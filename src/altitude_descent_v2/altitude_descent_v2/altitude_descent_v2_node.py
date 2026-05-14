#!/usr/bin/env python3

import json
import math
import time
from statistics import mean
from typing import Optional

import rclpy
from mavros_msgs.msg import (
    ESCStatus,
    ESCTelemetry,
    PositionTarget,
    RCOut,
    State,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Range
from std_msgs.msg import String


class PIDController:
    """Z 向高度轨迹 PID，带积分限幅、输出限幅和异常 dt 保护."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        out_limit: float,
        i_limit: float,
    ) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_limit = abs(float(out_limit))
        self.i_limit = abs(float(i_limit))
        self.reset()

    def reset(self) -> None:
        """清空历史误差，避免重新进入 OFFBOARD 时继承旧积分."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time: Optional[float] = None
        self.initialized = False
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.output = 0.0

    def update(self, error: float) -> float:
        """根据参考高度与测距高度的误差计算速度修正量."""
        now = time.time()
        error = float(error)
        self.p_term = self.kp * error

        if not self.initialized or self.prev_time is None:
            self.initialized = True
            self.prev_error = error
            self.prev_time = now
            self.i_term = self.ki * self.integral
            self.d_term = 0.0
            self.output = self._limit_output(self.p_term)
            return self.output

        dt = now - self.prev_time
        if dt <= 1e-6 or dt > 1.0 or not math.isfinite(dt):
            # 控制周期异常时禁用积分和微分更新，避免输出尖峰。
            self.prev_error = error
            self.prev_time = now
            self.i_term = self.ki * self.integral
            self.d_term = 0.0
            self.output = self._limit_output(self.p_term + self.i_term)
            return self.output

        # 积分项用于补偿下降速度跟踪中的稳态误差。
        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        self.i_term = self.ki * self.integral

        # 微分项根据误差变化率抑制贴近轨迹时的高度振荡。
        self.d_term = self.kd * (error - self.prev_error) / dt

        self.prev_error = error
        self.prev_time = now
        self.output = self._limit_output(
            self.p_term + self.i_term + self.d_term
        )
        return self.output

    def _limit_output(self, value: float) -> float:
        """对 PID 速度修正量做对称限幅."""
        return max(-self.out_limit, min(self.out_limit, float(value)))


class AltitudeDescentNode(Node):
    """基于测距仪高度轨迹的定速下降节点."""

    MOTOR_SOURCE_AUTO = 'auto'
    MOTOR_SOURCE_ESC_STATUS = 'esc_status'
    MOTOR_SOURCE_ESC_TELEMETRY = 'esc_telemetry'
    MOTOR_SOURCE_RC_OUT = 'rc_out'
    MOTOR_SOURCE_NONE = 'none'

    def __init__(self) -> None:
        super().__init__('altitude_descent_v2_node')

        # ===== MAVROS 输入/输出话题 =====
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('distance_sensor_topic', '/mavros/hrlv_ez4_pub')
        self.declare_parameter(
            'setpoint_raw_topic',
            '/mavros/setpoint_raw/local',
        )
        self.declare_parameter('status_topic', '/altitude_descent_v2/status')
        self.declare_parameter('esc_status_topic', '/mavros/esc_status/status')
        self.declare_parameter(
            'esc_telemetry_topic',
            '/mavros/esc_telemetry/telemetry',
        )
        self.declare_parameter('rc_out_topic', '/mavros/rc/out')

        # ===== 下降速度、轨迹 PID 与安全参数 =====
        self.declare_parameter('target_descent_speed', 0.2)
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.06)
        self.declare_parameter('pid_i_limit', 1.0)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('distance_sensor_timeout_sec', 0.5)
        self.declare_parameter('motor_signal_timeout_sec', 0.5)
        self.declare_parameter('vz_limit', 0.3)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('touchdown_range_offset', 0.18)
        self.declare_parameter('terminal_switch_height', 0.20)
        # 兼容旧 launch 参数；末端控制不再使用该参数做硬停止。
        self.declare_parameter('min_height_stop', 0.20)
        self.declare_parameter('require_offboard', True)

        # ===== 电机/油门观测参数 =====
        self.declare_parameter('motor_signal_source', self.MOTOR_SOURCE_AUTO)
        self.declare_parameter('rc_pwm_min', 1000.0)
        self.declare_parameter('rc_pwm_max', 2000.0)
        self.declare_parameter('rc_motor_channel_offset', 0)
        self.declare_parameter('rc_motor_channel_count', 4)

        self.state_topic = str(self.get_parameter('state_topic').value)
        self.distance_sensor_topic = str(
            self.get_parameter('distance_sensor_topic').value
        )
        self.setpoint_raw_topic = str(
            self.get_parameter('setpoint_raw_topic').value
        )
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.esc_status_topic = str(
            self.get_parameter('esc_status_topic').value
        )
        self.esc_telemetry_topic = str(
            self.get_parameter('esc_telemetry_topic').value
        )
        self.rc_out_topic = str(self.get_parameter('rc_out_topic').value)

        self.target_descent_speed = abs(
            float(self.get_parameter('target_descent_speed').value)
        )
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.ki_z = float(self.get_parameter('ki_z').value)
        self.kd_z = float(self.get_parameter('kd_z').value)
        self.pid_i_limit = abs(float(self.get_parameter('pid_i_limit').value))
        self.control_rate_hz = max(
            1.0,
            float(self.get_parameter('control_rate_hz').value),
        )
        self.distance_sensor_timeout_sec = float(
            self.get_parameter('distance_sensor_timeout_sec').value
        )
        self.motor_signal_timeout_sec = float(
            self.get_parameter('motor_signal_timeout_sec').value
        )
        self.vz_limit = abs(float(self.get_parameter('vz_limit').value))
        self.velocity_deadband = abs(
            float(self.get_parameter('velocity_deadband').value)
        )
        self.touchdown_range_offset = max(
            0.0,
            float(self.get_parameter('touchdown_range_offset').value),
        )
        self.terminal_switch_height = max(
            1e-6,
            abs(float(self.get_parameter('terminal_switch_height').value)),
        )
        self.terminal_descent_gain = (
            self.target_descent_speed / self.terminal_switch_height
        )
        self.min_height_stop = float(
            self.get_parameter('min_height_stop').value
        )
        self.require_offboard = self._as_bool(
            self.get_parameter('require_offboard').value
        )
        self.motor_signal_source = self._normalize_motor_signal_source(
            str(self.get_parameter('motor_signal_source').value)
        )
        self.rc_pwm_min = float(self.get_parameter('rc_pwm_min').value)
        self.rc_pwm_max = float(self.get_parameter('rc_pwm_max').value)
        self.rc_motor_channel_offset = max(
            0,
            int(self.get_parameter('rc_motor_channel_offset').value),
        )
        self.rc_motor_channel_count = int(
            self.get_parameter('rc_motor_channel_count').value
        )

        self.pid_z = PIDController(
            self.kp_z,
            self.ki_z,
            self.kd_z,
            self.vz_limit,
            self.pid_i_limit,
        )

        # ===== 最新输入缓存 =====
        self.current_state = State()
        self.distance_sensor_msg: Optional[Range] = None
        self.esc_status_msg: Optional[ESCStatus] = None
        self.esc_telemetry_msg: Optional[ESCTelemetry] = None
        self.rc_out_msg: Optional[RCOut] = None

        self.last_distance_sensor_time = None
        self.last_esc_status_time = None
        self.last_esc_telemetry_time = None
        self.last_rc_out_time = None

        # ===== 下降参考轨迹状态 =====
        self.trajectory_active = False
        self.trajectory_start_z = float('nan')
        self.trajectory_start_time_sec = float('nan')
        self.last_reference_z = float('nan')
        self.last_trajectory_elapsed_sec = float('nan')

        # ===== 状态发布字段缓存 =====
        self.last_height_error_m = float('nan')
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_vz = 0.0
        self.last_cmd_yaw_rate = 0.0
        self.last_height_above_touchdown = float('nan')
        self.last_terminal_descent_active = False
        self.last_descent_phase = 'waiting'
        self.last_distance_sensor_valid = False
        self.last_distance_sensor_status = '等待测距仪数据'
        self.last_low_height_stop_active = False
        self.latest_status = '等待 OFFBOARD 与测距仪数据'
        self.last_motor_signal = self._empty_motor_signal()

        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(State, self.state_topic, self._on_state, 10)
        self.create_subscription(
            Range,
            self.distance_sensor_topic,
            self._on_distance_sensor,
            mavros_qos,
        )
        self.create_subscription(
            ESCStatus,
            self.esc_status_topic,
            self._on_esc_status,
            mavros_qos,
        )
        self.create_subscription(
            ESCTelemetry,
            self.esc_telemetry_topic,
            self._on_esc_telemetry,
            mavros_qos,
        )
        self.create_subscription(
            RCOut,
            self.rc_out_topic,
            self._on_rc_out,
            mavros_qos,
        )

        self.setpoint_pub = self.create_publisher(
            PositionTarget,
            self.setpoint_raw_topic,
            10,
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.create_timer(1.0, self._log_status)

        self.get_logger().info(
            'altitude_descent_v2_node 已启动 | '
            f'state={self.state_topic} | '
            f'distance_sensor={self.distance_sensor_topic} | '
            f'setpoint_raw={self.setpoint_raw_topic}'
        )
        self.get_logger().info(
            '定速下降参数 | '
            f'target_descent_speed={self.target_descent_speed:.3f}m/s | '
            f'PID_Z=({self.kp_z:.3f},{self.ki_z:.3f},{self.kd_z:.3f}) | '
            f'vz_limit={self.vz_limit:.3f}m/s | '
            f'touchdown_range_offset={self.touchdown_range_offset:.3f}m | '
            f'terminal_switch_height={self.terminal_switch_height:.3f}m | '
            f'terminal_gain={self.terminal_descent_gain:.3f}1/s | '
            f'require_offboard={self.require_offboard}'
        )

    @staticmethod
    def _as_bool(value) -> bool:
        """兼容 launch 传入的 bool、数字与字符串."""
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    @classmethod
    def _normalize_motor_signal_source(cls, source: str) -> str:
        """将电机信号来源限制到允许枚举，非法值回退为 auto."""
        normalized = source.strip().lower()
        valid_sources = {
            cls.MOTOR_SOURCE_AUTO,
            cls.MOTOR_SOURCE_ESC_STATUS,
            cls.MOTOR_SOURCE_ESC_TELEMETRY,
            cls.MOTOR_SOURCE_RC_OUT,
            cls.MOTOR_SOURCE_NONE,
        }
        if normalized in valid_sources:
            return normalized
        return cls.MOTOR_SOURCE_AUTO

    @staticmethod
    def _apply_deadband(value: float, deadband: float) -> float:
        """把很小的速度指令压为零，减少接近目标轨迹时的抖动."""
        return 0.0 if abs(float(value)) < float(deadband) else float(value)

    @staticmethod
    def _limit_symmetric(value: float, limit: float) -> float:
        """对速度指令做对称限幅."""
        limit = abs(float(limit))
        return max(-limit, min(limit, float(value)))

    def _height_above_touchdown(self, distance_m: float) -> float:
        """把测距仪原始高度换算为离脚架接地点的有效高度."""
        if not math.isfinite(distance_m):
            return float('nan')
        return max(float(distance_m) - self.touchdown_range_offset, 0.0)

    @staticmethod
    def _mean_or_nan(values: list[float]) -> float:
        """计算均值；空列表返回 NaN，便于 CSV 后处理."""
        return float(mean(values)) if values else float('nan')

    def _empty_motor_signal(self) -> dict:
        """返回空电机信号结构，保证 status JSON 字段稳定."""
        return {
            'source': self.MOTOR_SOURCE_NONE,
            'esc_status_rpm': [],
            'esc_status_rpm_mean': float('nan'),
            'esc_status_age_sec': float('nan'),
            'esc_telemetry_rpm': [],
            'esc_telemetry_rpm_mean': float('nan'),
            'esc_telemetry_age_sec': float('nan'),
            'rc_out_pwm': [],
            'rc_out_pwm_mean': float('nan'),
            'rc_out_throttle_norm': float('nan'),
            'rc_out_age_sec': float('nan'),
        }

    def _now_sec(self) -> float:
        """返回当前 ROS 时间，单位秒."""
        return self.get_clock().now().nanoseconds / 1e9

    def _age_sec(self, stamp_time) -> float:
        """计算缓存样本年龄；从未收到时返回极大值."""
        if stamp_time is None:
            return 1e9
        return (self.get_clock().now() - stamp_time).nanoseconds / 1e9

    def _on_state(self, msg: State) -> None:
        """缓存飞控连接、解锁与模式状态."""
        self.current_state = msg

    def _on_distance_sensor(self, msg: Range) -> None:
        """缓存测距仪样本，供高度轨迹 PID 使用."""
        self.distance_sensor_msg = msg
        self.last_distance_sensor_time = self.get_clock().now()

    def _on_esc_status(self, msg: ESCStatus) -> None:
        """缓存 ESC_STATUS RPM 数据."""
        self.esc_status_msg = msg
        self.last_esc_status_time = self.get_clock().now()

    def _on_esc_telemetry(self, msg: ESCTelemetry) -> None:
        """缓存 ESC_TELEMETRY RPM 数据."""
        self.esc_telemetry_msg = msg
        self.last_esc_telemetry_time = self.get_clock().now()

    def _on_rc_out(self, msg: RCOut) -> None:
        """缓存 RCOut PWM 数据，作为无 RPM 时的油门等效观测量."""
        self.rc_out_msg = msg
        self.last_rc_out_time = self.get_clock().now()

    def _evaluate_distance_sensor(self) -> tuple[bool, float, float, str]:
        """检查测距仪数据是否 fresh、有限且位于有效量程内."""
        age_sec = self._age_sec(self.last_distance_sensor_time)
        if self.distance_sensor_msg is None:
            return False, float('nan'), age_sec, '测距仪无数据'

        distance_m = float(self.distance_sensor_msg.range)
        min_range = float(self.distance_sensor_msg.min_range)
        max_range = float(self.distance_sensor_msg.max_range)

        if age_sec > self.distance_sensor_timeout_sec:
            return False, distance_m, age_sec, f'测距仪超时 age={age_sec:.3f}s'
        if not math.isfinite(distance_m):
            return False, distance_m, age_sec, '测距值不是有限数'
        if distance_m < 0.0:
            return False, distance_m, age_sec, '测距值为负数'

        # 近地读数可能低于 min_range；只把负数和超过 max_range 当作硬故障。
        min_enabled = math.isfinite(min_range) and min_range > 0.0
        max_enabled = math.isfinite(max_range) and max_range > 0.0
        if min_enabled and distance_m < min_range:
            return (
                True,
                distance_m,
                age_sec,
                (
                    f'测距值低于 min_range={min_range:.3f}m，'
                    '按脚架偏置进入近地处理'
                ),
            )
        if max_enabled and distance_m > max_range:
            return (
                False,
                distance_m,
                age_sec,
                f'测距值高于 max_range={max_range:.3f}m',
            )

        return True, distance_m, age_sec, '测距仪有效'

    def _reset_descent_trajectory(self) -> None:
        """重置下降轨迹，下次有效控制会从当前高度重新开始."""
        self.trajectory_active = False
        self.trajectory_start_z = float('nan')
        self.trajectory_start_time_sec = float('nan')
        self.last_reference_z = float('nan')
        self.last_trajectory_elapsed_sec = float('nan')
        self.pid_z.reset()

    def _ensure_descent_trajectory(
        self,
        height_above_touchdown: float,
    ) -> None:
        """首次有效控制时用当前有效高度作为轨迹起点."""
        if self.trajectory_active:
            return
        self.trajectory_active = True
        self.trajectory_start_z = float(height_above_touchdown)
        self.trajectory_start_time_sec = self._now_sec()
        self.last_reference_z = float(height_above_touchdown)
        self.last_trajectory_elapsed_sec = 0.0
        self.pid_z.reset()

    def _update_reference_z(self) -> float:
        """按目标下降速度生成当前有效高度参考."""
        elapsed_sec = max(
            0.0,
            self._now_sec() - self.trajectory_start_time_sec,
        )
        self.last_trajectory_elapsed_sec = elapsed_sec
        self.last_reference_z = (
            self.trajectory_start_z
            - self.target_descent_speed * elapsed_sec
        )
        return self.last_reference_z

    def _extract_esc_status_rpm(self) -> list[int]:
        """从 ESCStatus 中提取非负 RPM 列表."""
        if self.esc_status_msg is None:
            return []
        rpm_values = []
        for item in self.esc_status_msg.esc_status:
            rpm = int(item.rpm)
            if rpm >= 0:
                rpm_values.append(rpm)
        return rpm_values

    def _extract_esc_telemetry_rpm(self) -> list[int]:
        """从 ESCTelemetry 中提取非负 RPM 列表."""
        if self.esc_telemetry_msg is None:
            return []
        rpm_values = []
        for item in self.esc_telemetry_msg.esc_telemetry:
            rpm = int(item.rpm)
            if rpm >= 0:
                rpm_values.append(rpm)
        return rpm_values

    def _selected_rc_pwm_values(self) -> list[int]:
        """按通道参数截取 RCOut 中的电机 PWM 通道."""
        if self.rc_out_msg is None:
            return []

        channels = [int(value) for value in self.rc_out_msg.channels]
        start = self.rc_motor_channel_offset
        if self.rc_motor_channel_count > 0:
            end = start + self.rc_motor_channel_count
            channels = channels[start:end]
        else:
            channels = channels[start:]

        # 0 通常表示通道缺失或未输出，避免拉低平均油门估计。
        return [value for value in channels if value > 0]

    def _normalize_pwm_values(self, pwm_values: list[int]) -> list[float]:
        """把 PWM 映射到 0~1 油门等效比例，并做边界裁剪."""
        span = self.rc_pwm_max - self.rc_pwm_min
        if span <= 1e-6 or not math.isfinite(span):
            return []

        normalized_values = []
        for pwm in pwm_values:
            normalized = (float(pwm) - self.rc_pwm_min) / span
            normalized_values.append(max(0.0, min(1.0, normalized)))
        return normalized_values

    def _build_motor_signal_snapshot(self) -> dict:
        """按优先级汇总 ESC RPM 与 RCOut PWM，并选择当前使用来源."""
        snapshot = self._empty_motor_signal()

        esc_status_age = self._age_sec(self.last_esc_status_time)
        esc_status_rpm = self._extract_esc_status_rpm()
        snapshot['esc_status_rpm'] = esc_status_rpm
        snapshot['esc_status_rpm_mean'] = self._mean_or_nan(esc_status_rpm)
        snapshot['esc_status_age_sec'] = esc_status_age

        esc_telemetry_age = self._age_sec(self.last_esc_telemetry_time)
        esc_telemetry_rpm = self._extract_esc_telemetry_rpm()
        snapshot['esc_telemetry_rpm'] = esc_telemetry_rpm
        snapshot['esc_telemetry_rpm_mean'] = self._mean_or_nan(
            esc_telemetry_rpm
        )
        snapshot['esc_telemetry_age_sec'] = esc_telemetry_age

        rc_out_age = self._age_sec(self.last_rc_out_time)
        rc_pwm_values = self._selected_rc_pwm_values()
        rc_norm_values = self._normalize_pwm_values(rc_pwm_values)
        snapshot['rc_out_pwm'] = rc_pwm_values
        snapshot['rc_out_pwm_mean'] = self._mean_or_nan(rc_pwm_values)
        snapshot['rc_out_throttle_norm'] = self._mean_or_nan(rc_norm_values)
        snapshot['rc_out_age_sec'] = rc_out_age

        esc_status_ok = (
            esc_status_rpm
            and esc_status_age <= self.motor_signal_timeout_sec
        )
        esc_telemetry_ok = (
            esc_telemetry_rpm
            and esc_telemetry_age <= self.motor_signal_timeout_sec
        )
        rc_out_ok = (
            rc_pwm_values
            and rc_out_age <= self.motor_signal_timeout_sec
        )

        source = self.motor_signal_source
        if source == self.MOTOR_SOURCE_AUTO:
            if esc_status_ok:
                snapshot['source'] = self.MOTOR_SOURCE_ESC_STATUS
            elif esc_telemetry_ok:
                snapshot['source'] = self.MOTOR_SOURCE_ESC_TELEMETRY
            elif rc_out_ok:
                snapshot['source'] = self.MOTOR_SOURCE_RC_OUT
            else:
                snapshot['source'] = self.MOTOR_SOURCE_NONE
        elif source == self.MOTOR_SOURCE_ESC_STATUS and esc_status_ok:
            snapshot['source'] = self.MOTOR_SOURCE_ESC_STATUS
        elif source == self.MOTOR_SOURCE_ESC_TELEMETRY and esc_telemetry_ok:
            snapshot['source'] = self.MOTOR_SOURCE_ESC_TELEMETRY
        elif source == self.MOTOR_SOURCE_RC_OUT and rc_out_ok:
            snapshot['source'] = self.MOTOR_SOURCE_RC_OUT
        else:
            snapshot['source'] = self.MOTOR_SOURCE_NONE

        return snapshot

    def _publish_body_velocity(
        self,
        vx_body: float,
        vy_body: float,
        vz_body: float,
        yaw_rate: float,
    ) -> None:
        """发布 BODY_NED 速度指令，下降按本包约定使用负 `vz`."""
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
        )
        msg.velocity.x = float(vx_body)
        msg.velocity.y = float(-vy_body)
        msg.velocity.z = float(vz_body)
        msg.yaw_rate = float(yaw_rate)
        self.setpoint_pub.publish(msg)

        self.last_cmd_vx = float(vx_body)
        self.last_cmd_vy = float(vy_body)
        self.last_cmd_vz = float(vz_body)
        self.last_cmd_yaw_rate = float(yaw_rate)

    def _publish_status(
        self,
        distance_m: float,
        distance_age_sec: float,
        control_active: bool,
    ) -> None:
        """发布结构化 JSON 状态，供 CSV logger 记录控制内部量."""
        status_msg = String()
        status_msg.data = json.dumps({
            'mode': self.current_state.mode,
            'armed': bool(self.current_state.armed),
            'connected': bool(self.current_state.connected),
            'control_active': bool(control_active),
            'target_descent_speed': self.target_descent_speed,
            'trajectory_start_z': self.trajectory_start_z,
            'reference_z': self.last_reference_z,
            'trajectory_elapsed_sec': self.last_trajectory_elapsed_sec,
            'distance_sensor_z': distance_m,
            'touchdown_range_offset': self.touchdown_range_offset,
            'height_above_touchdown': self.last_height_above_touchdown,
            'terminal_switch_height': self.terminal_switch_height,
            'terminal_descent_gain': self.terminal_descent_gain,
            'terminal_descent_active': self.last_terminal_descent_active,
            'descent_phase': self.last_descent_phase,
            'distance_sensor_age_sec': distance_age_sec,
            'distance_sensor_valid': self.last_distance_sensor_valid,
            'distance_sensor_status': self.last_distance_sensor_status,
            'min_height_stop': self.min_height_stop,
            'low_height_stop_active': self.last_low_height_stop_active,
            'height_error_m': self.last_height_error_m,
            'kp_z': self.kp_z,
            'ki_z': self.ki_z,
            'kd_z': self.kd_z,
            'pid_i_limit': self.pid_i_limit,
            'pid_p': self.pid_z.p_term,
            'pid_i': self.pid_z.i_term,
            'pid_d': self.pid_z.d_term,
            'pid_output': self.pid_z.output,
            'vz_limit': self.vz_limit,
            'velocity_deadband': self.velocity_deadband,
            'cmd_vx': self.last_cmd_vx,
            'cmd_vy': self.last_cmd_vy,
            'cmd_vz': self.last_cmd_vz,
            'cmd_yaw_rate': self.last_cmd_yaw_rate,
            'motor_signal_source_requested': self.motor_signal_source,
            'motor_signal_source_used': self.last_motor_signal['source'],
            'esc_status_rpm': self.last_motor_signal['esc_status_rpm'],
            'esc_status_rpm_mean': (
                self.last_motor_signal['esc_status_rpm_mean']
            ),
            'esc_status_age_sec': self.last_motor_signal['esc_status_age_sec'],
            'esc_telemetry_rpm': (
                self.last_motor_signal['esc_telemetry_rpm']
            ),
            'esc_telemetry_rpm_mean': (
                self.last_motor_signal['esc_telemetry_rpm_mean']
            ),
            'esc_telemetry_age_sec': (
                self.last_motor_signal['esc_telemetry_age_sec']
            ),
            'rc_out_pwm': self.last_motor_signal['rc_out_pwm'],
            'rc_out_pwm_mean': self.last_motor_signal['rc_out_pwm_mean'],
            'rc_out_throttle_norm': (
                self.last_motor_signal['rc_out_throttle_norm']
            ),
            'rc_out_age_sec': self.last_motor_signal['rc_out_age_sec'],
            'latest_status': self.latest_status,
        })
        self.status_pub.publish(status_msg)

    def _publish_zero_and_status(
        self,
        distance_m: float,
        distance_age_sec: float,
        status_text: str,
    ) -> None:
        """保护状态统一输出零速，并重置下降轨迹和 PID."""
        self._reset_descent_trajectory()
        self.last_height_error_m = float('nan')
        self.last_terminal_descent_active = False
        self._publish_body_velocity(0.0, 0.0, 0.0, 0.0)
        self.latest_status = status_text
        self._publish_status(distance_m, distance_age_sec, False)

    def _control_loop(self) -> None:
        """主控制循环：XY 与 yaw 固定为 0，Z 跟踪定速下降轨迹."""
        self.last_motor_signal = self._build_motor_signal_snapshot()
        valid, distance_m, distance_age_sec, distance_status = (
            self._evaluate_distance_sensor()
        )
        self.last_distance_sensor_valid = valid
        self.last_distance_sensor_status = distance_status
        self.last_height_above_touchdown = self._height_above_touchdown(
            distance_m
        )
        self.last_terminal_descent_active = False
        self.last_descent_phase = 'waiting'
        self.last_low_height_stop_active = False

        if self.require_offboard and self.current_state.mode != 'OFFBOARD':
            self.last_descent_phase = 'waiting_offboard'
            self._publish_zero_and_status(
                distance_m,
                distance_age_sec,
                f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速',
            )
            return

        if not valid:
            self.last_descent_phase = 'distance_invalid'
            self._publish_zero_and_status(
                distance_m,
                distance_age_sec,
                f'{distance_status}，输出零速并重置下降轨迹',
            )
            return

        height_above_touchdown = self.last_height_above_touchdown
        if not math.isfinite(height_above_touchdown):
            self.last_descent_phase = 'distance_invalid'
            self._publish_zero_and_status(
                distance_m,
                distance_age_sec,
                '有效高度不是有限数，输出零速并重置下降轨迹',
            )
            return

        if height_above_touchdown <= self.terminal_switch_height:
            # 末端阶段直接按 v_z=-k*h 输出，h 为离脚架接地点的有效高度。
            self._reset_descent_trajectory()
            self.last_reference_z = height_above_touchdown
            self.last_trajectory_elapsed_sec = 0.0
            self.last_height_error_m = 0.0
            self.last_terminal_descent_active = True
            self.last_descent_phase = 'terminal_linear'

            raw_vz = -self.terminal_descent_gain * height_above_touchdown
            vz = self._limit_symmetric(raw_vz, self.vz_limit)
            vz = self._apply_deadband(vz, self.velocity_deadband)

            self._publish_body_velocity(0.0, 0.0, vz, 0.0)
            self.latest_status = (
                '末端线性减速 | '
                f'speed={self.target_descent_speed:.3f}m/s | '
                f'raw_distance={distance_m:.3f}m | '
                f'h={height_above_touchdown:.3f}m | '
                f'h_s={self.terminal_switch_height:.3f}m | '
                f'k={self.terminal_descent_gain:.3f}1/s | '
                f'vz={vz:.3f}m/s | '
                f'motor_source={self.last_motor_signal["source"]}'
            )
            self._publish_status(distance_m, distance_age_sec, True)
            return

        self.last_descent_phase = 'trajectory_pid'
        self._ensure_descent_trajectory(height_above_touchdown)
        reference_z = self._update_reference_z()
        self.last_height_error_m = reference_z - height_above_touchdown

        pid_output = self.pid_z.update(self.last_height_error_m)
        raw_vz = -self.target_descent_speed + pid_output
        vz = self._limit_symmetric(raw_vz, self.vz_limit)
        vz = self._apply_deadband(vz, self.velocity_deadband)

        self._publish_body_velocity(0.0, 0.0, vz, 0.0)
        self.latest_status = (
            '定速下降闭环 | '
            f'speed={self.target_descent_speed:.3f}m/s | '
            f'ref_z={reference_z:.3f}m | '
            f'raw_distance={distance_m:.3f}m | '
            f'h={height_above_touchdown:.3f}m | '
            f'ez={self.last_height_error_m:.3f}m | '
            f'pid={pid_output:.3f}m/s | '
            f'vz={vz:.3f}m/s | '
            f'motor_source={self.last_motor_signal["source"]}'
        )
        self._publish_status(distance_m, distance_age_sec, True)

    def _log_status(self) -> None:
        """按 1Hz 输出摘要日志，便于实机观察."""
        self.get_logger().info(self.latest_status)


def main(args=None) -> None:
    """启动 altitude_descent_v2_node."""
    rclpy.init(args=args)
    node = AltitudeDescentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # SIGINT 可能落在清理阶段，避免 launch 停止时打印 traceback。
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except KeyboardInterrupt:
                    pass


if __name__ == '__main__':
    main()
