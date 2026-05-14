#!/usr/bin/env python3

import csv
import json
import math
import os
from datetime import datetime
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


class AltitudeDescentCsvLoggerNode(Node):
    """独立 CSV logger，记录定速下降控制与电机/油门观测信号."""

    def __init__(self) -> None:
        super().__init__('altitude_descent_v2_csv_logger_node')

        # ===== 输入话题 =====
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

        # ===== 输出与采样参数 =====
        self.declare_parameter(
            'output_dir',
            '/home/zjh/project/rasip_pi_ws/log/altitude_descent_v2_csv',
        )
        self.declare_parameter('file_prefix', 'altitude_descent_v2')
        self.declare_parameter('sample_rate_hz', 30.0)
        self.declare_parameter('stale_timeout_sec', 0.5)
        self.declare_parameter('flush_interval_sec', 1.0)

        # ===== 控制参数快照 =====
        self.declare_parameter('target_descent_speed', 0.2)
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.06)
        self.declare_parameter('pid_i_limit', 1.0)
        self.declare_parameter('vz_limit', 0.3)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('touchdown_range_offset', 0.18)
        self.declare_parameter('terminal_switch_height', 0.20)
        # 兼容旧字段；控制节点不再使用该参数做近地硬停止。
        self.declare_parameter('min_height_stop', 0.20)
        self.declare_parameter('distance_sensor_timeout_sec', 0.5)
        self.declare_parameter('motor_signal_timeout_sec', 0.5)
        self.declare_parameter('motor_signal_source', 'auto')
        self.declare_parameter('rc_pwm_min', 1000.0)
        self.declare_parameter('rc_pwm_max', 2000.0)
        self.declare_parameter('rc_motor_channel_offset', 0)
        self.declare_parameter('rc_motor_channel_count', 4)
        self.declare_parameter('require_offboard', True)

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

        self.output_dir = str(self.get_parameter('output_dir').value)
        self.file_prefix = str(self.get_parameter('file_prefix').value)
        self.sample_rate_hz = max(
            1.0,
            float(self.get_parameter('sample_rate_hz').value),
        )
        self.stale_timeout_sec = float(
            self.get_parameter('stale_timeout_sec').value
        )
        self.flush_interval_sec = float(
            self.get_parameter('flush_interval_sec').value
        )
        self.distance_sensor_timeout_sec = float(
            self.get_parameter('distance_sensor_timeout_sec').value
        )
        self.motor_signal_timeout_sec = float(
            self.get_parameter('motor_signal_timeout_sec').value
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

        self.param_snapshot = {
            'target_descent_speed': abs(
                float(self.get_parameter('target_descent_speed').value)
            ),
            'kp_z': float(self.get_parameter('kp_z').value),
            'ki_z': float(self.get_parameter('ki_z').value),
            'kd_z': float(self.get_parameter('kd_z').value),
            'pid_i_limit': float(self.get_parameter('pid_i_limit').value),
            'vz_limit': float(self.get_parameter('vz_limit').value),
            'velocity_deadband': float(
                self.get_parameter('velocity_deadband').value
            ),
            'touchdown_range_offset': max(
                0.0,
                float(self.get_parameter('touchdown_range_offset').value),
            ),
            'terminal_switch_height': max(
                1e-6,
                abs(float(self.get_parameter('terminal_switch_height').value)),
            ),
            'min_height_stop': float(
                self.get_parameter('min_height_stop').value
            ),
            'distance_sensor_timeout_sec': self.distance_sensor_timeout_sec,
            'motor_signal_timeout_sec': self.motor_signal_timeout_sec,
            'motor_signal_source': str(
                self.get_parameter('motor_signal_source').value
            ),
            'rc_pwm_min': self.rc_pwm_min,
            'rc_pwm_max': self.rc_pwm_max,
            'rc_motor_channel_offset': self.rc_motor_channel_offset,
            'rc_motor_channel_count': self.rc_motor_channel_count,
            'require_offboard': self._as_bool(
                self.get_parameter('require_offboard').value
            ),
        }
        self.param_snapshot['terminal_descent_gain'] = (
            self.param_snapshot['target_descent_speed']
            / self.param_snapshot['terminal_switch_height']
        )

        self.state_msg: Optional[State] = None
        self.distance_sensor_msg: Optional[Range] = None
        self.setpoint_msg: Optional[PositionTarget] = None
        self.status_msg: Optional[dict] = None
        self.esc_status_msg: Optional[ESCStatus] = None
        self.esc_telemetry_msg: Optional[ESCTelemetry] = None
        self.rc_out_msg: Optional[RCOut] = None

        self.distance_sensor_rx_time = None
        self.setpoint_rx_time = None
        self.status_rx_time = None
        self.esc_status_rx_time = None
        self.esc_telemetry_rx_time = None
        self.rc_out_rx_time = None
        self.last_flush_time = self.get_clock().now()

        os.makedirs(self.output_dir, exist_ok=True)
        now_text = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir,
            f'{self.file_prefix}_{now_text}.csv',
        )
        self.csv_file = open(
            self.csv_path,
            'w',
            newline='',
            encoding='utf-8',
        )
        self.csv_writer = csv.writer(self.csv_file, lineterminator='\n')
        self.csv_writer.writerow([
            'ros_time_sec',
            'mode',
            'armed',
            'connected',
            'control_active',
            'target_descent_speed',
            'trajectory_start_z',
            'reference_z',
            'trajectory_elapsed_sec',
            'distance_sensor_z',
            'touchdown_range_offset',
            'height_above_touchdown',
            'terminal_switch_height',
            'terminal_descent_gain',
            'terminal_descent_active',
            'descent_phase',
            'distance_sensor_fresh',
            'distance_sensor_age_sec',
            'distance_sensor_valid',
            'distance_sensor_status',
            'min_height_stop',
            'low_height_stop_active',
            'height_error_m',
            'kp_z',
            'ki_z',
            'kd_z',
            'pid_i_limit',
            'pid_p',
            'pid_i',
            'pid_d',
            'pid_output',
            'vz_limit',
            'velocity_deadband',
            'cmd_vx',
            'cmd_vy',
            'cmd_vz',
            'cmd_yaw_rate',
            'setpoint_fresh',
            'setpoint_age_sec',
            'sp_frame',
            'sp_type_mask',
            'sp_vx',
            'sp_vy',
            'sp_vz',
            'sp_yaw_rate',
            'esc_status_fresh',
            'esc_status_age_sec',
            'esc_status_rpm_values',
            'esc_status_rpm_mean',
            'esc_telemetry_fresh',
            'esc_telemetry_age_sec',
            'esc_telemetry_rpm_values',
            'esc_telemetry_rpm_mean',
            'rc_out_fresh',
            'rc_out_age_sec',
            'rc_out_pwm_values',
            'rc_out_pwm_mean',
            'rc_out_throttle_norm',
            'motor_signal_source_requested',
            'motor_signal_source_used',
            'status_fresh',
            'status_age_sec',
            'latest_status',
        ])
        self.csv_file.flush()

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
            PositionTarget,
            self.setpoint_raw_topic,
            self._on_setpoint,
            10,
        )
        self.create_subscription(
            String,
            self.status_topic,
            self._on_status,
            10,
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

        self.create_timer(1.0 / self.sample_rate_hz, self._write_row)
        self.create_timer(0.5, self._periodic_flush)

        self.get_logger().info('altitude_descent_v2_csv_logger_node 已启动')
        self.get_logger().info(f'CSV 输出: {self.csv_path}')

    @staticmethod
    def _as_bool(value) -> bool:
        """兼容 launch 传入的 bool、数字与字符串."""
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    @staticmethod
    def _stamp_now_sec(now_time) -> float:
        """将 ROS Time 转为秒."""
        now_msg = now_time.to_msg()
        return float(now_msg.sec) + float(now_msg.nanosec) * 1e-9

    @staticmethod
    def _format_sequence(values) -> str:
        """把数组压成分号分隔字符串，便于 CSV 单列保存."""
        return ';'.join(str(value) for value in values)

    @staticmethod
    def _mean_or_nan(values: list[float]) -> float:
        """计算均值；空列表返回 NaN."""
        return float(mean(values)) if values else float('nan')

    def _height_above_touchdown(self, distance_m: float) -> float:
        """按脚架偏置计算控制用有效高度."""
        if not math.isfinite(distance_m):
            return float('nan')
        return max(
            float(distance_m) - self.param_snapshot['touchdown_range_offset'],
            0.0,
        )

    def _age_sec(self, stamp_time, now_time) -> float:
        """计算样本年龄；从未收到时返回 NaN."""
        if stamp_time is None:
            return float('nan')
        return (now_time - stamp_time).nanoseconds / 1e9

    def _is_fresh(self, stamp_time, now_time, timeout_sec: float) -> bool:
        """判断样本是否在 timeout 内."""
        age_sec = self._age_sec(stamp_time, now_time)
        return math.isfinite(age_sec) and age_sec <= timeout_sec

    def _range_valid(self, msg: Optional[Range], fresh: bool) -> bool:
        """判断测距仪样本是否可作为有效高度记录."""
        if msg is None or not fresh:
            return False
        distance_m = float(msg.range)
        min_range = float(msg.min_range)
        max_range = float(msg.max_range)
        if not math.isfinite(distance_m):
            return False
        if distance_m < 0.0:
            return False

        min_enabled = math.isfinite(min_range) and min_range > 0.0
        max_enabled = math.isfinite(max_range) and max_range > 0.0
        # 近地阶段测距仪可能低于消息 min_range，但该值仍用于脚架偏置减速。
        if min_enabled and distance_m < min_range:
            return True
        if max_enabled and distance_m > max_range:
            return False
        return True

    def _extract_esc_status_rpm(self) -> list[int]:
        """从 ESCStatus 中提取 RPM 列表."""
        if self.esc_status_msg is None:
            return []
        return [
            int(item.rpm)
            for item in self.esc_status_msg.esc_status
            if int(item.rpm) >= 0
        ]

    def _extract_esc_telemetry_rpm(self) -> list[int]:
        """从 ESCTelemetry 中提取 RPM 列表."""
        if self.esc_telemetry_msg is None:
            return []
        return [
            int(item.rpm)
            for item in self.esc_telemetry_msg.esc_telemetry
            if int(item.rpm) >= 0
        ]

    def _selected_rc_pwm_values(self) -> list[int]:
        """按配置截取电机 PWM 通道."""
        if self.rc_out_msg is None:
            return []
        channels = [int(value) for value in self.rc_out_msg.channels]
        start = self.rc_motor_channel_offset
        if self.rc_motor_channel_count > 0:
            channels = channels[start:start + self.rc_motor_channel_count]
        else:
            channels = channels[start:]
        return [value for value in channels if value > 0]

    def _normalize_pwm_values(self, pwm_values: list[int]) -> list[float]:
        """把 PWM 映射为 0~1 油门等效比例."""
        span = self.rc_pwm_max - self.rc_pwm_min
        if span <= 1e-6 or not math.isfinite(span):
            return []
        normalized_values = []
        for pwm in pwm_values:
            normalized = (float(pwm) - self.rc_pwm_min) / span
            normalized_values.append(max(0.0, min(1.0, normalized)))
        return normalized_values

    def _infer_motor_signal_source(
        self,
        esc_status_fresh: bool,
        esc_status_rpm: list[int],
        esc_telemetry_fresh: bool,
        esc_telemetry_rpm: list[int],
        rc_out_fresh: bool,
        rc_pwm_values: list[int],
    ) -> str:
        """当 status 不可用时按与控制节点一致的优先级推断来源."""
        requested = self.param_snapshot['motor_signal_source'].strip().lower()
        if requested in ('none', ''):
            return 'none'

        esc_status_ok = esc_status_fresh and bool(esc_status_rpm)
        esc_telemetry_ok = esc_telemetry_fresh and bool(esc_telemetry_rpm)
        rc_out_ok = rc_out_fresh and bool(rc_pwm_values)

        if requested == 'esc_status':
            return 'esc_status' if esc_status_ok else 'none'
        if requested == 'esc_telemetry':
            return 'esc_telemetry' if esc_telemetry_ok else 'none'
        if requested == 'rc_out':
            return 'rc_out' if rc_out_ok else 'none'

        if esc_status_ok:
            return 'esc_status'
        if esc_telemetry_ok:
            return 'esc_telemetry'
        if rc_out_ok:
            return 'rc_out'
        return 'none'

    def _on_state(self, msg: State) -> None:
        """缓存飞控状态."""
        self.state_msg = msg

    def _on_distance_sensor(self, msg: Range) -> None:
        """缓存测距仪样本."""
        self.distance_sensor_msg = msg
        self.distance_sensor_rx_time = self.get_clock().now()

    def _on_setpoint(self, msg: PositionTarget) -> None:
        """缓存 setpoint_raw 指令."""
        self.setpoint_msg = msg
        self.setpoint_rx_time = self.get_clock().now()

    def _on_status(self, msg: String) -> None:
        """解析控制节点发布的结构化状态."""
        try:
            self.status_msg = json.loads(msg.data)
            self.status_rx_time = self.get_clock().now()
        except Exception as exc:
            self.get_logger().warn(
                f'解析 altitude_descent_v2/status 失败: {exc}'
            )

    def _on_esc_status(self, msg: ESCStatus) -> None:
        """缓存 ESCStatus 样本."""
        self.esc_status_msg = msg
        self.esc_status_rx_time = self.get_clock().now()

    def _on_esc_telemetry(self, msg: ESCTelemetry) -> None:
        """缓存 ESCTelemetry 样本."""
        self.esc_telemetry_msg = msg
        self.esc_telemetry_rx_time = self.get_clock().now()

    def _on_rc_out(self, msg: RCOut) -> None:
        """缓存 RCOut 样本."""
        self.rc_out_msg = msg
        self.rc_out_rx_time = self.get_clock().now()

    def _status_value(self, key: str, default):
        """读取 status JSON 字段；缺失时使用默认值."""
        if self.status_msg is None:
            return default
        return self.status_msg.get(key, default)

    def _write_row(self) -> None:
        """按固定采样频率写一行 CSV."""
        now_time = self.get_clock().now()
        ros_time_sec = self._stamp_now_sec(now_time)

        status_fresh = self._is_fresh(
            self.status_rx_time,
            now_time,
            self.stale_timeout_sec,
        )
        status_age = self._age_sec(self.status_rx_time, now_time)

        mode = self.state_msg.mode if self.state_msg is not None else ''
        armed = int(self.state_msg.armed) if self.state_msg is not None else -1
        connected = (
            int(self.state_msg.connected) if self.state_msg is not None else -1
        )

        distance_age = self._age_sec(self.distance_sensor_rx_time, now_time)
        distance_fresh = self._is_fresh(
            self.distance_sensor_rx_time,
            now_time,
            self.distance_sensor_timeout_sec,
        )
        if self.distance_sensor_msg is not None:
            distance_z = float(self.distance_sensor_msg.range)
        else:
            distance_z = float('nan')
        distance_valid = self._range_valid(
            self.distance_sensor_msg,
            distance_fresh,
        )
        height_above_touchdown = self._height_above_touchdown(distance_z)

        target_descent_speed = self._status_value(
            'target_descent_speed',
            self.param_snapshot['target_descent_speed'],
        )
        trajectory_start_z = self._status_value(
            'trajectory_start_z',
            float('nan'),
        )
        reference_z = self._status_value('reference_z', float('nan'))
        trajectory_elapsed_sec = self._status_value(
            'trajectory_elapsed_sec',
            float('nan'),
        )
        touchdown_range_offset = self._status_value(
            'touchdown_range_offset',
            self.param_snapshot['touchdown_range_offset'],
        )
        height_above_touchdown = self._status_value(
            'height_above_touchdown',
            height_above_touchdown,
        )
        terminal_switch_height = self._status_value(
            'terminal_switch_height',
            self.param_snapshot['terminal_switch_height'],
        )
        terminal_descent_gain = self._status_value(
            'terminal_descent_gain',
            self.param_snapshot['terminal_descent_gain'],
        )
        if status_fresh:
            control_active = int(self._status_value('control_active', False))
            terminal_descent_active = int(
                self._status_value('terminal_descent_active', False)
            )
            descent_phase = self._status_value('descent_phase', '')
            distance_status = self._status_value('distance_sensor_status', '')
            low_height_stop_active = int(
                self._status_value('low_height_stop_active', False)
            )
            height_error = self._status_value('height_error_m', float('nan'))
            pid_p = self._status_value('pid_p', float('nan'))
            pid_i = self._status_value('pid_i', float('nan'))
            pid_d = self._status_value('pid_d', float('nan'))
            pid_output = self._status_value('pid_output', float('nan'))
            cmd_vx = self._status_value('cmd_vx', float('nan'))
            cmd_vy = self._status_value('cmd_vy', float('nan'))
            cmd_vz = self._status_value('cmd_vz', float('nan'))
            cmd_yaw_rate = self._status_value('cmd_yaw_rate', float('nan'))
            motor_source_used = self._status_value(
                'motor_signal_source_used',
                'none',
            )
            latest_status = self._status_value('latest_status', '')
        else:
            control_active = 0
            terminal_descent_active = 0
            descent_phase = ''
            distance_status = ''
            low_height_stop_active = 0
            height_error = (
                reference_z - height_above_touchdown
                if distance_valid and math.isfinite(reference_z)
                else float('nan')
            )
            pid_p = float('nan')
            pid_i = float('nan')
            pid_d = float('nan')
            pid_output = float('nan')
            cmd_vx = float('nan')
            cmd_vy = float('nan')
            cmd_vz = float('nan')
            cmd_yaw_rate = float('nan')
            latest_status = ''
            motor_source_used = ''

        setpoint_age = self._age_sec(self.setpoint_rx_time, now_time)
        setpoint_fresh = self._is_fresh(
            self.setpoint_rx_time,
            now_time,
            self.stale_timeout_sec,
        )
        if self.setpoint_msg is not None:
            sp_frame = int(self.setpoint_msg.coordinate_frame)
            sp_type_mask = int(self.setpoint_msg.type_mask)
            sp_vx = float(self.setpoint_msg.velocity.x)
            sp_vy = float(self.setpoint_msg.velocity.y)
            sp_vz = float(self.setpoint_msg.velocity.z)
            sp_yaw_rate = float(self.setpoint_msg.yaw_rate)
        else:
            sp_frame = -1
            sp_type_mask = -1
            sp_vx = float('nan')
            sp_vy = float('nan')
            sp_vz = float('nan')
            sp_yaw_rate = float('nan')

        esc_status_age = self._age_sec(self.esc_status_rx_time, now_time)
        esc_status_fresh = self._is_fresh(
            self.esc_status_rx_time,
            now_time,
            self.motor_signal_timeout_sec,
        )
        esc_status_rpm = self._extract_esc_status_rpm()

        esc_telemetry_age = self._age_sec(
            self.esc_telemetry_rx_time,
            now_time,
        )
        esc_telemetry_fresh = self._is_fresh(
            self.esc_telemetry_rx_time,
            now_time,
            self.motor_signal_timeout_sec,
        )
        esc_telemetry_rpm = self._extract_esc_telemetry_rpm()

        rc_out_age = self._age_sec(self.rc_out_rx_time, now_time)
        rc_out_fresh = self._is_fresh(
            self.rc_out_rx_time,
            now_time,
            self.motor_signal_timeout_sec,
        )
        rc_pwm_values = self._selected_rc_pwm_values()
        rc_norm_values = self._normalize_pwm_values(rc_pwm_values)
        if not status_fresh:
            motor_source_used = self._infer_motor_signal_source(
                esc_status_fresh,
                esc_status_rpm,
                esc_telemetry_fresh,
                esc_telemetry_rpm,
                rc_out_fresh,
                rc_pwm_values,
            )

        self.csv_writer.writerow([
            ros_time_sec,
            mode,
            armed,
            connected,
            control_active,
            target_descent_speed,
            trajectory_start_z,
            reference_z,
            trajectory_elapsed_sec,
            distance_z,
            touchdown_range_offset,
            height_above_touchdown,
            terminal_switch_height,
            terminal_descent_gain,
            terminal_descent_active,
            descent_phase,
            int(distance_fresh),
            distance_age,
            int(distance_valid),
            distance_status,
            self.param_snapshot['min_height_stop'],
            low_height_stop_active,
            height_error,
            self.param_snapshot['kp_z'],
            self.param_snapshot['ki_z'],
            self.param_snapshot['kd_z'],
            self.param_snapshot['pid_i_limit'],
            pid_p,
            pid_i,
            pid_d,
            pid_output,
            self.param_snapshot['vz_limit'],
            self.param_snapshot['velocity_deadband'],
            cmd_vx,
            cmd_vy,
            cmd_vz,
            cmd_yaw_rate,
            int(setpoint_fresh),
            setpoint_age,
            sp_frame,
            sp_type_mask,
            sp_vx,
            sp_vy,
            sp_vz,
            sp_yaw_rate,
            int(esc_status_fresh),
            esc_status_age,
            self._format_sequence(esc_status_rpm),
            self._mean_or_nan(esc_status_rpm),
            int(esc_telemetry_fresh),
            esc_telemetry_age,
            self._format_sequence(esc_telemetry_rpm),
            self._mean_or_nan(esc_telemetry_rpm),
            int(rc_out_fresh),
            rc_out_age,
            self._format_sequence(rc_pwm_values),
            self._mean_or_nan(rc_pwm_values),
            self._mean_or_nan(rc_norm_values),
            self.param_snapshot['motor_signal_source'],
            motor_source_used,
            int(status_fresh),
            status_age,
            latest_status,
        ])

    def _periodic_flush(self) -> None:
        """按配置周期 flush，避免控制记录长时间停留在缓冲区."""
        elapsed_sec = (
            self.get_clock().now() - self.last_flush_time
        ).nanoseconds / 1e9
        if elapsed_sec < self.flush_interval_sec:
            return
        self.csv_file.flush()
        self.last_flush_time = self.get_clock().now()

    def destroy_node(self) -> bool:
        """节点退出前确保 CSV 文件落盘并关闭."""
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception as exc:
            self.get_logger().warn(f'关闭 CSV 文件失败: {exc}')
        return super().destroy_node()


def main(args=None) -> None:
    """启动 altitude_descent_v2_csv_logger_node."""
    rclpy.init(args=args)
    node = AltitudeDescentCsvLoggerNode()
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
