#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from debug_interface.msg import ArucoBasePose
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range


class PIDController:
    """简单 PID 控制器，带积分限幅和输出限幅."""

    def __init__(self, kp=0.5, ki=0.0, kd=0.05, out_limit=0.8, i_limit=1.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_limit = abs(float(out_limit))
        self.i_limit = abs(float(i_limit))

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def reset(self):
        """在模式切换或数据超时时清空状态，避免历史积分带来突跳."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def update(self, error: float) -> float:
        """根据当前误差计算控制输出."""
        now = time.time()
        dt = now - self.prev_time
        if dt <= 1e-6:
            dt = 1e-2

        # 比例项直接反映当前误差。
        p_term = self.kp * error

        # 积分项用于消除稳态误差，并做限幅防止积分饱和。
        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        i_term = self.ki * self.integral

        # 微分项利用误差变化率抑制过冲。
        d_term = self.kd * (error - self.prev_error) / dt

        out = p_term + i_term + d_term
        out = max(-self.out_limit, min(self.out_limit, out))

        self.prev_error = error
        self.prev_time = now
        return out


class PidTuningV4Node(Node):
    """BODY_NED 控制节点：在 v3 基础上将 Z 改为距离传感器闭环."""

    def __init__(self):
        super().__init__('pid_tuning_v4_node')

        # ===== 输入/输出话题 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('distance_sensor_topic', '/mavros/hrlv_ez4_pub')
        self.declare_parameter('setpoint_raw_topic', '/mavros/setpoint_raw/local')

        # ===== 控制目标 =====
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 2.5)

        # ===== XY PID：兼容粗调入口 =====
        self.declare_parameter('kp_xy', 0.5)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.08)

        # ===== XY PID：分轴精调入口（NaN 表示未单独配置） =====
        self.declare_parameter('kp_x', float('nan'))
        self.declare_parameter('ki_x', float('nan'))
        self.declare_parameter('kd_x', float('nan'))
        self.declare_parameter('kp_y', float('nan'))
        self.declare_parameter('ki_y', float('nan'))
        self.declare_parameter('kd_y', float('nan'))

        # ===== Z PID 与 yaw 修正参数 =====
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.06)
        self.declare_parameter('camera_yaw_compensation_deg', 0.0)

        # ===== 安全参数 =====
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('distance_sensor_timeout_sec', 0.5)
        self.declare_parameter('vxy_limit', 0.8)
        self.declare_parameter('vx_limit', float('nan'))
        self.declare_parameter('vy_limit', float('nan'))
        self.declare_parameter('vz_limit', 0.5)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('enable_z_hold', True)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.distance_sensor_topic = self.get_parameter('distance_sensor_topic').value
        self.setpoint_raw_topic = self.get_parameter('setpoint_raw_topic').value

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_z = float(self.get_parameter('target_z').value)

        kp_xy = float(self.get_parameter('kp_xy').value)
        ki_xy = float(self.get_parameter('ki_xy').value)
        kd_xy = float(self.get_parameter('kd_xy').value)

        kp_x_raw = float(self.get_parameter('kp_x').value)
        ki_x_raw = float(self.get_parameter('ki_x').value)
        kd_x_raw = float(self.get_parameter('kd_x').value)
        kp_y_raw = float(self.get_parameter('kp_y').value)
        ki_y_raw = float(self.get_parameter('ki_y').value)
        kd_y_raw = float(self.get_parameter('kd_y').value)

        # 分轴参数优先于粗调参数，方便从统一调参逐步切到分轴精调。
        self.x_use_split = (
            math.isfinite(kp_x_raw)
            or math.isfinite(ki_x_raw)
            or math.isfinite(kd_x_raw)
        )
        self.y_use_split = (
            math.isfinite(kp_y_raw)
            or math.isfinite(ki_y_raw)
            or math.isfinite(kd_y_raw)
        )

        self.kp_x = kp_x_raw if math.isfinite(kp_x_raw) else kp_xy
        self.ki_x = ki_x_raw if math.isfinite(ki_x_raw) else ki_xy
        self.kd_x = kd_x_raw if math.isfinite(kd_x_raw) else kd_xy
        self.kp_y = kp_y_raw if math.isfinite(kp_y_raw) else kp_xy
        self.ki_y = ki_y_raw if math.isfinite(ki_y_raw) else ki_xy
        self.kd_y = kd_y_raw if math.isfinite(kd_y_raw) else kd_xy

        self.kp_z = float(self.get_parameter('kp_z').value)
        self.ki_z = float(self.get_parameter('ki_z').value)
        self.kd_z = float(self.get_parameter('kd_z').value)

        self.camera_yaw_compensation_deg = float(
            self.get_parameter('camera_yaw_compensation_deg').value
        )
        self.camera_yaw_compensation_rad = math.radians(
            self.camera_yaw_compensation_deg
        )

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
        self.distance_sensor_timeout_sec = float(
            self.get_parameter('distance_sensor_timeout_sec').value
        )
        self.vxy_limit = float(self.get_parameter('vxy_limit').value)
        vx_limit_raw = float(self.get_parameter('vx_limit').value)
        vy_limit_raw = float(self.get_parameter('vy_limit').value)
        self.vx_limit = vx_limit_raw if math.isfinite(vx_limit_raw) else self.vxy_limit
        self.vy_limit = vy_limit_raw if math.isfinite(vy_limit_raw) else self.vxy_limit
        self.vz_limit = float(self.get_parameter('vz_limit').value)
        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)
        self.require_offboard = bool(self.get_parameter('require_offboard').value)
        self.enable_z_hold = bool(self.get_parameter('enable_z_hold').value)

        # ===== PID 控制器 =====
        self.pid_x = PIDController(
            self.kp_x,
            self.ki_x,
            self.kd_x,
            out_limit=self.vx_limit,
        )
        self.pid_y = PIDController(
            self.kp_y,
            self.ki_y,
            self.kd_y,
            out_limit=self.vy_limit,
        )
        self.pid_z = PIDController(
            self.kp_z,
            self.ki_z,
            self.kd_z,
            out_limit=self.vz_limit,
        )

        # ===== 状态缓存 =====
        self.current_state = State()
        self.pose = ArucoBasePose()
        self.distance_sensor_msg: Optional[Range] = None

        self.has_pose = False
        self.last_pose_time = None
        self.last_distance_sensor_time = None
        self.latest_status = '等待视觉、测距仪与飞控状态数据'

        mavros_sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.state_sub = self.create_subscription(
            State,
            self.state_topic,
            self.state_callback,
            10,
        )
        self.pose_sub = self.create_subscription(
            ArucoBasePose,
            self.pose_topic,
            self.pose_callback,
            10,
        )
        self.distance_sensor_sub = self.create_subscription(
            Range,
            self.distance_sensor_topic,
            self.distance_sensor_callback,
            mavros_sensor_qos,
        )
        self.setpoint_pub = self.create_publisher(
            PositionTarget,
            self.setpoint_raw_topic,
            10,
        )

        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1.0),
            self.control_loop,
        )
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'pid_tuning_v4_node 已启动 | '
            f'pose={self.pose_topic} | state={self.state_topic} | '
            f'distance_sensor={self.distance_sensor_topic} | '
            f'setpoint_raw={self.setpoint_raw_topic}'
        )
        self.get_logger().info(
            'PID参数 | '
            f'X=({self.kp_x:.3f},{self.ki_x:.3f},{self.kd_x:.3f}) '
            f'Y=({self.kp_y:.3f},{self.ki_y:.3f},{self.kd_y:.3f}) '
            f'Z=({self.kp_z:.3f},{self.ki_z:.3f},{self.kd_z:.3f})'
        )
        self.get_logger().info(
            '速度限幅 | '
            f'vxy_limit={self.vxy_limit:.3f} | vx_limit={self.vx_limit:.3f} | '
            f'vy_limit={self.vy_limit:.3f} | vz_limit={self.vz_limit:.3f}'
        )
        self.get_logger().info(
            '参数来源 | '
            f'X轴={"split(kp_x/ki_x/kd_x)" if self.x_use_split else "legacy(kp_xy/ki_xy/kd_xy)"} | '
            f'Y轴={"split(kp_y/ki_y/kd_y)" if self.y_use_split else "legacy(kp_xy/ki_xy/kd_xy)"}'
        )
        self.get_logger().info(
            'yaw 修正 | '
            f'camera_yaw_compensation_deg={self.camera_yaw_compensation_deg:.3f} '
            '(俯视逆时针为正；当前视觉链 yaw 在控制前先取反，再叠加固定补偿)'
        )
        self.get_logger().info(
            'Z 高度来源已切换为距离传感器 | '
            f'distance_sensor_timeout_sec={self.distance_sensor_timeout_sec:.3f}'
        )
        self.get_logger().info('yaw 闭环已关闭，yaw_rate 固定输出 0.0')

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        """将角度归一化到 [-pi, pi]."""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def apply_deadband(value: float, deadband: float) -> float:
        """把很小的速度指令压到零，减少悬停抖动."""
        return 0.0 if abs(value) < deadband else value

    @staticmethod
    def rotate_marker_error_to_body(
        ex_marker: float,
        ey_marker: float,
        yaw_rel_corrected: float,
    ) -> tuple[float, float]:
        """
        将 marker 平面误差旋转到机体平面误差.

        这里严格采用：
        e_body = R(-yaw_rel_corrected) * e_marker

        其中：
        - ex_marker = target_x - pose.x
        - ey_marker = pose.y - target_y
        - yaw_rel_corrected = 控制使用的相对 yaw + 固定补偿角
        """
        cos_yaw = math.cos(yaw_rel_corrected)
        sin_yaw = math.sin(yaw_rel_corrected)
        ex_body = cos_yaw * ex_marker + sin_yaw * ey_marker
        ey_body = -sin_yaw * ex_marker + cos_yaw * ey_marker
        return ex_body, ey_body

    def reset_all_pids(self):
        """统一重置所有控制器，避免状态残留影响下一次接管."""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()

    def state_callback(self, msg: State):
        self.current_state = msg

    def pose_callback(self, msg: ArucoBasePose):
        self.pose = msg
        self.has_pose = True
        self.last_pose_time = self.get_clock().now()

    def distance_sensor_callback(self, msg: Range):
        """缓存最近一次测距仪样本，供 Z 闭环与日志使用."""
        self.distance_sensor_msg = msg
        self.last_distance_sensor_time = self.get_clock().now()

    def is_pose_fresh(self) -> bool:
        """判断视觉位姿是否在允许时限内."""
        if not self.has_pose or self.last_pose_time is None:
            return False
        dt = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        return dt <= self.pose_timeout_sec

    def get_distance_sensor_age_sec(self) -> float:
        """返回测距仪样本年龄；从未收到时返回极大值."""
        if self.distance_sensor_msg is None or self.last_distance_sensor_time is None:
            return 1e9
        return (
            self.get_clock().now() - self.last_distance_sensor_time
        ).nanoseconds / 1e9

    def evaluate_distance_sensor(self) -> tuple[bool, float, float, str]:
        """
        检查测距仪当前样本是否可用于控制.

        返回：
        - valid: 是否 fresh + finite + in-range
        - distance_sensor_z: 当前高度值，缺失时为 NaN
        - age_sec: 样本年龄
        - status_text: 用于状态日志的说明
        """
        age_sec = self.get_distance_sensor_age_sec()
        if self.distance_sensor_msg is None:
            return False, float('nan'), age_sec, '距离传感器无效：从未收到数据'

        distance_sensor_z = float(self.distance_sensor_msg.range)
        min_range = float(self.distance_sensor_msg.min_range)
        max_range = float(self.distance_sensor_msg.max_range)

        if age_sec > self.distance_sensor_timeout_sec:
            return (
                False,
                distance_sensor_z,
                age_sec,
                f'距离传感器超时：age={age_sec:.3f}s',
            )

        if not math.isfinite(distance_sensor_z):
            return False, distance_sensor_z, age_sec, '距离传感器无效：Range.range 非有限数'

        lower_ok = (not math.isfinite(min_range)) or (distance_sensor_z >= min_range)
        upper_ok = (not math.isfinite(max_range)) or (distance_sensor_z <= max_range)
        if not (lower_ok and upper_ok):
            return (
                False,
                distance_sensor_z,
                age_sec,
                '距离值越界：'
                f'range={distance_sensor_z:.3f}m, '
                f'min={min_range:.3f}m, max={max_range:.3f}m',
            )

        return True, distance_sensor_z, age_sec, '距离传感器有效'

    def publish_body_velocity(
        self,
        vx_body: float,
        vy_body: float,
        vz_body_flu: float,
        yaw_rate: float,
    ):
        """
        发布 BODY_NED 速度指令.

        约定与 `pid_tuning_v3` 保持一致：
        - 内部控制按 FLU 机体系理解：x前、y左、z上
        - 发布到 `FRAME_BODY_NED` 时：`msg.velocity.y = -vy_body`
        - `msg.velocity.z = vz_body_flu`
        - yaw 不参与闭环，因此只固定写 `yaw_rate=0.0`
        """
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
        msg.velocity.z = float(vz_body_flu)
        msg.yaw_rate = float(yaw_rate)
        self.setpoint_pub.publish(msg)

    def control_loop(self):
        """主控制循环：XY 用视觉，Z 改为距离传感器，yaw_rate 固定 0."""
        if self.require_offboard and self.current_state.mode != 'OFFBOARD':
            self.reset_all_pids()
            self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
            self.latest_status = f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        if not self.is_pose_fresh():
            self.reset_all_pids()
            self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
            self.latest_status = '视觉位姿超时，输出零速并重置 PID'
            return

        # 先在 marker 平面计算误差，符号定义严格沿用 v3。
        ex_marker = self.target_x - float(self.pose.x)
        ey_marker = float(self.pose.y) - self.target_y

        # 当前 `/debug/aruco_pose.yaw` 与 BODY_NED 控制所需旋转方向相反，
        # 因此控制前先取反，再叠加固定相机安装补偿角。
        yaw_rel_raw = self.wrap_to_pi(-float(self.pose.yaw))
        yaw_rel_corrected = self.wrap_to_pi(
            yaw_rel_raw + self.camera_yaw_compensation_rad
        )
        ex_body, ey_body = self.rotate_marker_error_to_body(
            ex_marker,
            ey_marker,
            yaw_rel_corrected,
        )

        vx = self.apply_deadband(
            self.pid_x.update(ex_body),
            self.velocity_deadband,
        )
        vy = self.apply_deadband(
            self.pid_y.update(ey_body),
            self.velocity_deadband,
        )

        aruco_z = float(self.pose.z)
        ez_from_vision = self.target_z - aruco_z
        distance_valid, distance_sensor_z, distance_age_sec, distance_status = (
            self.evaluate_distance_sensor()
        )

        if self.enable_z_hold:
            if distance_valid:
                ez = self.target_z - distance_sensor_z
                vz = self.apply_deadband(
                    self.pid_z.update(ez),
                    self.velocity_deadband,
                )
                z_status = (
                    f'distance_sensor_z={distance_sensor_z:.3f}m, '
                    f'distance_age={distance_age_sec:.3f}s, ez={ez:.3f}'
                )
            else:
                # 测距仪异常时保持 XY 控制，单独冻结 Z，避免盲目上下冲。
                self.pid_z.reset()
                vz = 0.0
                z_status = (
                    f'{distance_status}，保持 XY 控制，Z 输出 0 并重置 PID_Z'
                )
        else:
            self.pid_z.reset()
            vz = 0.0
            z_status = 'enable_z_hold=false，Z 保持关闭'

        yaw_rate = 0.0
        self.publish_body_velocity(vx, vy, vz, yaw_rate)
        self.latest_status = (
            '闭环控制 | '
            f'ex_marker={ex_marker:.3f}, ey_marker={ey_marker:.3f} | '
            f'yaw_rel_raw={yaw_rel_raw:.3f} rad, '
            f'yaw_rel_corrected={yaw_rel_corrected:.3f} rad | '
            f'ex_body={ex_body:.3f}, ey_body={ey_body:.3f} | '
            f'aruco_z={aruco_z:.3f}, ez_from_vision={ez_from_vision:.3f} | '
            f'{z_status} | '
            f'vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}, yaw_rate={yaw_rate:.3f}'
        )

    def log_callback(self):
        """按 1Hz 输出状态摘要，便于在线调参."""
        self.get_logger().info(self.latest_status)


def main(args=None):
    rclpy.init(args=args)
    node = PidTuningV4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except BaseException:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except BaseException:
                pass


if __name__ == '__main__':
    main()
