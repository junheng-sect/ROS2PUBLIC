#!/usr/bin/env python3

import math
import time

import rclpy
from debug_interface.msg import ArucoBasePose
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node


class PIDController:
    """简单 PID 控制器，带积分限幅和输出限幅。"""

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
        """在模式切换或数据超时时清空状态，避免历史积分带来突跳。"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def update(self, error: float) -> float:
        """根据当前误差计算控制输出。"""
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


class PidTuningV3Node(Node):
    """BODY_NED 控制节点：固定 yaw 补偿 + 动态相对 yaw 修正。"""

    def __init__(self):
        super().__init__('pid_tuning_v3_node')

        # ===== 输入/输出话题 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('state_topic', '/mavros/state')
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
        self.declare_parameter('vxy_limit', 0.8)
        self.declare_parameter('vx_limit', float('nan'))
        self.declare_parameter('vy_limit', float('nan'))
        self.declare_parameter('vz_limit', 0.5)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('enable_z_hold', True)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.state_topic = self.get_parameter('state_topic').value
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
        self.x_use_split = math.isfinite(kp_x_raw) or math.isfinite(ki_x_raw) or math.isfinite(kd_x_raw)
        self.y_use_split = math.isfinite(kp_y_raw) or math.isfinite(ki_y_raw) or math.isfinite(kd_y_raw)

        self.kp_x = kp_x_raw if math.isfinite(kp_x_raw) else kp_xy
        self.ki_x = ki_x_raw if math.isfinite(ki_x_raw) else ki_xy
        self.kd_x = kd_x_raw if math.isfinite(kd_x_raw) else kd_xy
        self.kp_y = kp_y_raw if math.isfinite(kp_y_raw) else kp_xy
        self.ki_y = ki_y_raw if math.isfinite(ki_y_raw) else ki_xy
        self.kd_y = kd_y_raw if math.isfinite(kd_y_raw) else kd_xy

        self.kp_z = float(self.get_parameter('kp_z').value)
        self.ki_z = float(self.get_parameter('ki_z').value)
        self.kd_z = float(self.get_parameter('kd_z').value)

        self.camera_yaw_compensation_deg = float(self.get_parameter('camera_yaw_compensation_deg').value)
        self.camera_yaw_compensation_rad = math.radians(self.camera_yaw_compensation_deg)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
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
        self.pid_x = PIDController(self.kp_x, self.ki_x, self.kd_x, out_limit=self.vx_limit)
        self.pid_y = PIDController(self.kp_y, self.ki_y, self.kd_y, out_limit=self.vy_limit)
        self.pid_z = PIDController(self.kp_z, self.ki_z, self.kd_z, out_limit=self.vz_limit)

        # ===== 状态缓存 =====
        self.current_state = State()
        self.pose = ArucoBasePose()
        self.has_pose = False
        self.last_pose_time = None
        self.latest_status = '等待视觉与飞控状态数据'

        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.pose_sub = self.create_subscription(ArucoBasePose, self.pose_topic, self.pose_callback, 10)
        self.setpoint_pub = self.create_publisher(PositionTarget, self.setpoint_raw_topic, 10)

        self.control_timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'pid_tuning_v3_node 已启动 | '
            f'pose={self.pose_topic} | state={self.state_topic} | setpoint_raw={self.setpoint_raw_topic}'
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
        self.get_logger().info('yaw 闭环已关闭，yaw_rate 固定输出 0.0')

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        """将角度归一化到 [-pi, pi]。"""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def apply_deadband(value: float, deadband: float) -> float:
        """把很小的速度指令压到零，减少悬停抖动。"""
        return 0.0 if abs(value) < deadband else value

    @staticmethod
    def rotate_marker_error_to_body(
        ex_marker: float,
        ey_marker: float,
        yaw_rel_corrected: float,
    ) -> tuple[float, float]:
        """
        将 marker 平面误差旋转到机体平面误差。

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
        """统一重置所有控制器，避免状态残留影响下一次接管。"""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()

    def state_callback(self, msg: State):
        self.current_state = msg

    def pose_callback(self, msg: ArucoBasePose):
        self.pose = msg
        self.has_pose = True
        self.last_pose_time = self.get_clock().now()

    def is_pose_fresh(self) -> bool:
        """判断视觉位姿是否在允许时限内。"""
        if not self.has_pose or self.last_pose_time is None:
            return False
        dt = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        return dt <= self.pose_timeout_sec

    def publish_body_velocity(self, vx_body: float, vy_body: float, vz_body_flu: float, yaw_rate: float):
        """
        发布 BODY_NED 速度指令。

        约定与 `pid_tuning_v2` 保持一致：
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
        """主控制循环：XY 用 BODY_NED，Z 用视觉深度保持，yaw_rate 固定 0。"""
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

        # 先在 marker 平面计算误差，符号定义严格沿用本轮需求。
        ex_marker = self.target_x - float(self.pose.x)
        ey_marker = float(self.pose.y) - self.target_y

        # 结合 2026-04-06 仿真 CSV 复盘结果，当前 `/debug/aruco_pose.yaw`
        # 与 BODY_NED 控制所需的旋转方向相反，因此控制前先取反。
        yaw_rel_raw = self.wrap_to_pi(-float(self.pose.yaw))
        yaw_rel_corrected = self.wrap_to_pi(yaw_rel_raw + self.camera_yaw_compensation_rad)
        ex_body, ey_body = self.rotate_marker_error_to_body(ex_marker, ey_marker, yaw_rel_corrected)

        # Z 方向按视觉相对高度做固定目标保持。
        ez = self.target_z - float(self.pose.z)

        vx = self.apply_deadband(self.pid_x.update(ex_body), self.velocity_deadband)
        vy = self.apply_deadband(self.pid_y.update(ey_body), self.velocity_deadband)

        if self.enable_z_hold:
            vz = self.apply_deadband(self.pid_z.update(ez), self.velocity_deadband)
        else:
            self.pid_z.reset()
            vz = 0.0

        yaw_rate = 0.0
        self.publish_body_velocity(vx, vy, vz, yaw_rate)
        self.latest_status = (
            '闭环控制 | '
            f'ex_marker={ex_marker:.3f}, ey_marker={ey_marker:.3f} | '
            f'yaw_rel_raw={yaw_rel_raw:.3f} rad, '
            f'yaw_rel_corrected={yaw_rel_corrected:.3f} rad | '
            f'ex_body={ex_body:.3f}, ey_body={ey_body:.3f}, ez={ez:.3f} | '
            f'vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}, yaw_rate={yaw_rate:.3f}'
        )

    def log_callback(self):
        """按 1Hz 输出状态摘要，便于在线调参。"""
        self.get_logger().info(self.latest_status)


def main(args=None):
    rclpy.init(args=args)
    node = PidTuningV3Node()
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
