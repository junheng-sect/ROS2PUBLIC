#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from debug_interface.msg import ArucoBasePose


class PIDController:
    """简单 PID 控制器，带积分限幅和输出限幅。"""

    def __init__(self, kp=0.5, ki=0.0, kd=0.05, out_limit=0.8, i_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_limit = abs(out_limit)
        self.i_limit = abs(i_limit)

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def update(self, error: float) -> float:
        now = time.time()
        dt = now - self.prev_time
        if dt <= 1e-6:
            dt = 1e-2

        # P 项
        p = self.kp * error

        # I 项（带积分限幅，防止积分累积过大）
        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        i = self.ki * self.integral

        # D 项
        d = self.kd * (error - self.prev_error) / dt

        out = p + i + d
        out = max(-self.out_limit, min(self.out_limit, out))

        self.prev_error = error
        self.prev_time = now
        return out


class ArucoTrackingNode(Node):
    """Offboard 模式下基于 /debug/aruco_pose 的 PID 跟踪控制节点。"""

    def __init__(self):
        super().__init__('aruco_tracking_node')

        # ========= 参数 =========
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel')

        # 控制目标：飞到标志正上方并正对（x=0, y=0, yaw=0）。
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)

        # PID 参数
        self.declare_parameter('kp_xy', 0.5)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.08)

        self.declare_parameter('kp_yaw', 1.2)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.08)

        # 频率与安全参数
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('vxy_limit', 0.8)
        self.declare_parameter('vz_limit', 0.5)
        self.declare_parameter('wz_limit', 1.0)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('yaw_rate_deadband', 0.02)
        # 坐标系补偿：将 marker 坐标系中的 XY 误差旋转到速度命令坐标系（map/ENU）。
        self.declare_parameter('rotate_error_to_map', True)
        # map->arucomarker 的偏航角（度），默认 +90°（与当前静态 TF 一致）。
        self.declare_parameter('marker_in_map_yaw_deg', 90.0)
        # 是否启用“相对高度保持”（使用 /debug/aruco_pose.z 做参考）。
        self.declare_parameter('enable_relative_z_hold', True)
        # 进入 OFFBOARD 后是否重置相对高度参考值。
        self.declare_parameter('reset_z_ref_on_offboard', True)

        # 相对高度保持的 PID 参数（作用对象：z_ref - 当前 z）。
        self.declare_parameter('kp_z_hold', 0.8)
        self.declare_parameter('ki_z_hold', 0.0)
        self.declare_parameter('kd_z_hold', 0.06)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)

        kp_xy = float(self.get_parameter('kp_xy').value)
        ki_xy = float(self.get_parameter('ki_xy').value)
        kd_xy = float(self.get_parameter('kd_xy').value)

        kp_yaw = float(self.get_parameter('kp_yaw').value)
        ki_yaw = float(self.get_parameter('ki_yaw').value)
        kd_yaw = float(self.get_parameter('kd_yaw').value)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
        self.vxy_limit = float(self.get_parameter('vxy_limit').value)
        self.vz_limit = float(self.get_parameter('vz_limit').value)
        self.wz_limit = float(self.get_parameter('wz_limit').value)
        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)
        self.yaw_rate_deadband = float(self.get_parameter('yaw_rate_deadband').value)
        self.rotate_error_to_map = bool(self.get_parameter('rotate_error_to_map').value)
        self.marker_in_map_yaw_deg = float(self.get_parameter('marker_in_map_yaw_deg').value)
        self.enable_relative_z_hold = bool(self.get_parameter('enable_relative_z_hold').value)
        self.reset_z_ref_on_offboard = bool(self.get_parameter('reset_z_ref_on_offboard').value)

        kp_z_hold = float(self.get_parameter('kp_z_hold').value)
        ki_z_hold = float(self.get_parameter('ki_z_hold').value)
        kd_z_hold = float(self.get_parameter('kd_z_hold').value)

        # ========= PID 初始化 =========
        self.pid_x = PIDController(kp_xy, ki_xy, kd_xy, out_limit=self.vxy_limit)
        self.pid_y = PIDController(kp_xy, ki_xy, kd_xy, out_limit=self.vxy_limit)
        self.pid_z_hold = PIDController(kp_z_hold, ki_z_hold, kd_z_hold, out_limit=self.vz_limit)
        self.pid_yaw = PIDController(kp_yaw, ki_yaw, kd_yaw, out_limit=self.wz_limit)

        # ========= 运行状态 =========
        self.current_state = State()
        self.pose = ArucoBasePose()
        self.has_pose = False
        self.last_pose_time = None
        self.z_ref = None
        self.prev_offboard = False

        self.latest_status = ''

        # ========= 通信 =========
        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.pose_sub = self.create_subscription(ArucoBasePose, self.pose_topic, self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        self.control_timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'aruco_tracking_node 已启动 | '
            f'pose={self.pose_topic} | state={self.state_topic} | cmd={self.cmd_vel_topic}'
        )

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        """将角度归一化到 [-pi, pi]。"""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def rotate_xy(x: float, y: float, yaw_rad: float):
        """二维向量绕 Z 轴旋转：v_out = R(yaw) * v_in。"""
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)
        return c * x - s * y, s * x + c * y

    def state_callback(self, msg: State):
        # 检测 OFFBOARD 上升沿，可选地重置相对高度参考。
        if msg.mode == 'OFFBOARD' and not self.prev_offboard and self.reset_z_ref_on_offboard:
            self.z_ref = None
            self.pid_z_hold.reset()
        self.prev_offboard = (msg.mode == 'OFFBOARD')
        self.current_state = msg

    def pose_callback(self, msg: ArucoBasePose):
        self.pose = msg
        self.has_pose = True
        self.last_pose_time = self.get_clock().now()

    def is_pose_fresh(self) -> bool:
        """判断视觉位姿是否超时。"""
        if not self.has_pose or self.last_pose_time is None:
            return False
        dt = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        return dt <= self.pose_timeout_sec

    def publish_cmd(self, vx: float, vy: float, vz: float, wz: float):
        """发布速度控制指令。"""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        cmd.twist.angular.z = wz
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        """主控制循环：仅在 OFFBOARD + 视觉有效时执行 PID。"""
        # 不在 OFFBOARD，发零速并重置 PID，避免切回 OFFBOARD 时突变。
        if self.current_state.mode != 'OFFBOARD':
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z_hold.reset()
            self.pid_yaw.reset()
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.latest_status = f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        # 视觉数据超时，悬停。
        if not self.is_pose_fresh():
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.latest_status = '视觉数据超时，输出零速悬停'
            return

        # 计算误差：x/y/yaw 对齐到目标值；z 采用“相对高度保持”（锁定 z_ref）。
        # ex_m/ey_m：marker 坐标系下误差（输入位姿自身语义）。
        ex_m = self.target_x - float(self.pose.x)
        ey_m = self.target_y - float(self.pose.y)
        z_now = float(self.pose.z)
        eyaw = self.wrap_to_pi(self.target_yaw - float(self.pose.yaw))

        # /mavros/setpoint_velocity/cmd_vel 通常在局部坐标系（map/ENU）解释。
        # 因此默认将 marker 坐标误差旋转到 map 坐标再做 PID，避免“绕圈/走圆弧”。
        if self.rotate_error_to_map:
            yaw_map_marker = math.radians(self.marker_in_map_yaw_deg)
            ex_cmd, ey_cmd = self.rotate_xy(ex_m, ey_m, yaw_map_marker)
        else:
            ex_cmd, ey_cmd = ex_m, ey_m

        vx = self.pid_x.update(ex_cmd)
        vy = self.pid_y.update(ey_cmd)
        # 相对高度保持：首次进入时锁定 z_ref，后续用 PID 把 z 拉回 z_ref。
        if self.enable_relative_z_hold:
            if self.z_ref is None:
                self.z_ref = z_now
                self.pid_z_hold.reset()
            ez = self.z_ref - z_now
            vz = self.pid_z_hold.update(ez)
        else:
            ez = 0.0
            vz = 0.0
        wz = self.pid_yaw.update(eyaw)

        # 死区处理，减少抖动。
        if abs(vx) < self.velocity_deadband:
            vx = 0.0
        if abs(vy) < self.velocity_deadband:
            vy = 0.0
        if abs(wz) < self.yaw_rate_deadband:
            wz = 0.0

        self.publish_cmd(vx, vy, vz, wz)

        self.latest_status = (
            f'OFFBOARD跟踪中 | err_marker=({ex_m:.3f},{ey_m:.3f},{eyaw:.3f}) | '
            f'err_cmd=({ex_cmd:.3f},{ey_cmd:.3f}) | '
            f'z_ref={self.z_ref if self.z_ref is not None else float("nan"):.3f}, '
            f'z_now={z_now:.3f}, ez={ez:.3f} | '
            f'cmd=({vx:.3f},{vy:.3f},{vz:.3f},{wz:.3f})'
        )

    def log_callback(self):
        """1Hz 状态日志。"""
        self.get_logger().info(self.latest_status if self.latest_status else '等待状态数据...')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
