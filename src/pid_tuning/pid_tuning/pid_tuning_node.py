#!/usr/bin/env python3

import math
import time

import rclpy
from debug_interface.msg import ArucoBasePose
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node


class PIDController:
    """简单 PID 控制器（带输出限幅和积分限幅）。"""

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
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def update(self, error: float) -> float:
        now = time.time()
        dt = now - self.prev_time
        if dt <= 1e-6:
            dt = 1e-2

        # P 项：当前误差线性输出。
        p_term = self.kp * error

        # I 项：积分并限幅。
        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        i_term = self.ki * self.integral

        # D 项：误差变化率抑制振荡。
        d_term = self.kd * (error - self.prev_error) / dt

        out = p_term + i_term + d_term
        out = max(-self.out_limit, min(self.out_limit, out))

        self.prev_error = error
        self.prev_time = now
        return out


class PidTuningNode(Node):
    """PID 调参专用控制节点：XY/Yaw + 固定高度 Z 闭环。"""

    def __init__(self):
        super().__init__('pid_tuning_node')

        # ===== 话题参数 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('setpoint_raw_topic', '/mavros/setpoint_raw/local')

        # ===== 目标参数 =====
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('target_z', 2.5)

        # ===== 兼容参数：粗调入口 =====
        self.declare_parameter('kp_xy', 0.5)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.08)

        # ===== 分轴参数：精调入口（NaN 表示未显式配置） =====
        self.declare_parameter('kp_x', float('nan'))
        self.declare_parameter('ki_x', float('nan'))
        self.declare_parameter('kd_x', float('nan'))
        self.declare_parameter('kp_y', float('nan'))
        self.declare_parameter('ki_y', float('nan'))
        self.declare_parameter('kd_y', float('nan'))

        # ===== Yaw 与 Z PID =====
        self.declare_parameter('kp_yaw', 1.2)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.08)

        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.06)

        # ===== 安全参数 =====
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('vxy_limit', 0.8)
        self.declare_parameter('vx_limit', float('nan'))
        self.declare_parameter('vy_limit', float('nan'))
        self.declare_parameter('vz_limit', 0.5)
        self.declare_parameter('wz_limit', 1.0)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('yaw_rate_deadband', 0.02)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('enable_z_hold', True)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.setpoint_raw_topic = self.get_parameter('setpoint_raw_topic').value

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)
        self.target_z = float(self.get_parameter('target_z').value)

        # 读取粗调参数
        kp_xy = float(self.get_parameter('kp_xy').value)
        ki_xy = float(self.get_parameter('ki_xy').value)
        kd_xy = float(self.get_parameter('kd_xy').value)

        # 读取分轴参数（NaN 表示“未设置”）
        kp_x_raw = float(self.get_parameter('kp_x').value)
        ki_x_raw = float(self.get_parameter('ki_x').value)
        kd_x_raw = float(self.get_parameter('kd_x').value)
        kp_y_raw = float(self.get_parameter('kp_y').value)
        ki_y_raw = float(self.get_parameter('ki_y').value)
        kd_y_raw = float(self.get_parameter('kd_y').value)

        # 参数优先级：分轴 > 粗调。
        self.x_use_split = math.isfinite(kp_x_raw) or math.isfinite(ki_x_raw) or math.isfinite(kd_x_raw)
        self.y_use_split = math.isfinite(kp_y_raw) or math.isfinite(ki_y_raw) or math.isfinite(kd_y_raw)

        self.kp_x = kp_x_raw if math.isfinite(kp_x_raw) else kp_xy
        self.ki_x = ki_x_raw if math.isfinite(ki_x_raw) else ki_xy
        self.kd_x = kd_x_raw if math.isfinite(kd_x_raw) else kd_xy
        self.kp_y = kp_y_raw if math.isfinite(kp_y_raw) else kp_xy
        self.ki_y = ki_y_raw if math.isfinite(ki_y_raw) else ki_xy
        self.kd_y = kd_y_raw if math.isfinite(kd_y_raw) else kd_xy

        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.ki_yaw = float(self.get_parameter('ki_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)

        self.kp_z = float(self.get_parameter('kp_z').value)
        self.ki_z = float(self.get_parameter('ki_z').value)
        self.kd_z = float(self.get_parameter('kd_z').value)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
        self.vxy_limit = float(self.get_parameter('vxy_limit').value)
        vx_limit_raw = float(self.get_parameter('vx_limit').value)
        vy_limit_raw = float(self.get_parameter('vy_limit').value)
        # 兼容策略：
        # - 未显式设置 vx/vy 时，回退到统一 vxy_limit。
        # - 显式设置后，分别限制 X/Y 两个速度通道。
        self.vx_limit = vx_limit_raw if math.isfinite(vx_limit_raw) else self.vxy_limit
        self.vy_limit = vy_limit_raw if math.isfinite(vy_limit_raw) else self.vxy_limit
        self.vz_limit = float(self.get_parameter('vz_limit').value)
        self.wz_limit = float(self.get_parameter('wz_limit').value)
        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)
        self.yaw_rate_deadband = float(self.get_parameter('yaw_rate_deadband').value)
        self.require_offboard = bool(self.get_parameter('require_offboard').value)
        self.enable_z_hold = bool(self.get_parameter('enable_z_hold').value)

        # ===== 控制器 =====
        self.pid_x = PIDController(self.kp_x, self.ki_x, self.kd_x, out_limit=self.vx_limit)
        self.pid_y = PIDController(self.kp_y, self.ki_y, self.kd_y, out_limit=self.vy_limit)
        self.pid_z = PIDController(self.kp_z, self.ki_z, self.kd_z, out_limit=self.vz_limit)
        self.pid_yaw = PIDController(self.kp_yaw, self.ki_yaw, self.kd_yaw, out_limit=self.wz_limit)

        # ===== 状态 =====
        self.current_state = State()
        self.pose = ArucoBasePose()
        self.has_pose = False
        self.last_pose_time = None

        self.latest_status = '等待数据'

        # ===== 通信 =====
        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.pose_sub = self.create_subscription(ArucoBasePose, self.pose_topic, self.pose_callback, 10)
        self.setpoint_pub = self.create_publisher(PositionTarget, self.setpoint_raw_topic, 10)

        self.control_timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'pid_tuning_node 已启动 | '
            f'pose={self.pose_topic} | state={self.state_topic} | setpoint_raw={self.setpoint_raw_topic}'
        )
        self.get_logger().info(
            'PID参数 | '
            f'X=({self.kp_x:.3f},{self.ki_x:.3f},{self.kd_x:.3f}) '
            f'Y=({self.kp_y:.3f},{self.ki_y:.3f},{self.kd_y:.3f}) '
            f'Yaw=({self.kp_yaw:.3f},{self.ki_yaw:.3f},{self.kd_yaw:.3f}) '
            f'Z=({self.kp_z:.3f},{self.ki_z:.3f},{self.kd_z:.3f})'
        )
        self.get_logger().info(
            f'速度限幅 | vxy_limit={self.vxy_limit:.3f} | '
            f'vx_limit={self.vx_limit:.3f} | vy_limit={self.vy_limit:.3f} | '
            f'vz_limit={self.vz_limit:.3f} | wz_limit={self.wz_limit:.3f}'
        )
        self.get_logger().info(
            '参数来源 | '
            f'X轴={"split(kp_x/ki_x/kd_x)" if self.x_use_split else "legacy(kp_xy/ki_xy/kd_xy)"} | '
            f'Y轴={"split(kp_y/ki_y/kd_y)" if self.y_use_split else "legacy(kp_xy/ki_xy/kd_xy)"}'
        )

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def state_callback(self, msg: State):
        self.current_state = msg

    def pose_callback(self, msg: ArucoBasePose):
        self.pose = msg
        self.has_pose = True
        self.last_pose_time = self.get_clock().now()

    def is_pose_fresh(self) -> bool:
        if not self.has_pose or self.last_pose_time is None:
            return False
        dt = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        return dt <= self.pose_timeout_sec

    @staticmethod
    def apply_deadband(value: float, deadband: float) -> float:
        return 0.0 if abs(value) < deadband else value

    def publish_body_velocity(self, vx_body: float, vy_body: float, vz_body_flu: float, yaw_rate: float):
        """
        发布 BODY_NED 速度指令。

        说明：
        - 我们在控制内部按 FLU 机体系理解速度：x前、y左、z上。
        - PositionTarget.FRAME_BODY_NED 下，实机测试确认：
          y 需要取反，z 与内部控制输出同号更符合预期高度响应。
        - 故这里使用：vy_ned=-vy_flu, vz_ned=vz_flu。
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
        if self.require_offboard and self.current_state.mode != 'OFFBOARD':
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.pid_yaw.reset()
            self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
            self.latest_status = f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        if not self.is_pose_fresh():
            self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
            self.latest_status = '视觉数据超时，输出零速'
            return

        # X/Y/Yaw 误差
        ex = self.target_x - float(self.pose.x)
        ey = float(self.pose.y) - self.target_y
        eyaw = self.wrap_to_pi(self.target_yaw - float(self.pose.yaw))

        # Z 误差：按要求使用 aruco_pose.z 闭环到固定高度 target_z。
        ez = self.target_z - float(self.pose.z)

        vx = self.apply_deadband(self.pid_x.update(ex), self.velocity_deadband)
        vy = self.apply_deadband(self.pid_y.update(ey), self.velocity_deadband)
        wz = self.apply_deadband(self.pid_yaw.update(eyaw), self.yaw_rate_deadband)
        vz = 0.0
        if self.enable_z_hold:
            vz = self.apply_deadband(self.pid_z.update(ez), self.velocity_deadband)

        self.publish_body_velocity(vx, vy, vz, wz)
        self.latest_status = (
            f'OFFBOARD闭环 | ex={ex:.2f}, ey={ey:.2f}, ez={ez:.2f}, '
            f'eyaw={math.degrees(eyaw):.1f}deg | '
            f'vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, wz={wz:.2f}'
        )

    def log_callback(self):
        self.get_logger().info(self.latest_status)


def main(args=None):
    rclpy.init(args=args)
    node = PidTuningNode()
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
