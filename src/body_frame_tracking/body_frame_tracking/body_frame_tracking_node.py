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


class BodyFrameTrackingNode(Node):
    """机体坐标系速度闭环：直接输出 BODY_NED 速度指令。"""

    def __init__(self):
        super().__init__('body_frame_tracking_node')

        # ===== 话题参数 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('setpoint_raw_topic', '/mavros/setpoint_raw/local')

        # ===== 目标参数 =====
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)

        # ===== PID =====
        self.declare_parameter('kp_xy', 0.5)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.08)

        self.declare_parameter('kp_yaw', 1.2)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.08)

        # ===== 安全参数 =====
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('vxy_limit', 0.8)
        self.declare_parameter('vz_limit', 0.5)
        self.declare_parameter('wz_limit', 1.0)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('yaw_rate_deadband', 0.02)
        self.declare_parameter('require_offboard', True)

        # z 方向默认不控制，保持 0 速度。
        self.declare_parameter('enable_z_hold', False)
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.06)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.setpoint_raw_topic = self.get_parameter('setpoint_raw_topic').value

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
        self.require_offboard = bool(self.get_parameter('require_offboard').value)

        self.enable_z_hold = bool(self.get_parameter('enable_z_hold').value)
        kp_z = float(self.get_parameter('kp_z').value)
        ki_z = float(self.get_parameter('ki_z').value)
        kd_z = float(self.get_parameter('kd_z').value)

        # ===== 控制器 =====
        self.pid_x = PIDController(kp_xy, ki_xy, kd_xy, out_limit=self.vxy_limit)
        self.pid_y = PIDController(kp_xy, ki_xy, kd_xy, out_limit=self.vxy_limit)
        self.pid_z = PIDController(kp_z, ki_z, kd_z, out_limit=self.vz_limit)
        self.pid_yaw = PIDController(kp_yaw, ki_yaw, kd_yaw, out_limit=self.wz_limit)

        # ===== 状态 =====
        self.current_state = State()
        self.pose = ArucoBasePose()
        self.has_pose = False
        self.last_pose_time = None
        self.z_ref = None

        self.latest_status = '等待数据'

        # ===== 通信 =====
        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.pose_sub = self.create_subscription(ArucoBasePose, self.pose_topic, self.pose_callback, 10)
        self.setpoint_pub = self.create_publisher(PositionTarget, self.setpoint_raw_topic, 10)

        self.control_timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'body_frame_tracking_node 已启动 | '
            f'pose={self.pose_topic} | state={self.state_topic} | '
            f'setpoint_raw={self.setpoint_raw_topic}'
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

    def apply_deadband(self, value: float, deadband: float) -> float:
        return 0.0 if abs(value) < deadband else value

    def publish_body_velocity(self, vx_body: float, vy_body: float, vz_body_flu: float, yaw_rate: float):
        """
        发布 BODY_NED 速度指令。

        说明：
        - ArUco 链路里我们按 FLU 语义理解机体系速度：x前、y左、z上。
        - PositionTarget.FRAME_BODY_NED 需要 NED 机体系：x前、y右、z下。
        - 因此这里做符号转换：
          vy_ned = -vy_flu, vz_ned = -vz_flu。
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
        msg.velocity.z = float(-vz_body_flu)
        msg.yaw_rate = float(yaw_rate)

        self.setpoint_pub.publish(msg)

    def control_loop(self):
        if self.require_offboard and self.current_state.mode != 'OFFBOARD':
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.pid_yaw.reset()
            self.z_ref = None
            self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
            self.latest_status = f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        if not self.is_pose_fresh():
            self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
            self.latest_status = '视觉数据超时，输出零速'
            return

        # 直接在“机体/视觉当前误差坐标”下做闭环，不做动态 yaw->ENU 旋转。
        ex = self.target_x - float(self.pose.x)
        # 按实测修正：FLU 语义下 y 方向控制量需反向。
        ey = float(self.pose.y) - self.target_y
        eyaw = self.wrap_to_pi(self.target_yaw - float(self.pose.yaw))

        vx = self.apply_deadband(self.pid_x.update(ex), self.velocity_deadband)
        vy = self.apply_deadband(self.pid_y.update(ey), self.velocity_deadband)
        wz = self.apply_deadband(self.pid_yaw.update(eyaw), self.yaw_rate_deadband)

        vz = 0.0
        if self.enable_z_hold:
            z_now = float(self.pose.z)
            if self.z_ref is None:
                self.z_ref = z_now
                self.pid_z.reset()
            ez = self.z_ref - z_now
            vz = self.apply_deadband(self.pid_z.update(ez), self.velocity_deadband)

        self.publish_body_velocity(vx, vy, vz, wz)
        self.latest_status = (
            f'OFFBOARD 机体系闭环 | ex={ex:.2f}, ey={ey:.2f}, '
            f'eyaw={math.degrees(eyaw):.1f}deg | vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, wz={wz:.2f}'
        )

    def log_callback(self):
        self.get_logger().info(self.latest_status)


def main(args=None):
    rclpy.init(args=args)
    node = BodyFrameTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
