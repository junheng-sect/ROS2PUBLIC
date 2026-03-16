#!/usr/bin/env python3

import math
import time
from enum import Enum

import rclpy
from debug_interface.msg import ArucoBasePose
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node


class PIDController:
    """简单 PID 控制器（带积分限幅与输出限幅）。"""

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
        """重置积分项与微分历史，避免阶段切换时出现控制突变。"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def update(self, error: float) -> float:
        now = time.time()
        dt = now - self.prev_time
        if dt <= 1e-6:
            dt = 1e-2

        p_term = self.kp * error

        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        i_term = self.ki * self.integral

        d_term = self.kd * (error - self.prev_error) / dt

        out = p_term + i_term + d_term
        out = max(-self.out_limit, min(self.out_limit, out))

        self.prev_error = error
        self.prev_time = now
        return out


class TrackingPhase(Enum):
    """控制阶段：先对正偏航，再推进 XY 对齐。"""

    YAW_ALIGN = 1
    XY_ALIGN = 2


class YawThenXYTrackingNode(Node):
    """先偏航对齐，再 XYZ 对齐（且全程 yaw 闭环）的机体系控制节点。"""

    def __init__(self):
        super().__init__('yaw_then_xy_tracking_node')

        # ===== 话题参数 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('setpoint_raw_topic', '/mavros/setpoint_raw/local')

        # ===== 目标参数 =====
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('target_z', 2.5)

        # ===== PID 参数（保持与 pid_tuning 一致） =====
        self.declare_parameter('kp_x', 0.6)
        self.declare_parameter('ki_x', 0.0)
        self.declare_parameter('kd_x', 0.02)
        self.declare_parameter('kp_y', 0.6)
        self.declare_parameter('ki_y', 0.0)
        self.declare_parameter('kd_y', 0.02)
        self.declare_parameter('kp_yaw', 0.60)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.02)
        self.declare_parameter('kp_z', 0.60)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.06)

        # ===== 安全与限幅 =====
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('vx_limit', 1.0)
        self.declare_parameter('vy_limit', 1.0)
        self.declare_parameter('vz_limit', 0.5)
        self.declare_parameter('wz_limit', 1.0)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('yaw_rate_deadband', 0.03)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('enable_z_hold', True)

        # ===== 阶段切换判据 =====
        self.declare_parameter('yaw_align_threshold_deg', 5.0)
        self.declare_parameter('yaw_align_hold_sec', 0.5)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.setpoint_raw_topic = self.get_parameter('setpoint_raw_topic').value

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)
        self.target_z = float(self.get_parameter('target_z').value)

        self.kp_x = float(self.get_parameter('kp_x').value)
        self.ki_x = float(self.get_parameter('ki_x').value)
        self.kd_x = float(self.get_parameter('kd_x').value)
        self.kp_y = float(self.get_parameter('kp_y').value)
        self.ki_y = float(self.get_parameter('ki_y').value)
        self.kd_y = float(self.get_parameter('kd_y').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.ki_yaw = float(self.get_parameter('ki_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.ki_z = float(self.get_parameter('ki_z').value)
        self.kd_z = float(self.get_parameter('kd_z').value)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
        self.vx_limit = float(self.get_parameter('vx_limit').value)
        self.vy_limit = float(self.get_parameter('vy_limit').value)
        self.vz_limit = float(self.get_parameter('vz_limit').value)
        self.wz_limit = float(self.get_parameter('wz_limit').value)
        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)
        self.yaw_rate_deadband = float(self.get_parameter('yaw_rate_deadband').value)
        self.require_offboard = bool(self.get_parameter('require_offboard').value)
        self.enable_z_hold = bool(self.get_parameter('enable_z_hold').value)

        self.yaw_align_threshold_rad = math.radians(
            float(self.get_parameter('yaw_align_threshold_deg').value)
        )
        self.yaw_align_hold_sec = float(self.get_parameter('yaw_align_hold_sec').value)

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
        self.phase = TrackingPhase.YAW_ALIGN
        self.yaw_ok_start_time = None
        self.latest_status = '等待数据'

        # ===== 通信 =====
        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.pose_sub = self.create_subscription(ArucoBasePose, self.pose_topic, self.pose_callback, 10)
        self.setpoint_pub = self.create_publisher(PositionTarget, self.setpoint_raw_topic, 10)

        self.control_timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'yaw_then_xy_tracking_node 已启动 | '
            f'pose={self.pose_topic} | state={self.state_topic} | setpoint_raw={self.setpoint_raw_topic}'
        )
        self.get_logger().info(
            f'阶段切换判据 | yaw阈值={math.degrees(self.yaw_align_threshold_rad):.1f}deg | '
            f'持续时间={self.yaw_align_hold_sec:.2f}s | 流程=先Yaw,后XYZ(保持Yaw)'
        )

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        """将角度归一化到 [-pi, pi]。"""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def apply_deadband(value: float, deadband: float) -> float:
        """将小幅抖动压到 0，减轻执行器抖振。"""
        return 0.0 if abs(value) < deadband else value

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

    def reset_all_pids(self):
        """在模式切换/数据失效时统一清空 PID 内部状态。"""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()

    def reset_phase(self):
        """回到第一阶段：先偏航对齐。"""
        self.phase = TrackingPhase.YAW_ALIGN
        self.yaw_ok_start_time = None

    def publish_body_velocity(self, vx_body: float, vy_body: float, vz_body_flu: float, yaw_rate: float):
        """
        发布 BODY_NED 速度指令。

        约定：
        - 内部计算使用 FLU：x前、y左、z上。
        - 发布到 FRAME_BODY_NED 时需转换：vy_ned=-vy_flu，vz_ned=vz_flu。
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
            self.reset_all_pids()
            self.reset_phase()
            self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
            self.latest_status = f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        if not self.is_pose_fresh():
            self.reset_all_pids()
            self.reset_phase()
            self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
            self.latest_status = '视觉数据超时，输出零速并重置阶段'
            return

        ex = self.target_x - float(self.pose.x)
        ey = float(self.pose.y) - self.target_y
        eyaw = self.wrap_to_pi(self.target_yaw - float(self.pose.yaw))
        ez = self.target_z - float(self.pose.z)

        wz = self.apply_deadband(self.pid_yaw.update(eyaw), self.yaw_rate_deadband)
        if self.phase == TrackingPhase.YAW_ALIGN:
            # 第一阶段：仅允许 yaw 动作，XYZ 全锁死。
            vx = 0.0
            vy = 0.0
            vz = 0.0
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()

            if abs(eyaw) <= self.yaw_align_threshold_rad:
                if self.yaw_ok_start_time is None:
                    self.yaw_ok_start_time = self.get_clock().now()
                hold_sec = (self.get_clock().now() - self.yaw_ok_start_time).nanoseconds / 1e9
                if hold_sec >= self.yaw_align_hold_sec:
                    self.phase = TrackingPhase.XY_ALIGN
                    self.pid_x.reset()
                    self.pid_y.reset()
            else:
                self.yaw_ok_start_time = None

            self.publish_body_velocity(vx, vy, vz, wz)
            self.latest_status = (
                f'阶段=YAW_ALIGN(仅Yaw) | eyaw={math.degrees(eyaw):.1f}deg | '
                f'vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, wz={wz:.2f}'
            )
            return

        # 第二阶段：在保持 yaw 闭环的同时做 XYZ 闭环到目标。
        vx = self.apply_deadband(self.pid_x.update(ex), self.velocity_deadband)
        vy = self.apply_deadband(self.pid_y.update(ey), self.velocity_deadband)
        vz = 0.0
        if self.enable_z_hold:
            vz = self.apply_deadband(self.pid_z.update(ez), self.velocity_deadband)
        self.publish_body_velocity(vx, vy, vz, wz)
        self.latest_status = (
            f'阶段=XY_ALIGN(XYZ+Yaw) | ex={ex:.2f}, ey={ey:.2f}, ez={ez:.2f}, eyaw={math.degrees(eyaw):.1f}deg | '
            f'vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, wz={wz:.2f}'
        )

    def log_callback(self):
        self.get_logger().info(self.latest_status)


def main(args=None):
    rclpy.init(args=args)
    node = YawThenXYTrackingNode()
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
