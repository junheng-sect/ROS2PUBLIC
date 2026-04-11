#!/usr/bin/env python3

import math
import time

import rclpy
from debug_interface.msg import ArucoBasePose
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class PIDController:
    """简单 PID 控制器，带积分限幅和输出限幅。"""

    def __init__(self, kp=0.5, ki=0.0, kd=0.05, out_limit=0.8, i_limit=2.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_limit = abs(float(out_limit))
        self.i_limit = abs(float(i_limit))

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def reset(self):
        """在控制模式切换或数据失效时清空历史状态，避免输出突变。"""
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

        # 积分项用于消除稳态误差，同时做积分限幅避免饱和。
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


class XYTrackingNode(Node):
    """实机环境下基于 ArUco 的仅 XY 对准与固定高度保持节点。"""

    def __init__(self):
        super().__init__('xy_tracking_node')

        # ===== 输入/输出话题 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('base_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel')

        # ===== 跟踪目标 =====
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 2.5)

        # ===== PID 参数 =====
        self.declare_parameter('kp_xy', 0.6)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.08)
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.06)

        # ===== 控制频率与安全参数 =====
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('base_pose_timeout_sec', 0.5)
        self.declare_parameter('vxy_limit', 0.8)
        self.declare_parameter('vz_limit', 0.5)
        self.declare_parameter('velocity_deadband', 0.03)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.base_pose_topic = self.get_parameter('base_pose_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_z = float(self.get_parameter('target_z').value)

        kp_xy = float(self.get_parameter('kp_xy').value)
        ki_xy = float(self.get_parameter('ki_xy').value)
        kd_xy = float(self.get_parameter('kd_xy').value)
        kp_z = float(self.get_parameter('kp_z').value)
        ki_z = float(self.get_parameter('ki_z').value)
        kd_z = float(self.get_parameter('kd_z').value)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
        self.base_pose_timeout_sec = float(self.get_parameter('base_pose_timeout_sec').value)
        self.vxy_limit = float(self.get_parameter('vxy_limit').value)
        self.vz_limit = float(self.get_parameter('vz_limit').value)
        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)

        # ===== 控制器 =====
        self.pid_x = PIDController(kp_xy, ki_xy, kd_xy, out_limit=self.vxy_limit)
        self.pid_y = PIDController(kp_xy, ki_xy, kd_xy, out_limit=self.vxy_limit)
        self.pid_z = PIDController(kp_z, ki_z, kd_z, out_limit=self.vz_limit)

        # ===== 状态缓存 =====
        self.current_state = State()
        self.pose = ArucoBasePose()
        self.base_pose = PoseStamped()
        self.base_yaw = 0.0

        self.has_pose = False
        self.has_base_pose = False
        self.last_pose_time = None
        self.last_base_pose_time = None

        self.latest_status = '等待数据'

        # /mavros/local_position/pose 发布端常为 BEST_EFFORT，这里显式匹配。
        mavros_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.pose_sub = self.create_subscription(ArucoBasePose, self.pose_topic, self.pose_callback, 10)
        self.base_pose_sub = self.create_subscription(
            PoseStamped,
            self.base_pose_topic,
            self.base_pose_callback,
            mavros_pose_qos,
        )
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        self.control_timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'xy_tracking_node 已启动 | '
            f'pose={self.pose_topic} | base_pose={self.base_pose_topic} | '
            f'state={self.state_topic} | cmd={self.cmd_vel_topic}'
        )

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        """将角度归一化到 [-pi, pi]。"""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def rotate_xy(x: float, y: float, yaw_rad: float):
        """将 marker 坐标系下的二维误差旋转到 map 坐标系。"""
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)
        return c * x - s * y, s * x + c * y

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """四元数转 yaw 角。"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def reset_all_pids(self):
        """统一重置所有 PID 控制器。"""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()

    def state_callback(self, msg: State):
        self.current_state = msg

    def pose_callback(self, msg: ArucoBasePose):
        self.pose = msg
        self.has_pose = True
        self.last_pose_time = self.get_clock().now()

    def base_pose_callback(self, msg: PoseStamped):
        self.base_pose = msg
        q = msg.pose.orientation
        self.base_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.has_base_pose = True
        self.last_base_pose_time = self.get_clock().now()

    def is_pose_fresh(self) -> bool:
        """判断视觉位姿是否在可接受时限内。"""
        if not self.has_pose or self.last_pose_time is None:
            return False
        dt = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        return dt <= self.pose_timeout_sec

    def is_base_pose_fresh(self) -> bool:
        """判断 MAVROS 本地位姿是否在可接受时限内。"""
        if not self.has_base_pose or self.last_base_pose_time is None:
            return False
        dt = (self.get_clock().now() - self.last_base_pose_time).nanoseconds / 1e9
        return dt <= self.base_pose_timeout_sec

    def apply_deadband(self, value: float) -> float:
        """将小幅指令压到零，减少仿真抖动。"""
        return 0.0 if abs(value) < self.velocity_deadband else value

    def publish_cmd(self, vx: float, vy: float, vz: float):
        """发布 ENU 速度指令，并保持 yaw 不参与控制。"""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(vx)
        cmd.twist.linear.y = float(vy)
        cmd.twist.linear.z = float(vz)
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        """主控制循环：仅在 OFFBOARD 且数据有效时进行 XY/Z 闭环。"""
        if self.current_state.mode != 'OFFBOARD':
            self.reset_all_pids()
            self.publish_cmd(0.0, 0.0, 0.0)
            self.latest_status = f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        if not self.is_pose_fresh():
            self.reset_all_pids()
            self.publish_cmd(0.0, 0.0, 0.0)
            self.latest_status = '视觉位姿超时，输出零速悬停'
            return

        if not self.is_base_pose_fresh():
            self.reset_all_pids()
            self.publish_cmd(0.0, 0.0, 0.0)
            self.latest_status = '本地位姿超时，输出零速悬停'
            return

        # 先在 marker 坐标系中计算误差，再用动态 yaw 旋转到 map/ENU。
        ex_marker = self.target_x - float(self.pose.x)
        ey_marker = self.target_y - float(self.pose.y)
        yaw_marker_base = float(self.pose.yaw)
        yaw_map_marker = self.wrap_to_pi(self.base_yaw - yaw_marker_base)
        ex_map, ey_map = self.rotate_xy(ex_marker, ey_marker, yaw_map_marker)

        # 高度使用视觉测得的 aruco 相对高度闭环到 target_z。
        # 实机记录表明 /mavros/local_position/pose 的 z 方向可能与当前控制语义不一致，
        # 若直接用本地位姿做高度误差，会导致 ez 长时间为大正值并把 vz 打到上限。
        z_now = float(self.pose.z)
        ez = self.target_z - z_now

        vx = self.apply_deadband(self.pid_x.update(ex_map))
        vy = self.apply_deadband(self.pid_y.update(ey_map))
        vz = self.apply_deadband(self.pid_z.update(ez))

        self.publish_cmd(vx, vy, vz)
        self.latest_status = (
            f'OFFBOARD跟踪中 | '
            f'err_marker=({ex_marker:.3f},{ey_marker:.3f}) | '
            f'yaw_map_base={self.base_yaw:.3f}, yaw_marker_base={yaw_marker_base:.3f}, '
            f'yaw_map_marker={yaw_map_marker:.3f} | '
            f'err_map=({ex_map:.3f},{ey_map:.3f}) | '
            f'aruco_z={z_now:.3f}, target_z={self.target_z:.3f}, ez={ez:.3f} | '
            f'cmd=({vx:.3f},{vy:.3f},{vz:.3f},0.000)'
        )

    def log_callback(self):
        """按 1Hz 输出当前控制状态。"""
        self.get_logger().info(self.latest_status)


def main(args=None):
    """节点主入口。"""
    rclpy.init(args=args)
    node = XYTrackingNode()
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
