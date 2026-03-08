#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node


class OffboardTestNode(Node):
    """OFFBOARD 测试节点：向南飞 10 秒后悬停。"""

    def __init__(self):
        super().__init__('offboard_test_node')

        # ===== 话题参数 =====
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel')

        # ===== 行为参数 =====
        self.declare_parameter('south_speed_mps', 0.5)
        self.declare_parameter('fly_duration_sec', 10.0)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('start_on_offboard_entry', True)

        self.state_topic = self.get_parameter('state_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.south_speed_mps = abs(float(self.get_parameter('south_speed_mps').value))
        self.fly_duration_sec = max(0.0, float(self.get_parameter('fly_duration_sec').value))
        self.control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.require_offboard = bool(self.get_parameter('require_offboard').value)
        self.start_on_offboard_entry = bool(self.get_parameter('start_on_offboard_entry').value)

        # ===== 运行状态 =====
        self.current_state = State()
        self.prev_offboard = False
        self.mission_started = (not self.start_on_offboard_entry)
        self.start_time_sec = None
        self.mission_finished = False
        self.latest_status = '等待进入 OFFBOARD'

        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        self.control_timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'offboard_test_node 已启动 | '
            f'state={self.state_topic} | cmd={self.cmd_vel_topic} | '
            f'south_speed={self.south_speed_mps:.2f}m/s | duration={self.fly_duration_sec:.1f}s'
        )

    def state_callback(self, msg: State):
        self.current_state = msg

        is_offboard = (msg.mode == 'OFFBOARD')
        offboard_rising = (is_offboard and not self.prev_offboard)
        self.prev_offboard = is_offboard

        # OFFBOARD 上升沿触发一次任务：开始计时并清除完成标志。
        if self.start_on_offboard_entry and offboard_rising:
            self.mission_started = True
            self.mission_finished = False
            self.start_time_sec = self.get_clock().now().nanoseconds / 1e9
            self.latest_status = '检测到 OFFBOARD 上升沿，开始向南飞行'
            self.get_logger().info('检测到 OFFBOARD 上升沿，执行 south-fly 10s 测试')

    def publish_cmd(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """发布速度控制指令。cmd_vel 使用 ENU：x=East, y=North, z=Up。"""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(vx)
        cmd.twist.linear.y = float(vy)
        cmd.twist.linear.z = float(vz)
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = float(yaw_rate)
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        # 可选安全门控：非 OFFBOARD 时只发零速，避免误控制。
        if self.require_offboard and self.current_state.mode != 'OFFBOARD':
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.latest_status = f'当前模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        # 若要求 OFFBOARD 上升沿启动，则启动前保持零速等待。
        if not self.mission_started:
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.latest_status = '已进入 OFFBOARD，等待任务启动'
            return

        if self.start_time_sec is None:
            self.start_time_sec = self.get_clock().now().nanoseconds / 1e9

        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_sec - self.start_time_sec

        if (not self.mission_finished) and (elapsed < self.fly_duration_sec):
            # /mavros/setpoint_velocity/cmd_vel 采用 ENU：
            # y>0 表示向北，因此向南飞行应设置 y<0。
            self.publish_cmd(0.0, -self.south_speed_mps, 0.0, 0.0)
            self.latest_status = (
                f'执行中：向南飞行 | vy={-self.south_speed_mps:.2f}m/s | '
                f't={elapsed:.2f}/{self.fly_duration_sec:.2f}s'
            )
            return

        # 计时结束后持续悬停（零速）。
        self.mission_finished = True
        self.publish_cmd(0.0, 0.0, 0.0, 0.0)
        self.latest_status = f'任务完成：已向南飞行 {self.fly_duration_sec:.1f}s，当前悬停'

    def log_callback(self):
        # 每秒打印一次状态，便于确认任务阶段与输出。
        mode_text = self.current_state.mode if self.current_state.mode else 'UNKNOWN'
        self.get_logger().info(f'[offboard_test] mode={mode_text} | {self.latest_status}')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

