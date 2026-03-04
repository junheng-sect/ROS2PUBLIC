#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""OFFBOARD 基础任务节点。

任务流程：
1. 预发送 setpoint
2. 切换 OFFBOARD
3. 解锁
4. 起飞到目标高度
5. 悬停指定时间
6. 切换 AUTO.LAND
7. 上锁
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class MissionState(Enum):
    WAIT_CONNECTION = 0
    PRESTREAM_SETPOINT = 1
    REQUEST_OFFBOARD = 2
    REQUEST_ARM = 3
    TAKEOFF = 4
    HOVER = 5
    REQUEST_LAND = 6
    REQUEST_DISARM = 7
    DONE = 8


class OffboardMissionNode(Node):
    def __init__(self):
        super().__init__('offboard_mission_node')

        self.declare_parameter('takeoff_height', 2.5)
        self.declare_parameter('hover_time_sec', 5.0)
        self.declare_parameter('prestream_time_sec', 1.5)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('height_tolerance', 0.15)
        self.declare_parameter('landed_height_threshold', 0.15)
        self.declare_parameter('target_yaw_deg', 90.0)

        self.takeoff_height = float(self.get_parameter('takeoff_height').value)
        self.hover_time_sec = float(self.get_parameter('hover_time_sec').value)
        self.prestream_time_sec = float(self.get_parameter('prestream_time_sec').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.height_tolerance = float(self.get_parameter('height_tolerance').value)
        self.landed_height_threshold = float(self.get_parameter('landed_height_threshold').value)
        self.target_yaw_deg = float(self.get_parameter('target_yaw_deg').value)
        target_yaw_rad = math.radians(self.target_yaw_deg)
        self.target_qz = math.sin(target_yaw_rad * 0.5)
        self.target_qw = math.cos(target_yaw_rad * 0.5)

        self.drone_state = State()
        self.local_pose = None

        self.state = MissionState.WAIT_CONNECTION
        self.state_enter_time = self.get_clock().now()
        self.last_request_time = self.get_clock().now()

        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, qos_best_effort)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_best_effort,
        )

        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.pending_future = None

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.timer_callback)

        self.get_logger().info(
            f'offboard_control ready: arm -> takeoff 2.5m -> hover 5s -> land -> disarm, '
            f'target_yaw={self.target_yaw_deg:.1f}deg'
        )

    def state_callback(self, msg: State):
        self.drone_state = msg

    def pose_callback(self, msg: PoseStamped):
        self.local_pose = msg

    def _set_state(self, new_state: MissionState, reason: str):
        self.state = new_state
        self.state_enter_time = self.get_clock().now()
        self.get_logger().info(f'STATE -> {new_state.name}: {reason}')

    def _elapsed(self):
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    def _request_interval_ok(self, sec: float = 1.0):
        elapsed = (self.get_clock().now() - self.last_request_time).nanoseconds / 1e9
        return elapsed >= sec

    def _publish_setpoint(self, x: float, y: float, z: float):
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        self.target_pose.pose.orientation.w = self.target_qw
        self.target_pose.pose.orientation.x = 0.0
        self.target_pose.pose.orientation.y = 0.0
        self.target_pose.pose.orientation.z = self.target_qz
        self.setpoint_pub.publish(self.target_pose)

    def _send_mode(self, mode: str):
        if not self.mode_client.wait_for_service(timeout_sec=0.0):
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        self.pending_future = self.mode_client.call_async(req)
        self.last_request_time = self.get_clock().now()
        self.get_logger().info(f'Request mode: {mode}')
        return True

    def _send_arm(self, arm: bool):
        if not self.arm_client.wait_for_service(timeout_sec=0.0):
            return False
        req = CommandBool.Request()
        req.value = arm
        self.pending_future = self.arm_client.call_async(req)
        self.last_request_time = self.get_clock().now()
        self.get_logger().info(f'Request arm={arm}')
        return True

    def timer_callback(self):
        if self.local_pose is None:
            return

        current_x = self.local_pose.pose.position.x
        current_y = self.local_pose.pose.position.y
        current_z = self.local_pose.pose.position.z

        # 在 OFFBOARD 阶段持续发布位置 setpoint
        if self.state in (
            MissionState.PRESTREAM_SETPOINT,
            MissionState.REQUEST_OFFBOARD,
            MissionState.REQUEST_ARM,
            MissionState.TAKEOFF,
            MissionState.HOVER,
        ):
            target_z = self.takeoff_height if self.state in (MissionState.TAKEOFF, MissionState.HOVER) else current_z
            self._publish_setpoint(current_x, current_y, target_z)

        if self.state == MissionState.WAIT_CONNECTION:
            if self.drone_state.connected and self.local_pose is not None:
                self._set_state(MissionState.PRESTREAM_SETPOINT, 'FCU connected')
            return

        if self.state == MissionState.PRESTREAM_SETPOINT:
            if self._elapsed() >= self.prestream_time_sec:
                self._set_state(MissionState.REQUEST_OFFBOARD, 'setpoint prestream done')
            return

        if self.state == MissionState.REQUEST_OFFBOARD:
            if self.drone_state.mode == 'OFFBOARD':
                self._set_state(MissionState.REQUEST_ARM, 'OFFBOARD enabled')
                return
            if self.pending_future is None and self._request_interval_ok(1.0):
                self._send_mode('OFFBOARD')
            if self.pending_future is not None and self.pending_future.done():
                self.pending_future = None
            return

        if self.state == MissionState.REQUEST_ARM:
            if self.drone_state.armed:
                self._set_state(MissionState.TAKEOFF, 'armed')
                return
            if self.pending_future is None and self._request_interval_ok(1.0):
                self._send_arm(True)
            if self.pending_future is not None and self.pending_future.done():
                self.pending_future = None
            return

        if self.state == MissionState.TAKEOFF:
            if math.fabs(current_z - self.takeoff_height) <= self.height_tolerance:
                self._set_state(MissionState.HOVER, 'takeoff reached')
            return

        if self.state == MissionState.HOVER:
            if self._elapsed() >= self.hover_time_sec:
                self._set_state(MissionState.REQUEST_LAND, 'hover finished')
            return

        if self.state == MissionState.REQUEST_LAND:
            if self.drone_state.mode == 'AUTO.LAND':
                self._set_state(MissionState.REQUEST_DISARM, 'AUTO.LAND enabled')
                return
            if self.pending_future is None and self._request_interval_ok(1.0):
                self._send_mode('AUTO.LAND')
            if self.pending_future is not None and self.pending_future.done():
                self.pending_future = None
            return

        if self.state == MissionState.REQUEST_DISARM:
            if (not self.drone_state.armed) or (current_z <= self.landed_height_threshold):
                if self.drone_state.armed:
                    if self.pending_future is None and self._request_interval_ok(1.0):
                        self._send_arm(False)
                    if self.pending_future is not None and self.pending_future.done():
                        self.pending_future = None
                else:
                    self._set_state(MissionState.DONE, 'disarmed')
            return



def main(args=None):
    rclpy.init(args=args)
    node = OffboardMissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
