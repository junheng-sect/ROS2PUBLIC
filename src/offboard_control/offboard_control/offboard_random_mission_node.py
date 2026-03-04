#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""OFFBOARD 随机点任务节点。

流程：
1. 预发送 setpoint
2. 切换 OFFBOARD
3. 解锁
4. 起飞到 2.5m
5. 飞到以 (0,0) 为圆心、半径 1 的圆周随机点，且 yaw 随机
6. 悬停 5s
7. 回到原点上空
8. AUTO.LAND 降落
9. 上锁
"""

import math
import random
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
    MOVE_TO_RANDOM = 5
    HOVER_RANDOM = 6
    MOVE_TO_ORIGIN = 7
    REQUEST_LAND = 8
    REQUEST_DISARM = 9
    DONE = 10


class OffboardRandomMissionNode(Node):
    def __init__(self):
        super().__init__('offboard_random_mission_node')

        self.declare_parameter('takeoff_height', 2.5)
        self.declare_parameter('hover_time_sec', 5.0)
        self.declare_parameter('prestream_time_sec', 1.5)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('pos_tolerance', 0.15)
        self.declare_parameter('landed_height_threshold', 0.15)
        self.declare_parameter('radius', 1.0)

        self.takeoff_height = float(self.get_parameter('takeoff_height').value)
        self.hover_time_sec = float(self.get_parameter('hover_time_sec').value)
        self.prestream_time_sec = float(self.get_parameter('prestream_time_sec').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.pos_tolerance = float(self.get_parameter('pos_tolerance').value)
        self.landed_height_threshold = float(self.get_parameter('landed_height_threshold').value)
        self.radius = float(self.get_parameter('radius').value)

        self.drone_state = State()
        self.local_pose = None

        self.state = MissionState.WAIT_CONNECTION
        self.state_enter_time = self.get_clock().now()
        self.last_request_time = self.get_clock().now()

        self.pending_future = None

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw_deg = 0.0

        self.random_x = 0.0
        self.random_y = 0.0
        self.random_yaw_deg = 0.0

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

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.timer_callback)

        self.get_logger().info(
            'offboard_random_mission ready: arm -> takeoff -> random(x,y,yaw) -> hover -> origin -> land -> disarm'
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

    def _publish_setpoint(self):
        target_yaw_rad = math.radians(self.target_yaw_deg)
        qz = math.sin(target_yaw_rad * 0.5)
        qw = math.cos(target_yaw_rad * 0.5)

        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.pose.position.x = self.target_x
        self.target_pose.pose.position.y = self.target_y
        self.target_pose.pose.position.z = self.target_z
        self.target_pose.pose.orientation.x = 0.0
        self.target_pose.pose.orientation.y = 0.0
        self.target_pose.pose.orientation.z = qz
        self.target_pose.pose.orientation.w = qw
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

    def _distance_to_target(self, x: float, y: float, z: float):
        dx = x - self.target_x
        dy = y - self.target_y
        dz = z - self.target_z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _pick_random_goal(self):
        angle = random.uniform(0.0, 2.0 * math.pi)
        self.random_x = self.radius * math.cos(angle)
        self.random_y = self.radius * math.sin(angle)
        self.random_yaw_deg = random.uniform(-180.0, 180.0)

        self.get_logger().info(
            f'Random goal: x={self.random_x:.3f}, y={self.random_y:.3f}, yaw={self.random_yaw_deg:.1f}deg'
        )

    def timer_callback(self):
        if self.local_pose is None:
            return

        current_x = self.local_pose.pose.position.x
        current_y = self.local_pose.pose.position.y
        current_z = self.local_pose.pose.position.z

        if self.state == MissionState.WAIT_CONNECTION:
            if self.drone_state.connected:
                self.target_x = current_x
                self.target_y = current_y
                self.target_z = current_z
                self.target_yaw_deg = 0.0
                self._set_state(MissionState.PRESTREAM_SETPOINT, 'FCU connected')
            return

        # 非终止状态持续发布 setpoint
        if self.state != MissionState.DONE:
            self._publish_setpoint()

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
                self.target_z = self.takeoff_height
                self._set_state(MissionState.TAKEOFF, 'armed')
                return
            if self.pending_future is None and self._request_interval_ok(1.0):
                self._send_arm(True)
            if self.pending_future is not None and self.pending_future.done():
                self.pending_future = None
            return

        if self.state == MissionState.TAKEOFF:
            if abs(current_z - self.takeoff_height) <= self.pos_tolerance:
                self._pick_random_goal()
                self.target_x = self.random_x
                self.target_y = self.random_y
                self.target_z = self.takeoff_height
                self.target_yaw_deg = self.random_yaw_deg
                self._set_state(MissionState.MOVE_TO_RANDOM, 'takeoff reached')
            return

        if self.state == MissionState.MOVE_TO_RANDOM:
            if self._distance_to_target(current_x, current_y, current_z) <= self.pos_tolerance:
                self._set_state(MissionState.HOVER_RANDOM, 'random point reached')
            return

        if self.state == MissionState.HOVER_RANDOM:
            if self._elapsed() >= self.hover_time_sec:
                self.target_x = 0.0
                self.target_y = 0.0
                self.target_z = self.takeoff_height
                self.target_yaw_deg = 90.0
                self._set_state(MissionState.MOVE_TO_ORIGIN, 'hover finished, go origin')
            return

        if self.state == MissionState.MOVE_TO_ORIGIN:
            if self._distance_to_target(current_x, current_y, current_z) <= self.pos_tolerance:
                self._set_state(MissionState.REQUEST_LAND, 'origin reached')
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
    node = OffboardRandomMissionNode()
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
