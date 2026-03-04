#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class RoverAutoLoopNode(Node):
    """Looping mission: straight 1m, then right turn 90deg with fixed radius."""

    def __init__(self) -> None:
        super().__init__('rover_auto_loop')

        self.declare_parameter('cmd_topic', '/rover/ackermann_cmd')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('linear_speed', 0.30)
        self.declare_parameter('straight_distance_m', 1.0)
        self.declare_parameter('turn_radius_m', 1.0)
        self.declare_parameter('turn_angle_deg', 90.0)
        self.declare_parameter('wheelbase_m', 0.45)

        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._linear_speed = max(0.01, abs(float(self.get_parameter('linear_speed').value)))
        self._straight_distance = max(0.0, float(self.get_parameter('straight_distance_m').value))
        self._turn_radius = max(0.01, float(self.get_parameter('turn_radius_m').value))
        self._turn_angle_deg = abs(float(self.get_parameter('turn_angle_deg').value))
        self._wheelbase = max(0.01, float(self.get_parameter('wheelbase_m').value))

        self._straight_duration = self._straight_distance / self._linear_speed
        arc_len = self._turn_radius * math.radians(self._turn_angle_deg)
        self._turn_duration = arc_len / self._linear_speed

        # Ackermann steering angle for a right turn with target radius.
        self._right_delta = -math.atan(self._wheelbase / self._turn_radius)

        self._pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._stage = 'straight'
        self._stage_start = time.monotonic()

        period = 1.0 / max(self._publish_rate_hz, 1.0)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'Auto loop started | v={self._linear_speed:.2f} m/s | '
            f'straight={self._straight_distance:.2f} m | '
            f'turn={self._turn_angle_deg:.1f} deg @ R={self._turn_radius:.2f} m'
        )

    def _publish_cmd(self, v: float, delta: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(delta)
        self._pub.publish(msg)

    def _switch_stage(self, stage: str) -> None:
        self._stage = stage
        self._stage_start = time.monotonic()
        self.get_logger().info(f'Stage -> {stage}')

    def _on_timer(self) -> None:
        elapsed = time.monotonic() - self._stage_start

        if self._stage == 'straight':
            self._publish_cmd(self._linear_speed, 0.0)
            if elapsed >= self._straight_duration:
                self._switch_stage('turn_right')
            return

        self._publish_cmd(self._linear_speed, self._right_delta)
        if elapsed >= self._turn_duration:
            self._switch_stage('straight')

    def destroy_node(self) -> bool:
        self._publish_cmd(0.0, 0.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoverAutoLoopNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
