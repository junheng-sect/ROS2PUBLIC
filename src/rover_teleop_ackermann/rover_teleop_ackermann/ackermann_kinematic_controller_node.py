#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class AckermannKinematicControllerNode(Node):
    """Kinematic bicycle-model controller that drives Gazebo model pose."""

    def __init__(self) -> None:
        super().__init__('ackermann_kinematic_controller')

        self.declare_parameter('cmd_topic', '/rover/ackermann_cmd')
        self.declare_parameter('world_name', 'rover')
        self.declare_parameter('model_name', 'rover')
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('wheelbase', 0.45)
        self.declare_parameter('v_max', 0.6)
        self.declare_parameter('steer_max_deg', 25.0)
        self.declare_parameter('cmd_timeout_sec', 0.3)

        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw_deg', 0.0)
        self.declare_parameter('z_fixed', 0.15)

        self._cmd_topic = self.get_parameter('cmd_topic').value
        self._world_name = self.get_parameter('world_name').value
        self._model_name = self.get_parameter('model_name').value
        self._control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self._wheelbase = float(self.get_parameter('wheelbase').value)
        self._v_max = float(self.get_parameter('v_max').value)
        self._steer_max_rad = float(self.get_parameter('steer_max_deg').value) * math.pi / 180.0
        self._cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)

        self._x = float(self.get_parameter('init_x').value)
        self._y = float(self.get_parameter('init_y').value)
        self._yaw = float(self.get_parameter('init_yaw_deg').value) * math.pi / 180.0
        self._z_fixed = float(self.get_parameter('z_fixed').value)

        self._v_cmd = 0.0
        self._delta_cmd = 0.0
        self._last_cmd_time = time.monotonic()
        self._last_tick_time = time.monotonic()
        self._pending_future = None
        self._last_log_time = {}

        self.create_subscription(Twist, self._cmd_topic, self._cmd_callback, 10)

        service_name = f'/world/{self._world_name}/set_pose'
        self._set_pose_client = self.create_client(SetEntityPose, service_name)

        timer_period = 1.0 / max(self._control_rate_hz, 1.0)
        self._timer = self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            f'Controller started | Model={self._model_name} | '
            f'Service={service_name} | wheelbase={self._wheelbase:.2f} m'
        )

    def _throttled_log(self, key: str, msg: str, level: str = 'warn', period_sec: float = 2.0) -> None:
        now = time.monotonic()
        last = self._last_log_time.get(key, 0.0)
        if (now - last) < period_sec:
            return
        self._last_log_time[key] = now

        if level == 'error':
            self.get_logger().error(msg)
        else:
            self.get_logger().warn(msg)

    def _cmd_callback(self, msg: Twist) -> None:
        self._v_cmd = max(min(float(msg.linear.x), self._v_max), -self._v_max)
        self._delta_cmd = max(min(float(msg.angular.z), self._steer_max_rad), -self._steer_max_rad)
        self._last_cmd_time = time.monotonic()

    def _yaw_to_quaternion(self, yaw: float) -> tuple[float, float, float, float]:
        half = yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def _request_set_pose(self) -> None:
        if not self._set_pose_client.service_is_ready():
            self._throttled_log('service_unready', 'SetEntityPose service is not ready: waiting...')
            return

        if self._pending_future is not None and not self._pending_future.done():
            return

        request = SetEntityPose.Request()
        request.entity.name = self._model_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(self._x)
        request.pose.position.y = float(self._y)
        request.pose.position.z = float(self._z_fixed)

        qx, qy, qz, qw = self._yaw_to_quaternion(self._yaw)
        request.pose.orientation.x = qx
        request.pose.orientation.y = qy
        request.pose.orientation.z = qz
        request.pose.orientation.w = qw

        future = self._set_pose_client.call_async(request)
        future.add_done_callback(self._on_set_pose_done)
        self._pending_future = future

    def _on_set_pose_done(self, future) -> None:
        try:
            response = future.result()
            if not response.success:
                self._throttled_log('set_pose_failed', 'SetEntityPose returned success=false')
        except Exception as exc:  # noqa: BLE001
            self._throttled_log('set_pose_exception', f'SetEntityPose call failed: {exc}', level='error')
        finally:
            if self._pending_future is future:
                self._pending_future = None

    def _on_timer(self) -> None:
        now = time.monotonic()
        dt = max(0.0, now - self._last_tick_time)
        self._last_tick_time = now

        if dt <= 0.0:
            return

        if (now - self._last_cmd_time) > self._cmd_timeout_sec:
            v = 0.0
            delta = 0.0
        else:
            v = self._v_cmd
            delta = self._delta_cmd

        # Bicycle model integration.
        yaw_rate = 0.0
        if abs(self._wheelbase) > 1e-6:
            yaw_rate = (v / self._wheelbase) * math.tan(delta)

        yaw_mid = self._yaw + 0.5 * yaw_rate * dt
        self._x += v * math.cos(yaw_mid) * dt
        self._y += v * math.sin(yaw_mid) * dt
        self._yaw += yaw_rate * dt
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        self._request_set_pose()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AckermannKinematicControllerNode()

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
