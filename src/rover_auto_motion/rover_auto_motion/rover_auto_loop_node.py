#!/usr/bin/env python3

import math
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class RoverAutoLoopNode(Node):
    """循环任务：前进 1m，再右转 90deg，然后继续循环。"""

    def __init__(self) -> None:
        super().__init__('rover_auto_loop')

        self.declare_parameter('entity_name', 'rover')
        self.declare_parameter('set_pose_service', '/world/rover/set_pose')  # Gazebo Transport 服务名
        self.declare_parameter('z_height_m', 0.0)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('linear_speed', 0.30)
        self.declare_parameter('straight_distance_m', 1.0)
        self.declare_parameter('turn_radius_m', 1.0)
        self.declare_parameter('turn_angle_deg', 90.0)
        self.declare_parameter('cmd_smoothing_tau_sec', 0.18)

        self._entity_name = str(self.get_parameter('entity_name').value)
        self._set_pose_service = str(self.get_parameter('set_pose_service').value)
        self._z_height = float(self.get_parameter('z_height_m').value)
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._linear_speed = max(0.01, abs(float(self.get_parameter('linear_speed').value)))
        self._straight_distance = max(0.0, float(self.get_parameter('straight_distance_m').value))
        self._turn_radius = max(0.01, float(self.get_parameter('turn_radius_m').value))
        self._turn_angle_deg = abs(float(self.get_parameter('turn_angle_deg').value))
        self._cmd_smoothing_tau = max(0.0, float(self.get_parameter('cmd_smoothing_tau_sec').value))

        self._straight_duration = self._straight_distance / self._linear_speed
        arc_len = self._turn_radius * math.radians(self._turn_angle_deg)
        self._turn_duration = arc_len / self._linear_speed

        # 固定转弯半径下，右转角速度为负号（右手系下绕 z 轴顺时针）。
        self._right_yaw_rate = -self._linear_speed / self._turn_radius

        self._stage = 'straight'
        self._stage_start = time.monotonic()
        self._last_tick = self._stage_start

        # rover 在世界坐标系下的当前状态（数值积分）。
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._v_cmd = 0.0
        self._yaw_rate_cmd = 0.0
        self._last_state_log = self._stage_start

        period = 1.0 / max(self._publish_rate_hz, 1.0)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'Auto loop started | v={self._linear_speed:.2f} m/s | '
            f'straight={self._straight_distance:.2f} m | '
            f'turn={self._turn_angle_deg:.1f} deg @ R={self._turn_radius:.2f} m | '
            f'smooth_tau={self._cmd_smoothing_tau:.2f}s'
        )

    def _yaw_to_quaternion(self, yaw: float):
        half = 0.5 * yaw
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def _request_set_pose(self) -> None:
        # 直接调用 Gazebo CLI 服务，避免 ros_gz_bridge 在不同环境下的服务桥接差异。
        qx, qy, qz, qw = self._yaw_to_quaternion(self._yaw)
        req = (
            f'name:"{self._entity_name}" '
            f'position {{x: {self._x:.6f} y: {self._y:.6f} z: {self._z_height:.6f}}} '
            f'orientation {{x:{qx:.6f} y:{qy:.6f} z:{qz:.6f} w:{qw:.6f}}}'
        )
        cmd = [
            'gz', 'service',
            '-s', self._set_pose_service,
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '500',
            '--req', req,
        ]
        result = subprocess.run(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True
        )
        if result.returncode != 0:
            self.get_logger().warn(
                f'set_pose 调用失败: {result.stderr.strip()}',
                throttle_duration_sec=2.0
            )

    def _switch_stage(self, stage: str) -> None:
        self._stage = stage
        self._stage_start = time.monotonic()
        self.get_logger().info(f'Stage -> {stage}')

    def _on_timer(self) -> None:
        now = time.monotonic()
        dt = max(0.0, now - self._last_tick)
        self._last_tick = now

        elapsed = now - self._stage_start

        if self._stage == 'straight':
            v_target = self._linear_speed
            yaw_rate_target = 0.0
            if elapsed >= self._straight_duration:
                self._switch_stage('turn_right')
        else:
            v_target = self._linear_speed
            yaw_rate_target = self._right_yaw_rate
            if elapsed >= self._turn_duration:
                self._switch_stage('straight')

        # 一阶平滑，避免“直行/转弯”切换瞬间出现速度突变导致视觉卡顿。
        if self._cmd_smoothing_tau > 1e-6:
            alpha = dt / (self._cmd_smoothing_tau + dt)
        else:
            alpha = 1.0
        self._v_cmd += alpha * (v_target - self._v_cmd)
        self._yaw_rate_cmd += alpha * (yaw_rate_target - self._yaw_rate_cmd)

        # 简单运动学积分：先更新偏航，再按当前偏航推进平面位置。
        self._yaw += self._yaw_rate_cmd * dt
        self._x += self._v_cmd * math.cos(self._yaw) * dt
        self._y += self._v_cmd * math.sin(self._yaw) * dt
        self._request_set_pose()

        # 1Hz 输出当前位置，便于确认 rover 位姿指令确实在变化。
        if (now - self._last_state_log) >= 1.0:
            self._last_state_log = now
            self.get_logger().info(
                f'pose_cmd: x={self._x:.3f}, y={self._y:.3f}, yaw_deg={math.degrees(self._yaw):.1f}'
            )

    def destroy_node(self) -> bool:
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
