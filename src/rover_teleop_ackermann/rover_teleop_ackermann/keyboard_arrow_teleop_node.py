#!/usr/bin/env python3

import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class KeyboardArrowTeleopNode(Node):
    """Keyboard teleop node that outputs Ackermann-like speed/steering commands."""

    def __init__(self) -> None:
        super().__init__('keyboard_arrow_teleop')

        self.declare_parameter('cmd_topic', '/rover/ackermann_cmd')
        self.declare_parameter('publish_rate_hz', 40.0)
        self.declare_parameter('key_timeout_sec', 0.90)
        self.declare_parameter('v_max', 0.6)
        self.declare_parameter('steer_max_deg', 25.0)

        self._cmd_topic = self.get_parameter('cmd_topic').value
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._key_timeout_sec = float(self.get_parameter('key_timeout_sec').value)
        self._v_max = float(self.get_parameter('v_max').value)
        self._steer_max_rad = float(self.get_parameter('steer_max_deg').value) * 3.141592653589793 / 180.0

        self._pub = self.create_publisher(Twist, self._cmd_topic, 10)

        self._target_v = 0.0
        self._target_delta = 0.0
        self._last_any_key_time = 0.0

        self._stdin_fd = None
        self._owns_stdin_fd = False
        self._old_termios_settings = None
        self._init_keyboard()

        timer_period = 1.0 / max(self._publish_rate_hz, 1.0)
        self._timer = self.create_timer(timer_period, self._on_timer)

        steer_max_deg = self._steer_max_rad * 180.0 / 3.141592653589793
        self.get_logger().info(
            f'WASD teleop started | Topic: {self._cmd_topic} | '
            f'v_max={self._v_max:.2f} m/s | steer_max={steer_max_deg:.1f} deg'
        )
        self.get_logger().info('Use WASD: W/S speed, A/D steering, SPACE stop, Ctrl+C exit')

    def _init_keyboard(self) -> None:
        if sys.stdin.isatty():
            self._stdin_fd = sys.stdin.fileno()
            self._owns_stdin_fd = False
        else:
            try:
                # ros2 launch usually disconnects stdin. /dev/tty still works
                # if this process has a controlling terminal.
                self._stdin_fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
                self._owns_stdin_fd = True
            except OSError:
                self.get_logger().warn('No TTY available, keyboard input disabled')
                return

        self._old_termios_settings = termios.tcgetattr(self._stdin_fd)
        # cbreak keeps Ctrl+C signal handling while still reading single chars.
        tty.setcbreak(self._stdin_fd)

    def _restore_keyboard(self) -> None:
        if self._stdin_fd is not None and self._old_termios_settings is not None:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_termios_settings)
        if self._stdin_fd is not None and self._owns_stdin_fd:
            os.close(self._stdin_fd)
        self._stdin_fd = None
        self._owns_stdin_fd = False
        self._old_termios_settings = None

    def _read_key_chars(self) -> list[str]:
        if self._stdin_fd is None:
            return []

        chars: list[str] = []
        while True:
            if not select.select([self._stdin_fd], [], [], 0.0)[0]:
                break
            try:
                chunk = os.read(self._stdin_fd, 64)
            except BlockingIOError:
                break
            if not chunk:
                break
            chars.extend(chunk.decode('utf-8', errors='ignore'))
        return chars

    def _handle_key(self, key: str, now: float) -> None:
        if key in ('w', 'W'):
            self._target_v = self._v_max
            self._last_any_key_time = now
        elif key in ('s', 'S'):
            self._target_v = -self._v_max
            self._last_any_key_time = now
        elif key in ('a', 'A'):
            self._target_delta = self._steer_max_rad
            self._last_any_key_time = now
        elif key in ('d', 'D'):
            self._target_delta = -self._steer_max_rad
            self._last_any_key_time = now
        elif key == ' ':
            self._target_v = 0.0
            self._target_delta = 0.0
            self._last_any_key_time = now

    def _publish_cmd(self) -> None:
        msg = Twist()
        msg.linear.x = float(self._target_v)
        msg.angular.z = float(self._target_delta)
        self._pub.publish(msg)

    def _on_timer(self) -> None:
        now = time.monotonic()

        for key in self._read_key_chars():
            self._handle_key(key, now)

        # Use one inactivity timer for both axes.
        # Key repeat has an initial delay on many systems; keep a larger
        # timeout to avoid intermittent zeroing while a key is still held.
        if (now - self._last_any_key_time) > self._key_timeout_sec:
            self._target_v = 0.0
            self._target_delta = 0.0

        self._publish_cmd()

    def destroy_node(self) -> bool:
        # Publish a final zero command and restore terminal settings.
        self._target_v = 0.0
        self._target_delta = 0.0
        self._publish_cmd()
        self._restore_keyboard()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardArrowTeleopNode()

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
