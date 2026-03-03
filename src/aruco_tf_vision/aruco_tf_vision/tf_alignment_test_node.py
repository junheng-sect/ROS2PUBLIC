#!/usr/bin/env python3
"""
占位测试节点。
保留该模块用于兼容 setup.py 中既有 console_scripts 入口。
"""

import rclpy
from rclpy.node import Node


class TFAlignmentTestNode(Node):
    def __init__(self):
        super().__init__("tf_alignment_test_node")
        self.get_logger().info("tf_alignment_test_node placeholder started")


def main(args=None):
    rclpy.init(args=args)
    node = TFAlignmentTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
