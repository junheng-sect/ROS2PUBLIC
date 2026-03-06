#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from debug_interface.msg import TransformMatrix4x4


class MatrixBaselinkInCamNode(Node):
    """发布 baselink->cam 的固定 4x4 变换矩阵。"""

    def __init__(self):
        super().__init__('matrix_baselink_in_cam_node')

        # 输出话题：按既有命名规则，发布“baselink 在 cam 中”的矩阵。
        self.declare_parameter('output_topic', '/debug/matrix/baselink_in_cam')
        # frame_id 仅用于语义标记，不参与矩阵计算。
        self.declare_parameter('output_frame_id', 'cam_optical')
        # 发布频率，默认 10Hz，保证下游节点实时拿到固定外参。
        self.declare_parameter('publish_rate_hz', 10.0)

        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.pub_matrix = self.create_publisher(TransformMatrix4x4, self.output_topic, 10)

        # 坐标轴约束（以 cam 为参考）：
        # baselink_x -> cam_-y, baselink_y -> cam_-x, baselink_z -> cam_-z
        # 对应 p_cam = R_cb * p_baselink，列向量分别是 baselink 三轴在 cam 中的方向。
        self.T_bc = np.eye(4, dtype=np.float64)
        self.T_bc[0:3, 0:3] = np.array(
            [
                [0.0, -1.0, 0.0],
                [-1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
            ],
            dtype=np.float64,
        )
        # 平移按需求固定为 0。
        self.T_bc[0:3, 3] = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        self.timer = self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.publish_matrix)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            f'matrix_baselink_in_cam 节点已启动 | pub={self.output_topic} | rate={self.publish_rate_hz:.1f}Hz'
        )

    def publish_matrix(self):
        """周期发布固定外参矩阵。"""
        msg = TransformMatrix4x4()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame_id
        msg.matrix = self.T_bc.reshape(16).tolist()
        self.pub_matrix.publish(msg)

    def log_callback(self):
        """1Hz 输出关键矩阵元素，便于调试确认方向定义。"""
        r = self.T_bc[0:3, 0:3]
        self.get_logger().info(
            'T_baselink_to_cam | '
            f'R00={r[0,0]:.0f}, R01={r[0,1]:.0f}, R02={r[0,2]:.0f} | '
            f'R10={r[1,0]:.0f}, R11={r[1,1]:.0f}, R12={r[1,2]:.0f} | '
            f'R20={r[2,0]:.0f}, R21={r[2,1]:.0f}, R22={r[2,2]:.0f}'
        )


def main(args=None):
    """节点主入口。"""
    rclpy.init(args=args)
    node = MatrixBaselinkInCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
