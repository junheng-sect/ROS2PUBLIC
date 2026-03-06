#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from debug_interface.msg import TransformMatrix4x4


class MatrixCamInMarkerNode(Node):
    """将 marker->cam 矩阵反变换为 cam->marker 矩阵并发布。"""

    def __init__(self):
        super().__init__('matrix_cam_in_marker_node')

        # 输入为 matrix_marker_in_cam 发布的 marker->cam 变换矩阵。
        self.declare_parameter('input_topic', '/debug/matrix/marker_in_cam')
        # 输出为其逆变换 cam->marker。
        self.declare_parameter('output_topic', '/debug/matrix/cam_in_marker')
        # 输出 frame_id，仅用于语义标识。
        self.declare_parameter('output_frame_id', 'marker_optical')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        self.sub_matrix = self.create_subscription(
            TransformMatrix4x4, self.input_topic, self.matrix_callback, 10
        )
        self.pub_matrix = self.create_publisher(TransformMatrix4x4, self.output_topic, 10)

        # 1Hz 日志缓存，便于快速检查输出平移量。
        self.latest_matrix = None
        self.has_input = False
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            f'matrix_cam_in_marker 节点已启动 | sub={self.input_topic} | pub={self.output_topic}'
        )

    @staticmethod
    def inverse_rigid_transform(T_in: np.ndarray) -> np.ndarray:
        """对刚体变换矩阵求逆：T=[R t;0 1] -> T^-1=[R^T -R^T t;0 1]。"""
        R = T_in[0:3, 0:3]
        t = T_in[0:3, 3]

        R_inv = R.T
        t_inv = -R_inv @ t

        T_out = np.eye(4, dtype=np.float64)
        T_out[0:3, 0:3] = R_inv
        T_out[0:3, 3] = t_inv
        return T_out

    def matrix_callback(self, msg: TransformMatrix4x4):
        # 输入消息按行优先展开为 16 个浮点数，这里还原成 4x4 矩阵。
        T_marker_cam = np.array(msg.matrix, dtype=np.float64).reshape(4, 4)

        # 对 marker->cam 做逆变换，得到 cam->marker。
        T_cam_marker = self.inverse_rigid_transform(T_marker_cam)

        out = TransformMatrix4x4()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        # 重新按行优先展开后发布。
        out.matrix = T_cam_marker.reshape(16).tolist()

        self.pub_matrix.publish(out)
        self.latest_matrix = T_cam_marker
        self.has_input = True

    def log_callback(self):
        """按 1Hz 输出 cam->marker 矩阵中的平移项。"""
        if not self.has_input or self.latest_matrix is None:
            self.get_logger().info('尚未收到 /debug/matrix/marker_in_cam 数据')
            return

        t = self.latest_matrix[0:3, 3]
        self.get_logger().info(
            f'T_cam_to_marker | tx={t[0]:.4f}, ty={t[1]:.4f}, tz={t[2]:.4f}'
        )


def main(args=None):
    """节点主入口。"""
    rclpy.init(args=args)
    node = MatrixCamInMarkerNode()
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
