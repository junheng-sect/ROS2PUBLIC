#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from debug_interface.msg import TransformMatrix4x4


class MatrixBaselinkInMarkerNode(Node):
    """通过矩阵乘法计算 marker->baselink 并发布。"""

    def __init__(self):
        super().__init__('matrix_baselink_in_marker_node')

        # 输入 1：marker->cam 矩阵（来自 matrix_marker_in_cam）。
        self.declare_parameter('marker_in_cam_topic', '/debug/matrix/marker_in_cam')
        # 输入 2：baselink->cam 固定外参矩阵（来自 matrix_baselink_in_cam）。
        self.declare_parameter('baselink_in_cam_topic', '/debug/matrix/baselink_in_cam')

        # 输出：marker->baselink 矩阵。
        self.declare_parameter('output_topic', '/debug/matrix/baselink_in_marker')
        self.declare_parameter('output_frame_id', 'marker_optical')

        self.marker_in_cam_topic = self.get_parameter('marker_in_cam_topic').value
        self.baselink_in_cam_topic = self.get_parameter('baselink_in_cam_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        self.sub_marker_in_cam = self.create_subscription(
            TransformMatrix4x4, self.marker_in_cam_topic, self.marker_in_cam_callback, 10
        )
        self.sub_baselink_in_cam = self.create_subscription(
            TransformMatrix4x4, self.baselink_in_cam_topic, self.baselink_in_cam_callback, 10
        )
        self.pub_matrix = self.create_publisher(TransformMatrix4x4, self.output_topic, 10)

        # 缓存最近一次输入矩阵。
        self.T_mc = None  # marker->cam
        self.T_bc = None  # baselink->cam
        self.has_marker_in_cam = False
        self.has_baselink_in_cam = False

        # 1Hz 日志输出。
        self.latest_matrix = None
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'matrix_baselink_in_marker 节点已启动 | '
            f'sub1={self.marker_in_cam_topic} | sub2={self.baselink_in_cam_topic} | pub={self.output_topic}'
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

    def marker_in_cam_callback(self, msg: TransformMatrix4x4):
        """接收 marker->cam，刷新缓存并尝试计算输出。"""
        self.T_mc = np.array(msg.matrix, dtype=np.float64).reshape(4, 4)
        self.has_marker_in_cam = True
        self.try_publish(msg)

    def baselink_in_cam_callback(self, msg: TransformMatrix4x4):
        """接收 baselink->cam，刷新缓存并尝试计算输出。"""
        self.T_bc = np.array(msg.matrix, dtype=np.float64).reshape(4, 4)
        self.has_baselink_in_cam = True
        self.try_publish(msg)

    def try_publish(self, msg: TransformMatrix4x4):
        """当两路矩阵都就绪时执行矩阵链乘。"""
        if not (self.has_marker_in_cam and self.has_baselink_in_cam):
            return

        # 已知：
        # T_mc: marker->cam
        # T_bc: baselink->cam
        # 目标：T_mb: marker->baselink
        # 关系：T_mb = T_cb * T_mc，其中 T_cb = inv(T_bc)
        T_cb = self.inverse_rigid_transform(self.T_bc)
        T_mb = T_cb @ self.T_mc

        out = TransformMatrix4x4()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.output_frame_id
        out.matrix = T_mb.reshape(16).tolist()
        self.pub_matrix.publish(out)

        self.latest_matrix = T_mb

    def log_callback(self):
        """按 1Hz 输出状态与平移量，方便联调。"""
        if self.latest_matrix is None:
            self.get_logger().info(
                '等待输入矩阵 | '
                f'marker_in_cam={self.has_marker_in_cam} | baselink_in_cam={self.has_baselink_in_cam}'
            )
            return

        t = self.latest_matrix[0:3, 3]
        self.get_logger().info(
            f'T_marker_to_baselink | tx={t[0]:.4f}, ty={t[1]:.4f}, tz={t[2]:.4f}'
        )


def main(args=None):
    """节点主入口。"""
    rclpy.init(args=args)
    node = MatrixBaselinkInMarkerNode()
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
