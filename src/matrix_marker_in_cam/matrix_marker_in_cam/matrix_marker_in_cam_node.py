#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from debug_interface.msg import TVecRVec, TransformMatrix4x4


class MatrixMarkerInCamNode(Node):
    """将 tvec/rvec 组装为 marker->cam 的 4x4 变换矩阵并发布。"""

    def __init__(self):
        super().__init__('matrix_marker_in_cam_node')

        # 输入话题：tvec/rvec（OpenCV 语义，marker->cam，optical 坐标）。
        self.declare_parameter('input_topic', '/debug/tvec')
        # 输出话题：4x4 变换矩阵，默认发布到 /debug/matrix/marker_in_cam。
        self.declare_parameter('output_topic', '/debug/matrix/marker_in_cam')
        # 输出 frame_id，仅用于标识矩阵语义。
        self.declare_parameter('output_frame_id', 'cam_optical')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        self.sub_tvec = self.create_subscription(TVecRVec, self.input_topic, self.tvec_callback, 10)
        self.pub_matrix = self.create_publisher(TransformMatrix4x4, self.output_topic, 10)

        # 1Hz 日志缓存。
        self.latest_matrix = None
        self.has_input = False
        self.log_timer = self.create_timer(1.0, self.log_callback)

        # 旋转修正矩阵：绕 Z 轴 +90°。
        # 这里使用右乘（R_out = R_in @ Rz90），表示在当前坐标系局部 Z 轴上追加旋转。
        self.Rz_90 = np.array(
            [
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        self.get_logger().info(
            f'matrix_marker_in_cam 节点已启动 | sub={self.input_topic} | pub={self.output_topic}'
        )

    @staticmethod
    def rodrigues_to_matrix(rvec: np.ndarray) -> np.ndarray:
        """Rodrigues 旋转向量转 3x3 旋转矩阵。"""
        theta = float(np.linalg.norm(rvec))
        if theta < 1e-12:
            return np.eye(3, dtype=np.float64)

        k = rvec / theta
        kx, ky, kz = float(k[0]), float(k[1]), float(k[2])
        K = np.array(
            [
                [0.0, -kz, ky],
                [kz, 0.0, -kx],
                [-ky, kx, 0.0],
            ],
            dtype=np.float64,
        )
        I = np.eye(3, dtype=np.float64)
        return I + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)

    def tvec_callback(self, msg: TVecRVec):
        # tvec/rvec 的语义：
        # R_cm / t_cm 表示 marker 坐标系到 camera 坐标系的变换（optical）。
        t_cm = np.array(msg.tvec, dtype=np.float64).reshape(3)
        rvec = np.array(msg.rvec, dtype=np.float64).reshape(3)
        R_cm = self.rodrigues_to_matrix(rvec)
        # 按需求在输出前增加一次“绕 Z 轴 +90°”的旋转修正。
        R_cm = R_cm @ self.Rz_90

        # 构造 4x4 齐次变换矩阵 T_cm：
        # [ R_cm(3x3)  t_cm(3x1) ]
        # [   0 0 0       1      ]
        T_cm = np.eye(4, dtype=np.float64)
        T_cm[0:3, 0:3] = R_cm
        T_cm[0:3, 3] = t_cm

        out = TransformMatrix4x4()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        # 按行优先展开，匹配 TransformMatrix4x4.msg 约定。
        out.matrix = T_cm.reshape(16).tolist()

        self.pub_matrix.publish(out)
        self.latest_matrix = T_cm
        self.has_input = True

    def log_callback(self):
        """按 1Hz 输出平移列向量，便于快速核对矩阵有效性。"""
        if not self.has_input or self.latest_matrix is None:
            self.get_logger().info('尚未收到 /debug/tvec 数据')
            return

        t = self.latest_matrix[0:3, 3]
        self.get_logger().info(
            f'T_marker_to_cam | tx={t[0]:.4f}, ty={t[1]:.4f}, tz={t[2]:.4f}'
        )


def main(args=None):
    """节点主入口。"""
    rclpy.init(args=args)
    node = MatrixMarkerInCamNode()
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
