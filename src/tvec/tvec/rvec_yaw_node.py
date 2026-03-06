#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node

from debug_interface.msg import TVecRVec


class RVecYawNode(Node):
    """从 /debug/tvec 的 rvec 计算相对 yaw，并按 1Hz 输出日志。"""

    def __init__(self):
        super().__init__('rvec_yaw_node')

        # 输入话题：由 tvec_rvec_node 发布的调试位姿向量。
        self.declare_parameter('input_topic', '/debug/tvec')
        self.input_topic = self.get_parameter('input_topic').value

        self.sub_tvec = self.create_subscription(TVecRVec, self.input_topic, self.tvec_callback, 10)

        # 缓存最近一次计算结果，交由 1Hz 定时器输出日志，避免刷屏。
        self.latest_yaw_rad = None
        self.latest_yaw_deg = None
        self.latest_rvec = None
        self.has_input = False
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(f'rvec_yaw_node 已启动 | sub={self.input_topic}')

    @staticmethod
    def rodrigues_to_matrix(rvec: np.ndarray) -> np.ndarray:
        """Rodrigues 旋转向量转旋转矩阵。"""
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

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        """将角度归一化到 [-pi, pi]。"""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def tvec_callback(self, msg: TVecRVec):
        """接收 rvec 并计算 yaw。"""
        rvec = np.array(msg.rvec, dtype=np.float64).reshape(3)
        R = self.rodrigues_to_matrix(rvec)

        # 使用常见 ZYX 约定提取 yaw：yaw = atan2(R10, R00)。
        yaw_rad = self.wrap_to_pi(float(math.atan2(R[1, 0], R[0, 0])))
        yaw_deg = math.degrees(yaw_rad)

        self.latest_rvec = rvec
        self.latest_yaw_rad = yaw_rad
        self.latest_yaw_deg = yaw_deg
        self.has_input = True

    def log_callback(self):
        """1Hz 输出相对 yaw。"""
        if not self.has_input:
            # 按当前调试需求，关闭该节点的周期性 INFO 日志输出。
            # self.get_logger().info('尚未收到 /debug/tvec 数据')
            return

        r = self.latest_rvec
        # 按当前调试需求，关闭该节点的周期性 INFO 日志输出。
        # self.get_logger().info(
        #     f'rvec=({r[0]:.4f}, {r[1]:.4f}, {r[2]:.4f}) | '
        #     f'yaw={self.latest_yaw_rad:.4f} rad ({self.latest_yaw_deg:.2f} deg)'
        # )


def main(args=None):
    """节点主入口。"""
    rclpy.init(args=args)
    node = RVecYawNode()
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
