#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node

from debug_interface.msg import ArucoBasePose, TVecRVec


class MarkerInCamNode(Node):
    """将 marker 在 camera optical 坐标系下的位姿发布为 ArucoBasePose。"""

    def __init__(self):
        super().__init__('marker_in_cam_node')

        # 输入：tvec/rvec 调试话题；输出：marker 在 cam(optical) 下的 x/y/z/yaw。
        # camera optical 坐标定义：x向右、y向下、z向前。
        self.declare_parameter('input_topic', '/debug/tvec')
        self.declare_parameter('output_topic', '/debug/marker_in_cam')
        self.declare_parameter('output_frame_id', 'cam_optical')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        self.sub_tvec = self.create_subscription(TVecRVec, self.input_topic, self.tvec_callback, 10)
        self.pub_pose = self.create_publisher(ArucoBasePose, self.output_topic, 10)

        # 1Hz 日志缓存。
        self.latest_pose_msg = None
        self.has_input = False
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            f'marker_in_cam 节点已启动 | sub={self.input_topic} | pub={self.output_topic}'
        )

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
        return I + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)

    @staticmethod
    def normalize_angle(angle_rad: float) -> float:
        """归一化到 [-pi, pi]。"""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def tvec_callback(self, msg: TVecRVec):
        # tvec/rvec 语义：marker -> camera（optical 坐标表达）
        t_cm_opt = np.array(msg.tvec, dtype=np.float64).reshape(3)  # 直接就是 optical 坐标下的位置
        rvec = np.array(msg.rvec, dtype=np.float64).reshape(3)
        R_cm_opt = self.rodrigues_to_matrix(rvec)

        # 不再做 FLU 转换，直接使用 optical 坐标输出 xyz。
        # yaw 取自 optical 坐标下旋转矩阵的平面角（绕 z_opt）。
        yaw = math.atan2(R_cm_opt[1, 0], R_cm_opt[0, 0])
        yaw = self.normalize_angle(yaw)

        out = ArucoBasePose()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        out.x = float(t_cm_opt[0])
        out.y = float(t_cm_opt[1])
        out.z = float(t_cm_opt[2])
        out.yaw = float(yaw)

        self.pub_pose.publish(out)
        self.latest_pose_msg = out
        self.has_input = True

    def log_callback(self):
        """按 1Hz 输出 marker 在 cam(optical) 下的位姿。"""
        if not self.has_input or self.latest_pose_msg is None:
            self.get_logger().info('尚未收到 /debug/tvec 数据')
            return

        p = self.latest_pose_msg
        self.get_logger().info(
            f'marker_in_cam (optical) | '
            f'x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}, '
            f'yaw={p.yaw:.4f} rad ({math.degrees(p.yaw):.2f} deg)'
        )


def main(args=None):
    """节点主入口。"""
    rclpy.init(args=args)
    node = MarkerInCamNode()
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
