#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node

from debug_interface.msg import ArucoBasePose, TVecRVec


class RelativePoseNode(Node):
    """将 tvec/rvec 转换为 ArUco(FLU) 下机体(FLU)位姿的节点。"""

    def __init__(self):
        super().__init__('relative_pose_node')

        # 输入话题：来自 tvec 功能包的调试向量话题。
        self.declare_parameter('input_topic', '/debug/tvec')
        # 输出话题：发布 ArUco 坐标系下的机体位姿。
        self.declare_parameter('output_topic', '/debug/relative_pose')
        # 输出 frame_id，用于标识“这是在 aruco_flu 参考系下的位姿”。
        self.declare_parameter('output_frame_id', 'aruco_flu')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        self.sub_tvec = self.create_subscription(
            TVecRVec,
            self.input_topic,
            self.tvec_callback,
            10,
        )
        self.pub_pose = self.create_publisher(ArucoBasePose, self.output_topic, 10)

        # 用于 1Hz 日志输出的最新状态缓存。
        self.latest_pose_msg = None
        self.has_input = False
        self.log_timer = self.create_timer(1.0, self.log_callback)

        # 光学坐标系(optical) -> FLU 坐标系的基变换矩阵：
        # optical 约定: x右, y下, z前
        # FLU 约定:     x前, y左, z上
        # 因此：x_flu=z_opt, y_flu=-x_opt, z_flu=-y_opt
        self.R_flu_opt = np.array(
            [
                [0.0, 0.0, 1.0],
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
            ],
            dtype=np.float64,
        )
        self.R_opt_flu = self.R_flu_opt.T

        self.get_logger().info(
            f'relative_pose 节点已启动 | sub={self.input_topic} | pub={self.output_topic}'
        )

    @staticmethod
    def rodrigues_to_matrix(rvec: np.ndarray) -> np.ndarray:
        """将 Rodrigues 旋转向量转换为 3x3 旋转矩阵。"""
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
        """将角度归一化到 [-pi, pi]。"""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def tvec_callback(self, msg: TVecRVec):
        # 输入语义（来自 estimatePoseSingleMarkers）：
        # R_cm, t_cm：marker -> camera（两者均在 optical 坐标定义下）。
        t_cm_opt = np.array(msg.tvec, dtype=np.float64).reshape(3)
        rvec = np.array(msg.rvec, dtype=np.float64).reshape(3)
        R_cm_opt = self.rodrigues_to_matrix(rvec)

        # 将“marker->camera”从 optical 坐标表达转换为 FLU 坐标表达。
        # 这里对旋转做双边基变换，对平移做单边基变换。
        R_cm_flu = self.R_flu_opt @ R_cm_opt @ self.R_opt_flu
        t_cm_flu = self.R_flu_opt @ t_cm_opt

        # 需求目标是“aruco 坐标系下机体坐标系的位置”：
        # 先对 marker->camera 取逆，得到 camera->marker 或 marker系下camera位姿。
        # 若近似机体坐标系与相机坐标系重合，则可作为机体位姿近似。
        R_mc_flu = R_cm_flu.T
        p_m_c_flu = -R_mc_flu @ t_cm_flu

        # 从旋转矩阵提取偏航角（绕 FLU 的 z 轴）。
        yaw = math.atan2(R_mc_flu[1, 0], R_mc_flu[0, 0])
        yaw = self.normalize_angle(yaw)

        out = ArucoBasePose()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        out.x = float(p_m_c_flu[0])
        out.y = float(p_m_c_flu[1])
        out.z = float(p_m_c_flu[2])
        out.yaw = float(yaw)

        self.pub_pose.publish(out)
        self.latest_pose_msg = out
        self.has_input = True

    def log_callback(self):
        """1Hz 输出当前估计结果。"""
        if not self.has_input or self.latest_pose_msg is None:
            self.get_logger().info('尚未收到 /debug/tvec 数据')
            return

        p = self.latest_pose_msg
        self.get_logger().info(
            f'aruco->base (FLU) | '
            f'x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}, '
            f'yaw={p.yaw:.4f} rad ({math.degrees(p.yaw):.2f} deg)'
        )


def main(args=None):
    """节点主入口。"""
    rclpy.init(args=args)
    node = RelativePoseNode()
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
