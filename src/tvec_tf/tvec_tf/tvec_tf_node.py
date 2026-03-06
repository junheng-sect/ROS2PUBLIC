#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from debug_interface.msg import ArucoBasePose, TVecRVec


class TVecTfNode(Node):
    """将 /debug/tvec 中的 tvec 映射为 aruco marker 到 vision_pose 的 TF。"""

    def __init__(self):
        super().__init__('tvec_tf_node')

        # 输入为 tvec 功能包发布的调试向量。
        self.declare_parameter('input_topic', '/debug/tvec')
        # 按需求：父坐标系固定为 arucomarker。
        self.declare_parameter('parent_frame', 'arucomarker')
        # 按需求：子坐标系固定为 vision_pose。
        self.declare_parameter('child_frame', 'vision_pose')

        self.input_topic = self.get_parameter('input_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub_tvec = self.create_subscription(TVecRVec, self.input_topic, self.tvec_callback, 10)
        # 发布计算后的 xyz+yaw，便于通过 rostopic 直接查看。
        self.pose_pub = self.create_publisher(ArucoBasePose, '/debug/aruco_pose', 10)

        self.latest_log = ''
        self.latest_pose = None
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            f'tvec_tf_node 已启动 | sub={self.input_topic} | tf={self.parent_frame}->{self.child_frame}'
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
        return I + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        """将角度归一化到 [-pi, pi]。"""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def yaw_to_quaternion(yaw: float):
        """仅绕 Z 轴旋转时的四元数计算（roll=pitch=0）。"""
        half = 0.5 * yaw
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def tvec_callback(self, msg: TVecRVec):
        """按用户定义映射关系发布动态 TF。"""
        # 由 rvec 计算相对 yaw（ZYX 约定：yaw=atan2(R10,R00)）。
        rvec = np.array(msg.rvec, dtype=np.float64).reshape(3)
        R = self.rodrigues_to_matrix(rvec)
        yaw = self.wrap_to_pi(float(math.atan2(R[1, 0], R[0, 0])))
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)

        # 按你指定公式使用 yaw 对平移进行组合变换。
        tx = float(msg.tvec[1] * math.cos(yaw) - msg.tvec[0] * math.sin(yaw))
        ty = float(msg.tvec[1] * math.sin(yaw) + msg.tvec[0] * math.cos(yaw))
        tz = float(msg.tvec[2])

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.child_frame

        tf_msg.transform.translation.x = tx
        tf_msg.transform.translation.y = ty
        tf_msg.transform.translation.z = tz

        # 将由 rvec 提取到的 yaw 写入 TF 旋转（仅绕 Z 轴）。
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)

        # 同步发布 ArucoBasePose 调试话题。
        pose_msg = ArucoBasePose()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = self.parent_frame
        pose_msg.x = tx
        pose_msg.y = ty
        pose_msg.z = tz
        pose_msg.yaw = yaw
        self.pose_pub.publish(pose_msg)
        self.latest_pose = pose_msg

        self.latest_log = (
            f'aruco_pose | x={tx:.3f}, y={ty:.3f}, z={tz:.3f}, '
            f'yaw={yaw:.3f} rad ({math.degrees(yaw):.2f} deg)'
        )

    def log_callback(self):
        """1Hz 日志输出。"""
        if self.latest_log:
            self.get_logger().info(self.latest_log)
        else:
            self.get_logger().info('尚未收到 /debug/tvec 数据')


def main(args=None):
    """节点主入口。"""
    rclpy.init(args=args)
    node = TVecTfNode()
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
