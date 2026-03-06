#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from debug_interface.msg import TransformMatrix4x4


class VisionPoseTfNode(Node):
    """将矩阵组合后广播为 aruco_marker->vision_pose TF。"""

    def __init__(self):
        super().__init__('vision_pose_tf_node')

        # 姿态来源：marker->baselink（保持现有朝向逻辑不变）。
        self.declare_parameter('pose_matrix_topic', '/debug/matrix/baselink_in_marker')
        # 位置来源：marker->cam（用于让 vision_pose 原点与 cam 重合）。
        self.declare_parameter('cam_matrix_topic', '/debug/matrix/marker_in_cam')
        self.declare_parameter('parent_frame', 'aruco_marker')
        self.declare_parameter('child_frame', 'vision_pose')

        self.pose_matrix_topic = self.get_parameter('pose_matrix_topic').value
        self.cam_matrix_topic = self.get_parameter('cam_matrix_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub_pose_matrix = self.create_subscription(
            TransformMatrix4x4, self.pose_matrix_topic, self.pose_matrix_callback, 10
        )
        self.sub_cam_matrix = self.create_subscription(
            TransformMatrix4x4, self.cam_matrix_topic, self.cam_matrix_callback, 10
        )

        # 缓存两路矩阵：
        # T_mb：marker->baselink（取旋转）
        # T_mc：marker->cam（取平移）
        self.T_mb = None
        self.T_mc = None

        self.latest_log = ''
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'vision_pose_tf_node 已启动 | '
            f'sub_pose={self.pose_matrix_topic} | sub_cam={self.cam_matrix_topic} | '
            f'tf={self.parent_frame}->{self.child_frame}'
        )

    @staticmethod
    def rotation_matrix_to_quaternion(R: np.ndarray):
        """3x3 旋转矩阵转四元数（x,y,z,w）。"""
        trace = float(R[0, 0] + R[1, 1] + R[2, 2])
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        return float(qx), float(qy), float(qz), float(qw)

    def pose_matrix_callback(self, msg: TransformMatrix4x4):
        """接收 marker->baselink，提供 vision_pose 姿态。"""
        self.T_mb = np.array(msg.matrix, dtype=np.float64).reshape(4, 4)
        self.try_publish_tf()

    def cam_matrix_callback(self, msg: TransformMatrix4x4):
        """接收 marker->cam，提供 vision_pose 位置。"""
        self.T_mc = np.array(msg.matrix, dtype=np.float64).reshape(4, 4)
        self.try_publish_tf()

    def try_publish_tf(self):
        """当两路矩阵都就绪后，组合发布 vision_pose TF。"""
        if self.T_mb is None or self.T_mc is None:
            return

        # 姿态保持现有逻辑：来自 marker->baselink。
        R = self.T_mb[0:3, 0:3]
        # 位置改为 cam 原点：来自 marker->cam。
        t = self.T_mc[0:3, 3]
        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(R)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.child_frame
        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)
        self.latest_log = (
            f'发布TF {self.parent_frame}->{self.child_frame} | '
            f'x={t[0]:.3f}, y={t[1]:.3f}, z={t[2]:.3f} | '
            'pos<-marker->cam, rot<-marker->baselink'
        )

    def log_callback(self):
        """1Hz 日志。"""
        if self.latest_log:
            self.get_logger().info(self.latest_log)
        else:
            self.get_logger().info('尚未收到完整矩阵数据（需要 marker->cam 与 marker->baselink）')


def main(args=None):
    rclpy.init(args=args)
    node = VisionPoseTfNode()
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
