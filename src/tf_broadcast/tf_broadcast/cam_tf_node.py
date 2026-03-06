#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from debug_interface.msg import TransformMatrix4x4


class CamTfNode(Node):
    """将 /debug/matrix/cam_in_marker 转换并广播为 TF。"""

    def __init__(self):
        super().__init__('cam_tf_node')

        self.declare_parameter('input_topic', '/debug/matrix/cam_in_marker')
        self.declare_parameter('parent_frame', 'aruco_marker')
        self.declare_parameter('child_frame', 'cam')

        self.input_topic = self.get_parameter('input_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub_matrix = self.create_subscription(
            TransformMatrix4x4, self.input_topic, self.matrix_callback, 10
        )

        self.latest_log = ''
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            f'cam_tf_node 已启动 | sub={self.input_topic} | tf={self.parent_frame}->{self.child_frame}'
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

    def matrix_callback(self, msg: TransformMatrix4x4):
        T = np.array(msg.matrix, dtype=np.float64).reshape(4, 4)
        R = T[0:3, 0:3]
        t = T[0:3, 3]
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
            f'x={t[0]:.3f}, y={t[1]:.3f}, z={t[2]:.3f}'
        )

    def log_callback(self):
        """1Hz 日志。"""
        if self.latest_log:
            self.get_logger().info(self.latest_log)
        else:
            self.get_logger().info('尚未收到 cam_in_marker 矩阵数据')


def main(args=None):
    rclpy.init(args=args)
    node = CamTfNode()
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
