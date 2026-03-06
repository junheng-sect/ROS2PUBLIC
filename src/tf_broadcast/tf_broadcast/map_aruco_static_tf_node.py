#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class MapArucoStaticTfNode(Node):
    """发布静态坐标变换：map(ENU) -> aruco_marker(NWU)。"""

    def __init__(self):
        super().__init__('map_aruco_static_tf_node')

        # 保持参数可配置，默认值按当前需求设置。
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'aruco_marker')
        self.declare_parameter('tx', 0.0)
        self.declare_parameter('ty', 0.0)
        self.declare_parameter('tz', 0.3)

        parent_frame = self.get_parameter('parent_frame').value
        child_frame = self.get_parameter('child_frame').value
        tx = float(self.get_parameter('tx').value)
        ty = float(self.get_parameter('ty').value)
        tz = float(self.get_parameter('tz').value)

        # ENU -> NWU 的旋转关系：
        # x_NWU = y_ENU, y_NWU = -x_ENU, z_NWU = z_ENU
        # 等价于绕 Z 轴 +90°（yaw=+pi/2）。
        yaw = math.pi / 2.0
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = tx
        tf_msg.transform.translation.y = ty
        tf_msg.transform.translation.z = tz
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)
        self.get_logger().info(
            f'静态TF已发布: {parent_frame} -> {child_frame} | '
            f't=({tx:.3f}, {ty:.3f}, {tz:.3f}) | yaw=+90deg'
        )

    @staticmethod
    def yaw_to_quaternion(yaw: float):
        """仅绕 Z 轴旋转时的四元数计算（roll=pitch=0）。"""
        half = yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)


def main(args=None):
    """节点主入口。"""
    rclpy.init(args=args)
    node = MapArucoStaticTfNode()
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
