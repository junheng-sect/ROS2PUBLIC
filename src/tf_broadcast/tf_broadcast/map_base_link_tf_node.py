#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class MapBaseLinkTfNode(Node):
    """订阅 Pose 并广播 map->base_link TF。"""

    def __init__(self):
        super().__init__('map_base_link_tf_node')

        # 参数化配置，便于后续复用到不同话题或坐标系命名。
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')

        self.pose_topic = self.get_parameter('pose_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        # MAVROS 常用 BEST_EFFORT，需匹配避免 QoS 不兼容。
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.pose_callback, qos_profile
        )

        # 1Hz 调试日志缓存。
        self.latest_log = ''
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            f'map_base_link_tf_node 已启动 | sub={self.pose_topic} | '
            f'tf={self.parent_frame}->{self.child_frame}'
        )

    def pose_callback(self, msg: PoseStamped):
        # 直接把 PoseStamped 映射为 TransformStamped。
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.child_frame

        tf_msg.transform.translation.x = msg.pose.position.x
        tf_msg.transform.translation.y = msg.pose.position.y
        tf_msg.transform.translation.z = msg.pose.position.z+0.3
        tf_msg.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)

        self.latest_log = (
            f'发布TF {self.parent_frame}->{self.child_frame} | '
            f'x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}'
        )

    def log_callback(self):
        """1Hz 输出最近一次 TF 发布摘要。"""
        if self.latest_log:
            # 按当前调试需求，关闭该节点的周期性 INFO 日志输出。
            # self.get_logger().info(self.latest_log)
            return
        else:
            # 按当前调试需求，关闭该节点的周期性 INFO 日志输出。
            # self.get_logger().info('尚未收到 Pose 数据')
            return


def main(args=None):
    """节点主入口。"""
    rclpy.init(args=args)
    node = MapBaseLinkTfNode()
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
