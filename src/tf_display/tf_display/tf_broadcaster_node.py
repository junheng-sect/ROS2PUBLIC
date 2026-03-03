#!/usr/bin/env python3
"""
TF 坐标变换广播节点

功能：
- 订阅 MAVROS 的无人机位姿信息
- 发布 map → base_link 的 TF 变换
- 在 RViz 中显示世界坐标系（map）和无人机坐标系（base_link）

坐标系定义：
- map: 世界坐标系（ENU：东 - 北 - 天），即起飞点坐标系
- base_link: 无人机机体坐标系（原点为无人机质心）
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_broadcaster import TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math


class TFBroadcasterNode(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_node')

        # ========== 声明参数 ==========
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 50.0)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # ========== 创建 TF 广播器 ==========
        self.tf_broadcaster = TransformBroadcaster(self)

        # ========== QoS 配置（兼容 MAVROS）==========
        # MAVROS 使用 BEST_EFFORT 可靠性，需要匹配
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ========== 订阅 MAVROS 位姿话题 ==========
        # MAVROS 发布无人机在 ENU 坐标系下的位姿
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )

        # ========== 创建定时器（备用，用于静态 TF 发布）==========
        # 如果没有 MAVROS 数据，可以发布静态 TF
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # ========== 存储最新位姿 ==========
        self.latest_pose = None

        self.get_logger().info('TF Broadcaster 节点已启动')
        self.get_logger().info(f'  - 父坐标系：{self.frame_id}')
        self.get_logger().info(f'  - 子坐标系：{self.child_frame_id}')
        self.get_logger().info(f'  - 订阅话题：/mavros/local_position/pose')

    def pose_callback(self, msg: PoseStamped):
        """
        回调函数：处理 MAVROS 发布的无人机位姿

        参数:
            msg: PoseStamped 消息，包含无人机在 ENU 坐标系下的位置和姿态
        """
        self.latest_pose = msg
        self.get_logger().debug(f'收到位姿数据：x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}')

    def timer_callback(self):
        """
        定时器回调：发布 TF 变换

        如果有最新的位姿数据，使用位姿数据发布 TF
        否则发布静态 TF（原点重合）
        """
        transform_stamped = TransformStamped()

        # ========== 时间戳 ==========
        transform_stamped.header.stamp = self.get_clock().now().to_msg()

        # ========== 设置坐标系名称 ==========
        transform_stamped.header.frame_id = self.frame_id  # 父坐标系（map）
        transform_stamped.child_frame_id = self.child_frame_id  # 子坐标系（base_link）

        if self.latest_pose is not None:
            # ========== 使用 MAVROS 位姿数据 ==========
            # 位置（ENU 坐标系：东 - 北 - 天）
            transform_stamped.transform.translation.x = self.latest_pose.pose.position.x
            transform_stamped.transform.translation.y = self.latest_pose.pose.position.y
            transform_stamped.transform.translation.z = self.latest_pose.pose.position.z

            # 姿态（四元数）
            transform_stamped.transform.rotation = self.latest_pose.pose.orientation
            
            # 调试日志
            self.get_logger().info(f'发布 TF: x={self.latest_pose.pose.position.x:.3f}, y={self.latest_pose.pose.position.y:.3f}, z={self.latest_pose.pose.position.z:.3f}')
        else:
            # ========== 无数据时发布静态 TF ==========
            # 默认位置：原点
            transform_stamped.transform.translation.x = 0.0
            transform_stamped.transform.translation.y = 0.0
            transform_stamped.transform.translation.z = 0.0

            # 默认姿态：水平（四元数表示无旋转）
            transform_stamped.transform.rotation.x = 0.0
            transform_stamped.transform.rotation.y = 0.0
            transform_stamped.transform.rotation.z = 0.0
            transform_stamped.transform.rotation.w = 1.0
            
            # 调试日志
            self.get_logger().info('无 MAVROS 数据，发布静态 TF')

        # ========== 发布 TF 变换 ==========
        self.tf_broadcaster.sendTransform(transform_stamped)


def main(args=None):
    """主函数：初始化并运行节点"""
    rclpy.init(args=args)
    node = TFBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
