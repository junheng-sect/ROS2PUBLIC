#!/usr/bin/env python3
"""
ROS 2 摄像头节点
订阅压缩图像 → 解码 → 发布原始图像
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # ========== 参数 ==========
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('output_topic', '/camera/output')
        self.declare_parameter('log_fps', True)

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.log_fps = self.get_parameter('log_fps').get_parameter_value().bool_value

        self.bridge = CvBridge()

        # ========== QoS 配置 ==========
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ========== 确定订阅话题 ==========
        if self.use_compressed:
            self.subscribe_topic = f'{self.camera_topic}/compressed'
            self.msg_type = CompressedImage
        else:
            self.subscribe_topic = self.camera_topic
            self.msg_type = Image

        self.get_logger().info(f'Subscribing to: {self.subscribe_topic}')

        # ========== 订阅者 ==========
        self.subscription = self.create_subscription(
            self.msg_type,
            self.subscribe_topic,
            self.image_callback,
            qos_profile
        )

        # ========== 发布者 ==========
        self.image_pub = self.create_publisher(Image, self.output_topic, 10)
        self.get_logger().info(f'Publishing to: {self.output_topic}')

        # ========== 统计 ==========
        self.frame_count = 0
        self.last_time = self.get_clock().now()

        if self.log_fps:
            self.timer = self.create_timer(1.0, self.log_fps_callback)

        self.get_logger().info('Camera Node started!')

    def image_callback(self, msg):
        self.frame_count += 1

        try:
            if self.use_compressed:
                # 解码压缩图像
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                # 转换原始图像
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if cv_image is None or cv_image.size == 0:
                self.get_logger().warn('Failed to decode image')
                return

            # 发布解码后的图像
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            output_msg.header = msg.header
            self.image_pub.publish(output_msg)

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def log_fps_callback(self):
        if self.log_fps:
            current_time = self.get_clock().now()
            elapsed = (current_time - self.last_time).nanoseconds / 1e9
            if elapsed > 0:
                fps = self.frame_count / elapsed
                self.get_logger().info(f'📊 FPS: {fps:.1f} | Topic: {self.output_topic}')
            self.frame_count = 0
            self.last_time = current_time

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()