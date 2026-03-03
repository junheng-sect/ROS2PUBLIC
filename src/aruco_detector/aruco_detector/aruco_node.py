#!/usr/bin/env python3
"""
ROS 2 ArUco 检测节点（优化版）
- 直接订阅压缩话题 /image_raw/compressed
- 兼容不同 OpenCV 版本
- 输出位姿日志 + 发布带标记图像
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from aruco_interfaces.msg import ArucoPose
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ========== 声明参数 ==========
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('publish_annotated_image', True)
        
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value
        self.publish_annotated = self.get_parameter('publish_annotated_image').get_parameter_value().bool_value

        # ========== 确定订阅话题 ==========
        if self.use_compressed:
            self.subscribe_topic = f'{self.image_topic}/compressed'
            self.msg_type = CompressedImage
        else:
            self.subscribe_topic = self.image_topic
            self.msg_type = Image

        # ========== QoS 配置（优化性能）==========
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # 只保留最新帧，防止堆积
        )
        
        self.subscription = self.create_subscription(
            self.msg_type,
            self.subscribe_topic,
            self.listener_callback,
            qos_profile
        )
        
        # ========== 创建发布者 ==========
        self.pose_publisher = self.create_publisher(ArucoPose, '/aruco_pose', 10)
        
        if self.publish_annotated:
            self.image_publisher = self.create_publisher(Image, '/aruco/image_annotated', 10)
            self.get_logger().info('Publishing annotated images to /aruco/image_annotated')
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.bridge = CvBridge()

        # ========== ArUco 配置 ==========
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        
        try:
            # OpenCV 4.7.0+ 新 API
            self.parameters = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)
            self.use_new_api = True
            self.get_logger().info('Using OpenCV 4.7.0+ ArUco API')
        except AttributeError:
            # OpenCV < 4.7.0 旧 API
            self.parameters = aruco.DetectorParameters_create()
            self.use_new_api = False
            self.get_logger().info('Using legacy OpenCV ArUco API')

        # ========== 相机参数（使用你的标定值）==========
        self.camera_matrix = np.array([
            [637.1145761300412,  0.0,       326.72351990464625],
            [0.0,         635.4004044167282, 234.5232390422],
            [0.0,         0.0,        1.0       ]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([
            [0.10629533008540253, -0.27708130888165716, -0.0020389632465854473, 0.0012458148297138881, 0.0]
        ], dtype=np.float32)

        # 标记尺寸
        self.marker_lengths = {
            33: 0.135,   # 外层码
            42: 0.017,  # 内层码
        }
        self.default_marker_size = 0.05

        # 性能统计
        self.frame_count = 0
        self.detect_count = 0

        self.get_logger().info(f'ArUco Detector started, subscribing to: {self.subscribe_topic}')

    def listener_callback(self, msg):
        self.frame_count += 1

        try:
            # 根据消息类型解码图像
            if self.use_compressed:
                # 压缩图像解码（JPEG）
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                # 原始图像转换
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if cv_image is None or cv_image.size == 0:
                self.get_logger().warn('Failed to decode image')
                return

        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        # 检测 ArUco
        if self.use_new_api:
            corners, ids, rejected = self.detector.detectMarkers(cv_image)
        else:
            corners, ids, rejected = aruco.detectMarkers(
                cv_image, self.dictionary, parameters=self.parameters)

        detected_count = 0

        if ids is not None:
            detected_ids = ids.flatten().tolist()
            self.get_logger().info(f'🎯 Detected {len(ids)} marker(s), IDs: {detected_ids}')
            
            # 处理每个检测到的标记
            for i in range(len(ids)):
                marker_id = ids[i][0]
                selected_corners = corners[i]
                
                # 获取标记尺寸
                if marker_id in self.marker_lengths:
                    marker_length = self.marker_lengths[marker_id]
                else:
                    marker_length = self.default_marker_size

                # 估计位姿
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    selected_corners, 
                    marker_length, 
                    self.camera_matrix, 
                    self.dist_coeffs)

                r = rvec[0][0]
                t = tvec[0][0]

                # 旋转向量转欧拉角
                R, _ = cv2.Rodrigues(r)
                q = self.rotation_matrix_to_quaternion(R)
                euler = euler_from_quaternion(q)

                # 发布位姿消息
                pose_msg = ArucoPose()
                pose_msg.id = int(marker_id)
                pose_msg.x = float(t[0])
                pose_msg.y = float(t[1])
                pose_msg.z = float(t[2])
                pose_msg.roll = float(euler[0])
                pose_msg.pitch = float(euler[1])
                pose_msg.yaw = float(euler[2])
                self.pose_publisher.publish(pose_msg)

                # ========== 绘制坐标轴（兼容不同 OpenCV 版本）==========
                try:
                    # OpenCV 4.7.0+ 新 API
                    cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, r, t, marker_length * 0.5)
                except AttributeError:
                    # OpenCV < 4.7.0 旧 API
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, r, t, marker_length * 0.5)

                # 发布 TF 变换
                self.publish_tf(t, q, marker_id)

                # 日志输出位姿
                yaw_deg = np.degrees(euler[2])
                self.get_logger().info(
                    f'📍 ID {marker_id}: X={t[0]:.3f}m, Y={t[1]:.3f}m, Z={t[2]:.3f}m, Yaw={yaw_deg:6.2f}°'
                )
                
                detected_count += 1
                self.detect_count += 1

        # 绘制所有检测到的标记框
        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        # 添加检测状态和性能文字
        if detected_count > 0:
            status_text = f'Detected: {detected_count} marker(s)'
            cv2.putText(cv_image, status_text, (10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        else:
            status_text = 'No ArUco detected'
            cv2.putText(cv_image, status_text, (10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 添加帧率信息
        perf_text = f'Frames: {self.frame_count} | Detects: {self.detect_count}'
        cv2.putText(cv_image, perf_text, (10, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

        # 发布带标记的图像
        if self.publish_annotated:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                annotated_msg.header = msg.header
                self.image_publisher.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish annotated image: {e}')

    def rotation_matrix_to_quaternion(self, R):
        import tf_transformations as tf
        return tf.quaternion_from_matrix(np.vstack((np.hstack((R, [[0],[0],[0]])), [0,0,0,1])))

    def publish_tf(self, tvec, quat, marker_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_link"
        t.child_frame_id = f"aruco_marker_{marker_id}"
        t.transform.translation.x = tvec[0]
        t.transform.translation.y = tvec[1]
        t.transform.translation.z = tvec[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()