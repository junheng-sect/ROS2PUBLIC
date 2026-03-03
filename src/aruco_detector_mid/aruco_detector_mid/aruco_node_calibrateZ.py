#!/usr/bin/env python3
"""
ROS 2 ArUco 检测节点（Z 轴修正版）
- 修正相机焦距与分辨率匹配问题
- 支持参数化配置相机内参
- 添加调试输出验证 Z 轴
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
        super().__init__('aruco_detector_mid')

        # ========== 1. 声明参数 ==========
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('publish_annotated_image', True)
        
        # 【关键修正】相机内参参数（320x240 分辨率）
        # 如果标定是 640x480，焦距需要减半
        self.declare_parameter('camera_fx', 318.55)  # 637.11 / 2
        self.declare_parameter('camera_fy', 317.70)  # 635.40 / 2
        self.declare_parameter('camera_cx', 160.0)   # 320 / 2
        self.declare_parameter('camera_cy', 120.0)   # 240 / 2
        
        # 标记尺寸参数
        self.declare_parameter('marker_size_33', 0.135)
        self.declare_parameter('marker_size_42', 0.017)

        # 获取参数
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value
        self.publish_annotated = self.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        
        self.fx = self.get_parameter('camera_fx').get_parameter_value().double_value
        self.fy = self.get_parameter('camera_fy').get_parameter_value().double_value
        self.cx = self.get_parameter('camera_cx').get_parameter_value().double_value
        self.cy = self.get_parameter('camera_cy').get_parameter_value().double_value
        
        self.marker_lengths = {
            33: self.get_parameter('marker_size_33').get_parameter_value().double_value,
            42: self.get_parameter('marker_size_42').get_parameter_value().double_value,
        }
        self.default_marker_size = 0.05

        # ========== 2. 确定订阅话题 ==========
        if self.use_compressed:
            self.subscribe_topic = f'{self.image_topic}/compressed'
            self.msg_type = CompressedImage
        else:
            self.subscribe_topic = self.image_topic
            self.msg_type = Image

        # ========== 3. QoS 配置 ==========
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            self.msg_type,
            self.subscribe_topic,
            self.listener_callback,
            qos_profile
        )

        # ========== 4. 创建发布者 ==========
        self.pose_publisher = self.create_publisher(ArucoPose, '/aruco_pose', 10)
        if self.publish_annotated:
            self.image_publisher = self.create_publisher(Image, '/aruco/image_annotated', 10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.bridge = CvBridge()

        # ========== 5. ArUco 配置 ==========
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        try:
            self.parameters = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)
            self.use_new_api = True
            self.get_logger().info('Using OpenCV 4.7.0+ ArUco API')
        except AttributeError:
            self.parameters = aruco.DetectorParameters_create()
            self.use_new_api = False
            self.get_logger().info('Using legacy OpenCV ArUco API')

        # ========== 6. 构建相机矩阵（修正后的参数）==========
        self.camera_matrix = np.array([
            [self.fx,  0.0,    self.cx],
            [0.0,    self.fy,  self.cy],
            [0.0,    0.0,      1.0    ]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.array([
            [0.10629533008540253, -0.27708130888165716, -0.0020389632465854473, 0.0012458148297138881, 0.0]
        ], dtype=np.float32)

        # ========== 7. 日志输出配置信息 ==========
        self.get_logger().info('📷 ========== 相机参数配置 ==========')
        self.get_logger().info(f'  分辨率：320x240')
        self.get_logger().info(f'  焦距：fx={self.fx:.2f}, fy={self.fy:.2f}')
        self.get_logger().info(f'  主点：cx={self.cx:.2f}, cy={self.cy:.2f}')
        self.get_logger().info(f'  标记尺寸：{self.marker_lengths}')
        self.get_logger().info('======================================')

        self.frame_count = 0
        self.detect_count = 0
        self.get_logger().info(f'ArUco Detector Mid started, subscribing to: {self.subscribe_topic}')

    def listener_callback(self, msg):
        self.frame_count += 1
        try:
            if self.use_compressed:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if cv_image is None or cv_image.size == 0:
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

            for i in range(len(ids)):
                marker_id = ids[i][0]
                selected_corners = corners[i]
                marker_length = self.marker_lengths.get(marker_id, self.default_marker_size)

                # 估计位姿
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    selected_corners,
                    marker_length,
                    self.camera_matrix,
                    self.dist_coeffs)

                r = rvec[0][0]
                t = tvec[0][0]

                # ========== 调试信息：像素尺寸 ==========
                pixel_size = np.linalg.norm(selected_corners[0][0] - selected_corners[0][1])
                
                # 理论深度计算（用于验证）
                theoretical_z = (marker_length * self.fx) / pixel_size

                # 旋转向量转四元数
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

                # 绘制坐标轴
                try:
                    cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, r, t, marker_length * 0.5)
                except AttributeError:
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, r, t, marker_length * 0.5)

                # 发布 TF 变换
                self.publish_tf(t, q, marker_id)

                # ========== 日志输出 ==========
                yaw_deg = np.degrees(euler[2])
                
                # 输出详细调试信息
                self.get_logger().info(
                    f'📍 ID {marker_id}: X={t[0]:.3f}m, Y={t[1]:.3f}m, Z={t[2]:.3f}m | '
                    f'pixel_size={pixel_size:.1f}px, theoretical_Z={theoretical_z:.3f}m'
                )
                
                detected_count += 1
                self.detect_count += 1

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        # 添加状态文字
        status_text = f'Detected: {detected_count} marker(s)'
        color = (0, 255, 0) if detected_count > 0 else (0, 0, 255)
        cv2.putText(cv_image, status_text, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        perf_text = f'Frames: {self.frame_count} | Detects: {self.detect_count}'
        cv2.putText(cv_image, perf_text, (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

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