#!/usr/bin/env python3
"""
ROS 2 ArUco 检测节点（动态切换标记版 + CSV 日志记录）
- Z > 0.5m：使用外层码（ID 33，尺寸 0.135m）
- Z <= 0.5m：使用内层码（ID 42，尺寸 0.017m）
- 添加切换滞回区间，避免频繁跳变
- 【新增】将检测数据记录到 CSV 文件
46行：是否记录CSV文件
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
# 【新增】CSV 和系统相关导入
import csv
import os
from datetime import datetime

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_mid')
        # ========== 1. 声明参数 ==========
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('publish_annotated_image', True)
        # 相机内参参数（320x240 分辨率）
        self.declare_parameter('camera_fx', 318.55)
        self.declare_parameter('camera_fy', 317.70)
        self.declare_parameter('camera_cx', 160.0)
        self.declare_parameter('camera_cy', 120.0)
        # 标记尺寸参数
        self.declare_parameter('marker_size_33', 0.135)
        self.declare_parameter('marker_size_42', 0.017)
        # 切换阈值参数
        self.declare_parameter('switch_distance', 0.5)
        self.declare_parameter('switch_hysteresis', 0.05)
        
        # 【新增】CSV 日志参数
        self.declare_parameter('enable_csv_log', True)  # 是否启用日志
        self.declare_parameter('csv_log_path', os.path.expanduser('~/aruco_detection_log.csv')) # 默认主目录下

        # 获取参数
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value
        self.publish_annotated = self.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        self.fx = self.get_parameter('camera_fx').get_parameter_value().double_value
        self.fy = self.get_parameter('camera_fy').get_parameter_value().double_value
        self.cx = self.get_parameter('camera_cx').get_parameter_value().double_value
        self.cy = self.get_parameter('camera_cy').get_parameter_value().double_value
        self.switch_distance = self.get_parameter('switch_distance').get_parameter_value().double_value
        self.hysteresis = self.get_parameter('switch_hysteresis').get_parameter_value().double_value
        
        # 【新增】获取日志参数
        self.enable_csv_log = self.get_parameter('enable_csv_log').get_parameter_value().bool_value
        self.csv_path = self.get_parameter('csv_log_path').get_parameter_value().string_value
        self.csv_file = None
        self.csv_writer = None

        # 标记切换状态
        self.current_marker_id = 33
        self.last_z_value = 0.0
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

        # ========== 6. 构建相机矩阵 ==========
        self.camera_matrix = np.array([
            [self.fx,  0.0,    self.cx],
            [0.0,    self.fy,  self.cy],
            [0.0,    0.0,      1.0    ]
        ], dtype=np.float32)
        self.dist_coeffs = np.array([
            [0.10629533008540253, -0.27708130888165716, -0.0020389632465854473, 0.0012458148297138881, 0.0]
        ], dtype=np.float32)

        # ========== 7. 初始化 CSV 日志文件 ==========
        self._init_csv_log()

        # 日志输出配置信息
        self.get_logger().info('📷 ========== 相机参数配置 ==========')
        self.get_logger().info(f'  分辨率：320x240')
        self.get_logger().info(f'  焦距：fx={self.fx:.2f}, fy={self.fy:.2f}')
        self.get_logger().info(f'  主点：cx={self.cx:.2f}, cy={self.cy:.2f}')
        self.get_logger().info(f'  标记尺寸：{self.marker_lengths}')
        self.get_logger().info(f'  切换距离：{self.switch_distance}m (滞回：±{self.hysteresis}m)')
        self.get_logger().info(f'  CSV 日志：{self.csv_path} (启用：{self.enable_csv_log})')
        self.get_logger().info('======================================')
        self.frame_count = 0
        self.detect_count = 0
        self.switch_count = 0
        self.get_logger().info(f'ArUco Detector Mid started, subscribing to: {self.subscribe_topic}')

    def _init_csv_log(self):
        """初始化 CSV 文件，创建目录并写入表头"""
        if not self.enable_csv_log:
            return
        
        try:
            # 确保目录存在
            dir_name = os.path.dirname(self.csv_path)
            if dir_name and not os.path.exists(dir_name):
                os.makedirs(dir_name)
            
            # 打开文件 (模式 'a' 表示追加，如果不存在则创建)
            self.csv_file = open(self.csv_path, mode='a', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # 如果文件是新的（大小为 0），写入表头
            if os.path.getsize(self.csv_path) == 0:
                header = ['timestamp', 'aruco_id', 'x', 'y', 'z', 'yaw']
                self.csv_writer.writerow(header)
                self.get_logger().info(f'📝 CSV 日志文件已创建：{self.csv_path}')
            else:
                self.get_logger().info(f'📝 追加写入现有 CSV 日志：{self.csv_path}')
                
        except Exception as e:
            self.get_logger().error(f'❌ 无法初始化 CSV 日志文件：{e}')
            self.enable_csv_log = False

    def _write_csv_log(self, timestamp, marker_id, x, y, z, yaw):
        """将数据写入 CSV 文件"""
        if not self.enable_csv_log or self.csv_writer is None:
            return
        
        try:
            row = [timestamp, marker_id, f'{x:.6f}', f'{y:.6f}', f'{z:.6f}', f'{yaw:.6f}']
            self.csv_writer.writerow(row)
            # 实时刷新缓冲区，防止程序崩溃丢失数据
            self.csv_file.flush() 
        except Exception as e:
            self.get_logger().error(f'❌ 写入 CSV 失败：{e}')

    def destroy_node(self):
        """节点销毁时关闭文件"""
        if self.csv_file:
            try:
                self.csv_file.close()
                self.get_logger().info('📝 CSV 文件已关闭')
            except Exception as e:
                self.get_logger().error(f'关闭 CSV 文件出错：{e}')
        super().destroy_node()

    def _select_marker_id(self, z_value):
        """
        根据 Z 值选择使用的标记 ID（带滞回逻辑）
        """
        if self.current_marker_id == 33:
            if z_value <= self.switch_distance - self.hysteresis:
                self.current_marker_id = 42
                self.switch_count += 1
                self.get_logger().warn(f'🔄 切换标记：外层码 (33) → 内层码 (42), Z={z_value:.3f}m')
        else:
            if z_value > self.switch_distance + self.hysteresis:
                self.current_marker_id = 33
                self.switch_count += 1
                self.get_logger().warn(f'🔄 切换标记：内层码 (42) → 外层码 (33), Z={z_value:.3f}m')
        return self.current_marker_id

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
        best_pose = None
        if ids is not None:
            detected_ids = ids.flatten().tolist()
            marker_poses = {}
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id not in [33, 42]:
                    continue
                selected_corners = corners[i]
                marker_length = self.marker_lengths.get(marker_id, self.default_marker_size)
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    selected_corners,
                    marker_length,
                    self.camera_matrix,
                    self.dist_coeffs)
                r = rvec[0][0]
                t = tvec[0][0]
                z_value = t[2]
                pixel_size = np.linalg.norm(selected_corners[0][0] - selected_corners[0][1])
                theoretical_z = (marker_length * self.fx) / pixel_size
                R, _ = cv2.Rodrigues(r)
                q = self.rotation_matrix_to_quaternion(R)
                euler = euler_from_quaternion(q)
                yaw_deg = np.degrees(euler[2])
                marker_poses[marker_id] = {
                    't': t, 'r': r, 'q': q, 'euler': euler,
                    'yaw_deg': yaw_deg, 'z': z_value, 'pixel_size': pixel_size,
                    'theoretical_z': theoretical_z, 'corners': selected_corners,
                    'marker_length': marker_length,
                }

            if len(marker_poses) > 0:
                if self.current_marker_id in marker_poses:
                    selected_id = self.current_marker_id
                    pose_info = marker_poses[selected_id]
                    self._select_marker_id(pose_info['z'])
                    self.last_z_value = pose_info['z']
                elif len(marker_poses) > 0:
                    selected_id = list(marker_poses.keys())[0]
                    pose_info = marker_poses[selected_id]
                    self.current_marker_id = selected_id
                    self.last_z_value = pose_info['z']
                    self.get_logger().info(f'⚠️  使用备用标记 ID {selected_id}')
                else:
                    return

                t = pose_info['t']
                r = pose_info['r']
                q = pose_info['q']
                euler = pose_info['euler']
                yaw_deg = pose_info['yaw_deg']
                z_value = pose_info['z']
                pixel_size = pose_info['pixel_size']
                theoretical_z = pose_info['theoretical_z']
                marker_length = pose_info['marker_length']
                corners = pose_info['corners']

                # 发布位姿消息
                pose_msg = ArucoPose()
                pose_msg.id = int(selected_id)
                pose_msg.x = float(t[0])
                pose_msg.y = float(t[1])
                pose_msg.z = float(t[2])
                pose_msg.roll = float(euler[0])
                pose_msg.pitch = float(euler[1])
                pose_msg.yaw = float(euler[2])
                self.pose_publisher.publish(pose_msg)

                # 【新增】记录 CSV 日志
                # 使用系统时间作为时间戳，方便外部分析
                now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                self._write_csv_log(now_str, int(selected_id), t[0], t[1], t[2], euler[2])

                # 绘制坐标轴
                try:
                    cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, r, t, marker_length * 0.5)
                except AttributeError:
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, r, t, marker_length * 0.5)
                
                # 发布 TF 变换
                self.publish_tf(t, q, selected_id)

                # 日志输出
                marker_type = "外层码" if selected_id == 33 else "内层码"
                self.get_logger().info(
                    f'📍 [{marker_type}] ID {selected_id}: '
                    f'X={t[0]:.3f}m, Y={t[1]:.3f}m, Z={t[2]:.3f}m, '
                    f'Yaw={yaw_deg:6.2f}°, pixel={pixel_size:.1f}px'
                )
                detected_count += 1
                self.detect_count += 1
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

                # 添加状态文字
                status_text = f'Detected: {detected_count} marker(s)'
                color = (0, 255, 0) if detected_count > 0 else (0, 0, 255)
                cv2.putText(cv_image, status_text, (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                if self.current_marker_id == 33:
                    mode_text = f'Mode: FAR (>0.5m) - ID 33'
                else:
                    mode_text = f'Mode: NEAR (<=0.5m) - ID 42'
                cv2.putText(cv_image, mode_text, (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                switch_text = f'Switches: {self.switch_count}'
                cv2.putText(cv_image, switch_text, (10, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)
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