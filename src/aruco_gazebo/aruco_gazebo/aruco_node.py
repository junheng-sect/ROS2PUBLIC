#!/usr/bin/env python3
"""
ROS 2 ArUco 检测节点（aruco_gazebo 包）
- 动态切换标记 + CSV 日志记录 + 修复图像发布逻辑
- 适配 Gazebo 仿真相机
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from aruco_interfaces.msg import ArucoPose
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_matrix
from geometry_msgs.msg import TransformStamped
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import csv
import os
from datetime import datetime


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_gazebo')
        
        # ========== 参数声明 ==========
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('use_compressed', False)  # Gazebo 通常发布原始图像
        self.declare_parameter('publish_annotated_image', True)
        
        # 相机内参（默认值，可通过 launch 文件覆盖）
        self.declare_parameter('camera_fx', 269.5)
        self.declare_parameter('camera_fy', 269.5)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        
        # ArUco 标记参数
        self.declare_parameter('marker_size_33', 0.135)
        self.declare_parameter('marker_size_42', 0.017)
        self.declare_parameter('switch_distance', 0.5)
        self.declare_parameter('switch_hysteresis', 0.05)
        
        # CSV 日志参数
        self.declare_parameter('enable_csv_log', True)
        self.declare_parameter('csv_log_path', 
            os.path.expanduser('~/project/project_ws/src/aruco_gazebo/aruco_log/gazebo_detection_log.csv'))
        
        # ========== 获取参数 ==========
        self.image_topic = self.get_parameter('image_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        self.publish_annotated = self.get_parameter('publish_annotated_image').value
        self.fx = self.get_parameter('camera_fx').value
        self.fy = self.get_parameter('camera_fy').value
        self.cx = self.get_parameter('camera_cx').value
        self.cy = self.get_parameter('camera_cy').value
        self.switch_distance = self.get_parameter('switch_distance').value
        self.hysteresis = self.get_parameter('switch_hysteresis').value
        self.enable_csv_log = self.get_parameter('enable_csv_log').value
        self.csv_path = self.get_parameter('csv_log_path').value
        
        self.csv_file = None
        self.csv_writer = None
        self.current_marker_id = 33
        self.last_z_value = 0.0
        self.marker_lengths = {33: 0.135, 42: 0.017}
        self.default_marker_size = 0.05
        
        # ========== 订阅话题 ==========
        self.subscribe_topic = f'{self.image_topic}/compressed' if self.use_compressed else self.image_topic
        self.msg_type = CompressedImage if self.use_compressed else Image
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.subscription = self.create_subscription(
            self.msg_type, 
            self.subscribe_topic, 
            self.listener_callback, 
            qos
        )
        
        # ========== 发布者 ==========
        self.pose_publisher = self.create_publisher(ArucoPose, '/aruco_pose', 10)
        if self.publish_annotated:
            self.image_publisher = self.create_publisher(Image, '/aruco/image_annotated', 10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.bridge = CvBridge()
        
        # ========== ArUco 配置 ==========
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        try:
            self.parameters = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)
            self.use_new_api = True
        except AttributeError:
            self.parameters = aruco.DetectorParameters_create()
            self.use_new_api = False
        
        # ========== 相机矩阵 ==========
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx], 
            [0, self.fy, self.cy], 
            [0, 0, 1]
        ], dtype=np.float32)
        # Gazebo 理想相机畸变较小，可设为零
        self.dist_coeffs = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]], dtype=np.float32)
        
        # ========== 初始化 CSV ==========
        self._init_csv_log()
        
        # ========== 日志 ==========
        self.get_logger().info(f'📷 ArUco Gazebo Node started | Sub: {self.subscribe_topic}')
        self.get_logger().info(f'📐 Camera Matrix: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}')
        self.frame_count = 0
        self.detect_count = 0
        self.switch_count = 0

    def _init_csv_log(self):
        """初始化 CSV 文件，每次运行生成独立文件（带时间戳）"""
        if not self.enable_csv_log:
            return
        try:
            base_path = self.csv_path
            dir_name = os.path.dirname(base_path)
            file_name = os.path.basename(base_path)
            name, ext = os.path.splitext(file_name)
            
            # 添加启动时间戳
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            new_file_name = f"{name}_{timestamp}{ext}"
            
            if dir_name:
                self.csv_path = os.path.join(dir_name, new_file_name)
            else:
                self.csv_path = new_file_name
            
            # 确保目录存在
            if dir_name and not os.path.exists(dir_name):
                os.makedirs(dir_name)
            
            # 使用 'w' 模式新建文件
            self.csv_file = open(self.csv_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # 写入表头
            header = ['timestamp', 'aruco_id', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
            self.csv_writer.writerow(header)
            
            self.get_logger().info(f'📝 CSV 日志文件已创建：{self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'❌ 无法初始化 CSV 日志文件：{e}')
            self.enable_csv_log = False

    def _write_csv_log(self, ts, mid, x, y, z, roll, pitch, yaw):
        """写入 CSV 日志"""
        if not self.enable_csv_log or not self.csv_writer:
            return
        try:
            self.csv_writer.writerow([
                ts, mid, 
                f'{x:.6f}', f'{y:.6f}', f'{z:.6f}',
                f'{roll:.6f}', f'{pitch:.6f}', f'{yaw:.6f}'
            ])
            self.csv_file.flush()
        except Exception as e:
            self.get_logger().error(f'CSV write failed: {e}')

    def destroy_node(self):
        """节点销毁时关闭 CSV 文件"""
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()

    def _select_marker_id(self, z):
        """根据距离动态切换 ArUco 标记 ID"""
        if self.current_marker_id == 33 and z <= self.switch_distance - self.hysteresis:
            self.current_marker_id = 42
            self.switch_count += 1
            self.get_logger().warn(f'🔄 Switch: 33→42 @ Z={z:.3f}m')
        elif self.current_marker_id == 42 and z > self.switch_distance + self.hysteresis:
            self.current_marker_id = 33
            self.switch_count += 1
            self.get_logger().warn(f'🔄 Switch: 42→33 @ Z={z:.3f}m')
        return self.current_marker_id

    def listener_callback(self, msg):
        """图像回调函数"""
        self.frame_count += 1
        
        # ========== 图像转换 ==========
        try:
            if self.use_compressed:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            if cv_image is None or cv_image.size == 0:
                return
        except Exception as e:
            self.get_logger().error(f'Image convert error: {e}')
            return
        
        # ========== ArUco 检测 ==========
        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(cv_image)
        else:
            corners, ids, _ = aruco.detectMarkers(
                cv_image, 
                self.dictionary, 
                parameters=self.parameters
            )
        
        # ========== 处理检测结果 ==========
        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id not in [33, 42]:
                    continue
                
                # 获取标记尺寸
                marker_len = self.marker_lengths.get(marker_id, self.default_marker_size)
                
                # 位姿估计
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], 
                    marker_len, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                r, t = rvec[0][0], tvec[0][0]
                
                # 旋转矩阵转四元数
                R, _ = cv2.Rodrigues(r)
                q = quaternion_from_matrix(
                    np.vstack((
                        np.hstack((R, [[0], [0], [0]])), 
                        [0, 0, 0, 1]
                    ))
                )
                euler = euler_from_quaternion(q)
                roll, pitch, yaw = euler[0], euler[1], euler[2]
                
                # ========== 发布位姿 ==========
                pose_msg = ArucoPose()
                pose_msg.id = int(marker_id)
                pose_msg.x, pose_msg.y, pose_msg.z = float(t[0]), float(t[1]), float(t[2])
                pose_msg.roll, pose_msg.pitch, pose_msg.yaw = float(roll), float(pitch), float(yaw)
                self.pose_publisher.publish(pose_msg)
                
                # ========== CSV 记录 ==========
                ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                self._write_csv_log(ts, int(marker_id), t[0], t[1], t[2], roll, pitch, yaw)
                
                # ========== 绘制坐标轴 ==========
                try:
                    cv2.aruco.drawAxis(
                        cv_image, 
                        self.camera_matrix, 
                        self.dist_coeffs, 
                        r, 
                        t, 
                        marker_len * 0.5
                    )
                except (AttributeError, cv2.error):
                    pass
                
                # ========== 绘制标记框 ==========
                pts = corners[i][0].astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)
                center = np.mean(corners[i][0], axis=0).astype(np.int32)
                cv2.putText(
                    cv_image, 
                    f'ID:{marker_id}', 
                    tuple(center),
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.4, 
                    (0, 255, 0), 
                    1
                )
                
                # ========== TF 发布 ==========
                self._publish_tf(t, q, marker_id)
                
                # ========== 日志 ==========
                self.get_logger().info(
                    f'📍 ID {marker_id}: '
                    f'X={t[0]:.3f} Y={t[1]:.3f} Z={t[2]:.3f} '
                    f'Yaw={np.degrees(yaw):.1f}°'
                )
                self.detect_count += 1
        
        # ========== 🔥【关键】无论是否检测到标记，都发布图像 ==========
        # 添加状态文字
        cv2.putText(
            cv_image, 
            f'Frames: {self.frame_count} | Detects: {self.detect_count}', 
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.4, 
            (255, 255, 255), 
            1
        )
        cv2.putText(
            cv_image, 
            f'Mode: {"FAR" if self.current_marker_id==33 else "NEAR"} (ID {self.current_marker_id})',
            (10, 40), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.4, 
            (255, 255, 0), 
            1
        )
        
        if self.publish_annotated:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                annotated_msg.header = msg.header
                self.image_publisher.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'Publish image failed: {e}')

    def _publish_tf(self, tvec, quat, mid):
        """发布 TF 变换"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_link"
        t.child_frame_id = f"aruco_marker_{mid}"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = tvec
        t.transform.rotation.x, t.transform.rotation.y = quat[0], quat[1]
        t.transform.rotation.z, t.transform.rotation.w = quat[2], quat[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # launch 退出时可能已由其他路径触发 shutdown，避免重复关闭异常
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
