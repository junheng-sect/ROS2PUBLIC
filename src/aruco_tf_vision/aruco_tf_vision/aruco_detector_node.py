#!/usr/bin/env python3
"""
ArUco 检测节点（aruco_tf_vision 包）

功能：
- 从相机图像中检测 ArUco 标记
- 发布带标注的图像到 /aruco/image_annotated
- 发布 ArUco 位姿到 /aruco_pose
- 发布 TF 变换：map → vision_pose（基于 ArUco 位姿）

坐标系：
- map: 世界坐标系（ENU），起飞点
- vision_pose: 视觉估计的无人机位姿（基于 ArUco）
- camera_link: 相机坐标系
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from aruco_interfaces.msg import ArucoPose
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_matrix, quaternion_multiply
from geometry_msgs.msg import TransformStamped
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy
import os
import csv
from datetime import datetime


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # ========== 参数声明 ==========
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('use_compressed', False)
        self.declare_parameter('publish_annotated_image', True)

        # 相机内参（根据 SDF 文件：640x480, hfov=1.74 rad）
        # fx = fy = width / (2 * tan(hfov/2)) = 640 / (2 * tan(0.87)) ≈ 268.5
        # cx = width / 2 = 320.0
        # cy = height / 2 = 240.0
        self.declare_parameter('camera_fx', 268.5)
        self.declare_parameter('camera_fy', 268.5)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)

        # ArUco 标记参数
        self.declare_parameter('marker_size_33', 0.5)
        self.declare_parameter('marker_size_42', 0.063)
        self.declare_parameter('switch_distance', 0.5)
        self.declare_parameter('switch_hysteresis', 0.05)

        # TF 参数
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('vision_frame', 'vision_pose')
        self.declare_parameter('camera_frame', 'camera_link')
        
        # vision_pose 位置偏移（用于消除初始高度误差）
        self.declare_parameter('vision_offset_x', 0.0)
        self.declare_parameter('vision_offset_y', 0.0)
        self.declare_parameter('vision_offset_z', 0.0)
        # CSV 日志参数
        self.declare_parameter('enable_csv_log', True)
        self.declare_parameter(
            'csv_log_path',
            os.path.expanduser('~/project/project_ws/src/aruco_tf_vision/aruco_log/aruco_tf_vision_log.csv')
        )

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

        self.world_frame = self.get_parameter('world_frame').value
        self.vision_frame = self.get_parameter('vision_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # 获取 vision_pose 位置偏移
        self.vision_offset_x = self.get_parameter('vision_offset_x').value
        self.vision_offset_y = self.get_parameter('vision_offset_y').value
        self.vision_offset_z = self.get_parameter('vision_offset_z').value
        self.enable_csv_log = self.get_parameter('enable_csv_log').value
        self.csv_path = self.get_parameter('csv_log_path').value

        self.current_marker_id = 33
        self.marker_lengths = {33: 0.5, 42: 0.063}
        self.default_marker_size = 0.05
        self.latest_marker_yaw_deg = None
        self.latest_tvec = None
        self.latest_base_pose = None
        self.latest_vision_pose = None
        self.latest_mode = ""
        self.csv_file = None
        self.csv_writer = None

        # ========== 订阅话题 ==========
        self.subscribe_topic = f'{self.image_topic}/compressed' if self.use_compressed else self.image_topic
        self.msg_type = CompressedImage if self.use_compressed else Image
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscription = self.create_subscription(
            self.msg_type,
            self.subscribe_topic,
            self.listener_callback,
            qos
        )
        self.base_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.base_pose_callback,
            qos
        )
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
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
        ], dtype=np.float64)
        self.dist_coeffs = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]], dtype=np.float64)

        # ========== Z 轴修正旋转（绕 X 轴 180°）==========
        self.z_axis_correction = np.array([1.0, 0.0, 0.0, 0.0])  # [x, y, z, w]

        # ========== 日志相关 ==========
        self.frame_count = 0
        self.detect_count = 0
        self.latest_detection_log = ""  # 存储最新的检测结果用于日志
        self.latest_tvec_log = ""       # 存储最新的 tvec 三轴值用于 1Hz 日志
        self.latest_theta_log = ""      # 存储最新的 theta 用于 1Hz 日志
        # 1Hz 日志定时器
        self.log_timer = self.create_timer(1.0, self.log_callback)
        self._init_csv_log()

        # ========== 初始化日志 ==========
        self.get_logger().info(f'ArUco Detector Node started | Sub: {self.subscribe_topic}')
        self.get_logger().info(f'Camera Matrix: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}')

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
        detected = False
        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id not in [33, 42]:
                    continue

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

                # ========== TF 发布：map → vision_pose ==========
                # vision_pose 表示基于 ArUco 视觉估计的无人机位姿
                # 在 map 坐标系中，vision_pose = -aruco_pose（因为 ArUco 位姿是相机→标记）
                self._publish_vision_tf(t, q, marker_id)

                # 存储检测结果用于日志（1Hz）
                self.latest_detection_log = (
                    f'ID {marker_id}: '
                    f'X={t[0]:.3f} Y={t[1]:.3f} Z={t[2]:.3f} '
                    f'Yaw={np.degrees(yaw):.1f}°'
                )
                self.latest_marker_yaw_deg = float(np.degrees(yaw))
                self.latest_tvec = (
                    float(t[0]),
                    float(t[1]),
                    float(t[2]),
                )
                self.latest_tvec_log = (
                    f'tvec[0]={t[0]:.3f}, tvec[1]={t[1]:.3f}, tvec[2]={t[2]:.3f}'
                )
                self.detect_count += 1
                detected = True

        # ========== 发布标注图像 ==========
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
                annotated_msg.header.stamp = self.get_clock().now().to_msg()
                annotated_msg.header.frame_id = self.camera_frame
                self.image_publisher.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'Publish image failed: {e}')

    def log_callback(self):
        """1Hz 日志回调函数"""
        if self.latest_detection_log:
            self.get_logger().info(self.latest_detection_log)
            self.get_logger().info(self.latest_tvec_log)
            self.get_logger().info(self.latest_theta_log)
        self._write_csv_row()

    def state_callback(self, msg: State):
        self.latest_mode = msg.mode

    def base_pose_callback(self, msg: PoseStamped):
        q = msg.pose.orientation
        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        self.latest_base_pose = {
            'x': float(msg.pose.position.x),
            'y': float(msg.pose.position.y),
            'z': float(msg.pose.position.z),
            'yaw_deg': float(np.degrees(yaw))
        }

    @staticmethod
    def _quat_to_yaw(qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return np.arctan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_deg(deg):
        """将角度归一化到 [-180, 180]（度）"""
        return ((deg + 180.0) % 360.0) - 180.0

    def _init_csv_log(self):
        if not self.enable_csv_log:
            return
        try:
            base_path = self.csv_path
            dir_name = os.path.dirname(base_path)
            file_name = os.path.basename(base_path)
            name, ext = os.path.splitext(file_name)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            with_ts = f"{name}_{timestamp}{ext}"
            if dir_name:
                os.makedirs(dir_name, exist_ok=True)
                self.csv_path = os.path.join(dir_name, with_ts)
            else:
                self.csv_path = with_ts

            self.csv_file = open(self.csv_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'timestamp',
                'stage',
                'base_x',
                'base_y',
                'base_z',
                'base_yaw_deg',
                'vision_x',
                'vision_y',
                'vision_z',
                'vision_yaw_deg',
                'detector_yaw_deg',
                'tvec_x',
                'tvec_y',
                'tvec_z',
            ])
            self.get_logger().info(f'CSV 日志文件已创建：{self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'CSV 初始化失败: {e}')
            self.enable_csv_log = False

    def _write_csv_row(self):
        if not self.enable_csv_log or self.csv_writer is None:
            return
        base = self.latest_base_pose
        vision = self.latest_vision_pose
        stage = 'NO_BASE'
        if base is not None and vision is None:
            stage = 'NO_VISION'
        elif base is not None and vision is not None:
            stage = self.latest_mode if self.latest_mode else 'TRACKING'

        def _fmt(obj, key):
            if obj is None:
                return ''
            return f'{obj[key]:.6f}'

        marker_yaw = '' if self.latest_marker_yaw_deg is None else f'{self.latest_marker_yaw_deg:.6f}'
        tvec_x = '' if self.latest_tvec is None else f'{self.latest_tvec[0]:.6f}'
        tvec_y = '' if self.latest_tvec is None else f'{self.latest_tvec[1]:.6f}'
        tvec_z = '' if self.latest_tvec is None else f'{self.latest_tvec[2]:.6f}'
        self.csv_writer.writerow([
            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            stage,
            _fmt(base, 'x'),
            _fmt(base, 'y'),
            _fmt(base, 'z'),
            _fmt(base, 'yaw_deg'),
            _fmt(vision, 'x'),
            _fmt(vision, 'y'),
            _fmt(vision, 'z'),
            _fmt(vision, 'yaw_deg'),
            marker_yaw,
            tvec_x,
            tvec_y,
            tvec_z,
        ])
        self.csv_file.flush()

    def _publish_vision_tf(self, tvec, quat, mid):
        """
        发布 TF 变换：map → vision_pose

        vision_pose 表示基于 ArUco 视觉估计的无人机在 map 坐标系中的位姿

        坐标系说明：
        - 相机坐标系：X 向前，Y 向右，Z 向下
        - map 坐标系 (ENU)：X 向东，Y 向北，Z 向上
        - ArUco 位姿 tvec 表示：标记原点到相机原点的向量（在相机坐标系中）
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame  # map
        t.child_frame_id = self.vision_frame  # vision_pose

        # ========== 旋转修正 ==========
        # 1. Z 轴修正（绕 X 轴 180°）
        corrected_quat = quaternion_multiply(self.z_axis_correction, quat)

        # 2. 四元数求逆（共轭）
        inv_quat = np.array([
            -corrected_quat[0],
            -corrected_quat[1],
            -corrected_quat[2],
            corrected_quat[3]
        ])

        # 3. 绕自身 Z 轴旋转 +90° 修正（右乘：局部坐标系）
        z_90_quat = np.array([0.0, 0.0, 0.7071067811865476, 0.7071067811865476])
        final_quat = quaternion_multiply(inv_quat, z_90_quat)

        # ========== 位置计算（与 yaw 耦合）==========
        # theta 取 vision_pose 的 yaw 角（度）并转弧度，用于平移旋转补偿
        yaw_rad = self._quat_to_yaw(
            final_quat[0],
            final_quat[1],
            final_quat[2],
            final_quat[3]
        )
        yaw_deg = float(np.degrees(yaw_rad))
        theta = np.deg2rad(yaw_deg)
        yaw_aruco_deg = self._normalize_deg(yaw_deg - 90.0)
        self.latest_theta_log = (
            f'theta={theta:.6f} rad ({yaw_deg:.2f} deg), '
            f'yaw_aruco={yaw_aruco_deg:.2f} deg'
        )
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        t.transform.translation.x = -tvec[0] * sin_theta + tvec[1] * cos_theta + self.vision_offset_x
        t.transform.translation.y = tvec[0] * cos_theta + tvec[1] * sin_theta + self.vision_offset_y
        t.transform.translation.z = tvec[2] + self.vision_offset_z

        t.transform.rotation.x = final_quat[0]
        t.transform.rotation.y = final_quat[1]
        t.transform.rotation.z = final_quat[2]
        t.transform.rotation.w = final_quat[3]
        self.latest_vision_pose = {
            'x': float(t.transform.translation.x),
            'y': float(t.transform.translation.y),
            'z': float(t.transform.translation.z),
            'yaw_deg': yaw_deg
        }

        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
