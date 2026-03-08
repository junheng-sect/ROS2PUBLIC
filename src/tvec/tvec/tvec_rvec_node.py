#!/usr/bin/env python3

import csv
import os
from datetime import datetime

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from debug_interface.msg import TVecRVec
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class TVecRVecNode(Node):
    """ArUco 相对位姿测试节点：1Hz 日志 + CSV 记录 + RViz 图像显示。"""

    def __init__(self):
        super().__init__('tvec_rvec_node')

        # ---- 运行参数 ----
        # image_topic：ArUco 检测输入图像话题。
        # camera_*：用于位姿解算（solvePnP）的相机内参。
        # marker_size_*：不同 ArUco ID 对应的物理边长（单位：米）。
        # enable_csv_log/csv_log_path：是否保存 tvec/rvec 到 CSV 及其路径。
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_fx', 810.78076)
        self.declare_parameter('camera_fy', 813.75141)
        self.declare_parameter('camera_cx', 346.24076)
        self.declare_parameter('camera_cy', 251.50143)
        self.declare_parameter('dist_k1', -0.410508)
        self.declare_parameter('dist_k2', 0.102062)
        self.declare_parameter('dist_p1', 0.001503)
        self.declare_parameter('dist_p2', -0.000384)
        self.declare_parameter('dist_k3', 0.0)
        self.declare_parameter('marker_size_33', 0.5)
        self.declare_parameter('marker_size_42', 0.063)
        self.declare_parameter('enable_csv_log', True)
        self.declare_parameter(
            'csv_log_path',
            os.path.expanduser('~/project/zjh_ws/src/tvec/log/tvec_rvec_log.csv')
        )

        self.image_topic = self.get_parameter('image_topic').value
        self.fx = float(self.get_parameter('camera_fx').value)
        self.fy = float(self.get_parameter('camera_fy').value)
        self.cx = float(self.get_parameter('camera_cx').value)
        self.cy = float(self.get_parameter('camera_cy').value)
        self.k1 = float(self.get_parameter('dist_k1').value)
        self.k2 = float(self.get_parameter('dist_k2').value)
        self.p1 = float(self.get_parameter('dist_p1').value)
        self.p2 = float(self.get_parameter('dist_p2').value)
        self.k3 = float(self.get_parameter('dist_k3').value)
        self.marker_size_33 = float(self.get_parameter('marker_size_33').value)
        self.marker_size_42 = float(self.get_parameter('marker_size_42').value)
        self.enable_csv_log = bool(self.get_parameter('enable_csv_log').value)
        self.csv_path = self.get_parameter('csv_log_path').value

        # CvBridge 用于 ROS Image 与 OpenCV BGR 图像之间的转换。
        self.bridge = CvBridge()
        # 发布带标注图像，便于在 RViz 中观察检测结果。
        self.annotated_pub = self.create_publisher(Image, '/tvec/image_annotated', 10)
        # 发布调试位姿向量：用于 `ros2 topic echo /debug/tvec` 实时查看 tvec/rvec。
        self.debug_tvec_pub = self.create_publisher(TVecRVec, '/debug/tvec', 10)

        # 仿真桥接图像通常使用 BEST_EFFORT，QoS 需对齐以避免不兼容。
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, qos)

        # ArUco 检测接口兼容处理：
        # - 新版 OpenCV：cv2.aruco.ArucoDetector（OpenCV >= 4.7）。
        # - 旧版回退：cv2.aruco.detectMarkers。
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        try:
            self.detector = aruco.ArucoDetector(self.dictionary, aruco.DetectorParameters())
            self.use_new_api = True
        except AttributeError:
            self.detector = None
            self.parameters = aruco.DetectorParameters_create()
            self.use_new_api = False

        # 相机内参矩阵 K 与零畸变系数。
        # estimatePoseSingleMarkers 输出的 tvec/rvec 都以“相机坐标系”为参考。
        self.camera_matrix = np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        self.dist_coeffs = np.array(
            [[self.k1, self.k2, self.p1, self.p2, self.k3]],
            dtype=np.float64,
        )

        # 缓存最近一次检测结果，供 1Hz 定时日志与 CSV 写入使用。
        self.latest_id = None
        self.latest_tvec = None
        self.latest_rvec = None
        self.frame_count = 0
        self.detect_count = 0

        self.csv_file = None
        self.csv_writer = None
        self._init_csv()

        self.log_timer = self.create_timer(1.0, self.log_callback)
        self.get_logger().info(f'tvec/rvec test node started | sub={self.image_topic}')

    def _init_csv(self):
        # 使用时间戳创建 CSV，避免覆盖历史实验数据。
        if not self.enable_csv_log:
            return
        base = self.csv_path
        dirname = os.path.dirname(base)
        name, ext = os.path.splitext(os.path.basename(base))
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        if dirname:
            os.makedirs(dirname, exist_ok=True)
            self.csv_path = os.path.join(dirname, f'{name}_{ts}{ext}')
        else:
            self.csv_path = f'{name}_{ts}{ext}'

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'marker_id',
            'tvec_x', 'tvec_y', 'tvec_z',
            'rvec_x', 'rvec_y', 'rvec_z',
        ])
        self.get_logger().info(f'CSV created: {self.csv_path}')

    def image_callback(self, msg: Image):
        # 高频回调仅做检测与可视化；
        # 日志与 CSV 写盘在 log_callback 中按 1Hz 节流执行。
        self.frame_count += 1

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'Image convert error: {exc}')
            return

        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(cv_image)
        else:
            corners, ids, _ = aruco.detectMarkers(cv_image, self.dictionary, parameters=self.parameters)

        if ids is not None:
            for i in range(len(ids)):
                # 不同 marker_id 允许配置不同物理尺寸。
                marker_id = int(ids[i][0])
                marker_len = self.marker_size_33 if marker_id == 33 else self.marker_size_42

                # 位姿估计结果语义：
                # rvec：罗德里格斯旋转向量（marker 坐标系 -> 相机坐标系）。
                # tvec：平移向量（marker 原点在相机坐标系中的位置，单位米）。
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i],
                    marker_len,
                    self.camera_matrix,
                    self.dist_coeffs,
                )

                r = rvec[0][0]
                t = tvec[0][0]
                # 缓存最近值，用于周期日志打印与 CSV 输出。
                self.latest_id = marker_id
                self.latest_tvec = (float(t[0]), float(t[1]), float(t[2]))
                self.latest_rvec = (float(r[0]), float(r[1]), float(r[2]))
                self.detect_count += 1

                # 将当前检测到的向量发布到调试话题，方便在线观察中间变量。
                debug_msg = TVecRVec()
                debug_msg.header = msg.header
                debug_msg.tvec = [float(t[0]), float(t[1]), float(t[2])]
                debug_msg.rvec = [float(r[0]), float(r[1]), float(r[2])]
                self.debug_tvec_pub.publish(debug_msg)

                # 可视化元素：坐标轴 + 轮廓线 + 简要文本。
                try:
                    cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, r, t, marker_len * 0.5)
                except Exception:
                    pass
                pts = corners[i][0].astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)
                label = f'ID:{marker_id} t=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})'
                cv2.putText(cv_image, label, tuple(pts[0][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        cv2.putText(
            cv_image,
            f'Frames={self.frame_count} Detects={self.detect_count}',
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )

        try:
            # 发布标注后的图像到 RViz 图像面板。
            out = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'camera_link'
            self.annotated_pub.publish(out)
        except Exception as exc:
            self.get_logger().error(f'Publish annotated image failed: {exc}')

    def log_callback(self):
        # 1Hz 摘要日志 + CSV 刷盘，兼顾可读性与数据可靠落盘。
        if self.latest_tvec is None or self.latest_rvec is None:
            # 按当前调试需求，关闭该节点的周期性 INFO 日志输出。
            # self.get_logger().info('No detection in last period')
            return

        t = self.latest_tvec
        r = self.latest_rvec
        # 按当前调试需求，关闭该节点的周期性 INFO 日志输出。
        # self.get_logger().info(
        #     f'ID={self.latest_id} | '
        #     f'tvec=({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}) | '
        #     f'rvec=({r[0]:.4f}, {r[1]:.4f}, {r[2]:.4f})'
        # )

        if self.csv_writer is not None:
            self.csv_writer.writerow([
                datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                self.latest_id,
                f'{t[0]:.6f}', f'{t[1]:.6f}', f'{t[2]:.6f}',
                f'{r[0]:.6f}', f'{r[1]:.6f}', f'{r[2]:.6f}',
            ])
            self.csv_file.flush()

    def destroy_node(self):
        # 节点退出前确保 CSV 文件句柄正确关闭。
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        super().destroy_node()


def main(args=None):
    # 标准 ROS 2 节点生命周期封装。
    rclpy.init(args=args)
    node = TVecRVecNode()
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
