#!/usr/bin/env python3

import csv
import os
from datetime import datetime
from typing import Optional

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from debug_interface.msg import TVecRVec
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from tvec.camera_profiles import (
    get_camera_profile,
    get_default_camera_profile_name,
)


DEFAULT_CAMERA_PROFILE = get_default_camera_profile_name()
DEFAULT_CAMERA_SETTINGS = get_camera_profile(DEFAULT_CAMERA_PROFILE)


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
        # camera_profile 默认取统一配置文件中的默认项，避免旧相机参数继续散落在代码里。
        self.declare_parameter('camera_profile', DEFAULT_CAMERA_PROFILE)
        self.declare_parameter('camera_fx', DEFAULT_CAMERA_SETTINGS['camera_fx'])
        self.declare_parameter('camera_fy', DEFAULT_CAMERA_SETTINGS['camera_fy'])
        self.declare_parameter('camera_cx', DEFAULT_CAMERA_SETTINGS['camera_cx'])
        self.declare_parameter('camera_cy', DEFAULT_CAMERA_SETTINGS['camera_cy'])
        self.declare_parameter('dist_k1', DEFAULT_CAMERA_SETTINGS['dist_k1'])
        self.declare_parameter('dist_k2', DEFAULT_CAMERA_SETTINGS['dist_k2'])
        self.declare_parameter('dist_p1', DEFAULT_CAMERA_SETTINGS['dist_p1'])
        self.declare_parameter('dist_p2', DEFAULT_CAMERA_SETTINGS['dist_p2'])
        self.declare_parameter('dist_k3', DEFAULT_CAMERA_SETTINGS['dist_k3'])
        self.declare_parameter('marker_size_33', 0.193)
        self.declare_parameter('marker_size_42', 0.025)
        # ArUco 字典参数，支持在 launch 中快速切换不同码本。
        self.declare_parameter('aruco_dictionary', 'DICT_5X5_1000')
        # 图像订阅 QoS 默认使用 best_effort，以兼容 usb_cam 的常见发布配置。
        self.declare_parameter('image_qos_reliability', 'best_effort')
        # 关闭该参数后仅保留位姿解算与调试数据发布，不再生成标注图像。
        self.declare_parameter('publish_annotated_image', True)
        # 检测循环使用独立定时器，只处理当前缓存中的最新一帧。
        # 周期保持较小值，用于尽快消费最新缓存而不在订阅回调中阻塞。
        self.declare_parameter('processing_period_sec', 0.001)
        self.declare_parameter('enable_csv_log', True)
        self.declare_parameter(
            'csv_log_path',
            os.path.expanduser('~/project/zjh_ws/src/tvec/log/tvec_rvec_log.csv')
        )

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_profile = str(self.get_parameter('camera_profile').value).strip()
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
        self.aruco_dictionary_name = str(self.get_parameter('aruco_dictionary').value).strip()
        self.image_qos_reliability = str(self.get_parameter('image_qos_reliability').value).strip().lower()
        self.publish_annotated_image = bool(
            self.get_parameter('publish_annotated_image').value
        )
        self.processing_period_sec = max(
            float(self.get_parameter('processing_period_sec').value),
            0.001,
        )
        self.enable_csv_log = bool(self.get_parameter('enable_csv_log').value)
        self.csv_path = self.get_parameter('csv_log_path').value

        # CvBridge 用于 ROS Image 与 OpenCV BGR 图像之间的转换。
        self.bridge = CvBridge()
        # 发布带标注图像，便于在 RViz 中观察检测结果。
        # 若当前只关心位姿链路，则直接关闭该发布器，减少整帧复制与序列化开销。
        self.annotated_pub = None
        if self.publish_annotated_image:
            self.annotated_pub = self.create_publisher(Image, '/tvec/image_annotated', 10)
        # 发布调试位姿向量：用于 `ros2 topic echo /debug/tvec` 实时查看 tvec/rvec。
        self.debug_tvec_pub = self.create_publisher(TVecRVec, '/debug/tvec', 10)

        # 图像 QoS 默认按实机 USB 相机使用 RELIABLE；
        # 如需兼容某些仿真链路可传 image_qos_reliability:=best_effort。
        if self.image_qos_reliability == 'best_effort':
            reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            reliability = ReliabilityPolicy.RELIABLE
        # 将输入图像队列压到 1，优先减少旧帧积压。
        # 若当前帧尚未处理完，新到达的图像不会继续在订阅端排成长队。
        qos = QoSProfile(reliability=reliability, depth=1)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, qos)

        # ArUco 检测接口兼容处理：
        # - 新版 OpenCV：cv2.aruco.ArucoDetector（OpenCV >= 4.7）。
        # - 旧版回退：cv2.aruco.detectMarkers。
        # 支持通过参数切换常见 ArUco 字典，避免“能收图但始终不识别”的情况。
        aruco_dict_map = {
            'DICT_4X4_50': aruco.DICT_4X4_50,
            'DICT_4X4_100': aruco.DICT_4X4_100,
            'DICT_4X4_250': aruco.DICT_4X4_250,
            'DICT_4X4_1000': aruco.DICT_4X4_1000,
            'DICT_5X5_50': aruco.DICT_5X5_50,
            'DICT_5X5_100': aruco.DICT_5X5_100,
            'DICT_5X5_250': aruco.DICT_5X5_250,
            'DICT_5X5_1000': aruco.DICT_5X5_1000,
            'DICT_6X6_50': aruco.DICT_6X6_50,
            'DICT_6X6_100': aruco.DICT_6X6_100,
            'DICT_6X6_250': aruco.DICT_6X6_250,
            'DICT_6X6_1000': aruco.DICT_6X6_1000,
            'DICT_7X7_50': aruco.DICT_7X7_50,
            'DICT_7X7_100': aruco.DICT_7X7_100,
            'DICT_7X7_250': aruco.DICT_7X7_250,
            'DICT_7X7_1000': aruco.DICT_7X7_1000,
            'DICT_ARUCO_ORIGINAL': aruco.DICT_ARUCO_ORIGINAL,
        }
        dict_key = self.aruco_dictionary_name.upper()
        if dict_key not in aruco_dict_map:
            self.get_logger().warn(
                f'未知 aruco_dictionary={self.aruco_dictionary_name}，回退为 DICT_5X5_1000'
            )
            dict_key = 'DICT_5X5_1000'
        self.dictionary = aruco.getPredefinedDictionary(aruco_dict_map[dict_key])
        try:
            self.detector = aruco.ArucoDetector(self.dictionary, aruco.DetectorParameters())
            self.use_new_api = True
        except AttributeError:
            self.detector = None
            self.parameters = aruco.DetectorParameters_create()
            self.use_new_api = False

        # 相机内参矩阵 K 与畸变系数。
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
        # 图像缓存改为“最新帧优先”：
        # - 订阅回调只负责覆盖最新一帧，不在回调里执行 ArUco 检测。
        # - 定时处理逻辑每次只拿当前最新缓存，避免顺序吃完整个历史队列。
        self.cached_image_msg: Optional[Image] = None
        self.cached_tvec_cb_start_stamp = None
        self.cached_frame_serial = 0
        self.last_processed_frame_serial = 0

        self.csv_file = None
        self.csv_writer = None
        self._init_csv()

        # 高频处理定时器只消费当前缓存的最新帧。
        self.processing_timer = self.create_timer(
            self.processing_period_sec,
            self.process_latest_image,
        )
        self.log_timer = self.create_timer(1.0, self.log_callback)
        self.get_logger().info(
            'tvec/rvec test node started | '
            f'profile={self.camera_profile} | sub={self.image_topic} | '
            f'image_qos={self.image_qos_reliability} | annotated_image='
            f'{"enabled" if self.publish_annotated_image else "disabled"} | '
            f'processing_period_sec={self.processing_period_sec:.3f} | '
            f'dict={dict_key} | '
            f'K=({self.fx:.3f}, {self.fy:.3f}, {self.cx:.3f}, {self.cy:.3f})'
        )

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
        # 高频回调只缓存最新图像，不在这里执行检测。
        # 这样做的目标是：新帧来了就覆盖旧缓存，避免顺序吃大量历史帧。
        self.frame_count += 1
        # 记录第 2 级回调起点，用于分析图像从发布到真正开始处理前的排队时间。
        tvec_cb_start_stamp = self.get_clock().now().to_msg()
        self.cached_image_msg = msg
        self.cached_tvec_cb_start_stamp = tvec_cb_start_stamp
        self.cached_frame_serial += 1

    def process_latest_image(self):
        # 若没有新缓存，则直接返回，避免空转做无意义工作。
        if self.cached_image_msg is None:
            return
        if self.cached_frame_serial == self.last_processed_frame_serial:
            return

        # 取出当前最新缓存对应的快照。
        # 即使在处理期间又来了新帧，下一轮也只会继续拿最新缓存，不回头补旧帧。
        msg = self.cached_image_msg
        tvec_cb_start_stamp = self.cached_tvec_cb_start_stamp
        frame_serial = self.cached_frame_serial
        self.last_processed_frame_serial = frame_serial

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'Image convert error: {exc}')
            return
        # 当前节点没有额外图像预处理，本时间点表示 CvBridge 转换完成。
        tvec_cv_bridge_done_stamp = self.get_clock().now().to_msg()

        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(cv_image)
        else:
            corners, ids, _ = aruco.detectMarkers(cv_image, self.dictionary, parameters=self.parameters)
        # 检测阶段结束后记录时间，用于区分图像转换与检测开销。
        tvec_detect_done_stamp = self.get_clock().now().to_msg()

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
                # 单帧可能发布多条结果，因此位姿解算完成时刻按每条结果单独记录。
                tvec_pose_done_stamp = self.get_clock().now().to_msg()

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
                debug_msg.tvec_cb_start_stamp = tvec_cb_start_stamp
                debug_msg.tvec_cv_bridge_done_stamp = tvec_cv_bridge_done_stamp
                debug_msg.tvec_detect_done_stamp = tvec_detect_done_stamp
                debug_msg.tvec_pose_done_stamp = tvec_pose_done_stamp
                debug_msg.tvec_pub_stamp = self.get_clock().now().to_msg()
                debug_msg.tvec = [float(t[0]), float(t[1]), float(t[2])]
                debug_msg.rvec = [float(r[0]), float(r[1]), float(r[2])]
                self.debug_tvec_pub.publish(debug_msg)

                if self.publish_annotated_image:
                    # 可视化元素：坐标轴 + 轮廓线 + 简要文本。
                    try:
                        cv2.aruco.drawAxis(
                            cv_image,
                            self.camera_matrix,
                            self.dist_coeffs,
                            r,
                            t,
                            marker_len * 0.5,
                        )
                    except Exception:
                        pass
                    pts = corners[i][0].astype(np.int32).reshape((-1, 1, 2))
                    cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)
                    label = f'ID:{marker_id} t=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})'
                    cv2.putText(
                        cv_image,
                        label,
                        tuple(pts[0][0]),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (0, 255, 255),
                        1,
                    )

        if not self.publish_annotated_image:
            return

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
