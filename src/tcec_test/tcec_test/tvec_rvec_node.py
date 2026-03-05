#!/usr/bin/env python3

import csv
import os
from datetime import datetime

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class TVecRVecNode(Node):
    """ArUco relative pose test node: 1Hz log + CSV + RViz image."""

    def __init__(self):
        super().__init__('tvec_rvec_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_fx', 268.5)
        self.declare_parameter('camera_fy', 268.5)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        self.declare_parameter('marker_size_33', 0.5)
        self.declare_parameter('marker_size_42', 0.063)
        self.declare_parameter('enable_csv_log', True)
        self.declare_parameter(
            'csv_log_path',
            os.path.expanduser('~/project/project_ws/src/tcec_test/log/tvec_rvec_log.csv')
        )

        self.image_topic = self.get_parameter('image_topic').value
        self.fx = float(self.get_parameter('camera_fx').value)
        self.fy = float(self.get_parameter('camera_fy').value)
        self.cx = float(self.get_parameter('camera_cx').value)
        self.cy = float(self.get_parameter('camera_cy').value)
        self.marker_size_33 = float(self.get_parameter('marker_size_33').value)
        self.marker_size_42 = float(self.get_parameter('marker_size_42').value)
        self.enable_csv_log = bool(self.get_parameter('enable_csv_log').value)
        self.csv_path = self.get_parameter('csv_log_path').value

        self.bridge = CvBridge()
        self.annotated_pub = self.create_publisher(Image, '/tcec_test/image_annotated', 10)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, qos)

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        try:
            self.detector = aruco.ArucoDetector(self.dictionary, aruco.DetectorParameters())
            self.use_new_api = True
        except AttributeError:
            self.detector = None
            self.parameters = aruco.DetectorParameters_create()
            self.use_new_api = False

        self.camera_matrix = np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        self.dist_coeffs = np.zeros((1, 5), dtype=np.float64)

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
                marker_id = int(ids[i][0])
                marker_len = self.marker_size_33 if marker_id == 33 else self.marker_size_42

                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i],
                    marker_len,
                    self.camera_matrix,
                    self.dist_coeffs,
                )

                r = rvec[0][0]
                t = tvec[0][0]
                self.latest_id = marker_id
                self.latest_tvec = (float(t[0]), float(t[1]), float(t[2]))
                self.latest_rvec = (float(r[0]), float(r[1]), float(r[2]))
                self.detect_count += 1

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
            out = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'camera_link'
            self.annotated_pub.publish(out)
        except Exception as exc:
            self.get_logger().error(f'Publish annotated image failed: {exc}')

    def log_callback(self):
        if self.latest_tvec is None or self.latest_rvec is None:
            self.get_logger().info('No detection in last period')
            return

        t = self.latest_tvec
        r = self.latest_rvec
        self.get_logger().info(
            f'ID={self.latest_id} | '
            f'tvec=({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}) | '
            f'rvec=({r[0]:.4f}, {r[1]:.4f}, {r[2]:.4f})'
        )

        if self.csv_writer is not None:
            self.csv_writer.writerow([
                datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                self.latest_id,
                f'{t[0]:.6f}', f'{t[1]:.6f}', f'{t[2]:.6f}',
                f'{r[0]:.6f}', f'{r[1]:.6f}', f'{r[2]:.6f}',
            ])
            self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        super().destroy_node()


def main(args=None):
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
