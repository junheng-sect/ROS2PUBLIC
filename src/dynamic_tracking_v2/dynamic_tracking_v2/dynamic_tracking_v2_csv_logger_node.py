#!/usr/bin/env python3

import csv
import io
import math
import os
from datetime import datetime
from typing import Optional

import rclpy
from debug_interface.msg import ArucoBasePose, ImageRawTiming, PipelineTiming, TVecRVec
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu


class DynamicTrackingV2CsvLoggerNode(Node):
    """记录调参过程到 CSV，并在退出时输出摘要指标."""

    def __init__(self):
        super().__init__('dynamic_tracking_v2_csv_logger_node')

        # ===== 输入话题 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('image_raw_timing_topic', '/debug/image_raw_timing')
        self.declare_parameter('raw_tvec_topic', '/debug/tvec')
        self.declare_parameter('pipeline_timing_topic', '/debug/pipeline_timing')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('local_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('attitude_topic', '/mavros/imu/data')
        self.declare_parameter('setpoint_raw_topic', '/mavros/setpoint_raw/local')

        # ===== 输出路径 =====
        self.declare_parameter(
            'output_dir',
            '/home/zjh/project/rasip_pi_ws/log/tracking_csv',
        )
        self.declare_parameter('file_prefix', 'dynamic_tracking_v2')
        self.declare_parameter(
            'summary_csv_path',
            '/home/zjh/project/rasip_pi_ws/log/tracking_csv/'
            'dynamic_tracking_v2_summary.csv',
        )

        # ===== 采样参数 =====
        self.declare_parameter('sample_rate_hz', 30.0)
        self.declare_parameter('stale_timeout_sec', 0.5)
        self.declare_parameter('flush_interval_sec', 1.0)

        # ===== 目标/控制参数快照 =====
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 2.5)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('kp_xy', 0.5)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.08)
        self.declare_parameter('kp_x', float('nan'))
        self.declare_parameter('ki_x', float('nan'))
        self.declare_parameter('kd_x', float('nan'))
        self.declare_parameter('kp_y', float('nan'))
        self.declare_parameter('ki_y', float('nan'))
        self.declare_parameter('kd_y', float('nan'))
        self.declare_parameter('kp_z', 0.5)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.03)
        self.declare_parameter('kp_yaw', 0.4)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.03)
        self.declare_parameter('camera_yaw_compensation_deg', 0.0)
        self.declare_parameter('v_limit', 0.8)
        self.declare_parameter('vz_limit', 0.3)
        self.declare_parameter('yaw_rate_limit', 0.4)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('enable_z_hold', True)
        # 记录运行时的相机元数据，便于后续按分辨率汇总延迟结果。
        # 这里开启动态类型，兼容 launch 里传入的字符串/整数/布尔值。
        dynamic_meta_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter(
            'camera_profile',
            '',
            dynamic_meta_descriptor,
        )
        self.declare_parameter(
            'image_width',
            '',
            dynamic_meta_descriptor,
        )
        self.declare_parameter(
            'image_height',
            '',
            dynamic_meta_descriptor,
        )
        self.declare_parameter(
            'publish_annotated_image',
            True,
            dynamic_meta_descriptor,
        )

        self.pose_topic = self.get_parameter('pose_topic').value
        self.image_raw_timing_topic = self.get_parameter('image_raw_timing_topic').value
        self.raw_tvec_topic = self.get_parameter('raw_tvec_topic').value
        self.pipeline_timing_topic = self.get_parameter('pipeline_timing_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.local_pose_topic = self.get_parameter('local_pose_topic').value
        self.attitude_topic = self.get_parameter('attitude_topic').value
        self.setpoint_raw_topic = self.get_parameter('setpoint_raw_topic').value

        self.output_dir = self.get_parameter('output_dir').value
        self.file_prefix = self.get_parameter('file_prefix').value
        self.summary_csv_path = self.get_parameter('summary_csv_path').value
        self.sample_rate_hz = float(self.get_parameter('sample_rate_hz').value)
        self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
        self.flush_interval_sec = float(
            self.get_parameter('flush_interval_sec').value
        )

        # 记录完整参数快照，便于后续回溯“这一条数据对应哪组参数”。
        self.param_snapshot = {
            'target_x': float(self.get_parameter('target_x').value),
            'target_y': float(self.get_parameter('target_y').value),
            'target_z': float(self.get_parameter('target_z').value),
            'target_yaw': float(self.get_parameter('target_yaw').value),
            'kp_xy': float(self.get_parameter('kp_xy').value),
            'ki_xy': float(self.get_parameter('ki_xy').value),
            'kd_xy': float(self.get_parameter('kd_xy').value),
            'kp_x': float(self.get_parameter('kp_x').value),
            'ki_x': float(self.get_parameter('ki_x').value),
            'kd_x': float(self.get_parameter('kd_x').value),
            'kp_y': float(self.get_parameter('kp_y').value),
            'ki_y': float(self.get_parameter('ki_y').value),
            'kd_y': float(self.get_parameter('kd_y').value),
            'kp_z': float(self.get_parameter('kp_z').value),
            'ki_z': float(self.get_parameter('ki_z').value),
            'kd_z': float(self.get_parameter('kd_z').value),
            'kp_yaw': float(self.get_parameter('kp_yaw').value),
            'ki_yaw': float(self.get_parameter('ki_yaw').value),
            'kd_yaw': float(self.get_parameter('kd_yaw').value),
            'camera_yaw_compensation_deg': float(
                self.get_parameter('camera_yaw_compensation_deg').value
            ),
            'v_limit': float(self.get_parameter('v_limit').value),
            'vz_limit': float(self.get_parameter('vz_limit').value),
            'yaw_rate_limit': float(
                self.get_parameter('yaw_rate_limit').value
            ),
            'velocity_deadband': float(
                self.get_parameter('velocity_deadband').value
            ),
            'control_rate_hz': float(self.get_parameter('control_rate_hz').value),
            'pose_timeout_sec': float(self.get_parameter('pose_timeout_sec').value),
            'require_offboard': int(
                bool(self.get_parameter('require_offboard').value)
            ),
            'enable_z_hold': int(bool(self.get_parameter('enable_z_hold').value)),
            'camera_profile': str(self.get_parameter('camera_profile').value),
            'image_width': str(self.get_parameter('image_width').value),
            'image_height': str(self.get_parameter('image_height').value),
            'publish_annotated_image': int(
                bool(self.get_parameter('publish_annotated_image').value)
            ),
        }
        self.camera_yaw_compensation_rad = math.radians(
            self.param_snapshot['camera_yaw_compensation_deg']
        )

        # ===== 最新数据缓存 =====
        self.state_msg: Optional[State] = None
        self.pose_msg: Optional[ArucoBasePose] = None
        self.image_raw_timing_msg: Optional[ImageRawTiming] = None
        self.raw_tvec_msg: Optional[TVecRVec] = None
        self.pipeline_timing_msg: Optional[PipelineTiming] = None
        self.local_pose_msg: Optional[PoseStamped] = None
        self.attitude_msg: Optional[Imu] = None
        self.setpoint_msg: Optional[PositionTarget] = None

        self.pose_rx_time = None
        self.image_raw_timing_rx_time = None
        self.raw_tvec_rx_time = None
        self.pipeline_timing_rx_time = None
        self.local_pose_rx_time = None
        self.attitude_rx_time = None
        self.setpoint_rx_time = None
        self.latest_image_header_stamp_time = None
        self.latest_setpoint_publish_stamp_time = None
        self.latest_image_to_setpoint_latency_sec = float('nan')
        self.last_flush_time = self.get_clock().now()
        self.image_raw_timing_by_key = {}
        self.image_raw_timing_key_order = []

        # ===== 指标缓存 =====
        self.metric_rows = []

        os.makedirs(self.output_dir, exist_ok=True)
        now_text = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir,
            f'{self.file_prefix}_{now_text}.csv',
        )
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file, lineterminator='\n')
        self.csv_writer.writerow([
            'ros_time_sec',
            'mode',
            'armed',
            'connected',
            'aruco_fresh',
            'aruco_age_sec',
            'aruco_x',
            'aruco_y',
            'aruco_z',
            'aruco_yaw_rad',
            'yaw_rel_raw_rad',
            'yaw_rel_corrected_rad',
            'ex_marker',
            'ey_marker',
            'ex_body',
            'ey_body',
            'ez',
            'eyaw',
            'local_pose_fresh',
            'local_pose_age_sec',
            'local_x',
            'local_y',
            'local_z',
            'local_yaw_rad',
            'setpoint_fresh',
            'setpoint_age_sec',
            'sp_frame',
            'sp_type_mask',
            'sp_vx',
            'sp_vy',
            'sp_vz',
            'sp_yaw_rate',
            'image_header_stamp_sec',
            'image_source_stamp_sec',
            'image_received_stamp_sec',
            'image_probe_pub_sec',
            'tvec_cb_start_sec',
            'tvec_cv_bridge_done_sec',
            'tvec_detect_done_sec',
            'tvec_pose_done_sec',
            'tvec_pub_sec',
            'tf_cb_start_sec',
            'tf_pub_sec',
            'ctrl_cb_start_sec',
            'setpoint_pub_sec',
            'setpoint_publish_stamp_sec',
            'image_to_setpoint_latency_sec',
            'image_header_to_source_sec',
            'image_source_to_received_sec',
            'image_source_to_tvec_cb_sec',
            'tvec_queue_sec',
            'tvec_cv_bridge_sec',
            'tvec_detect_sec',
            'tvec_pose_estimate_sec',
            'tvec_publish_gap_sec',
            'tf_queue_sec',
            'tf_compute_sec',
            'ctrl_queue_sec',
            'ctrl_compute_sec',
            'tvec_total_compute_sec',
            'total_latency_sec',
            'target_x',
            'target_y',
            'target_z',
            'target_yaw',
            'kp_xy',
            'ki_xy',
            'kd_xy',
            'kp_x',
            'ki_x',
            'kd_x',
            'kp_y',
            'ki_y',
            'kd_y',
            'kp_z',
            'ki_z',
            'kd_z',
            'kp_yaw',
            'ki_yaw',
            'kd_yaw',
            'camera_yaw_compensation_deg',
            'v_limit',
            'vz_limit',
            'yaw_rate_limit',
            'velocity_deadband',
            'control_rate_hz',
            'pose_timeout_sec',
            'require_offboard',
            'enable_z_hold',
            'camera_profile',
            'image_width',
            'image_height',
            'publish_annotated_image',
            'raw_tvec_x',
            'raw_tvec_y',
            'raw_tvec_z',
            'raw_rvec_x',
            'raw_rvec_y',
            'raw_rvec_z',
            'local_roll_rad',
            'local_pitch_rad',
        ])
        self.csv_file.flush()

        mavros_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(State, self.state_topic, self._on_state, 10)
        self.create_subscription(ArucoBasePose, self.pose_topic, self._on_pose, 10)
        self.create_subscription(
            ImageRawTiming,
            self.image_raw_timing_topic,
            self._on_image_raw_timing,
            10,
        )
        self.create_subscription(TVecRVec, self.raw_tvec_topic, self._on_raw_tvec, 10)
        self.create_subscription(
            PipelineTiming,
            self.pipeline_timing_topic,
            self._on_pipeline_timing,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.local_pose_topic,
            self._on_local_pose,
            mavros_pose_qos,
        )
        self.create_subscription(
            Imu,
            self.attitude_topic,
            self._on_attitude,
            mavros_pose_qos,
        )
        self.create_subscription(
            PositionTarget,
            self.setpoint_raw_topic,
            self._on_setpoint,
            10,
        )

        timer_period = 1.0 / max(self.sample_rate_hz, 1.0)
        self.create_timer(timer_period, self._write_row)

        self.get_logger().info(
            f'PID调参 CSV 记录已启动，输出文件: {self.csv_path}'
        )
        self.get_logger().info(
            '参数快照 | '
            f'image_raw_timing_topic={self.image_raw_timing_topic} | '
            f'raw_tvec_topic={self.raw_tvec_topic} | '
            f'pipeline_timing_topic={self.pipeline_timing_topic} | '
            f'attitude_topic={self.attitude_topic} | '
            f'camera_yaw_compensation_deg='
            f'{self.param_snapshot["camera_yaw_compensation_deg"]:.3f} | '
            f'v_limit={self.param_snapshot["v_limit"]:.3f} | '
            f'vz_limit={self.param_snapshot["vz_limit"]:.3f} | '
            f'yaw_rate_limit={self.param_snapshot["yaw_rate_limit"]:.3f} | '
            f'camera_profile={self.param_snapshot["camera_profile"]} | '
            f'image_width={self.param_snapshot["image_width"]} | '
            f'image_height={self.param_snapshot["image_height"]} | '
            f'publish_annotated_image='
            f'{self.param_snapshot["publish_annotated_image"]} | '
            'Z 使用 aruco_z，yaw 使用 yaw_rel_corrected 闭环'
        )

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        """将角度归一化到 [-pi, pi]."""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def rotate_marker_error_to_body(
        ex_marker: float,
        ey_marker: float,
        yaw_rel_corrected: float,
    ) -> tuple[float, float]:
        """与控制节点保持一致：e_body = R(-yaw_rel_corrected) * e_marker."""
        cos_yaw = math.cos(yaw_rel_corrected)
        sin_yaw = math.sin(yaw_rel_corrected)
        ex_body = cos_yaw * ex_marker + sin_yaw * ey_marker
        ey_body = -sin_yaw * ex_marker + cos_yaw * ey_marker
        return ex_body, ey_body

    @staticmethod
    def _quat_to_rpy(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> tuple[float, float, float]:
        """四元数转 roll/pitch/yaw（单位：rad）."""
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _age_sec(stamp_time, now_time) -> float:
        """计算数据距离当前时刻的年龄，缺失时返回一个极大值."""
        if stamp_time is None:
            return 1e9
        return (now_time - stamp_time).nanoseconds / 1e9

    @staticmethod
    def _stamp_key(stamp_msg) -> Optional[tuple[int, int]]:
        """把 builtin Time 转成字典键，便于跨话题按同一帧匹配 timing."""
        if stamp_msg is None:
            return None
        try:
            return (int(stamp_msg.sec), int(stamp_msg.nanosec))
        except (AttributeError, TypeError, ValueError):
            return None

    @staticmethod
    def _time_from_stamp_msg(stamp_msg) -> Optional[Time]:
        """把消息头时间戳转成 Time，便于统一计算端到端延迟."""
        if stamp_msg is None:
            return None
        try:
            return Time.from_msg(stamp_msg)
        except (AttributeError, TypeError, ValueError):
            return None

    @staticmethod
    def _time_to_sec(stamp_time: Optional[Time]) -> float:
        """把 Time 转成浮点秒，缺失时返回 NaN."""
        if stamp_time is None:
            return float('nan')
        return stamp_time.nanoseconds / 1e9

    @staticmethod
    def _delta_sec(start_time: Optional[Time], end_time: Optional[Time]) -> float:
        """计算两个时间戳差值；任一缺失时返回 NaN。"""
        if start_time is None or end_time is None:
            return float('nan')
        return (end_time - start_time).nanoseconds / 1e9

    def _is_fresh(self, stamp_time, now_time, timeout_sec: float) -> bool:
        """判断数据是否仍处于新鲜时间窗内."""
        return self._age_sec(stamp_time, now_time) <= timeout_sec

    def _on_state(self, msg: State):
        self.state_msg = msg

    def _on_pose(self, msg: ArucoBasePose):
        self.pose_msg = msg
        self.pose_rx_time = self.get_clock().now()
        # /debug/aruco_pose.header.stamp 沿用图像原始时间戳，可直接视为感知链输入时刻。
        self.latest_image_header_stamp_time = self._time_from_stamp_msg(
            msg.header.stamp
        )

    def _on_image_raw_timing(self, msg: ImageRawTiming):
        """缓存原始图像发布时间元信息，供后续按 header.stamp 精确匹配。"""
        self.image_raw_timing_msg = msg
        self.image_raw_timing_rx_time = self.get_clock().now()
        key = self._stamp_key(msg.header.stamp)
        if key is None:
            return
        self.image_raw_timing_by_key[key] = msg
        self.image_raw_timing_key_order.append(key)
        while len(self.image_raw_timing_key_order) > 256:
            expired_key = self.image_raw_timing_key_order.pop(0)
            if expired_key == key:
                continue
            self.image_raw_timing_by_key.pop(expired_key, None)

    def _on_raw_tvec(self, msg: TVecRVec):
        self.raw_tvec_msg = msg
        self.raw_tvec_rx_time = self.get_clock().now()

    def _on_pipeline_timing(self, msg: PipelineTiming):
        """优先记录各节点内部真实埋点，避免 logger 自己的接收时刻污染分析。"""
        self.pipeline_timing_msg = msg
        self.pipeline_timing_rx_time = self.get_clock().now()
        self.latest_image_header_stamp_time = self._time_from_stamp_msg(
            msg.header.stamp
        )
        self.latest_setpoint_publish_stamp_time = self._time_from_stamp_msg(
            msg.setpoint_pub_stamp
        )
        self.latest_image_to_setpoint_latency_sec = float('nan')
        if (
            self.latest_image_header_stamp_time is not None
            and self.latest_setpoint_publish_stamp_time is not None
        ):
            latency_sec = self._delta_sec(
                self.latest_image_header_stamp_time,
                self.latest_setpoint_publish_stamp_time,
            )
            if math.isfinite(latency_sec):
                self.latest_image_to_setpoint_latency_sec = latency_sec

    def _on_local_pose(self, msg: PoseStamped):
        self.local_pose_msg = msg
        self.local_pose_rx_time = self.get_clock().now()

    def _on_attitude(self, msg: Imu):
        self.attitude_msg = msg
        self.attitude_rx_time = self.get_clock().now()

    def _on_setpoint(self, msg: PositionTarget):
        self.setpoint_msg = msg
        self.setpoint_rx_time = self.get_clock().now()
        # 端到端延迟优先以 /debug/pipeline_timing 中的真实埋点为准；
        # 若 timing 尚未到达，再临时回退为 setpoint 自身 header.stamp。
        if self.pipeline_timing_msg is None:
            self.latest_setpoint_publish_stamp_time = self._time_from_stamp_msg(
                msg.header.stamp
            )
            self.latest_image_to_setpoint_latency_sec = float('nan')
            if (
                self.latest_image_header_stamp_time is not None
                and self.latest_setpoint_publish_stamp_time is not None
            ):
                latency_sec = self._delta_sec(
                    self.latest_image_header_stamp_time,
                    self.latest_setpoint_publish_stamp_time,
                )
                if math.isfinite(latency_sec):
                    self.latest_image_to_setpoint_latency_sec = latency_sec

    def _write_row(self):
        """按固定频率把当前控制状态写入单次 CSV."""
        now_time = self.get_clock().now()
        now_msg = now_time.to_msg()
        ros_time_sec = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9

        mode = self.state_msg.mode if self.state_msg is not None else ''
        armed = int(self.state_msg.armed) if self.state_msg is not None else -1
        connected = (
            int(self.state_msg.connected)
            if self.state_msg is not None else -1
        )

        aruco_fresh = int(
            self._is_fresh(self.pose_rx_time, now_time, self.stale_timeout_sec)
        )
        aruco_age = self._age_sec(self.pose_rx_time, now_time)
        if self.pose_msg is not None:
            aruco_x = float(self.pose_msg.x)
            aruco_y = float(self.pose_msg.y)
            aruco_z = float(self.pose_msg.z)
            aruco_yaw = float(self.pose_msg.yaw)
            yaw_rel_raw = self.wrap_to_pi(-aruco_yaw)
            yaw_rel_corrected = self.wrap_to_pi(
                yaw_rel_raw + self.camera_yaw_compensation_rad
            )
            ex_marker = self.param_snapshot['target_x'] - aruco_x
            ey_marker = aruco_y - self.param_snapshot['target_y']
            ex_body, ey_body = self.rotate_marker_error_to_body(
                ex_marker,
                ey_marker,
                yaw_rel_corrected,
            )
            ez = self.param_snapshot['target_z'] - aruco_z
            eyaw = self.wrap_to_pi(
                self.param_snapshot['target_yaw'] - yaw_rel_corrected
            )
        else:
            aruco_x = float('nan')
            aruco_y = float('nan')
            aruco_z = float('nan')
            aruco_yaw = float('nan')
            yaw_rel_raw = float('nan')
            yaw_rel_corrected = float('nan')
            ex_marker = float('nan')
            ey_marker = float('nan')
            ex_body = float('nan')
            ey_body = float('nan')
            ez = float('nan')
            eyaw = float('nan')

        if self.raw_tvec_msg is not None:
            raw_tvec_x = float(self.raw_tvec_msg.tvec[0])
            raw_tvec_y = float(self.raw_tvec_msg.tvec[1])
            raw_tvec_z = float(self.raw_tvec_msg.tvec[2])
            raw_rvec_x = float(self.raw_tvec_msg.rvec[0])
            raw_rvec_y = float(self.raw_tvec_msg.rvec[1])
            raw_rvec_z = float(self.raw_tvec_msg.rvec[2])
        else:
            raw_tvec_x = float('nan')
            raw_tvec_y = float('nan')
            raw_tvec_z = float('nan')
            raw_rvec_x = float('nan')
            raw_rvec_y = float('nan')
            raw_rvec_z = float('nan')

        local_fresh = int(
            self._is_fresh(
                self.local_pose_rx_time,
                now_time,
                self.stale_timeout_sec,
            )
        )
        local_age = self._age_sec(self.local_pose_rx_time, now_time)
        if self.local_pose_msg is not None:
            pose = self.local_pose_msg.pose
            local_x = float(pose.position.x)
            local_y = float(pose.position.y)
            local_z = float(pose.position.z)
        else:
            local_x = float('nan')
            local_y = float('nan')
            local_z = float('nan')

        if self.attitude_msg is not None:
            attitude_orientation = self.attitude_msg.orientation
        elif self.local_pose_msg is not None:
            attitude_orientation = self.local_pose_msg.pose.orientation
        else:
            attitude_orientation = None

        if attitude_orientation is not None:
            local_roll, local_pitch, local_yaw = self._quat_to_rpy(
                attitude_orientation.x,
                attitude_orientation.y,
                attitude_orientation.z,
                attitude_orientation.w,
            )
        else:
            local_roll = float('nan')
            local_pitch = float('nan')
            local_yaw = float('nan')

        sp_fresh = int(
            self._is_fresh(
                self.setpoint_rx_time,
                now_time,
                self.stale_timeout_sec,
            )
        )
        sp_age = self._age_sec(self.setpoint_rx_time, now_time)
        if self.setpoint_msg is not None:
            sp_frame = int(self.setpoint_msg.coordinate_frame)
            sp_type_mask = int(self.setpoint_msg.type_mask)
            sp_vx = float(self.setpoint_msg.velocity.x)
            sp_vy = float(self.setpoint_msg.velocity.y)
            sp_vz = float(self.setpoint_msg.velocity.z)
            sp_yaw_rate = float(self.setpoint_msg.yaw_rate)
        else:
            sp_frame = -1
            sp_type_mask = -1
            sp_vx = float('nan')
            sp_vy = float('nan')
            sp_vz = float('nan')
            sp_yaw_rate = float('nan')

        image_header_stamp_sec = self._time_to_sec(
            self.latest_image_header_stamp_time
        )
        image_header_key = None
        if self.pipeline_timing_msg is not None:
            image_header_key = self._stamp_key(self.pipeline_timing_msg.header.stamp)
        elif self.pose_msg is not None:
            image_header_key = self._stamp_key(self.pose_msg.header.stamp)
        image_raw_timing_msg = self.image_raw_timing_by_key.get(image_header_key)
        if image_raw_timing_msg is not None:
            image_source_time = self._time_from_stamp_msg(
                image_raw_timing_msg.image_source_stamp
            )
            image_received_time = self._time_from_stamp_msg(
                image_raw_timing_msg.image_received_stamp
            )
            image_probe_pub_time = self._time_from_stamp_msg(
                image_raw_timing_msg.image_probe_pub_stamp
            )
        else:
            image_source_time = None
            image_received_time = None
            image_probe_pub_time = None

        timing_msg = self.pipeline_timing_msg
        if timing_msg is not None:
            tvec_cb_start_time = self._time_from_stamp_msg(
                timing_msg.tvec_cb_start_stamp
            )
            tvec_cv_bridge_done_time = self._time_from_stamp_msg(
                timing_msg.tvec_cv_bridge_done_stamp
            )
            tvec_detect_done_time = self._time_from_stamp_msg(
                timing_msg.tvec_detect_done_stamp
            )
            tvec_pose_done_time = self._time_from_stamp_msg(
                timing_msg.tvec_pose_done_stamp
            )
            tvec_pub_time = self._time_from_stamp_msg(timing_msg.tvec_pub_stamp)
            tf_cb_start_time = self._time_from_stamp_msg(
                timing_msg.tf_cb_start_stamp
            )
            tf_pub_time = self._time_from_stamp_msg(timing_msg.tf_pub_stamp)
            ctrl_cb_start_time = self._time_from_stamp_msg(
                timing_msg.ctrl_cb_start_stamp
            )
            setpoint_pub_time = self._time_from_stamp_msg(
                timing_msg.setpoint_pub_stamp
            )
        else:
            tvec_cb_start_time = None
            tvec_cv_bridge_done_time = None
            tvec_detect_done_time = None
            tvec_pose_done_time = None
            tvec_pub_time = None
            tf_cb_start_time = None
            tf_pub_time = None
            ctrl_cb_start_time = None
            setpoint_pub_time = self.latest_setpoint_publish_stamp_time

        image_source_stamp_sec = self._time_to_sec(image_source_time)
        image_received_stamp_sec = self._time_to_sec(image_received_time)
        image_probe_pub_sec = self._time_to_sec(image_probe_pub_time)
        tvec_cb_start_sec = self._time_to_sec(tvec_cb_start_time)
        tvec_cv_bridge_done_sec = self._time_to_sec(tvec_cv_bridge_done_time)
        tvec_detect_done_sec = self._time_to_sec(tvec_detect_done_time)
        tvec_pose_done_sec = self._time_to_sec(tvec_pose_done_time)
        tvec_pub_sec = self._time_to_sec(tvec_pub_time)
        tf_cb_start_sec = self._time_to_sec(tf_cb_start_time)
        tf_pub_sec = self._time_to_sec(tf_pub_time)
        ctrl_cb_start_sec = self._time_to_sec(ctrl_cb_start_time)
        setpoint_pub_sec = self._time_to_sec(setpoint_pub_time)
        setpoint_publish_stamp_sec = self._time_to_sec(
            self.latest_setpoint_publish_stamp_time
        )
        if math.isfinite(setpoint_pub_sec):
            setpoint_publish_stamp_sec = setpoint_pub_sec

        image_header_to_source_sec = self._delta_sec(
            self.latest_image_header_stamp_time,
            image_source_time,
        )
        image_source_to_received_sec = self._delta_sec(
            image_source_time,
            image_received_time,
        )
        image_source_to_tvec_cb_sec = self._delta_sec(
            image_source_time,
            tvec_cb_start_time,
        )
        tvec_queue_sec = self._delta_sec(
            self.latest_image_header_stamp_time,
            tvec_cb_start_time,
        )
        tvec_cv_bridge_sec = self._delta_sec(
            tvec_cb_start_time,
            tvec_cv_bridge_done_time,
        )
        tvec_detect_sec = self._delta_sec(
            tvec_cv_bridge_done_time,
            tvec_detect_done_time,
        )
        tvec_pose_estimate_sec = self._delta_sec(
            tvec_detect_done_time,
            tvec_pose_done_time,
        )
        tvec_publish_gap_sec = self._delta_sec(
            tvec_pose_done_time,
            tvec_pub_time,
        )
        tf_queue_sec = self._delta_sec(tvec_pub_time, tf_cb_start_time)
        tf_compute_sec = self._delta_sec(tf_cb_start_time, tf_pub_time)
        ctrl_queue_sec = self._delta_sec(tf_pub_time, ctrl_cb_start_time)
        ctrl_compute_sec = self._delta_sec(
            ctrl_cb_start_time,
            setpoint_pub_time,
        )
        tvec_total_compute_sec = self._delta_sec(
            tvec_cb_start_time,
            tvec_pub_time,
        )
        total_latency_sec = self._delta_sec(
            self.latest_image_header_stamp_time,
            setpoint_pub_time,
        )
        image_to_setpoint_latency_sec = total_latency_sec
        if not math.isfinite(image_to_setpoint_latency_sec):
            image_to_setpoint_latency_sec = self.latest_image_to_setpoint_latency_sec
            total_latency_sec = image_to_setpoint_latency_sec

        self.csv_writer.writerow([
            ros_time_sec,
            mode,
            armed,
            connected,
            aruco_fresh,
            aruco_age,
            aruco_x,
            aruco_y,
            aruco_z,
            aruco_yaw,
            yaw_rel_raw,
            yaw_rel_corrected,
            ex_marker,
            ey_marker,
            ex_body,
            ey_body,
            ez,
            eyaw,
            local_fresh,
            local_age,
            local_x,
            local_y,
            local_z,
            local_yaw,
            sp_fresh,
            sp_age,
            sp_frame,
            sp_type_mask,
            sp_vx,
            sp_vy,
            sp_vz,
            sp_yaw_rate,
            image_header_stamp_sec,
            image_source_stamp_sec,
            image_received_stamp_sec,
            image_probe_pub_sec,
            tvec_cb_start_sec,
            tvec_cv_bridge_done_sec,
            tvec_detect_done_sec,
            tvec_pose_done_sec,
            tvec_pub_sec,
            tf_cb_start_sec,
            tf_pub_sec,
            ctrl_cb_start_sec,
            setpoint_pub_sec,
            setpoint_publish_stamp_sec,
            image_to_setpoint_latency_sec,
            image_header_to_source_sec,
            image_source_to_received_sec,
            image_source_to_tvec_cb_sec,
            tvec_queue_sec,
            tvec_cv_bridge_sec,
            tvec_detect_sec,
            tvec_pose_estimate_sec,
            tvec_publish_gap_sec,
            tf_queue_sec,
            tf_compute_sec,
            ctrl_queue_sec,
            ctrl_compute_sec,
            tvec_total_compute_sec,
            total_latency_sec,
            self.param_snapshot['target_x'],
            self.param_snapshot['target_y'],
            self.param_snapshot['target_z'],
            self.param_snapshot['target_yaw'],
            self.param_snapshot['kp_xy'],
            self.param_snapshot['ki_xy'],
            self.param_snapshot['kd_xy'],
            self.param_snapshot['kp_x'],
            self.param_snapshot['ki_x'],
            self.param_snapshot['kd_x'],
            self.param_snapshot['kp_y'],
            self.param_snapshot['ki_y'],
            self.param_snapshot['kd_y'],
            self.param_snapshot['kp_z'],
            self.param_snapshot['ki_z'],
            self.param_snapshot['kd_z'],
            self.param_snapshot['kp_yaw'],
            self.param_snapshot['ki_yaw'],
            self.param_snapshot['kd_yaw'],
            self.param_snapshot['camera_yaw_compensation_deg'],
            self.param_snapshot['v_limit'],
            self.param_snapshot['vz_limit'],
            self.param_snapshot['yaw_rate_limit'],
            self.param_snapshot['velocity_deadband'],
            self.param_snapshot['control_rate_hz'],
            self.param_snapshot['pose_timeout_sec'],
            self.param_snapshot['require_offboard'],
            self.param_snapshot['enable_z_hold'],
            self.param_snapshot['camera_profile'],
            self.param_snapshot['image_width'],
            self.param_snapshot['image_height'],
            self.param_snapshot['publish_annotated_image'],
            raw_tvec_x,
            raw_tvec_y,
            raw_tvec_z,
            raw_rvec_x,
            raw_rvec_y,
            raw_rvec_z,
            local_roll,
            local_pitch,
        ])

        self.metric_rows.append({
            't': ros_time_sec,
            'mode': mode,
            'aruco_fresh': aruco_fresh,
            'setpoint_fresh': sp_fresh,
            'aruco_x': aruco_x,
            'aruco_y': aruco_y,
            'aruco_z': aruco_z,
            'eyaw': eyaw,
            'sp_vx': sp_vx,
            'sp_vy': sp_vy,
            'sp_yaw_rate': sp_yaw_rate,
            'latency_sec': image_to_setpoint_latency_sec,
            'image_header_to_source_sec': image_header_to_source_sec,
            'image_source_to_received_sec': image_source_to_received_sec,
            'image_source_to_tvec_cb_sec': image_source_to_tvec_cb_sec,
            'tvec_queue_sec': tvec_queue_sec,
            'tvec_cv_bridge_sec': tvec_cv_bridge_sec,
            'tvec_detect_sec': tvec_detect_sec,
            'tvec_pose_estimate_sec': tvec_pose_estimate_sec,
            'tvec_publish_gap_sec': tvec_publish_gap_sec,
            'tvec_total_compute_sec': tvec_total_compute_sec,
            'tf_queue_sec': tf_queue_sec,
            'tf_compute_sec': tf_compute_sec,
            'ctrl_queue_sec': ctrl_queue_sec,
            'ctrl_compute_sec': ctrl_compute_sec,
            'total_latency_sec': total_latency_sec,
        })

        if self._age_sec(self.last_flush_time, now_time) >= self.flush_interval_sec:
            self.csv_file.flush()
            self.last_flush_time = now_time

    @staticmethod
    def _quantile(vals, p):
        """线性插值分位数计算."""
        if not vals:
            return float('nan')
        sorted_vals = sorted(vals)
        idx = (len(sorted_vals) - 1) * p
        lo = int(math.floor(idx))
        hi = int(math.ceil(idx))
        if lo == hi:
            return sorted_vals[lo]
        return sorted_vals[lo] * (hi - idx) + sorted_vals[hi] * (idx - lo)

    @staticmethod
    def _rmse(vals):
        """均方根误差."""
        if not vals:
            return float('nan')
        return math.sqrt(sum(v * v for v in vals) / len(vals))

    @staticmethod
    def _std(vals):
        """总体标准差，用于观察命令抖动强度."""
        if len(vals) < 2:
            return 0.0
        mean_val = sum(vals) / len(vals)
        return math.sqrt(
            sum((v - mean_val) * (v - mean_val) for v in vals) / len(vals)
        )

    def _compute_metrics(self):
        """计算 OFFBOARD 窗口下的调参摘要指标."""
        off_rows = [row for row in self.metric_rows if row['mode'] == 'OFFBOARD']
        eval_rows = [row for row in off_rows if row['aruco_fresh'] == 1]

        def calc_stats(rows, key):
            values = [
                row[key]
                for row in rows
                if math.isfinite(row.get(key, float('nan')))
            ]
            return {
                f'{key}_mean': (
                    sum(values) / len(values) if values else float('nan')
                ),
                f'{key}_p50': self._quantile(values, 0.50),
                f'{key}_p95': self._quantile(values, 0.95),
                f'{key}_max': max(values) if values else float('nan'),
            }

        result = {
            'eval_rows': len(eval_rows),
            'offboard_rows': len(off_rows),
            'z_eval_rows': len(eval_rows),
            'latency_eval_rows': 0,
            'status': 'ok' if len(eval_rows) >= 1 else 'insufficient_data',
        }

        if not off_rows:
            result.update({
                'fresh_ratio': float('nan'),
                'max_stale_s': float('nan'),
                'max_abs_yaw_rate': float('nan'),
                'latency_mean_sec': float('nan'),
                'latency_p50_sec': float('nan'),
                'latency_p95_sec': float('nan'),
                'latency_max_sec': float('nan'),
                'rmse_yaw': float('nan'),
                'p95_abs_yaw': float('nan'),
            })
            for metric_name in (
                'image_header_to_source_sec',
                'image_source_to_received_sec',
                'image_source_to_tvec_cb_sec',
                'tvec_queue_sec',
                'tvec_total_compute_sec',
                'tvec_cv_bridge_sec',
                'tvec_detect_sec',
                'tvec_pose_estimate_sec',
                'tf_queue_sec',
                'tf_compute_sec',
                'ctrl_queue_sec',
                'ctrl_compute_sec',
                'total_latency_sec',
            ):
                result.update(calc_stats([], metric_name))
            return result

        fresh_ratio = len(eval_rows) / len(off_rows)

        stale_run = 0
        max_stale_run = 0
        dt_vals = []
        for idx in range(1, len(off_rows)):
            dt_vals.append(max(0.0, off_rows[idx]['t'] - off_rows[idx - 1]['t']))
        mean_dt = sum(dt_vals) / len(dt_vals) if dt_vals else 0.0

        for row in off_rows:
            if row['aruco_fresh'] == 0:
                stale_run += 1
                max_stale_run = max(max_stale_run, stale_run)
            else:
                stale_run = 0
        max_stale_s = max_stale_run * mean_dt

        yaw_rate_vals = [
            abs(row['sp_yaw_rate'])
            for row in off_rows
            if math.isfinite(row['sp_yaw_rate'])
        ]

        result.update({
            'fresh_ratio': fresh_ratio,
            'max_stale_s': max_stale_s,
            'max_abs_yaw_rate': (
                max(yaw_rate_vals) if yaw_rate_vals else float('nan')
            ),
        })

        latency_vals = [
            row['latency_sec']
            for row in off_rows
            if row['aruco_fresh'] == 1
            and row['setpoint_fresh'] == 1
            and math.isfinite(row['latency_sec'])
            and row['latency_sec'] >= 0.0
        ]
        result.update({
            'latency_eval_rows': len(latency_vals),
            'latency_mean_sec': (
                sum(latency_vals) / len(latency_vals)
                if latency_vals else float('nan')
            ),
            'latency_p50_sec': self._quantile(latency_vals, 0.50),
            'latency_p95_sec': self._quantile(latency_vals, 0.95),
            'latency_max_sec': (
                max(latency_vals) if latency_vals else float('nan')
            ),
        })
        for metric_name in (
            'image_header_to_source_sec',
            'image_source_to_received_sec',
            'image_source_to_tvec_cb_sec',
            'tvec_queue_sec',
            'tvec_total_compute_sec',
            'tvec_cv_bridge_sec',
            'tvec_detect_sec',
            'tvec_pose_estimate_sec',
            'tf_queue_sec',
            'tf_compute_sec',
            'ctrl_queue_sec',
            'ctrl_compute_sec',
            'total_latency_sec',
        ):
            result.update(calc_stats(off_rows, metric_name))

        if not eval_rows:
            result.update({
                'rmse_yaw': float('nan'),
                'p95_abs_yaw': float('nan'),
            })
            return result

        target_x = self.param_snapshot['target_x']
        target_y = self.param_snapshot['target_y']
        target_z = self.param_snapshot['target_z']

        ex_marker = [target_x - row['aruco_x'] for row in eval_rows]
        ey_marker = [row['aruco_y'] - target_y for row in eval_rows]
        ez_vals = [target_z - row['aruco_z'] for row in eval_rows]
        eyaw_vals = [
            row['eyaw'] for row in eval_rows if math.isfinite(row['eyaw'])
        ]

        abs_ex = [abs(val) for val in ex_marker]
        abs_ey = [abs(val) for val in ey_marker]
        abs_ez = [abs(val) for val in ez_vals]
        abs_eyaw = [abs(val) for val in eyaw_vals]
        exy = [
            math.hypot(ex_marker[idx], ey_marker[idx])
            for idx in range(len(ex_marker))
        ]

        sp_vx = [row['sp_vx'] for row in eval_rows if math.isfinite(row['sp_vx'])]
        sp_vy = [row['sp_vy'] for row in eval_rows if math.isfinite(row['sp_vy'])]
        d_vx = [sp_vx[idx] - sp_vx[idx - 1] for idx in range(1, len(sp_vx))]
        d_vy = [sp_vy[idx] - sp_vy[idx - 1] for idx in range(1, len(sp_vy))]

        cmd_jitter_x = self._std(d_vx)
        cmd_jitter_y = self._std(d_vy)
        cmd_jitter_xy = math.hypot(cmd_jitter_x, cmd_jitter_y)

        result.update({
            'rmse_x': self._rmse(ex_marker),
            'rmse_y': self._rmse(ey_marker),
            'rmse_z': self._rmse(ez_vals),
            'rmse_xy': self._rmse(exy),
            'rmse_yaw': self._rmse(eyaw_vals),
            'p95_abs_x': self._quantile(abs_ex, 0.95),
            'p95_abs_y': self._quantile(abs_ey, 0.95),
            'p95_abs_z': self._quantile(abs_ez, 0.95),
            'p95_xy': self._quantile(exy, 0.95),
            'p95_abs_yaw': self._quantile(abs_eyaw, 0.95),
            'cmd_jitter_x': cmd_jitter_x,
            'cmd_jitter_y': cmd_jitter_y,
            'cmd_jitter_xy': cmd_jitter_xy,
        })
        return result

    def _repair_summary_csv_if_needed(self, summary_path: str, fieldnames):
        """统一 summary 表头与行结束符，兼容历史列集并补齐新增延迟字段."""
        if not os.path.exists(summary_path):
            return
        try:
            with open(summary_path, 'r', encoding='utf-8', newline='') as summary_file:
                raw = summary_file.read()
            if raw == '':
                return

            normalized = raw.replace('\r\n', '\n').replace('\r', '\n')
            if normalized.strip() == '':
                return

            reader = csv.DictReader(io.StringIO(normalized))
            existing_fieldnames = reader.fieldnames or []
            if not existing_fieldnames:
                return

            cleaned_rows = []
            for row in reader:
                if row is None:
                    continue
                # 过滤历史文件里重复混入的数据表头行。
                if all(
                    str(row.get(name, '')).strip() == str(name)
                    for name in existing_fieldnames
                ):
                    continue
                cleaned_rows.append(row)

            if normalized != raw or existing_fieldnames != fieldnames:
                with open(summary_path, 'w', encoding='utf-8', newline='') as summary_file:
                    writer = csv.DictWriter(
                        summary_file,
                        fieldnames=fieldnames,
                        lineterminator='\n',
                    )
                    writer.writeheader()
                    for row in cleaned_rows:
                        writer.writerow({
                            name: row.get(name, '')
                            for name in fieldnames
                        })
                self.get_logger().info(f'已修复 summary 格式: {summary_path}')
        except Exception as exc:
            self.get_logger().warn(f'summary 格式修复跳过: {exc}')

    def _append_summary_csv(self, metrics):
        """把本次运行摘要追加到 summary CSV."""
        row = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'run_csv': self.csv_path,
            'file_prefix': self.file_prefix,
            'status': metrics.get('status', 'unknown'),
            'offboard_rows': metrics.get('offboard_rows', 0),
            'eval_rows': metrics.get('eval_rows', 0),
            'z_eval_rows': metrics.get('z_eval_rows', 0),
            'latency_eval_rows': metrics.get('latency_eval_rows', 0),
            'fresh_ratio': metrics.get('fresh_ratio', float('nan')),
            'max_stale_s': metrics.get('max_stale_s', float('nan')),
            'latency_mean_sec': metrics.get('latency_mean_sec', float('nan')),
            'latency_p50_sec': metrics.get('latency_p50_sec', float('nan')),
            'latency_p95_sec': metrics.get('latency_p95_sec', float('nan')),
            'latency_max_sec': metrics.get('latency_max_sec', float('nan')),
            'image_header_to_source_sec_mean': metrics.get('image_header_to_source_sec_mean', float('nan')),
            'image_header_to_source_sec_p50': metrics.get('image_header_to_source_sec_p50', float('nan')),
            'image_header_to_source_sec_p95': metrics.get('image_header_to_source_sec_p95', float('nan')),
            'image_header_to_source_sec_max': metrics.get('image_header_to_source_sec_max', float('nan')),
            'image_source_to_received_sec_mean': metrics.get('image_source_to_received_sec_mean', float('nan')),
            'image_source_to_received_sec_p50': metrics.get('image_source_to_received_sec_p50', float('nan')),
            'image_source_to_received_sec_p95': metrics.get('image_source_to_received_sec_p95', float('nan')),
            'image_source_to_received_sec_max': metrics.get('image_source_to_received_sec_max', float('nan')),
            'image_source_to_tvec_cb_sec_mean': metrics.get('image_source_to_tvec_cb_sec_mean', float('nan')),
            'image_source_to_tvec_cb_sec_p50': metrics.get('image_source_to_tvec_cb_sec_p50', float('nan')),
            'image_source_to_tvec_cb_sec_p95': metrics.get('image_source_to_tvec_cb_sec_p95', float('nan')),
            'image_source_to_tvec_cb_sec_max': metrics.get('image_source_to_tvec_cb_sec_max', float('nan')),
            'tvec_queue_sec_mean': metrics.get('tvec_queue_sec_mean', float('nan')),
            'tvec_queue_sec_p50': metrics.get('tvec_queue_sec_p50', float('nan')),
            'tvec_queue_sec_p95': metrics.get('tvec_queue_sec_p95', float('nan')),
            'tvec_queue_sec_max': metrics.get('tvec_queue_sec_max', float('nan')),
            'tvec_total_compute_sec_mean': metrics.get('tvec_total_compute_sec_mean', float('nan')),
            'tvec_total_compute_sec_p50': metrics.get('tvec_total_compute_sec_p50', float('nan')),
            'tvec_total_compute_sec_p95': metrics.get('tvec_total_compute_sec_p95', float('nan')),
            'tvec_total_compute_sec_max': metrics.get('tvec_total_compute_sec_max', float('nan')),
            'tvec_cv_bridge_sec_mean': metrics.get('tvec_cv_bridge_sec_mean', float('nan')),
            'tvec_cv_bridge_sec_p50': metrics.get('tvec_cv_bridge_sec_p50', float('nan')),
            'tvec_cv_bridge_sec_p95': metrics.get('tvec_cv_bridge_sec_p95', float('nan')),
            'tvec_cv_bridge_sec_max': metrics.get('tvec_cv_bridge_sec_max', float('nan')),
            'tvec_detect_sec_mean': metrics.get('tvec_detect_sec_mean', float('nan')),
            'tvec_detect_sec_p50': metrics.get('tvec_detect_sec_p50', float('nan')),
            'tvec_detect_sec_p95': metrics.get('tvec_detect_sec_p95', float('nan')),
            'tvec_detect_sec_max': metrics.get('tvec_detect_sec_max', float('nan')),
            'tvec_pose_estimate_sec_mean': metrics.get('tvec_pose_estimate_sec_mean', float('nan')),
            'tvec_pose_estimate_sec_p50': metrics.get('tvec_pose_estimate_sec_p50', float('nan')),
            'tvec_pose_estimate_sec_p95': metrics.get('tvec_pose_estimate_sec_p95', float('nan')),
            'tvec_pose_estimate_sec_max': metrics.get('tvec_pose_estimate_sec_max', float('nan')),
            'tf_queue_sec_mean': metrics.get('tf_queue_sec_mean', float('nan')),
            'tf_queue_sec_p50': metrics.get('tf_queue_sec_p50', float('nan')),
            'tf_queue_sec_p95': metrics.get('tf_queue_sec_p95', float('nan')),
            'tf_queue_sec_max': metrics.get('tf_queue_sec_max', float('nan')),
            'tf_compute_sec_mean': metrics.get('tf_compute_sec_mean', float('nan')),
            'tf_compute_sec_p50': metrics.get('tf_compute_sec_p50', float('nan')),
            'tf_compute_sec_p95': metrics.get('tf_compute_sec_p95', float('nan')),
            'tf_compute_sec_max': metrics.get('tf_compute_sec_max', float('nan')),
            'ctrl_queue_sec_mean': metrics.get('ctrl_queue_sec_mean', float('nan')),
            'ctrl_queue_sec_p50': metrics.get('ctrl_queue_sec_p50', float('nan')),
            'ctrl_queue_sec_p95': metrics.get('ctrl_queue_sec_p95', float('nan')),
            'ctrl_queue_sec_max': metrics.get('ctrl_queue_sec_max', float('nan')),
            'ctrl_compute_sec_mean': metrics.get('ctrl_compute_sec_mean', float('nan')),
            'ctrl_compute_sec_p50': metrics.get('ctrl_compute_sec_p50', float('nan')),
            'ctrl_compute_sec_p95': metrics.get('ctrl_compute_sec_p95', float('nan')),
            'ctrl_compute_sec_max': metrics.get('ctrl_compute_sec_max', float('nan')),
            'total_latency_sec_mean': metrics.get('total_latency_sec_mean', float('nan')),
            'total_latency_sec_p50': metrics.get('total_latency_sec_p50', float('nan')),
            'total_latency_sec_p95': metrics.get('total_latency_sec_p95', float('nan')),
            'total_latency_sec_max': metrics.get('total_latency_sec_max', float('nan')),
            'rmse_x': metrics.get('rmse_x', float('nan')),
            'rmse_y': metrics.get('rmse_y', float('nan')),
            'rmse_z': metrics.get('rmse_z', float('nan')),
            'rmse_xy': metrics.get('rmse_xy', float('nan')),
            'rmse_yaw': metrics.get('rmse_yaw', float('nan')),
            'p95_abs_x': metrics.get('p95_abs_x', float('nan')),
            'p95_abs_y': metrics.get('p95_abs_y', float('nan')),
            'p95_abs_z': metrics.get('p95_abs_z', float('nan')),
            'p95_xy': metrics.get('p95_xy', float('nan')),
            'p95_abs_yaw': metrics.get('p95_abs_yaw', float('nan')),
            'cmd_jitter_x': metrics.get('cmd_jitter_x', float('nan')),
            'cmd_jitter_y': metrics.get('cmd_jitter_y', float('nan')),
            'cmd_jitter_xy': metrics.get('cmd_jitter_xy', float('nan')),
            'max_abs_yaw_rate': metrics.get('max_abs_yaw_rate', float('nan')),
            **self.param_snapshot,
        }

        fieldnames = list(row.keys())
        os.makedirs(os.path.dirname(self.summary_csv_path), exist_ok=True)
        self._repair_summary_csv_if_needed(self.summary_csv_path, fieldnames)
        file_exists = os.path.exists(self.summary_csv_path)
        with open(self.summary_csv_path, 'a', newline='', encoding='utf-8') as summary_file:
            writer = csv.DictWriter(
                summary_file,
                fieldnames=fieldnames,
                lineterminator='\n',
            )
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)

    def _print_summary(self, metrics):
        """在终端打印本次调参摘要."""
        self.get_logger().info(
            '===== dynamic_tracking_v2 调参指标汇总（OFFBOARD） ====='
        )
        self.get_logger().info(
            f"status={metrics.get('status')} | "
            f"offboard_rows={metrics.get('offboard_rows')} | "
            f"eval_rows={metrics.get('eval_rows')} | "
            f"z_eval_rows={metrics.get('z_eval_rows')} | "
            f"latency_eval_rows={metrics.get('latency_eval_rows')} | "
            f"fresh_ratio={metrics.get('fresh_ratio', float('nan')):.4f} | "
            f"max_stale_s={metrics.get('max_stale_s', float('nan')):.3f} | "
            f"max_abs_yaw_rate="
            f"{metrics.get('max_abs_yaw_rate', float('nan')):.4f}"
        )
        if metrics.get('latency_eval_rows', 0) >= 1:
            self.get_logger().info(
                f"Latency: mean={metrics['latency_mean_sec']:.4f}s, "
                f"p50={metrics['latency_p50_sec']:.4f}s, "
                f"p95={metrics['latency_p95_sec']:.4f}s, "
                f"max={metrics['latency_max_sec']:.4f}s"
            )
            self.get_logger().info(
                'PipelineLatency: '
                f"header_to_source(mean/p95)="
                f"{metrics.get('image_header_to_source_sec_mean', float('nan')):.4f}/"
                f"{metrics.get('image_header_to_source_sec_p95', float('nan')):.4f}s, "
                f"source_to_tvec_cb(mean/p95)="
                f"{metrics.get('image_source_to_tvec_cb_sec_mean', float('nan')):.4f}/"
                f"{metrics.get('image_source_to_tvec_cb_sec_p95', float('nan')):.4f}s, "
                f"tvec_queue(mean/p95)="
                f"{metrics.get('tvec_queue_sec_mean', float('nan')):.4f}/"
                f"{metrics.get('tvec_queue_sec_p95', float('nan')):.4f}s, "
                f"tvec_total_compute(mean/p95)="
                f"{metrics.get('tvec_total_compute_sec_mean', float('nan')):.4f}/"
                f"{metrics.get('tvec_total_compute_sec_p95', float('nan')):.4f}s, "
                f"tf_queue(mean/p95)="
                f"{metrics.get('tf_queue_sec_mean', float('nan')):.4f}/"
                f"{metrics.get('tf_queue_sec_p95', float('nan')):.4f}s, "
                f"tf_compute(mean/p95)="
                f"{metrics.get('tf_compute_sec_mean', float('nan')):.4f}/"
                f"{metrics.get('tf_compute_sec_p95', float('nan')):.4f}s, "
                f"ctrl_queue(mean/p95)="
                f"{metrics.get('ctrl_queue_sec_mean', float('nan')):.4f}/"
                f"{metrics.get('ctrl_queue_sec_p95', float('nan')):.4f}s, "
                f"ctrl_compute(mean/p95)="
                f"{metrics.get('ctrl_compute_sec_mean', float('nan')):.4f}/"
                f"{metrics.get('ctrl_compute_sec_p95', float('nan')):.4f}s"
            )
        if metrics.get('status') == 'ok':
            self.get_logger().info(
                f"RMSE: x={metrics['rmse_x']:.4f}, "
                f"y={metrics['rmse_y']:.4f}, "
                f"z={metrics['rmse_z']:.4f}, "
                f"xy={metrics['rmse_xy']:.4f}, "
                f"yaw={metrics['rmse_yaw']:.4f}"
            )
            self.get_logger().info(
                f"P95: |x|={metrics['p95_abs_x']:.4f}, "
                f"|y|={metrics['p95_abs_y']:.4f}, "
                f"|z|={metrics['p95_abs_z']:.4f}, "
                f"xy={metrics['p95_xy']:.4f}, "
                f"|yaw|={metrics['p95_abs_yaw']:.4f}"
            )
            self.get_logger().info(
                f"CmdJitter: x={metrics['cmd_jitter_x']:.5f}, "
                f"y={metrics['cmd_jitter_y']:.5f}, "
                f"xy={metrics['cmd_jitter_xy']:.5f}"
            )

    def destroy_node(self):
        """退出前落盘、汇总并追加 summary."""
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except BaseException:
            pass

        metrics = self._compute_metrics()
        self._print_summary(metrics)
        try:
            self._append_summary_csv(metrics)
            self.get_logger().info(f'汇总 CSV 已追加: {self.summary_csv_path}')
        except Exception as exc:
            self.get_logger().error(f'写入汇总 CSV 失败: {exc}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamicTrackingV2CsvLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except BaseException:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except BaseException:
                pass


if __name__ == '__main__':
    main()
