#!/usr/bin/env python3

import csv
import json
import math
import os
from datetime import datetime
from typing import Optional

import rclpy
from debug_interface.msg import ArucoBasePose
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, String


class LandV2CsvLoggerNode(Node):
    """记录 land_v2 运行过程到 CSV，并在退出时写入 summary。"""

    def __init__(self):
        super().__init__('land_v2_csv_logger_node')

        # ===== 输入话题 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('extended_state_topic', '/mavros/extended_state')
        self.declare_parameter('distance_sensor_topic', '/mavros/hrlv_ez4_pub')
        self.declare_parameter('rel_alt_topic', '/mavros/global_position/rel_alt')
        self.declare_parameter('vel_local_topic', '/mavros/local_position/velocity_local')
        self.declare_parameter('setpoint_raw_topic', '/mavros/setpoint_raw/local')
        self.declare_parameter('status_topic', '/land_v2/status')

        # ===== 输出路径 =====
        self.declare_parameter(
            'output_dir',
            '/home/zjh/project/rasip_pi_ws/log/tracking_csv',
        )
        self.declare_parameter('file_prefix', 'land_v2')
        self.declare_parameter(
            'summary_csv_path',
            '/home/zjh/project/rasip_pi_ws/log/tracking_csv/land_v2_summary.csv',
        )

        # ===== 采样参数 =====
        self.declare_parameter('sample_rate_hz', 30.0)
        self.declare_parameter('stale_timeout_sec', 0.5)
        self.declare_parameter('flush_interval_sec', 1.0)

        # ===== 关键控制参数快照 =====
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
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.06)
        self.declare_parameter('kp_yaw', 0.4)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.03)
        self.declare_parameter('camera_yaw_compensation_deg', 0.0)
        self.declare_parameter('invert_yaw_rate_output', True)
        self.declare_parameter('align_mode', 'sliding_window')
        self.declare_parameter('align_window_sec', 1.0)
        self.declare_parameter('align_window_min_ratio', 0.75)
        self.declare_parameter('align_window_min_samples', 12)
        self.declare_parameter('align_xy_tolerance_m', 0.18)
        self.declare_parameter('align_xy_mean_tolerance_m', 0.12)
        self.declare_parameter('align_z_tolerance_m', 0.15)
        self.declare_parameter('align_z_mean_tolerance_m', 0.10)
        self.declare_parameter('align_yaw_tolerance_rad', 0.12)
        self.declare_parameter('align_yaw_mean_tolerance_rad', 0.08)
        self.declare_parameter('descent_speed_mps', 0.2)
        self.declare_parameter('marker_loss_hold_sec', 0.4)
        self.declare_parameter('touchdown_range_threshold_m', 0.10)
        self.declare_parameter('touchdown_rel_alt_threshold_m', 0.15)
        self.declare_parameter('land_vz_abs_max_mps', 0.20)
        self.declare_parameter('land_vxy_abs_max_mps', 0.25)
        self.declare_parameter('land_detect_hold_sec', 1.0)
        self.declare_parameter('heuristic_disarm_hold_sec', 3.0)
        self.declare_parameter('min_throttle_descent_speed_mps', 0.35)
        self.declare_parameter('min_throttle_disarm_duration_sec', 5.0)
        self.declare_parameter('disarm_retry_interval_sec', 1.0)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('distance_sensor_timeout_sec', 0.5)
        self.declare_parameter('rel_alt_timeout_sec', 0.5)
        self.declare_parameter('local_velocity_timeout_sec', 0.5)
        self.declare_parameter('v_limit', 0.8)
        self.declare_parameter('vxy_limit', 0.8)
        self.declare_parameter('vx_limit', float('nan'))
        self.declare_parameter('vy_limit', float('nan'))
        self.declare_parameter('vz_limit', 0.2)
        self.declare_parameter('yaw_rate_limit', 0.4)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('enable_z_yaw_smoothing', True)
        self.declare_parameter('yaw_lpf_tau_sec', 0.35)
        self.declare_parameter('yaw_error_deadband', 0.04)
        self.declare_parameter('yaw_rate_slew_rate_limit', 0.35)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.extended_state_topic = self.get_parameter('extended_state_topic').value
        self.distance_sensor_topic = self.get_parameter('distance_sensor_topic').value
        self.rel_alt_topic = self.get_parameter('rel_alt_topic').value
        self.vel_local_topic = self.get_parameter('vel_local_topic').value
        self.setpoint_raw_topic = self.get_parameter('setpoint_raw_topic').value
        self.status_topic = self.get_parameter('status_topic').value

        self.output_dir = self.get_parameter('output_dir').value
        self.file_prefix = self.get_parameter('file_prefix').value
        self.summary_csv_path = self.get_parameter('summary_csv_path').value
        self.sample_rate_hz = float(self.get_parameter('sample_rate_hz').value)
        self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
        self.flush_interval_sec = float(self.get_parameter('flush_interval_sec').value)

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
            'invert_yaw_rate_output': int(
                self.parameter_as_bool(
                    self.get_parameter('invert_yaw_rate_output').value
                )
            ),
            'align_mode': self.get_parameter('align_mode').value,
            'align_window_sec': float(self.get_parameter('align_window_sec').value),
            'align_window_min_ratio': float(
                self.get_parameter('align_window_min_ratio').value
            ),
            'align_window_min_samples': int(
                self.get_parameter('align_window_min_samples').value
            ),
            'align_xy_tolerance_m': float(
                self.get_parameter('align_xy_tolerance_m').value
            ),
            'align_xy_mean_tolerance_m': float(
                self.get_parameter('align_xy_mean_tolerance_m').value
            ),
            'align_z_tolerance_m': float(
                self.get_parameter('align_z_tolerance_m').value
            ),
            'align_z_mean_tolerance_m': float(
                self.get_parameter('align_z_mean_tolerance_m').value
            ),
            'align_yaw_tolerance_rad': float(
                self.get_parameter('align_yaw_tolerance_rad').value
            ),
            'align_yaw_mean_tolerance_rad': float(
                self.get_parameter('align_yaw_mean_tolerance_rad').value
            ),
            'descent_speed_mps': float(self.get_parameter('descent_speed_mps').value),
            'marker_loss_hold_sec': float(
                self.get_parameter('marker_loss_hold_sec').value
            ),
            'touchdown_range_threshold_m': float(
                self.get_parameter('touchdown_range_threshold_m').value
            ),
            'touchdown_rel_alt_threshold_m': float(
                self.get_parameter('touchdown_rel_alt_threshold_m').value
            ),
            'land_vz_abs_max_mps': float(
                self.get_parameter('land_vz_abs_max_mps').value
            ),
            'land_vxy_abs_max_mps': float(
                self.get_parameter('land_vxy_abs_max_mps').value
            ),
            'land_detect_hold_sec': float(
                self.get_parameter('land_detect_hold_sec').value
            ),
            'heuristic_disarm_hold_sec': float(
                self.get_parameter('heuristic_disarm_hold_sec').value
            ),
            'min_throttle_descent_speed_mps': float(
                self.get_parameter('min_throttle_descent_speed_mps').value
            ),
            'min_throttle_disarm_duration_sec': float(
                self.get_parameter('min_throttle_disarm_duration_sec').value
            ),
            'disarm_retry_interval_sec': float(
                self.get_parameter('disarm_retry_interval_sec').value
            ),
            'control_rate_hz': float(self.get_parameter('control_rate_hz').value),
            'pose_timeout_sec': float(self.get_parameter('pose_timeout_sec').value),
            'distance_sensor_timeout_sec': float(
                self.get_parameter('distance_sensor_timeout_sec').value
            ),
            'rel_alt_timeout_sec': float(
                self.get_parameter('rel_alt_timeout_sec').value
            ),
            'local_velocity_timeout_sec': float(
                self.get_parameter('local_velocity_timeout_sec').value
            ),
            'v_limit': float(self.get_parameter('v_limit').value),
            'vxy_limit': float(self.get_parameter('vxy_limit').value),
            'vx_limit': float(self.get_parameter('vx_limit').value),
            'vy_limit': float(self.get_parameter('vy_limit').value),
            'vz_limit': float(self.get_parameter('vz_limit').value),
            'yaw_rate_limit': float(self.get_parameter('yaw_rate_limit').value),
            'velocity_deadband': float(
                self.get_parameter('velocity_deadband').value
            ),
            'require_offboard': int(
                self.parameter_as_bool(self.get_parameter('require_offboard').value)
            ),
            'enable_z_yaw_smoothing': int(
                self.parameter_as_bool(
                    self.get_parameter('enable_z_yaw_smoothing').value
                )
            ),
            'yaw_lpf_tau_sec': float(self.get_parameter('yaw_lpf_tau_sec').value),
            'yaw_error_deadband': float(
                self.get_parameter('yaw_error_deadband').value
            ),
            'yaw_rate_slew_rate_limit': float(
                self.get_parameter('yaw_rate_slew_rate_limit').value
            ),
        }
        self.camera_yaw_compensation_rad = math.radians(
            self.param_snapshot['camera_yaw_compensation_deg']
        )

        # ===== 最新数据缓存 =====
        self.state_msg: Optional[State] = None
        self.extended_state_msg: Optional[ExtendedState] = None
        self.pose_msg: Optional[ArucoBasePose] = None
        self.distance_sensor_msg: Optional[Range] = None
        self.rel_alt_msg: Optional[Float64] = None
        self.vel_local_msg: Optional[TwistStamped] = None
        self.setpoint_msg: Optional[PositionTarget] = None
        self.status_msg: Optional[dict] = None

        self.pose_rx_time = None
        self.distance_sensor_rx_time = None
        self.rel_alt_rx_time = None
        self.vel_local_rx_time = None
        self.setpoint_rx_time = None
        self.status_rx_time = None
        self.last_flush_time = self.get_clock().now()

        self.metric_rows = []

        os.makedirs(self.output_dir, exist_ok=True)
        now_text = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.output_dir, f'{self.file_prefix}_{now_text}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file, lineterminator='\n')
        self.csv_writer.writerow([
            'ros_time_sec',
            'mode',
            'armed',
            'connected',
            'phase',
            'align_mode',
            'align_ok',
            'align_window_ratio',
            'align_window_sample_count',
            'align_window_mean_yaw',
            'aruco_fresh',
            'aruco_age_sec',
            'aruco_x',
            'aruco_y',
            'aruco_z',
            'aruco_yaw_rad',
            'yaw_rel_raw_rad',
            'yaw_rel_corrected_rad',
            'eyaw_rad',
            'status_eyaw_rad',
            'status_eyaw_ctrl_rad',
            'status_yaw_rate',
            'distance_sensor_fresh',
            'distance_sensor_age_sec',
            'distance_sensor_valid',
            'distance_sensor_z',
            'rel_alt_fresh',
            'rel_alt_age_sec',
            'rel_alt',
            'local_velocity_fresh',
            'local_velocity_age_sec',
            'local_vx',
            'local_vy',
            'local_vz',
            'local_vxy',
            'ex_marker',
            'ey_marker',
            'ex_body',
            'ey_body',
            'setpoint_fresh',
            'setpoint_age_sec',
            'sp_frame',
            'sp_type_mask',
            'sp_vx',
            'sp_vy',
            'sp_vz',
            'sp_yaw_rate',
            'tracking_active',
            'range_only_active',
            'range_only_hold_active',
            'marker_loss_active',
            'marker_lost_total_count',
            'last_marker_loss_duration_sec',
            'touchdown_candidate_active',
            'touchdown_candidate_ready',
            'landed_by_extended_state',
            'landed_by_heuristic',
            'touchdown_height_source',
            'touchdown_height_value',
            'disarm_requested',
            'status_fresh',
            'status_age_sec',
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
            'invert_yaw_rate_output',
            'align_window_sec',
            'align_window_min_ratio',
            'align_window_min_samples',
            'align_xy_tolerance_m',
            'align_xy_mean_tolerance_m',
            'align_z_tolerance_m',
            'align_z_mean_tolerance_m',
            'align_yaw_tolerance_rad',
            'align_yaw_mean_tolerance_rad',
            'descent_speed_mps',
            'marker_loss_hold_sec',
            'touchdown_range_threshold_m',
            'touchdown_rel_alt_threshold_m',
            'land_vz_abs_max_mps',
            'land_vxy_abs_max_mps',
            'land_detect_hold_sec',
            'heuristic_disarm_hold_sec',
            'min_throttle_descent_speed_mps',
            'min_throttle_disarm_duration_sec',
            'disarm_retry_interval_sec',
            'control_rate_hz',
            'pose_timeout_sec',
            'distance_sensor_timeout_sec',
            'rel_alt_timeout_sec',
            'local_velocity_timeout_sec',
            'v_limit',
            'vxy_limit',
            'vx_limit',
            'vy_limit',
            'vz_limit',
            'yaw_rate_limit',
            'velocity_deadband',
            'require_offboard',
            'enable_z_yaw_smoothing',
            'yaw_lpf_tau_sec',
            'yaw_error_deadband',
            'yaw_rate_slew_rate_limit',
        ])
        self.csv_file.flush()

        mavros_best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(State, self.state_topic, self._on_state, 10)
        self.create_subscription(
            ExtendedState,
            self.extended_state_topic,
            self._on_extended_state,
            mavros_best_effort_qos,
        )
        self.create_subscription(ArucoBasePose, self.pose_topic, self._on_pose, 10)
        self.create_subscription(
            Range,
            self.distance_sensor_topic,
            self._on_distance_sensor,
            mavros_best_effort_qos,
        )
        self.create_subscription(
            Float64,
            self.rel_alt_topic,
            self._on_rel_alt,
            mavros_best_effort_qos,
        )
        self.create_subscription(
            TwistStamped,
            self.vel_local_topic,
            self._on_vel_local,
            mavros_best_effort_qos,
        )
        self.create_subscription(
            PositionTarget,
            self.setpoint_raw_topic,
            self._on_setpoint,
            10,
        )
        self.create_subscription(
            String,
            self.status_topic,
            self._on_status,
            10,
        )

        timer_period = 1.0 / max(self.sample_rate_hz, 1.0)
        self.create_timer(timer_period, self._write_row)

        self.get_logger().info(f'land_v2 CSV 记录已启动，输出文件: {self.csv_path}')
        self.get_logger().info(
            '参数快照 | '
            f'align_mode={self.param_snapshot["align_mode"]} | '
            f'distance_sensor_topic={self.distance_sensor_topic} | '
            f'status_topic={self.status_topic}'
        )

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        """将角度归一化到 [-pi, pi]。"""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def parameter_as_bool(value) -> bool:
        """兼容 launch 字符串和原生 bool，避免 'false' 被 bool() 判真。"""
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    @staticmethod
    def rotate_marker_error_to_body(
        ex_marker: float,
        ey_marker: float,
        yaw_rel_corrected: float,
    ) -> tuple[float, float]:
        """与控制节点保持一致：e_body = R(-yaw_rel_corrected) * e_marker。"""
        cos_yaw = math.cos(yaw_rel_corrected)
        sin_yaw = math.sin(yaw_rel_corrected)
        ex_body = cos_yaw * ex_marker + sin_yaw * ey_marker
        ey_body = -sin_yaw * ex_marker + cos_yaw * ey_marker
        return ex_body, ey_body

    @staticmethod
    def _age_sec(stamp_time, now_time) -> float:
        """计算样本年龄；缺失时返回极大值。"""
        if stamp_time is None:
            return 1e9
        return (now_time - stamp_time).nanoseconds / 1e9

    def _is_fresh(self, stamp_time, now_time, timeout_sec: float) -> bool:
        """判断数据是否位于新鲜时间窗内。"""
        return self._age_sec(stamp_time, now_time) <= timeout_sec

    @staticmethod
    def _range_is_valid(msg: Range) -> bool:
        """检查测距值是否有限且位于量程范围内。"""
        range_value = float(msg.range)
        min_range = float(msg.min_range)
        max_range = float(msg.max_range)
        if not math.isfinite(range_value):
            return False
        lower_ok = (not math.isfinite(min_range)) or (range_value >= min_range)
        upper_ok = (not math.isfinite(max_range)) or (range_value <= max_range)
        return lower_ok and upper_ok

    def _on_state(self, msg: State):
        self.state_msg = msg

    def _on_extended_state(self, msg: ExtendedState):
        self.extended_state_msg = msg

    def _on_pose(self, msg: ArucoBasePose):
        self.pose_msg = msg
        self.pose_rx_time = self.get_clock().now()

    def _on_distance_sensor(self, msg: Range):
        self.distance_sensor_msg = msg
        self.distance_sensor_rx_time = self.get_clock().now()

    def _on_rel_alt(self, msg: Float64):
        self.rel_alt_msg = msg
        self.rel_alt_rx_time = self.get_clock().now()

    def _on_vel_local(self, msg: TwistStamped):
        self.vel_local_msg = msg
        self.vel_local_rx_time = self.get_clock().now()

    def _on_setpoint(self, msg: PositionTarget):
        self.setpoint_msg = msg
        self.setpoint_rx_time = self.get_clock().now()

    def _on_status(self, msg: String):
        try:
            self.status_msg = json.loads(msg.data)
            self.status_rx_time = self.get_clock().now()
        except Exception as exc:
            self.get_logger().warn(f'解析 status 失败，已忽略该帧: {exc}')

    def _write_row(self):
        """按固定频率把当前状态写入单次 CSV。"""
        now_time = self.get_clock().now()
        now_msg = now_time.to_msg()
        ros_time_sec = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9

        mode = self.state_msg.mode if self.state_msg is not None else ''
        armed = int(self.state_msg.armed) if self.state_msg is not None else -1
        connected = int(self.state_msg.connected) if self.state_msg is not None else -1

        aruco_fresh = int(
            self._is_fresh(
                self.pose_rx_time,
                now_time,
                self.param_snapshot['pose_timeout_sec'],
            )
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
            eyaw = self.wrap_to_pi(
                self.param_snapshot['target_yaw'] - yaw_rel_corrected
            )
            ex_marker = self.param_snapshot['target_x'] - aruco_x
            ey_marker = aruco_y - self.param_snapshot['target_y']
            ex_body, ey_body = self.rotate_marker_error_to_body(
                ex_marker,
                ey_marker,
                yaw_rel_corrected,
            )
        else:
            aruco_x = float('nan')
            aruco_y = float('nan')
            aruco_z = float('nan')
            aruco_yaw = float('nan')
            yaw_rel_raw = float('nan')
            yaw_rel_corrected = float('nan')
            eyaw = float('nan')
            ex_marker = float('nan')
            ey_marker = float('nan')
            ex_body = float('nan')
            ey_body = float('nan')

        distance_sensor_fresh = int(
            self._is_fresh(
                self.distance_sensor_rx_time,
                now_time,
                self.param_snapshot['distance_sensor_timeout_sec'],
            )
        )
        distance_sensor_age = self._age_sec(self.distance_sensor_rx_time, now_time)
        if self.distance_sensor_msg is not None:
            distance_sensor_valid = int(
                distance_sensor_fresh and self._range_is_valid(self.distance_sensor_msg)
            )
            distance_sensor_z = float(self.distance_sensor_msg.range)
        else:
            distance_sensor_valid = 0
            distance_sensor_z = float('nan')

        rel_alt_fresh = int(
            self._is_fresh(
                self.rel_alt_rx_time,
                now_time,
                self.param_snapshot['rel_alt_timeout_sec'],
            )
        )
        rel_alt_age = self._age_sec(self.rel_alt_rx_time, now_time)
        if self.rel_alt_msg is not None:
            rel_alt = float(self.rel_alt_msg.data)
        else:
            rel_alt = float('nan')

        local_velocity_fresh = int(
            self._is_fresh(
                self.vel_local_rx_time,
                now_time,
                self.param_snapshot['local_velocity_timeout_sec'],
            )
        )
        local_velocity_age = self._age_sec(self.vel_local_rx_time, now_time)
        if self.vel_local_msg is not None:
            local_vx = float(self.vel_local_msg.twist.linear.x)
            local_vy = float(self.vel_local_msg.twist.linear.y)
            local_vz = float(self.vel_local_msg.twist.linear.z)
            local_vxy = math.hypot(local_vx, local_vy)
        else:
            local_vx = float('nan')
            local_vy = float('nan')
            local_vz = float('nan')
            local_vxy = float('nan')

        setpoint_fresh = int(
            self._is_fresh(self.setpoint_rx_time, now_time, self.stale_timeout_sec)
        )
        setpoint_age = self._age_sec(self.setpoint_rx_time, now_time)
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

        status_fresh = int(
            self._is_fresh(self.status_rx_time, now_time, self.stale_timeout_sec)
        )
        status_age = self._age_sec(self.status_rx_time, now_time)
        status_data = self.status_msg if status_fresh == 1 and self.status_msg is not None else {}

        phase = status_data.get('phase', '')
        align_mode = status_data.get('align_mode', '')
        align_ok = int(bool(status_data.get('align_ok', False)))
        align_window_ratio = float(status_data.get('align_window_ratio', float('nan')))
        align_window_sample_count = int(status_data.get('align_window_sample_count', 0))
        align_window_mean_yaw = float(
            status_data.get('align_window_mean_yaw', float('nan'))
        )
        status_eyaw = float(status_data.get('eyaw', float('nan')))
        status_eyaw_ctrl = float(status_data.get('eyaw_ctrl', float('nan')))
        status_yaw_rate = float(status_data.get('yaw_rate', float('nan')))
        tracking_active = int(bool(status_data.get('tracking_active', False)))
        range_only_active = int(bool(status_data.get('range_only_active', False)))
        range_only_hold_active = int(bool(status_data.get('range_only_hold_active', False)))
        marker_loss_active = int(bool(status_data.get('marker_loss_active', False)))
        marker_lost_total_count = int(status_data.get('marker_lost_total_count', 0))
        last_marker_loss_duration_sec = float(
            status_data.get('last_marker_loss_duration_sec', float('nan'))
        )
        touchdown_candidate_active = int(
            bool(status_data.get('touchdown_candidate_active', False))
        )
        touchdown_candidate_ready = int(
            bool(status_data.get('touchdown_candidate_ready', False))
        )
        landed_by_extended_state = int(
            bool(status_data.get('landed_by_extended_state', False))
        )
        landed_by_heuristic = int(bool(status_data.get('landed_by_heuristic', False)))
        touchdown_height_source = str(status_data.get('touchdown_height_source', ''))
        touchdown_height_value = float(
            status_data.get('touchdown_height_value', float('nan'))
        )
        disarm_requested = int(bool(status_data.get('disarm_requested', False)))

        row = {
            'ros_time_sec': ros_time_sec,
            'mode': mode,
            'armed': armed,
            'connected': connected,
            'phase': phase,
            'align_mode': align_mode,
            'align_ok': align_ok,
            'align_window_ratio': align_window_ratio,
            'align_window_sample_count': align_window_sample_count,
            'align_window_mean_yaw': align_window_mean_yaw,
            'aruco_fresh': aruco_fresh,
            'aruco_age_sec': aruco_age,
            'aruco_x': aruco_x,
            'aruco_y': aruco_y,
            'aruco_z': aruco_z,
            'aruco_yaw_rad': aruco_yaw,
            'yaw_rel_raw_rad': yaw_rel_raw,
            'yaw_rel_corrected_rad': yaw_rel_corrected,
            'eyaw_rad': eyaw,
            'status_eyaw_rad': status_eyaw,
            'status_eyaw_ctrl_rad': status_eyaw_ctrl,
            'status_yaw_rate': status_yaw_rate,
            'distance_sensor_fresh': distance_sensor_fresh,
            'distance_sensor_age_sec': distance_sensor_age,
            'distance_sensor_valid': distance_sensor_valid,
            'distance_sensor_z': distance_sensor_z,
            'rel_alt_fresh': rel_alt_fresh,
            'rel_alt_age_sec': rel_alt_age,
            'rel_alt': rel_alt,
            'local_velocity_fresh': local_velocity_fresh,
            'local_velocity_age_sec': local_velocity_age,
            'local_vx': local_vx,
            'local_vy': local_vy,
            'local_vz': local_vz,
            'local_vxy': local_vxy,
            'ex_marker': ex_marker,
            'ey_marker': ey_marker,
            'ex_body': ex_body,
            'ey_body': ey_body,
            'setpoint_fresh': setpoint_fresh,
            'setpoint_age_sec': setpoint_age,
            'sp_frame': sp_frame,
            'sp_type_mask': sp_type_mask,
            'sp_vx': sp_vx,
            'sp_vy': sp_vy,
            'sp_vz': sp_vz,
            'sp_yaw_rate': sp_yaw_rate,
            'tracking_active': tracking_active,
            'range_only_active': range_only_active,
            'range_only_hold_active': range_only_hold_active,
            'marker_loss_active': marker_loss_active,
            'marker_lost_total_count': marker_lost_total_count,
            'last_marker_loss_duration_sec': last_marker_loss_duration_sec,
            'touchdown_candidate_active': touchdown_candidate_active,
            'touchdown_candidate_ready': touchdown_candidate_ready,
            'landed_by_extended_state': landed_by_extended_state,
            'landed_by_heuristic': landed_by_heuristic,
            'touchdown_height_source': touchdown_height_source,
            'touchdown_height_value': touchdown_height_value,
            'disarm_requested': disarm_requested,
            'status_fresh': status_fresh,
            'status_age_sec': status_age,
            **self.param_snapshot,
        }

        self.metric_rows.append(row)
        self.csv_writer.writerow(list(row.values()))

        if self.flush_interval_sec <= 0.0:
            self.csv_file.flush()
        else:
            flush_age = self._age_sec(self.last_flush_time, now_time)
            if flush_age >= self.flush_interval_sec:
                self.csv_file.flush()
                self.last_flush_time = now_time

    def _compute_metrics(self):
        """计算本次运行的简要摘要。"""
        rows = self.metric_rows
        if not rows:
            return {
                'status': 'empty',
                'row_count': 0,
            }

        offboard_rows = [row for row in rows if row['mode'] == 'OFFBOARD']
        align_rows = [row for row in rows if row['phase'] == 'ALIGN_AT_TARGET_HEIGHT']
        track_rows = [row for row in rows if row['tracking_active'] == 1]
        range_only_rows = [row for row in rows if row['range_only_active'] == 1]
        landed_ext_rows = [row for row in rows if row['landed_by_extended_state'] == 1]
        landed_heuristic_rows = [row for row in rows if row['landed_by_heuristic'] == 1]

        max_aruco_age = max(row['aruco_age_sec'] for row in rows)
        max_distance_age = max(row['distance_sensor_age_sec'] for row in rows)
        marker_loss_count_max = max(
            row['marker_lost_total_count'] for row in rows
        )

        return {
            'status': 'ok',
            'row_count': len(rows),
            'offboard_row_count': len(offboard_rows),
            'align_row_count': len(align_rows),
            'tracking_row_count': len(track_rows),
            'range_only_row_count': len(range_only_rows),
            'landed_ext_row_count': len(landed_ext_rows),
            'landed_heuristic_row_count': len(landed_heuristic_rows),
            'max_aruco_age_sec': max_aruco_age,
            'max_distance_sensor_age_sec': max_distance_age,
            'marker_lost_total_count_max': marker_loss_count_max,
        }

    def _append_summary_csv(self, metrics):
        """把本次摘要追加到 summary CSV。"""
        os.makedirs(os.path.dirname(self.summary_csv_path), exist_ok=True)
        row = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'run_csv': self.csv_path,
            'file_prefix': self.file_prefix,
            **metrics,
            **self.param_snapshot,
        }
        fieldnames = list(row.keys())
        file_exists = os.path.exists(self.summary_csv_path)
        with open(self.summary_csv_path, 'a', newline='', encoding='utf-8') as summary_file:
            writer = csv.DictWriter(summary_file, fieldnames=fieldnames, lineterminator='\n')
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)

    def destroy_node(self):
        """退出前落盘并写 summary。"""
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except BaseException:
            pass

        metrics = self._compute_metrics()
        try:
            self._append_summary_csv(metrics)
            self.get_logger().info(f'汇总 CSV 已追加: {self.summary_csv_path}')
        except Exception as exc:
            self.get_logger().error(f'写入汇总 CSV 失败: {exc}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LandV2CsvLoggerNode()
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
