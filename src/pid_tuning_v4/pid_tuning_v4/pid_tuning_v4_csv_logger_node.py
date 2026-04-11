#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime
from typing import Optional

import rclpy
from debug_interface.msg import ArucoBasePose
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range


class PidTuningV4CsvLoggerNode(Node):
    """记录调参过程到 CSV，并在退出时输出摘要指标."""

    def __init__(self):
        super().__init__('pid_tuning_v4_csv_logger_node')

        # ===== 输入话题 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('distance_sensor_topic', '/mavros/hrlv_ez4_pub')
        self.declare_parameter('local_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('setpoint_raw_topic', '/mavros/setpoint_raw/local')

        # ===== 输出路径 =====
        self.declare_parameter('output_dir', '/home/zjh/project/rasip_pi_ws/log/tracking_csv')
        self.declare_parameter('file_prefix', 'pid_tuning_v4')
        self.declare_parameter(
            'summary_csv_path',
            '/home/zjh/project/rasip_pi_ws/log/tracking_csv/pid_tuning_v4_summary.csv',
        )

        # ===== 采样参数 =====
        self.declare_parameter('sample_rate_hz', 30.0)
        self.declare_parameter('stale_timeout_sec', 0.5)
        self.declare_parameter('flush_interval_sec', 1.0)

        # ===== 目标/控制参数快照 =====
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 2.5)
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
        self.declare_parameter('camera_yaw_compensation_deg', 0.0)
        self.declare_parameter('vxy_limit', 0.8)
        self.declare_parameter('vx_limit', float('nan'))
        self.declare_parameter('vy_limit', float('nan'))
        self.declare_parameter('vz_limit', 0.5)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('distance_sensor_timeout_sec', 0.5)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('enable_z_hold', True)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.distance_sensor_topic = self.get_parameter('distance_sensor_topic').value
        self.local_pose_topic = self.get_parameter('local_pose_topic').value
        self.setpoint_raw_topic = self.get_parameter('setpoint_raw_topic').value

        self.output_dir = self.get_parameter('output_dir').value
        self.file_prefix = self.get_parameter('file_prefix').value
        self.summary_csv_path = self.get_parameter('summary_csv_path').value
        self.sample_rate_hz = float(self.get_parameter('sample_rate_hz').value)
        self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
        self.flush_interval_sec = float(self.get_parameter('flush_interval_sec').value)
        self.distance_sensor_timeout_sec = float(
            self.get_parameter('distance_sensor_timeout_sec').value
        )

        # 记录完整参数快照，便于后续回溯“这一条数据对应哪组参数”。
        self.param_snapshot = {
            'distance_sensor_topic': self.distance_sensor_topic,
            'target_x': float(self.get_parameter('target_x').value),
            'target_y': float(self.get_parameter('target_y').value),
            'target_z': float(self.get_parameter('target_z').value),
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
            'camera_yaw_compensation_deg': float(
                self.get_parameter('camera_yaw_compensation_deg').value
            ),
            'vxy_limit': float(self.get_parameter('vxy_limit').value),
            'vx_limit': float(self.get_parameter('vx_limit').value),
            'vy_limit': float(self.get_parameter('vy_limit').value),
            'vz_limit': float(self.get_parameter('vz_limit').value),
            'velocity_deadband': float(
                self.get_parameter('velocity_deadband').value
            ),
            'control_rate_hz': float(self.get_parameter('control_rate_hz').value),
            'pose_timeout_sec': float(self.get_parameter('pose_timeout_sec').value),
            'distance_sensor_timeout_sec': self.distance_sensor_timeout_sec,
            'require_offboard': int(
                bool(self.get_parameter('require_offboard').value)
            ),
            'enable_z_hold': int(bool(self.get_parameter('enable_z_hold').value)),
        }
        self.camera_yaw_compensation_rad = math.radians(
            self.param_snapshot['camera_yaw_compensation_deg']
        )

        # ===== 最新数据缓存 =====
        self.state_msg: Optional[State] = None
        self.pose_msg: Optional[ArucoBasePose] = None
        self.distance_sensor_msg: Optional[Range] = None
        self.local_pose_msg: Optional[PoseStamped] = None
        self.setpoint_msg: Optional[PositionTarget] = None

        self.pose_rx_time = None
        self.distance_sensor_rx_time = None
        self.local_pose_rx_time = None
        self.setpoint_rx_time = None
        self.last_flush_time = self.get_clock().now()

        # ===== 指标缓存 =====
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
            'aruco_fresh',
            'aruco_age_sec',
            'aruco_x',
            'aruco_y',
            'aruco_z',
            'distance_sensor_z',
            'distance_sensor_fresh',
            'distance_sensor_age_sec',
            'distance_sensor_valid',
            'z_source_used',
            'ez_from_vision',
            'ez_from_distance_sensor',
            'aruco_yaw_rad',
            'yaw_rel_raw_rad',
            'yaw_rel_corrected_rad',
            'ex_marker',
            'ey_marker',
            'ex_body',
            'ey_body',
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
            'target_x',
            'target_y',
            'target_z',
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
            'camera_yaw_compensation_deg',
            'vxy_limit',
            'vx_limit',
            'vy_limit',
            'vz_limit',
            'velocity_deadband',
            'control_rate_hz',
            'pose_timeout_sec',
            'distance_sensor_timeout_sec',
            'require_offboard',
            'enable_z_hold',
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
            Range,
            self.distance_sensor_topic,
            self._on_distance_sensor,
            mavros_pose_qos,
        )
        self.create_subscription(
            PoseStamped,
            self.local_pose_topic,
            self._on_local_pose,
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

        self.get_logger().info(f'PID调参 CSV 记录已启动，输出文件: {self.csv_path}')
        self.get_logger().info(
            '参数快照 | '
            f'distance_sensor_topic={self.distance_sensor_topic} | '
            f'distance_sensor_timeout_sec={self.distance_sensor_timeout_sec:.3f} | '
            f'camera_yaw_compensation_deg='
            f'{self.param_snapshot["camera_yaw_compensation_deg"]:.3f} | '
            'yaw 不参与控制，sp_yaw_rate 仅用于观测且应保持 0.0'
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
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """四元数转 yaw."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _age_sec(stamp_time, now_time) -> float:
        """计算数据距离当前时刻的年龄，缺失时返回一个极大值."""
        if stamp_time is None:
            return 1e9
        return (now_time - stamp_time).nanoseconds / 1e9

    def _is_fresh(self, stamp_time, now_time, timeout_sec: float) -> bool:
        """判断数据是否仍处于新鲜时间窗内."""
        return self._age_sec(stamp_time, now_time) <= timeout_sec

    @staticmethod
    def _range_is_in_bounds(range_value: float, min_range: float, max_range: float) -> bool:
        """检查距离值是否落在消息声明的合法量程内."""
        lower_ok = (not math.isfinite(min_range)) or (range_value >= min_range)
        upper_ok = (not math.isfinite(max_range)) or (range_value <= max_range)
        return lower_ok and upper_ok

    def _on_state(self, msg: State):
        self.state_msg = msg

    def _on_pose(self, msg: ArucoBasePose):
        self.pose_msg = msg
        self.pose_rx_time = self.get_clock().now()

    def _on_distance_sensor(self, msg: Range):
        self.distance_sensor_msg = msg
        self.distance_sensor_rx_time = self.get_clock().now()

    def _on_local_pose(self, msg: PoseStamped):
        self.local_pose_msg = msg
        self.local_pose_rx_time = self.get_clock().now()

    def _on_setpoint(self, msg: PositionTarget):
        self.setpoint_msg = msg
        self.setpoint_rx_time = self.get_clock().now()

    def _write_row(self):
        """按固定频率把当前控制状态写入单次 CSV."""
        now_time = self.get_clock().now()
        now_msg = now_time.to_msg()
        ros_time_sec = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9

        mode = self.state_msg.mode if self.state_msg is not None else ''
        armed = int(self.state_msg.armed) if self.state_msg is not None else -1
        connected = int(self.state_msg.connected) if self.state_msg is not None else -1

        aruco_fresh = int(
            self._is_fresh(self.pose_rx_time, now_time, self.stale_timeout_sec)
        )
        aruco_age = self._age_sec(self.pose_rx_time, now_time)
        if self.pose_msg is not None:
            aruco_x = float(self.pose_msg.x)
            aruco_y = float(self.pose_msg.y)
            aruco_z = float(self.pose_msg.z)
            aruco_yaw = float(self.pose_msg.yaw)
            # 与控制节点保持一致：
            # `aruco_yaw_rad` 保留视觉链原始输出，
            # `yaw_rel_raw_rad`/`yaw_rel_corrected_rad` 记录真正参与控制的 yaw。
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
            ez_from_vision = self.param_snapshot['target_z'] - aruco_z
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
            ez_from_vision = float('nan')

        distance_sensor_fresh = int(
            self._is_fresh(
                self.distance_sensor_rx_time,
                now_time,
                self.distance_sensor_timeout_sec,
            )
        )
        distance_sensor_age = self._age_sec(self.distance_sensor_rx_time, now_time)
        if self.distance_sensor_msg is not None:
            distance_sensor_z = float(self.distance_sensor_msg.range)
            min_range = float(self.distance_sensor_msg.min_range)
            max_range = float(self.distance_sensor_msg.max_range)
            distance_sensor_finite = math.isfinite(distance_sensor_z)
            distance_sensor_in_range = (
                distance_sensor_finite
                and self._range_is_in_bounds(
                    distance_sensor_z,
                    min_range,
                    max_range,
                )
            )
            distance_sensor_valid = int(
                distance_sensor_fresh and distance_sensor_in_range
            )
            ez_from_distance_sensor = (
                self.param_snapshot['target_z'] - distance_sensor_z
                if distance_sensor_finite else float('nan')
            )
        else:
            distance_sensor_z = float('nan')
            distance_sensor_valid = 0
            ez_from_distance_sensor = float('nan')

        if not self.param_snapshot['enable_z_hold']:
            z_source_used = 'disabled'
        elif distance_sensor_valid == 1:
            z_source_used = 'distance_sensor'
        else:
            z_source_used = 'invalid'

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
            local_yaw = self._quat_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        else:
            local_x = float('nan')
            local_y = float('nan')
            local_z = float('nan')
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
            distance_sensor_z,
            distance_sensor_fresh,
            distance_sensor_age,
            distance_sensor_valid,
            z_source_used,
            ez_from_vision,
            ez_from_distance_sensor,
            aruco_yaw,
            yaw_rel_raw,
            yaw_rel_corrected,
            ex_marker,
            ey_marker,
            ex_body,
            ey_body,
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
            self.param_snapshot['target_x'],
            self.param_snapshot['target_y'],
            self.param_snapshot['target_z'],
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
            self.param_snapshot['camera_yaw_compensation_deg'],
            self.param_snapshot['vxy_limit'],
            self.param_snapshot['vx_limit'],
            self.param_snapshot['vy_limit'],
            self.param_snapshot['vz_limit'],
            self.param_snapshot['velocity_deadband'],
            self.param_snapshot['control_rate_hz'],
            self.param_snapshot['pose_timeout_sec'],
            self.param_snapshot['distance_sensor_timeout_sec'],
            self.param_snapshot['require_offboard'],
            self.param_snapshot['enable_z_hold'],
        ])

        # 指标统计仍以 OFFBOARD + aruco fresh 为主，保证 XY 与 yaw 对齐口径；
        # Z 指标则只在距离传感器有效样本上计算。
        self.metric_rows.append({
            't': ros_time_sec,
            'mode': mode,
            'aruco_fresh': aruco_fresh,
            'distance_sensor_valid': distance_sensor_valid,
            'aruco_x': aruco_x,
            'aruco_y': aruco_y,
            'aruco_z': aruco_z,
            'distance_sensor_z': distance_sensor_z,
            'sp_vx': sp_vx,
            'sp_vy': sp_vy,
            'sp_yaw_rate': sp_yaw_rate,
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
        return math.sqrt(sum((v - mean_val) * (v - mean_val) for v in vals) / len(vals))

    def _compute_metrics(self):
        """计算 OFFBOARD 窗口下的调参摘要指标."""
        off_rows = [row for row in self.metric_rows if row['mode'] == 'OFFBOARD']
        eval_rows = [row for row in off_rows if row['aruco_fresh'] == 1]
        z_eval_rows = [
            row for row in eval_rows if row['distance_sensor_valid'] == 1
        ]

        result = {
            'eval_rows': len(eval_rows),
            'offboard_rows': len(off_rows),
            'z_eval_rows': len(z_eval_rows),
            'status': 'ok' if len(eval_rows) >= 1 else 'insufficient_data',
        }

        if not off_rows:
            result.update({
                'fresh_ratio': float('nan'),
                'distance_valid_ratio': float('nan'),
                'max_stale_s': float('nan'),
                'max_abs_yaw_rate': float('nan'),
            })
            return result

        fresh_ratio = len(eval_rows) / len(off_rows)
        distance_valid_ratio = (
            len(z_eval_rows) / len(eval_rows) if eval_rows else float('nan')
        )

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
            'distance_valid_ratio': distance_valid_ratio,
            'max_stale_s': max_stale_s,
            'max_abs_yaw_rate': max(yaw_rate_vals) if yaw_rate_vals else float('nan'),
        })

        if not eval_rows:
            return result

        target_x = self.param_snapshot['target_x']
        target_y = self.param_snapshot['target_y']
        target_z = self.param_snapshot['target_z']

        ex_marker = [target_x - row['aruco_x'] for row in eval_rows]
        ey_marker = [row['aruco_y'] - target_y for row in eval_rows]

        abs_ex = [abs(val) for val in ex_marker]
        abs_ey = [abs(val) for val in ey_marker]
        exy = [
            math.hypot(ex_marker[idx], ey_marker[idx])
            for idx in range(len(ex_marker))
        ]

        if z_eval_rows:
            ez = [target_z - row['distance_sensor_z'] for row in z_eval_rows]
            abs_ez = [abs(val) for val in ez]
            rmse_z = self._rmse(ez)
            p95_abs_z = self._quantile(abs_ez, 0.95)
        else:
            rmse_z = float('nan')
            p95_abs_z = float('nan')

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
            'rmse_z': rmse_z,
            'rmse_xy': self._rmse(exy),
            'p95_abs_x': self._quantile(abs_ex, 0.95),
            'p95_abs_y': self._quantile(abs_ey, 0.95),
            'p95_abs_z': p95_abs_z,
            'p95_xy': self._quantile(exy, 0.95),
            'cmd_jitter_x': cmd_jitter_x,
            'cmd_jitter_y': cmd_jitter_y,
            'cmd_jitter_xy': cmd_jitter_xy,
        })
        return result

    def _repair_summary_csv_if_needed(self, summary_path: str):
        """统一行结束符并清理重复表头，避免历史格式问题影响后续追加."""
        if not os.path.exists(summary_path):
            return
        try:
            with open(summary_path, 'r', encoding='utf-8', newline='') as summary_file:
                raw = summary_file.read()
            if raw == '':
                return

            normalized = raw.replace('\r\n', '\n').replace('\r', '\n')
            lines = [line for line in normalized.split('\n') if line.strip() != '']
            if not lines:
                return

            header = lines[0]
            cleaned = [header]
            for line in lines[1:]:
                if line == header:
                    continue
                cleaned.append(line)

            repaired = '\n'.join(cleaned) + '\n'
            if repaired != raw:
                with open(summary_path, 'w', encoding='utf-8', newline='') as summary_file:
                    summary_file.write(repaired)
                self.get_logger().info(f'已修复 summary 格式: {summary_path}')
        except Exception as exc:
            self.get_logger().warn(f'summary 格式修复跳过: {exc}')

    def _append_summary_csv(self, metrics):
        """把本次运行摘要追加到 summary CSV."""
        os.makedirs(os.path.dirname(self.summary_csv_path), exist_ok=True)
        self._repair_summary_csv_if_needed(self.summary_csv_path)
        file_exists = os.path.exists(self.summary_csv_path)

        row = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'run_csv': self.csv_path,
            'file_prefix': self.file_prefix,
            'status': metrics.get('status', 'unknown'),
            'offboard_rows': metrics.get('offboard_rows', 0),
            'eval_rows': metrics.get('eval_rows', 0),
            'z_eval_rows': metrics.get('z_eval_rows', 0),
            'fresh_ratio': metrics.get('fresh_ratio', float('nan')),
            'distance_valid_ratio': metrics.get(
                'distance_valid_ratio',
                float('nan'),
            ),
            'max_stale_s': metrics.get('max_stale_s', float('nan')),
            'rmse_x': metrics.get('rmse_x', float('nan')),
            'rmse_y': metrics.get('rmse_y', float('nan')),
            'rmse_z': metrics.get('rmse_z', float('nan')),
            'rmse_xy': metrics.get('rmse_xy', float('nan')),
            'p95_abs_x': metrics.get('p95_abs_x', float('nan')),
            'p95_abs_y': metrics.get('p95_abs_y', float('nan')),
            'p95_abs_z': metrics.get('p95_abs_z', float('nan')),
            'p95_xy': metrics.get('p95_xy', float('nan')),
            'cmd_jitter_x': metrics.get('cmd_jitter_x', float('nan')),
            'cmd_jitter_y': metrics.get('cmd_jitter_y', float('nan')),
            'cmd_jitter_xy': metrics.get('cmd_jitter_xy', float('nan')),
            'max_abs_yaw_rate': metrics.get('max_abs_yaw_rate', float('nan')),
            **self.param_snapshot,
        }

        fieldnames = list(row.keys())
        with open(self.summary_csv_path, 'a', newline='', encoding='utf-8') as summary_file:
            writer = csv.DictWriter(summary_file, fieldnames=fieldnames, lineterminator='\n')
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)

    def _print_summary(self, metrics):
        """在终端打印本次调参摘要."""
        self.get_logger().info('===== pid_tuning_v4 调参指标汇总（OFFBOARD） =====')
        self.get_logger().info(
            f"status={metrics.get('status')} | offboard_rows={metrics.get('offboard_rows')} | "
            f"eval_rows={metrics.get('eval_rows')} | z_eval_rows={metrics.get('z_eval_rows')} | "
            f"fresh_ratio={metrics.get('fresh_ratio', float('nan')):.4f} | "
            f"distance_valid_ratio={metrics.get('distance_valid_ratio', float('nan')):.4f} | "
            f"max_stale_s={metrics.get('max_stale_s', float('nan')):.3f} | "
            f"max_abs_yaw_rate={metrics.get('max_abs_yaw_rate', float('nan')):.4f}"
        )
        if metrics.get('status') == 'ok':
            self.get_logger().info(
                f"RMSE: x={metrics['rmse_x']:.4f}, y={metrics['rmse_y']:.4f}, "
                f"z={metrics['rmse_z']:.4f}, xy={metrics['rmse_xy']:.4f}"
            )
            self.get_logger().info(
                f"P95: |x|={metrics['p95_abs_x']:.4f}, |y|={metrics['p95_abs_y']:.4f}, "
                f"|z|={metrics['p95_abs_z']:.4f}, xy={metrics['p95_xy']:.4f}"
            )
            self.get_logger().info(
                f"CmdJitter: x={metrics['cmd_jitter_x']:.5f}, y={metrics['cmd_jitter_y']:.5f}, "
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
    node = PidTuningV4CsvLoggerNode()
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
