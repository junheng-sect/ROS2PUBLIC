#!/usr/bin/env python3

import csv
import math
import os
import signal
from bisect import bisect_right
from dataclasses import dataclass
from datetime import datetime
from statistics import mean
from typing import Dict, List, Optional, Sequence

import rclpy
from debug_interface.msg import ArucoBasePose
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Range


@dataclass
class PoseSample:
    """保存单个 Pose 类样本的时间戳与三轴位置."""

    stamp_sec: float
    x: float
    y: float
    z: float


@dataclass
class VisualPoseSample:
    """保存视觉位姿样本的分析时间戳与原始诊断时间戳."""

    stamp_sec: float
    header_stamp_sec: float
    tf_pub_stamp_sec: float
    rx_stamp_sec: float
    visual_stamp_source: str
    x: float
    y: float
    z: float


@dataclass
class DistanceSensorSample:
    """保存单个测距仪样本及其有效性判定结果."""

    stamp_sec: float
    range_m: float
    min_range_m: float
    max_range_m: float
    valid: bool
    status: str
    gap_sec_from_prev: float
    gap_exceeds_timeout: bool


class PoseDelayAnalyzerNode(Node):
    """记录原始数据并在退出阶段输出延迟分析结果."""

    def __init__(self):
        super().__init__('pose_delay_analyzer_node')
        self._plt = None

        # ===== 输入与分析参数 =====
        self.declare_parameter('local_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter(
            'distance_sensor_topic',
            '/mavros/hrlv_ez4_pub',
        )
        self.declare_parameter('visual_pose_topic', '/debug/aruco_pose')
        self.declare_parameter('distance_sensor_timeout_sec', 0.5)
        self.declare_parameter(
            'output_dir',
            '/home/zjh/project/rasip_pi_ws/log/pose_delay',
        )
        self.declare_parameter('file_prefix', 'pose_delay')
        self.declare_parameter('analysis_axis', 'z')
        self.declare_parameter('min_samples', 30)
        self.declare_parameter('smooth_window_sec', 0.20)
        self.declare_parameter('slope_threshold', 0.05)
        self.declare_parameter('merge_gap_sec', 0.30)
        self.declare_parameter('major_edge_merge_gap_sec', 1.0)
        self.declare_parameter('major_edge_min_amplitude', 0.08)
        self.declare_parameter('major_edge_max_match_offset_sec', 2.0)
        self.declare_parameter('plot_relative_time', True)
        self.declare_parameter('auto_report', True)

        self.local_pose_topic = str(
            self.get_parameter('local_pose_topic').value
        )
        self.distance_sensor_topic = str(
            self.get_parameter('distance_sensor_topic').value
        )
        self.visual_pose_topic = str(
            self.get_parameter('visual_pose_topic').value
        )
        self.distance_sensor_timeout_sec = float(
            self.get_parameter('distance_sensor_timeout_sec').value
        )
        self.output_dir = str(self.get_parameter('output_dir').value)
        self.file_prefix = str(self.get_parameter('file_prefix').value)
        self.analysis_axis = self._normalize_axis(
            str(self.get_parameter('analysis_axis').value)
        )
        self.min_samples = int(self.get_parameter('min_samples').value)
        self.smooth_window_sec = float(
            self.get_parameter('smooth_window_sec').value
        )
        self.slope_threshold = float(
            self.get_parameter('slope_threshold').value
        )
        self.merge_gap_sec = float(self.get_parameter('merge_gap_sec').value)
        self.major_edge_merge_gap_sec = float(
            self.get_parameter('major_edge_merge_gap_sec').value
        )
        self.major_edge_min_amplitude = float(
            self.get_parameter('major_edge_min_amplitude').value
        )
        self.major_edge_max_match_offset_sec = float(
            self.get_parameter('major_edge_max_match_offset_sec').value
        )
        self.plot_relative_time = self._as_bool(
            self.get_parameter('plot_relative_time').value
        )
        self.auto_report = self._as_bool(
            self.get_parameter('auto_report').value
        )
        self._prepare_plot_backend()

        # ===== 数据缓存 =====
        self.local_pose_samples: List[PoseSample] = []
        self.distance_sensor_samples: List[DistanceSensorSample] = []
        self.visual_samples: List[VisualPoseSample] = []
        self.analysis_done = False

        # ===== 输出目录 =====
        os.makedirs(self.output_dir, exist_ok=True)
        self.session_name = (
            f'{self.file_prefix}_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        )
        self.session_dir = os.path.join(self.output_dir, self.session_name)
        os.makedirs(self.session_dir, exist_ok=True)

        # ===== 订阅 =====
        # MAVROS 常用 best_effort；这里统一显式声明，避免 QoS 不兼容。
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            PoseStamped,
            self.local_pose_topic,
            self._on_local_pose,
            mavros_qos,
        )
        self.create_subscription(
            Range,
            self.distance_sensor_topic,
            self._on_distance_sensor,
            mavros_qos,
        )
        self.create_subscription(
            ArucoBasePose,
            self.visual_pose_topic,
            self._on_visual_pose,
            10,
        )

        self.get_logger().info('pose_delay_analyzer 已启动，等待记录数据。')
        self.get_logger().info(f'输出目录: {self.session_dir}')
        self.get_logger().info(
            '参考流口径 | '
            f'analysis_axis={self.analysis_axis} | '
            f'{self._get_reference_description()}'
        )

    def _prepare_plot_backend(self):
        """预加载绘图后端，避免退出阶段首次建缓存导致超时."""
        if self._plt is not None:
            return

        import matplotlib

        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        self._plt = plt

    def _normalize_axis(self, axis: str) -> str:
        """将分析轴限制为 x/y/z，非法值自动回退到 z."""
        normalized = axis.strip().lower()
        if normalized in ('x', 'y', 'z'):
            return normalized

        self.get_logger().warning(
            f'analysis_axis={axis} 非法，自动回退到 z。'
        )
        return 'z'

    def _as_bool(self, value) -> bool:
        """兼容 launch 传入的 bool、字符串与数字."""
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in (
            '1',
            'true',
            'yes',
            'on',
        )

    def _on_local_pose(self, msg: PoseStamped):
        """记录飞控本地位姿原始样本."""
        self.local_pose_samples.append(
            PoseSample(
                stamp_sec=self._stamp_to_sec(msg.header.stamp),
                x=float(msg.pose.position.x),
                y=float(msg.pose.position.y),
                z=float(msg.pose.position.z),
            )
        )

    def _on_distance_sensor(self, msg: Range):
        """记录测距仪原始样本，并提前完成有效性判定."""
        stamp_sec = self._stamp_to_sec(msg.header.stamp)
        range_m = float(msg.range)
        min_range_m = float(msg.min_range)
        max_range_m = float(msg.max_range)

        gap_sec_from_prev = float('nan')
        if self.distance_sensor_samples:
            gap_sec_from_prev = (
                stamp_sec - self.distance_sensor_samples[-1].stamp_sec
            )

        if not math.isfinite(range_m):
            valid = False
            status = 'range_non_finite'
        else:
            upper_ok = (
                (not math.isfinite(max_range_m)) or
                (range_m <= max_range_m)
            )
            if upper_ok:
                # 现场 hrlv_ez4 经常在近地阶段给出小于 min_range 的真实回波，
                # 当前实验需要保留这些样本参与 reference(z) 分析，因此这里
                # 明确忽略 min_range，只继续保留有限值与 max_range 上限校验。
                valid = True
                if (
                    math.isfinite(min_range_m) and
                    range_m < min_range_m
                ):
                    status = 'valid_ignore_min_range'
                else:
                    status = 'valid'
            else:
                valid = False
                status = 'above_max_range'

        gap_exceeds_timeout = (
            math.isfinite(gap_sec_from_prev) and
            gap_sec_from_prev > self.distance_sensor_timeout_sec
        )

        self.distance_sensor_samples.append(
            DistanceSensorSample(
                stamp_sec=stamp_sec,
                range_m=range_m,
                min_range_m=min_range_m,
                max_range_m=max_range_m,
                valid=valid,
                status=status,
                gap_sec_from_prev=gap_sec_from_prev,
                gap_exceeds_timeout=gap_exceeds_timeout,
            )
        )

    def _on_visual_pose(self, msg: ArucoBasePose):
        """记录视觉位姿原始样本."""
        rx_stamp_sec = self._now_to_sec()
        header_stamp_sec = self._stamp_to_sec(msg.header.stamp)
        tf_pub_stamp_sec = self._stamp_to_sec(msg.tf_pub_stamp)

        if tf_pub_stamp_sec > 0.0:
            # 视觉链对齐使用 /debug/aruco_pose 的实际发布时间，
            # 避免原始图像 header.stamp 让视觉曲线看起来过早。
            stamp_sec = tf_pub_stamp_sec
            visual_stamp_source = 'tf_pub_stamp'
        else:
            # 老数据或异常发布端可能没有填充 tf_pub_stamp，此时只用
            # analyzer 接收时刻兜底，并在 CSV/报告中保留 fallback 标记。
            stamp_sec = rx_stamp_sec
            visual_stamp_source = 'fallback_rx_stamp'

        self.visual_samples.append(
            VisualPoseSample(
                stamp_sec=stamp_sec,
                header_stamp_sec=header_stamp_sec,
                tf_pub_stamp_sec=tf_pub_stamp_sec,
                rx_stamp_sec=rx_stamp_sec,
                visual_stamp_source=visual_stamp_source,
                x=float(msg.x),
                y=float(msg.y),
                z=float(msg.z),
            )
        )

    def finalize_analysis(self):
        """在节点退出时统一执行落盘、绘图和报告生成."""
        if self.analysis_done:
            return

        self.analysis_done = True
        self._emit_shutdown_message('开始输出 pose_delay_analyzer 分析结果。')

        local_pose_csv_path = os.path.join(
            self.session_dir,
            f'{self.file_prefix}_local_pose.csv',
        )
        distance_sensor_csv_path = os.path.join(
            self.session_dir,
            f'{self.file_prefix}_distance_sensor.csv',
        )
        reference_csv_path = os.path.join(
            self.session_dir,
            f'{self.file_prefix}_reference.csv',
        )
        visual_csv_path = os.path.join(
            self.session_dir,
            f'{self.file_prefix}_visual.csv',
        )
        reference_events_csv_path = os.path.join(
            self.session_dir,
            f'{self.file_prefix}_reference_events.csv',
        )
        visual_events_csv_path = os.path.join(
            self.session_dir,
            f'{self.file_prefix}_visual_events.csv',
        )
        matches_csv_path = os.path.join(
            self.session_dir,
            f'{self.file_prefix}_matches.csv',
        )
        plot_path = os.path.join(
            self.session_dir,
            f'{self.file_prefix}_plot.png',
        )
        report_path = os.path.join(
            self.session_dir,
            f'{self.file_prefix}_report.md',
        )

        # ROS 话题在现场可能出现近邻乱序样本；这里统一按各流分析用
        # stamp_sec 排序，避免 gap 统计、reference 对齐和一阶差分被负
        # 时间间隔污染。
        sorted_local_pose_samples = sorted(
            self.local_pose_samples,
            key=lambda sample: sample.stamp_sec,
        )
        sorted_distance_sensor_samples = sorted(
            self.distance_sensor_samples,
            key=lambda sample: sample.stamp_sec,
        )
        sorted_visual_samples = sorted(
            self.visual_samples,
            key=lambda sample: sample.stamp_sec,
        )

        base_time_sec = self._get_base_time_sec()
        local_pose_rows = self._build_pose_rows(
            sorted_local_pose_samples,
            base_time_sec,
        )
        distance_sensor_rows = self._build_distance_sensor_rows(
            sorted_distance_sensor_samples,
            base_time_sec,
        )
        reference_rows = self._build_reference_rows(
            sorted_local_pose_samples,
            sorted_distance_sensor_samples,
            base_time_sec,
        )
        visual_rows = self._build_visual_pose_rows(
            sorted_visual_samples,
            base_time_sec,
        )

        self._write_pose_csv(local_pose_csv_path, local_pose_rows)
        self._write_distance_sensor_csv(
            distance_sensor_csv_path,
            distance_sensor_rows,
        )
        self._write_reference_csv(reference_csv_path, reference_rows)
        self._write_visual_pose_csv(visual_csv_path, visual_rows)

        reference_analysis = self._analyze_stream(reference_rows, 'reference')
        visual_analysis = self._analyze_stream(visual_rows, 'visual')

        self._write_events_csv(
            reference_events_csv_path,
            reference_analysis['events'],
        )
        self._write_events_csv(
            visual_events_csv_path,
            visual_analysis['events'],
        )

        matches = self._match_events(
            reference_analysis['events'],
            visual_analysis['events'],
        )
        self._write_matches_csv(matches_csv_path, matches)

        summary = self._build_summary(
            local_pose_rows=local_pose_rows,
            distance_sensor_rows=distance_sensor_rows,
            reference_rows=reference_rows,
            visual_rows=visual_rows,
            reference_analysis=reference_analysis,
            visual_analysis=visual_analysis,
            matches=matches,
            local_pose_csv_path=local_pose_csv_path,
            distance_sensor_csv_path=distance_sensor_csv_path,
            reference_csv_path=reference_csv_path,
            visual_csv_path=visual_csv_path,
            reference_events_csv_path=reference_events_csv_path,
            visual_events_csv_path=visual_events_csv_path,
            matches_csv_path=matches_csv_path,
            plot_path=plot_path,
            report_path=report_path,
        )

        self._plot_comparison(
            reference_rows=reference_rows,
            visual_rows=visual_rows,
            reference_analysis=reference_analysis,
            visual_analysis=visual_analysis,
            summary=summary,
            plot_path=plot_path,
        )

        if self.auto_report:
            self._write_report(report_path, summary)

        if summary['mean_delay_sec'] is None:
            self._emit_shutdown_message(summary['status'])
        else:
            self._emit_shutdown_message(
                f'平均延迟: {summary["mean_delay_sec"]:.4f} s'
            )
        self._emit_shutdown_message(f'结果目录: {self.session_dir}')

    def _stamp_to_sec(self, stamp) -> float:
        """把 ROS 时间戳转换为秒."""
        return float(Time.from_msg(stamp).nanoseconds) / 1e9

    def _now_to_sec(self) -> float:
        """读取 analyzer 当前 ROS 时钟秒数，用作接收时刻诊断."""
        return float(self.get_clock().now().nanoseconds) / 1e9

    def _emit_shutdown_message(self, message: str):
        """退出阶段直接写标准输出，避免 rosout 已关闭时再次报错."""
        print(f'[pose_delay_analyzer_node] {message}', flush=True)

    def _get_base_time_sec(self) -> float:
        """取三路原始流中的最早时间，作为相对时间轴零点."""
        first_stamps = [
            sample.stamp_sec for sample in self.local_pose_samples
        ]
        first_stamps.extend(
            sample.stamp_sec for sample in self.distance_sensor_samples
        )
        first_stamps.extend(
            sample.stamp_sec for sample in self.visual_samples
        )

        if first_stamps:
            return min(first_stamps)

        return float(self.get_clock().now().nanoseconds) / 1e9

    def _build_pose_rows(
        self,
        samples: Sequence[PoseSample],
        base_time_sec: float,
    ) -> List[Dict[str, object]]:
        """把 Pose 类样本统一转换为 CSV/分析可复用的行结构."""
        rows = []
        for index, sample in enumerate(samples):
            rows.append({
                'index': index,
                'stamp_sec': sample.stamp_sec,
                'relative_time_sec': sample.stamp_sec - base_time_sec,
                'x': sample.x,
                'y': sample.y,
                'z': sample.z,
                'analysis_axis': self.analysis_axis,
                'analysis_value': getattr(sample, self.analysis_axis),
            })
        return rows

    def _build_visual_pose_rows(
        self,
        samples: Sequence[VisualPoseSample],
        base_time_sec: float,
    ) -> List[Dict[str, object]]:
        """把视觉样本转换为分析行，并保留原始时间戳诊断列."""
        rows = []
        for index, sample in enumerate(samples):
            rows.append({
                'index': index,
                'stamp_sec': sample.stamp_sec,
                'relative_time_sec': sample.stamp_sec - base_time_sec,
                'x': sample.x,
                'y': sample.y,
                'z': sample.z,
                'analysis_axis': self.analysis_axis,
                'analysis_value': getattr(sample, self.analysis_axis),
                'header_stamp_sec': sample.header_stamp_sec,
                'tf_pub_stamp_sec': sample.tf_pub_stamp_sec,
                'rx_stamp_sec': sample.rx_stamp_sec,
                'visual_stamp_source': sample.visual_stamp_source,
            })
        return rows

    def _build_distance_sensor_rows(
        self,
        samples: Sequence[DistanceSensorSample],
        base_time_sec: float,
    ) -> List[Dict[str, object]]:
        """把测距仪原始样本转换为独立 CSV 行结构."""
        rows = []
        previous_stamp_sec = None
        for index, sample in enumerate(samples):
            if previous_stamp_sec is None:
                gap_sec_from_prev = float('nan')
            else:
                gap_sec_from_prev = sample.stamp_sec - previous_stamp_sec
            gap_exceeds_timeout = (
                math.isfinite(gap_sec_from_prev) and
                gap_sec_from_prev > self.distance_sensor_timeout_sec
            )

            rows.append({
                'index': index,
                'stamp_sec': sample.stamp_sec,
                'relative_time_sec': sample.stamp_sec - base_time_sec,
                'range_m': sample.range_m,
                'min_range_m': sample.min_range_m,
                'max_range_m': sample.max_range_m,
                'valid': sample.valid,
                'status': sample.status,
                'gap_sec_from_prev': gap_sec_from_prev,
                'gap_exceeds_timeout': gap_exceeds_timeout,
            })
            previous_stamp_sec = sample.stamp_sec
        return rows

    def _build_reference_rows(
        self,
        local_pose_samples: Sequence[PoseSample],
        distance_sensor_samples: Sequence[DistanceSensorSample],
        base_time_sec: float,
    ) -> List[Dict[str, object]]:
        """构造用于延迟分析的参考流."""
        if self.analysis_axis == 'z':
            return self._build_reference_rows_for_z(
                local_pose_samples,
                distance_sensor_samples,
                base_time_sec,
            )
        return self._build_reference_rows_for_xy(
            local_pose_samples,
            distance_sensor_samples,
            base_time_sec,
        )

    def _build_reference_rows_for_z(
        self,
        local_pose_samples: Sequence[PoseSample],
        distance_sensor_samples: Sequence[DistanceSensorSample],
        base_time_sec: float,
    ) -> List[Dict[str, object]]:
        """Z 轴分析时，以有效测距仪样本为主构造参考流."""
        rows = []
        local_pose_stamps = [
            sample.stamp_sec for sample in local_pose_samples
        ]

        for distance_sample in distance_sensor_samples:
            if not distance_sample.valid:
                continue

            local_pose_sample = self._find_latest_pose_before_or_at(
                local_pose_samples,
                local_pose_stamps,
                distance_sample.stamp_sec,
            )

            if local_pose_sample is None:
                x_value = float('nan')
                y_value = float('nan')
                local_pose_z = float('nan')
                local_pose_stamp_sec = float('nan')
                has_local_pose_match = False
            else:
                x_value = local_pose_sample.x
                y_value = local_pose_sample.y
                local_pose_z = local_pose_sample.z
                local_pose_stamp_sec = local_pose_sample.stamp_sec
                has_local_pose_match = True

            rows.append({
                'index': len(rows),
                'stamp_sec': distance_sample.stamp_sec,
                'relative_time_sec': distance_sample.stamp_sec - base_time_sec,
                'x': x_value,
                'y': y_value,
                'z': distance_sample.range_m,
                'local_pose_z': local_pose_z,
                'distance_sensor_z': distance_sample.range_m,
                'local_pose_stamp_sec': local_pose_stamp_sec,
                'distance_sensor_stamp_sec': distance_sample.stamp_sec,
                'has_local_pose_match': has_local_pose_match,
                'has_distance_sensor_match': True,
                'analysis_axis': self.analysis_axis,
                'analysis_value': distance_sample.range_m,
                'reference_z_source': 'distance_sensor',
            })
        return rows

    def _build_reference_rows_for_xy(
        self,
        local_pose_samples: Sequence[PoseSample],
        distance_sensor_samples: Sequence[DistanceSensorSample],
        base_time_sec: float,
    ) -> List[Dict[str, object]]:
        """X/Y 轴分析时，以 local pose 为主并附带最近测距值."""
        rows = []
        valid_distance_samples = [
            sample for sample in distance_sensor_samples if sample.valid
        ]
        valid_distance_stamps = [
            sample.stamp_sec for sample in valid_distance_samples
        ]

        for pose_sample in local_pose_samples:
            distance_sample = self._find_latest_distance_sensor_before_or_at(
                valid_distance_samples,
                valid_distance_stamps,
                pose_sample.stamp_sec,
            )

            if distance_sample is None:
                distance_sensor_z = float('nan')
                distance_sensor_stamp_sec = float('nan')
                has_distance_sensor_match = False
            else:
                distance_sensor_z = distance_sample.range_m
                distance_sensor_stamp_sec = distance_sample.stamp_sec
                has_distance_sensor_match = True

            rows.append({
                'index': len(rows),
                'stamp_sec': pose_sample.stamp_sec,
                'relative_time_sec': pose_sample.stamp_sec - base_time_sec,
                'x': pose_sample.x,
                'y': pose_sample.y,
                'z': pose_sample.z,
                'local_pose_z': pose_sample.z,
                'distance_sensor_z': distance_sensor_z,
                'local_pose_stamp_sec': pose_sample.stamp_sec,
                'distance_sensor_stamp_sec': distance_sensor_stamp_sec,
                'has_local_pose_match': True,
                'has_distance_sensor_match': has_distance_sensor_match,
                'analysis_axis': self.analysis_axis,
                'analysis_value': getattr(pose_sample, self.analysis_axis),
                'reference_z_source': 'local_pose',
            })
        return rows

    def _find_latest_pose_before_or_at(
        self,
        samples: Sequence[PoseSample],
        stamps: Sequence[float],
        target_stamp_sec: float,
    ) -> Optional[PoseSample]:
        """找到时间不晚于目标时刻的最近 local pose 样本."""
        index = bisect_right(stamps, target_stamp_sec) - 1
        if index < 0:
            return None
        return samples[index]

    def _find_latest_distance_sensor_before_or_at(
        self,
        samples: Sequence[DistanceSensorSample],
        stamps: Sequence[float],
        target_stamp_sec: float,
    ) -> Optional[DistanceSensorSample]:
        """找到时间不晚于目标时刻的最近有效测距仪样本."""
        index = bisect_right(stamps, target_stamp_sec) - 1
        if index < 0:
            return None
        return samples[index]

    def _write_pose_csv(
        self,
        path: str,
        rows: Sequence[Dict[str, object]],
    ):
        """输出 local pose 或 visual pose 原始 CSV."""
        with open(path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file, lineterminator='\n')
            writer.writerow([
                'index',
                'stamp_sec',
                'relative_time_sec',
                'x',
                'y',
                'z',
                'analysis_axis',
                'analysis_value',
            ])
            for row in rows:
                writer.writerow([
                    row['index'],
                    self._format_float(row['stamp_sec'], 9),
                    self._format_float(row['relative_time_sec'], 9),
                    self._format_float(row['x'], 6),
                    self._format_float(row['y'], 6),
                    self._format_float(row['z'], 6),
                    row['analysis_axis'],
                    self._format_float(row['analysis_value'], 6),
                ])

    def _write_visual_pose_csv(
        self,
        path: str,
        rows: Sequence[Dict[str, object]],
    ):
        """输出视觉位姿 CSV，stamp_sec 为分析时间，附带诊断时间戳."""
        with open(path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file, lineterminator='\n')
            writer.writerow([
                'index',
                'stamp_sec',
                'relative_time_sec',
                'x',
                'y',
                'z',
                'analysis_axis',
                'analysis_value',
                'header_stamp_sec',
                'tf_pub_stamp_sec',
                'rx_stamp_sec',
                'visual_stamp_source',
            ])
            for row in rows:
                writer.writerow([
                    row['index'],
                    self._format_float(row['stamp_sec'], 9),
                    self._format_float(row['relative_time_sec'], 9),
                    self._format_float(row['x'], 6),
                    self._format_float(row['y'], 6),
                    self._format_float(row['z'], 6),
                    row['analysis_axis'],
                    self._format_float(row['analysis_value'], 6),
                    self._format_float(row['header_stamp_sec'], 9),
                    self._format_float(row['tf_pub_stamp_sec'], 9),
                    self._format_float(row['rx_stamp_sec'], 9),
                    row['visual_stamp_source'],
                ])

    def _write_distance_sensor_csv(
        self,
        path: str,
        rows: Sequence[Dict[str, object]],
    ):
        """输出测距仪原始 CSV，便于回溯有效性判定."""
        with open(path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file, lineterminator='\n')
            writer.writerow([
                'index',
                'stamp_sec',
                'relative_time_sec',
                'range_m',
                'min_range_m',
                'max_range_m',
                'valid',
                'status',
                'gap_sec_from_prev',
                'gap_exceeds_timeout',
            ])
            for row in rows:
                writer.writerow([
                    row['index'],
                    self._format_float(row['stamp_sec'], 9),
                    self._format_float(row['relative_time_sec'], 9),
                    self._format_float(row['range_m'], 6),
                    self._format_float(row['min_range_m'], 6),
                    self._format_float(row['max_range_m'], 6),
                    str(bool(row['valid'])).lower(),
                    row['status'],
                    self._format_float(row['gap_sec_from_prev'], 6),
                    str(bool(row['gap_exceeds_timeout'])).lower(),
                ])

    def _write_reference_csv(
        self,
        path: str,
        rows: Sequence[Dict[str, object]],
    ):
        """输出参考流 CSV，明确分析使用的高度/位置来源."""
        with open(path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file, lineterminator='\n')
            writer.writerow([
                'index',
                'stamp_sec',
                'relative_time_sec',
                'x',
                'y',
                'z',
                'local_pose_z',
                'distance_sensor_z',
                'local_pose_stamp_sec',
                'distance_sensor_stamp_sec',
                'has_local_pose_match',
                'has_distance_sensor_match',
                'reference_z_source',
                'analysis_axis',
                'analysis_value',
            ])
            for row in rows:
                writer.writerow([
                    row['index'],
                    self._format_float(row['stamp_sec'], 9),
                    self._format_float(row['relative_time_sec'], 9),
                    self._format_float(row['x'], 6),
                    self._format_float(row['y'], 6),
                    self._format_float(row['z'], 6),
                    self._format_float(row['local_pose_z'], 6),
                    self._format_float(row['distance_sensor_z'], 6),
                    self._format_float(row['local_pose_stamp_sec'], 9),
                    self._format_float(row['distance_sensor_stamp_sec'], 9),
                    str(bool(row['has_local_pose_match'])).lower(),
                    str(bool(row['has_distance_sensor_match'])).lower(),
                    row['reference_z_source'],
                    row['analysis_axis'],
                    self._format_float(row['analysis_value'], 6),
                ])

    def _format_float(self, value, precision: int) -> str:
        """统一浮点格式，NaN 与 inf 保持可读."""
        numeric_value = float(value)
        if math.isnan(numeric_value):
            return 'nan'
        if math.isinf(numeric_value):
            return 'inf' if numeric_value > 0.0 else '-inf'
        return f'{numeric_value:.{precision}f}'

    def _analyze_stream(
        self,
        rows: Sequence[Dict[str, object]],
        stream_name: str,
    ) -> Dict[str, object]:
        """对单路参考流执行平滑、求导、碎事件检测与大动作聚合."""
        if not rows:
            return {
                'stream_name': stream_name,
                'times': [],
                'plot_times': [],
                'raw_values': [],
                'smoothed_values': [],
                'slopes': [],
                'raw_events': [],
                'events': [],
                'sample_count': 0,
                'has_enough_samples': False,
            }

        times = [float(row['stamp_sec']) for row in rows]
        plot_times = [
            float(row['relative_time_sec']) if self.plot_relative_time
            else float(row['stamp_sec'])
            for row in rows
        ]
        raw_values = [float(row['analysis_value']) for row in rows]
        smoothed_values = self._smooth_series(
            times,
            raw_values,
            self.smooth_window_sec,
        )
        slopes = self._compute_slopes(times, smoothed_values)
        raw_events = self._detect_events(
            times=times,
            plot_times=plot_times,
            raw_values=raw_values,
            smoothed_values=smoothed_values,
            slopes=slopes,
            stream_name=stream_name,
        )
        events = self._aggregate_major_events(
            times=times,
            plot_times=plot_times,
            raw_values=raw_values,
            smoothed_values=smoothed_values,
            slopes=slopes,
            stream_name=stream_name,
            raw_events=raw_events,
        )

        return {
            'stream_name': stream_name,
            'times': times,
            'plot_times': plot_times,
            'raw_values': raw_values,
            'smoothed_values': smoothed_values,
            'slopes': slopes,
            'raw_events': raw_events,
            'events': events,
            'sample_count': len(rows),
            'has_enough_samples': len(rows) >= self.min_samples,
        }

    def _smooth_series(
        self,
        times: Sequence[float],
        values: Sequence[float],
        window_sec: float,
    ) -> List[float]:
        """按时间窗做中心滑动平均，降低噪声同时尽量减小相位偏移."""
        if len(values) <= 1 or window_sec <= 0.0:
            return list(values)

        half_window_sec = window_sec / 2.0
        prefix_sum = [0.0]
        for value in values:
            prefix_sum.append(prefix_sum[-1] + value)

        smoothed_values = []
        left = 0
        right = -1
        sample_count = len(values)

        for current_time in times:
            while (
                left < sample_count and
                times[left] < current_time - half_window_sec
            ):
                left += 1
            while (
                right + 1 < sample_count and
                times[right + 1] <= current_time + half_window_sec
            ):
                right += 1

            if right < left:
                smoothed_values.append(values[min(left, sample_count - 1)])
                continue

            window_size = right - left + 1
            window_sum = prefix_sum[right + 1] - prefix_sum[left]
            smoothed_values.append(window_sum / window_size)

        return smoothed_values

    def _compute_slopes(
        self,
        times: Sequence[float],
        values: Sequence[float],
    ) -> List[float]:
        """对平滑序列做一阶差分，得到速度近似值."""
        if not values:
            return []

        slopes = [0.0]
        for index in range(1, len(values)):
            delta_time_sec = times[index] - times[index - 1]
            if delta_time_sec <= 1e-9:
                slopes.append(0.0)
                continue
            slopes.append(
                (values[index] - values[index - 1]) / delta_time_sec
            )
        return slopes

    def _detect_events(
        self,
        *,
        times: Sequence[float],
        plot_times: Sequence[float],
        raw_values: Sequence[float],
        smoothed_values: Sequence[float],
        slopes: Sequence[float],
        stream_name: str,
    ) -> List[Dict[str, float]]:
        """根据速度阈值和间隙合并规则识别动作事件."""
        if len(times) < 2:
            return []

        active_segments = []
        start_index = None
        previous_active_index = None

        for index in range(1, len(times)):
            if math.fabs(slopes[index]) < self.slope_threshold:
                continue

            if start_index is None:
                start_index = index
            elif (
                previous_active_index is not None and
                times[index] - times[previous_active_index] > self.merge_gap_sec
            ):
                active_segments.append((start_index, previous_active_index))
                start_index = index

            previous_active_index = index

        if start_index is not None and previous_active_index is not None:
            active_segments.append((start_index, previous_active_index))

        events = []
        for event_index, (segment_start, segment_end) in enumerate(
            active_segments,
            start=1,
        ):
            ramp_start_index = max(0, segment_start - 1)
            segment_slopes = slopes[segment_start:segment_end + 1]
            mean_slope = mean(segment_slopes)
            max_abs_slope = max(math.fabs(value) for value in segment_slopes)
            direction = 'rise' if mean_slope >= 0.0 else 'fall'

            events.append({
                'stream': stream_name,
                'event_index': event_index,
                'direction': direction,
                'start_index': ramp_start_index,
                'end_index': segment_end,
                'start_stamp_sec': times[ramp_start_index],
                'end_stamp_sec': times[segment_end],
                'start_plot_time_sec': plot_times[ramp_start_index],
                'end_plot_time_sec': plot_times[segment_end],
                'start_value': raw_values[ramp_start_index],
                'end_value': raw_values[segment_end],
                'start_smoothed_value': smoothed_values[ramp_start_index],
                'end_smoothed_value': smoothed_values[segment_end],
                'mean_slope': mean_slope,
                'max_abs_slope': max_abs_slope,
                'sample_count': segment_end - ramp_start_index + 1,
            })

        return events

    def _aggregate_major_events(
        self,
        *,
        times: Sequence[float],
        plot_times: Sequence[float],
        raw_values: Sequence[float],
        smoothed_values: Sequence[float],
        slopes: Sequence[float],
        stream_name: str,
        raw_events: Sequence[Dict[str, object]],
    ) -> List[Dict[str, object]]:
        """把低层碎事件聚合成仅用于最终结论的大动作边沿."""
        if not raw_events:
            return []

        grouped_fragments: List[List[Dict[str, object]]] = []
        current_group: List[Dict[str, object]] = [dict(raw_events[0])]

        for raw_event in raw_events[1:]:
            previous_event = current_group[-1]
            same_direction = (
                previous_event['direction'] == raw_event['direction']
            )
            gap_sec = (
                float(raw_event['start_stamp_sec']) -
                float(previous_event['end_stamp_sec'])
            )
            if same_direction and gap_sec <= self.major_edge_merge_gap_sec:
                current_group.append(dict(raw_event))
                continue

            grouped_fragments.append(current_group)
            current_group = [dict(raw_event)]

        grouped_fragments.append(current_group)

        major_events = []
        for fragment_group in grouped_fragments:
            major_event = self._build_major_event(
                times=times,
                plot_times=plot_times,
                raw_values=raw_values,
                smoothed_values=smoothed_values,
                slopes=slopes,
                stream_name=stream_name,
                fragment_group=fragment_group,
            )
            if major_event is None:
                continue

            major_event['event_index'] = len(major_events) + 1
            major_events.append(major_event)

        return major_events

    def _build_major_event(
        self,
        *,
        times: Sequence[float],
        plot_times: Sequence[float],
        raw_values: Sequence[float],
        smoothed_values: Sequence[float],
        slopes: Sequence[float],
        stream_name: str,
        fragment_group: Sequence[Dict[str, object]],
    ) -> Optional[Dict[str, object]]:
        """根据同方向碎事件组构造一个可用于匹配的大动作边沿."""
        if not fragment_group:
            return None

        start_index = int(fragment_group[0]['start_index'])
        end_index = int(fragment_group[-1]['end_index'])
        if end_index < start_index:
            return None

        start_smoothed_value = float(smoothed_values[start_index])
        end_smoothed_value = float(smoothed_values[end_index])
        signed_amplitude = end_smoothed_value - start_smoothed_value
        abs_amplitude = math.fabs(signed_amplitude)
        if abs_amplitude < self.major_edge_min_amplitude:
            return None

        # 大动作的统计仍基于聚合后的整段斜率，避免只看首个碎片。
        slope_start_index = min(start_index + 1, end_index)
        segment_slopes = list(slopes[slope_start_index:end_index + 1])
        if not segment_slopes:
            segment_slopes = [0.0]

        mid_value = (
            start_smoothed_value + 0.5 * signed_amplitude
        )
        midpoint = self._estimate_midpoint_crossing(
            start_index=start_index,
            end_index=end_index,
            times=times,
            plot_times=plot_times,
            smoothed_values=smoothed_values,
            mid_value=mid_value,
        )

        return {
            'stream': stream_name,
            'direction': str(fragment_group[0]['direction']),
            'start_index': start_index,
            'end_index': end_index,
            'start_stamp_sec': float(times[start_index]),
            'end_stamp_sec': float(times[end_index]),
            'start_plot_time_sec': float(plot_times[start_index]),
            'end_plot_time_sec': float(plot_times[end_index]),
            'start_value': float(raw_values[start_index]),
            'end_value': float(raw_values[end_index]),
            'start_smoothed_value': start_smoothed_value,
            'end_smoothed_value': end_smoothed_value,
            'signed_amplitude': signed_amplitude,
            'abs_amplitude': abs_amplitude,
            'mid_value': mid_value,
            'mid_plot_value': midpoint['mid_plot_value'],
            'mid_stamp_sec': midpoint['mid_stamp_sec'],
            'mid_plot_time_sec': midpoint['mid_plot_time_sec'],
            'mid_detection_method': midpoint['mid_detection_method'],
            'mean_slope': mean(segment_slopes),
            'max_abs_slope': max(
                math.fabs(value) for value in segment_slopes
            ),
            'sample_count': end_index - start_index + 1,
            'fragment_count': len(fragment_group),
        }

    def _estimate_midpoint_crossing(
        self,
        *,
        start_index: int,
        end_index: int,
        times: Sequence[float],
        plot_times: Sequence[float],
        smoothed_values: Sequence[float],
        mid_value: float,
    ) -> Dict[str, object]:
        """估计大动作边沿 50% 中点的穿越时刻."""
        if end_index <= start_index:
            return {
                'mid_stamp_sec': float(times[start_index]),
                'mid_plot_time_sec': float(plot_times[start_index]),
                'mid_plot_value': float(smoothed_values[start_index]),
                'mid_detection_method': 'single_sample',
            }

        for index in range(start_index + 1, end_index + 1):
            previous_value = float(smoothed_values[index - 1])
            current_value = float(smoothed_values[index])
            previous_time = float(times[index - 1])
            current_time = float(times[index])
            previous_plot_time = float(plot_times[index - 1])
            current_plot_time = float(plot_times[index])

            if math.isclose(previous_value, mid_value, abs_tol=1e-9):
                return {
                    'mid_stamp_sec': previous_time,
                    'mid_plot_time_sec': previous_plot_time,
                    'mid_plot_value': previous_value,
                    'mid_detection_method': 'exact_sample',
                }
            if math.isclose(current_value, mid_value, abs_tol=1e-9):
                return {
                    'mid_stamp_sec': current_time,
                    'mid_plot_time_sec': current_plot_time,
                    'mid_plot_value': current_value,
                    'mid_detection_method': 'exact_sample',
                }

            crossed_midpoint = (
                min(previous_value, current_value) <= mid_value <=
                max(previous_value, current_value)
            )
            if not crossed_midpoint:
                continue

            value_delta = current_value - previous_value
            if math.isclose(value_delta, 0.0, abs_tol=1e-12):
                ratio = 0.0
            else:
                ratio = (mid_value - previous_value) / value_delta

            return {
                'mid_stamp_sec': (
                    previous_time + ratio * (current_time - previous_time)
                ),
                'mid_plot_time_sec': (
                    previous_plot_time +
                    ratio * (current_plot_time - previous_plot_time)
                ),
                'mid_plot_value': mid_value,
                'mid_detection_method': 'interpolated',
            }

        nearest_index = min(
            range(start_index, end_index + 1),
            key=lambda index: math.fabs(
                float(smoothed_values[index]) - mid_value
            ),
        )
        return {
            'mid_stamp_sec': float(times[nearest_index]),
            'mid_plot_time_sec': float(plot_times[nearest_index]),
            'mid_plot_value': float(smoothed_values[nearest_index]),
            'mid_detection_method': 'nearest_sample',
        }

    def _write_events_csv(
        self,
        path: str,
        events: Sequence[Dict[str, object]],
    ):
        """输出大动作边沿 CSV，而不是低层碎事件结果."""
        with open(path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file, lineterminator='\n')
            writer.writerow([
                'stream',
                'event_index',
                'direction',
                'start_index',
                'end_index',
                'start_stamp_sec',
                'end_stamp_sec',
                'start_plot_time_sec',
                'mid_stamp_sec',
                'mid_plot_time_sec',
                'end_plot_time_sec',
                'start_value',
                'end_value',
                'start_smoothed_value',
                'mid_value',
                'mid_plot_value',
                'end_smoothed_value',
                'signed_amplitude',
                'abs_amplitude',
                'mean_slope',
                'max_abs_slope',
                'sample_count',
                'fragment_count',
                'mid_detection_method',
            ])
            for event in events:
                writer.writerow([
                    event['stream'],
                    event['event_index'],
                    event['direction'],
                    event['start_index'],
                    event['end_index'],
                    self._format_float(event['start_stamp_sec'], 9),
                    self._format_float(event['end_stamp_sec'], 9),
                    self._format_float(event['start_plot_time_sec'], 9),
                    self._format_float(event['mid_stamp_sec'], 9),
                    self._format_float(event['mid_plot_time_sec'], 9),
                    self._format_float(event['end_plot_time_sec'], 9),
                    self._format_float(event['start_value'], 6),
                    self._format_float(event['end_value'], 6),
                    self._format_float(event['start_smoothed_value'], 6),
                    self._format_float(event['mid_value'], 6),
                    self._format_float(event['mid_plot_value'], 6),
                    self._format_float(event['end_smoothed_value'], 6),
                    self._format_float(event['signed_amplitude'], 6),
                    self._format_float(event['abs_amplitude'], 6),
                    self._format_float(event['mean_slope'], 6),
                    self._format_float(event['max_abs_slope'], 6),
                    event['sample_count'],
                    event['fragment_count'],
                    event['mid_detection_method'],
                ])

    def _match_events(
        self,
        reference_events: Sequence[Dict[str, object]],
        visual_events: Sequence[Dict[str, object]],
    ) -> List[Dict[str, object]]:
        """按方向拆分并顺序匹配大动作边沿，避免跨周期误配."""
        matches = []
        for direction in ('rise', 'fall'):
            reference_direction_events = [
                event for event in reference_events
                if event['direction'] == direction
            ]
            visual_direction_events = [
                event for event in visual_events
                if event['direction'] == direction
            ]

            reference_index = 0
            visual_index = 0
            while (
                reference_index < len(reference_direction_events) and
                visual_index < len(visual_direction_events)
            ):
                reference_event = reference_direction_events[reference_index]
                visual_event = visual_direction_events[visual_index]
                delay_sec = (
                    float(visual_event['mid_stamp_sec']) -
                    float(reference_event['mid_stamp_sec'])
                )

                if (
                    math.fabs(delay_sec) <=
                    self.major_edge_max_match_offset_sec
                ):
                    matches.append({
                        'visual_event_index': visual_event['event_index'],
                        'reference_event_index': reference_event['event_index'],
                        'visual_direction': visual_event['direction'],
                        'reference_direction': reference_event['direction'],
                        'visual_mid_stamp_sec': visual_event['mid_stamp_sec'],
                        'reference_mid_stamp_sec': reference_event[
                            'mid_stamp_sec'
                        ],
                        'visual_mid_plot_time_sec': visual_event[
                            'mid_plot_time_sec'
                        ],
                        'reference_mid_plot_time_sec': reference_event[
                            'mid_plot_time_sec'
                        ],
                        'visual_abs_amplitude': visual_event['abs_amplitude'],
                        'reference_abs_amplitude': reference_event[
                            'abs_amplitude'
                        ],
                        'delay_sec': delay_sec,
                    })
                    reference_index += 1
                    visual_index += 1
                    continue

                if delay_sec < -self.major_edge_max_match_offset_sec:
                    visual_index += 1
                else:
                    reference_index += 1

        return sorted(
            matches,
            key=lambda match: float(match['reference_mid_stamp_sec']),
        )

    def _write_matches_csv(
        self,
        path: str,
        matches: Sequence[Dict[str, object]],
    ):
        """输出大动作边沿的顺序匹配结果."""
        with open(path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file, lineterminator='\n')
            writer.writerow([
                'visual_event_index',
                'reference_event_index',
                'visual_direction',
                'reference_direction',
                'visual_mid_stamp_sec',
                'reference_mid_stamp_sec',
                'visual_mid_plot_time_sec',
                'reference_mid_plot_time_sec',
                'visual_abs_amplitude',
                'reference_abs_amplitude',
                'delay_sec',
            ])
            for match in matches:
                writer.writerow([
                    match['visual_event_index'],
                    match['reference_event_index'],
                    match['visual_direction'],
                    match['reference_direction'],
                    self._format_float(match['visual_mid_stamp_sec'], 9),
                    self._format_float(match['reference_mid_stamp_sec'], 9),
                    self._format_float(
                        match['visual_mid_plot_time_sec'],
                        9,
                    ),
                    self._format_float(
                        match['reference_mid_plot_time_sec'],
                        9,
                    ),
                    self._format_float(match['visual_abs_amplitude'], 6),
                    self._format_float(match['reference_abs_amplitude'], 6),
                    self._format_float(match['delay_sec'], 6),
                ])

    def _count_events_by_direction(
        self,
        events: Sequence[Dict[str, object]],
    ) -> Dict[str, int]:
        """统计大动作边沿在 rise/fall 两个方向上的数量."""
        counts = {'rise': 0, 'fall': 0}
        for event in events:
            direction = str(event.get('direction', ''))
            if direction in counts:
                counts[direction] += 1
        return counts

    def _count_matches_by_direction(
        self,
        matches: Sequence[Dict[str, object]],
    ) -> Dict[str, int]:
        """统计匹配成功的大动作边沿在 rise/fall 两个方向上的数量."""
        counts = {'rise': 0, 'fall': 0}
        for match in matches:
            direction = str(match.get('reference_direction', ''))
            if direction in counts:
                counts[direction] += 1
        return counts

    def _build_summary(
        self,
        *,
        local_pose_rows: Sequence[Dict[str, object]],
        distance_sensor_rows: Sequence[Dict[str, object]],
        reference_rows: Sequence[Dict[str, object]],
        visual_rows: Sequence[Dict[str, object]],
        reference_analysis: Dict[str, object],
        visual_analysis: Dict[str, object],
        matches: Sequence[Dict[str, object]],
        local_pose_csv_path: str,
        distance_sensor_csv_path: str,
        reference_csv_path: str,
        visual_csv_path: str,
        reference_events_csv_path: str,
        visual_events_csv_path: str,
        matches_csv_path: str,
        plot_path: str,
        report_path: str,
    ) -> Dict[str, object]:
        """汇总报告所需信息，并给出最终状态."""
        mean_delay_sec = None
        if matches:
            mean_delay_sec = mean(match['delay_sec'] for match in matches)

        reference_direction_counts = self._count_events_by_direction(
            reference_analysis['events']
        )
        visual_direction_counts = self._count_events_by_direction(
            visual_analysis['events']
        )
        matched_direction_counts = self._count_matches_by_direction(matches)

        status = '已完成延迟分析。'
        if (
            len(reference_rows) < self.min_samples or
            len(visual_rows) < self.min_samples
        ):
            status = (
                '样本不足：reference 或 visual 样本数低于 min_samples，'
                '因此不输出强结论。'
            )
        elif not matches:
            status = '未匹配到足够的大动作边沿，无法估计延迟。'

        return {
            'generated_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'analysis_axis': self.analysis_axis,
            'min_samples': self.min_samples,
            'smooth_window_sec': self.smooth_window_sec,
            'slope_threshold': self.slope_threshold,
            'merge_gap_sec': self.merge_gap_sec,
            'major_edge_merge_gap_sec': self.major_edge_merge_gap_sec,
            'major_edge_min_amplitude': self.major_edge_min_amplitude,
            'major_edge_max_match_offset_sec': (
                self.major_edge_max_match_offset_sec
            ),
            'plot_relative_time': self.plot_relative_time,
            'auto_report': self.auto_report,
            'distance_sensor_timeout_sec': self.distance_sensor_timeout_sec,
            'local_pose_topic': self.local_pose_topic,
            'distance_sensor_topic': self.distance_sensor_topic,
            'visual_pose_topic': self.visual_pose_topic,
            'reference_description': self._get_reference_description(),
            'reference_z_source_text': self._get_reference_z_source_text(),
            'timestamp_policy': self._get_timestamp_policy(),
            'visual_stamp_policy': self._get_visual_stamp_policy(),
            'local_pose_sample_count': len(local_pose_rows),
            'distance_sensor_sample_count': len(distance_sensor_rows),
            'distance_sensor_valid_sample_count': sum(
                1 for row in distance_sensor_rows if row['valid']
            ),
            'reference_sample_count': len(reference_rows),
            'visual_sample_count': len(visual_rows),
            'visual_tf_pub_stamp_sample_count': sum(
                1 for row in visual_rows
                if row['visual_stamp_source'] == 'tf_pub_stamp'
            ),
            'visual_fallback_stamp_sample_count': sum(
                1 for row in visual_rows
                if row['visual_stamp_source'] != 'tf_pub_stamp'
            ),
            'reference_event_count': len(reference_analysis['events']),
            'visual_event_count': len(visual_analysis['events']),
            'reference_rise_event_count': reference_direction_counts['rise'],
            'reference_fall_event_count': reference_direction_counts['fall'],
            'visual_rise_event_count': visual_direction_counts['rise'],
            'visual_fall_event_count': visual_direction_counts['fall'],
            'matched_rise_event_count': matched_direction_counts['rise'],
            'matched_fall_event_count': matched_direction_counts['fall'],
            'reference_unmatched_rise_event_count': (
                reference_direction_counts['rise'] -
                matched_direction_counts['rise']
            ),
            'reference_unmatched_fall_event_count': (
                reference_direction_counts['fall'] -
                matched_direction_counts['fall']
            ),
            'visual_unmatched_rise_event_count': (
                visual_direction_counts['rise'] -
                matched_direction_counts['rise']
            ),
            'visual_unmatched_fall_event_count': (
                visual_direction_counts['fall'] -
                matched_direction_counts['fall']
            ),
            'matches': list(matches),
            'mean_delay_sec': mean_delay_sec,
            'status': status,
            'reference_events': list(reference_analysis['events']),
            'visual_events': list(visual_analysis['events']),
            'local_pose_csv_path': local_pose_csv_path,
            'distance_sensor_csv_path': distance_sensor_csv_path,
            'reference_csv_path': reference_csv_path,
            'visual_csv_path': visual_csv_path,
            'reference_events_csv_path': reference_events_csv_path,
            'visual_events_csv_path': visual_events_csv_path,
            'matches_csv_path': matches_csv_path,
            'plot_path': plot_path,
            'report_path': report_path,
        }

    def _plot_comparison(
        self,
        *,
        reference_rows: Sequence[Dict[str, object]],
        visual_rows: Sequence[Dict[str, object]],
        reference_analysis: Dict[str, object],
        visual_analysis: Dict[str, object],
        summary: Dict[str, object],
        plot_path: str,
    ):
        """绘制参考流与视觉流的原始曲线、平滑曲线和大动作标记."""
        figure, axes = self._plt.subplots(
            2,
            1,
            figsize=(12, 8),
            sharex=True,
            constrained_layout=True,
        )
        value_axis, slope_axis = axes

        reference_plot_times = reference_analysis['plot_times']
        visual_plot_times = visual_analysis['plot_times']
        value_has_lines = False
        slope_has_lines = False

        if reference_rows:
            value_axis.plot(
                reference_plot_times,
                reference_analysis['raw_values'],
                label='reference raw',
                color='tab:blue',
                linewidth=1.2,
                alpha=0.7,
            )
            value_has_lines = True
            value_axis.plot(
                reference_plot_times,
                reference_analysis['smoothed_values'],
                label='reference smooth',
                color='tab:blue',
                linewidth=2.0,
            )
            slope_axis.plot(
                reference_plot_times,
                reference_analysis['slopes'],
                label='reference slope',
                color='tab:blue',
                linewidth=1.5,
            )
            slope_has_lines = True

        if visual_rows:
            value_axis.plot(
                visual_plot_times,
                visual_analysis['raw_values'],
                label='visual raw',
                color='tab:orange',
                linewidth=1.2,
                alpha=0.7,
            )
            value_has_lines = True
            value_axis.plot(
                visual_plot_times,
                visual_analysis['smoothed_values'],
                label='visual smooth',
                color='tab:orange',
                linewidth=2.0,
            )
            slope_axis.plot(
                visual_plot_times,
                visual_analysis['slopes'],
                label='visual slope',
                color='tab:orange',
                linewidth=1.5,
            )
            slope_has_lines = True

        reference_events = reference_analysis['events']
        visual_events = visual_analysis['events']
        if reference_events:
            value_axis.scatter(
                [event['mid_plot_time_sec'] for event in reference_events],
                [event['mid_plot_value'] for event in reference_events],
                label='reference major edge mid',
                color='tab:blue',
                marker='o',
                s=32,
                zorder=3,
            )
        if visual_events:
            value_axis.scatter(
                [event['mid_plot_time_sec'] for event in visual_events],
                [event['mid_plot_value'] for event in visual_events],
                label='visual major edge mid',
                color='tab:orange',
                marker='s',
                s=32,
                zorder=3,
            )

        slope_axis.axhline(
            self.slope_threshold,
            color='tab:red',
            linestyle=':',
            linewidth=1.2,
            label='threshold',
        )
        slope_axis.axhline(
            -self.slope_threshold,
            color='tab:red',
            linestyle=':',
            linewidth=1.2,
        )

        value_axis.set_ylabel(f'{self.analysis_axis} (m)')
        slope_axis.set_ylabel(f'd{self.analysis_axis}/dt (m/s)')
        slope_axis.set_xlabel(
            't (s)' if self.plot_relative_time else 'stamp_sec (s)'
        )

        if summary['mean_delay_sec'] is None:
            title_suffix = 'insufficient_samples_or_no_matches'
        else:
            title_suffix = (
                f'mean_delay={summary["mean_delay_sec"]:.4f} s'
            )
        value_axis.set_title(
            'Pose Delay Comparison | '
            f'{summary["reference_z_source_text"]} | '
            f'{title_suffix}'
        )

        if not value_has_lines:
            value_axis.text(
                0.5,
                0.5,
                'No reference/visual samples recorded',
                transform=value_axis.transAxes,
                ha='center',
                va='center',
            )
        if not slope_has_lines:
            slope_axis.text(
                0.5,
                0.5,
                'No slope samples recorded',
                transform=slope_axis.transAxes,
                ha='center',
                va='center',
            )

        value_axis.grid(True, linestyle='--', alpha=0.3)
        slope_axis.grid(True, linestyle='--', alpha=0.3)
        if value_has_lines:
            value_axis.legend(loc='best')
        if slope_has_lines:
            slope_axis.legend(loc='best')

        figure.savefig(plot_path, dpi=150)
        self._plt.close(figure)

    def _get_reference_description(self) -> str:
        """返回当前参考流的整体口径说明."""
        if self.analysis_axis == 'z':
            return (
                'reference(z)=distance sensor，'
                'reference.x/reference.y 取最近且不晚于测距时刻的 local pose'
            )
        return (
            f'reference({self.analysis_axis})=local pose，'
            'distance sensor 仅作为附加列记录'
        )

    def _get_reference_z_source_text(self) -> str:
        """返回图与报告中用于说明 z 来源的简短文本."""
        if self.analysis_axis == 'z':
            return 'reference(z)=distance sensor'
        return 'reference(x/y)=local pose'

    def _get_timestamp_policy(self) -> str:
        """返回三路数据参与分析时采用的时间戳口径."""
        return (
            'reference(z)=Range.header.stamp；'
            'visual=/debug/aruco_pose.tf_pub_stamp；'
            'local pose=PoseStamped.header.stamp'
        )

    def _get_visual_stamp_policy(self) -> str:
        """返回视觉流 stamp_sec 的来源说明."""
        return (
            '/debug/aruco_pose.tf_pub_stamp；'
            'tf_pub_stamp<=0 时 fallback 到 analyzer rx_stamp_sec'
        )

    def _write_report(self, path: str, summary: Dict[str, object]):
        """生成 Markdown 实验报告."""
        lines = [
            '# Pose Delay Analysis Report',
            '',
            f'- 生成时间：{summary["generated_at"]}',
            f'- 分析轴：`{summary["analysis_axis"]}`',
            f'- reference 口径：{summary["reference_description"]}',
            f'- 时间戳口径：{summary["timestamp_policy"]}',
            f'- local pose 话题：`{summary["local_pose_topic"]}`',
            f'- distance sensor 话题：`{summary["distance_sensor_topic"]}`',
            f'- visual pose 话题：`{summary["visual_pose_topic"]}`',
            f'- 状态：{summary["status"]}',
            '',
            '## 原始数据与输出文件',
            '',
            f'- local pose CSV：`{summary["local_pose_csv_path"]}`',
            f'- distance sensor CSV：`{summary["distance_sensor_csv_path"]}`',
            f'- reference CSV：`{summary["reference_csv_path"]}`',
            f'- visual CSV：`{summary["visual_csv_path"]}`',
            f'- reference events CSV：`{summary["reference_events_csv_path"]}`',
            f'- visual events CSV：`{summary["visual_events_csv_path"]}`',
            f'- matches CSV：`{summary["matches_csv_path"]}`',
            f'- plot：`{summary["plot_path"]}`',
        ]

        if self.auto_report:
            lines.append(f'- report：`{summary["report_path"]}`')

        lines.extend([
            '',
            '## 检测参数',
            '',
            '| 参数 | 数值 |',
            '| --- | --- |',
            f'| min_samples | {summary["min_samples"]} |',
            f'| smooth_window_sec | {summary["smooth_window_sec"]:.3f} |',
            f'| slope_threshold | {summary["slope_threshold"]:.3f} |',
            f'| merge_gap_sec | {summary["merge_gap_sec"]:.3f} |',
            (
                '| major_edge_merge_gap_sec | '
                f'{summary["major_edge_merge_gap_sec"]:.3f} |'
            ),
            (
                '| major_edge_min_amplitude | '
                f'{summary["major_edge_min_amplitude"]:.3f} |'
            ),
            (
                '| major_edge_max_match_offset_sec | '
                f'{summary["major_edge_max_match_offset_sec"]:.3f} |'
            ),
            (
                '| distance_sensor_timeout_sec | '
                f'{summary["distance_sensor_timeout_sec"]:.3f} |'
            ),
            f'| visual_stamp_source | {summary["visual_stamp_policy"]} |',
            f'| plot_relative_time | {summary["plot_relative_time"]} |',
            '',
            '## 样本统计',
            '',
            '| 数据流 | 样本数 | 说明 |',
            '| --- | --- | --- |',
            (
                f'| local pose raw | {summary["local_pose_sample_count"]} | '
                '原始 `/mavros/local_position/pose` |'
            ),
            (
                '| distance sensor raw | '
                f'{summary["distance_sensor_sample_count"]} | '
                '原始 `Range` 全量样本 |'
            ),
            (
                '| distance sensor valid | '
                f'{summary["distance_sensor_valid_sample_count"]} | '
                'finite 且未超过 max_range 的样本 |'
            ),
            (
                f'| reference | {summary["reference_sample_count"]} | '
                f'{summary["reference_z_source_text"]} |'
            ),
            (
                f'| visual | {summary["visual_sample_count"]} | '
                '原始 `/debug/aruco_pose`；'
                'stamp_sec 使用 tf_pub_stamp='
                f'{summary["visual_tf_pub_stamp_sample_count"]}，'
                'fallback_rx_stamp='
                f'{summary["visual_fallback_stamp_sample_count"]} |'
            ),
            '',
            '## 事件统计',
            '',
            '| 数据流 | direction | 检测到的大动作边沿 | 匹配成功 | 未匹配 |',
            '| --- | --- | --- | --- | --- |',
            (
                '| reference | rise | '
                f'{summary["reference_rise_event_count"]} | '
                f'{summary["matched_rise_event_count"]} | '
                f'{summary["reference_unmatched_rise_event_count"]} |'
            ),
            (
                '| reference | fall | '
                f'{summary["reference_fall_event_count"]} | '
                f'{summary["matched_fall_event_count"]} | '
                f'{summary["reference_unmatched_fall_event_count"]} |'
            ),
            (
                '| visual | rise | '
                f'{summary["visual_rise_event_count"]} | '
                f'{summary["matched_rise_event_count"]} | '
                f'{summary["visual_unmatched_rise_event_count"]} |'
            ),
            (
                '| visual | fall | '
                f'{summary["visual_fall_event_count"]} | '
                f'{summary["matched_fall_event_count"]} | '
                f'{summary["visual_unmatched_fall_event_count"]} |'
            ),
            '',
            '## Reference 大动作边沿',
            '',
        ])

        lines.extend(self._events_table_lines(summary['reference_events']))
        lines.extend([
            '',
            '## Visual 大动作边沿',
            '',
        ])
        lines.extend(self._events_table_lines(summary['visual_events']))
        lines.extend([
            '',
            '## 大动作边沿延迟匹配结果',
            '',
        ])
        lines.extend(self._matches_table_lines(summary['matches']))
        lines.extend([
            '',
            '## 结论',
            '',
        ])

        if summary['mean_delay_sec'] is None:
            lines.append(f'- {summary["status"]}')
        else:
            lines.append(
                f'- 大动作边沿平均延迟：`{summary["mean_delay_sec"]:.6f} s`'
            )
            lines.append(
                '- 计算方式：'
                '`delay_sec = visual_mid_stamp_sec - '
                'reference_mid_stamp_sec`'
            )
            lines.append(
                '- 单次延迟以大动作边沿的 50% 中点穿越时刻为准。'
            )

        with open(path, 'w', encoding='utf-8') as report_file:
            report_file.write('\n'.join(lines) + '\n')

    def _events_table_lines(
        self,
        events: Sequence[Dict[str, object]],
    ) -> List[str]:
        """把大动作边沿列表转成 Markdown 表格."""
        if not events:
            return ['- 无大动作边沿。']

        lines = [
            '| event | direction | start_t(s) | mid_t(s) | end_t(s) | '
            'start_smooth(m) | end_smooth(m) | abs_amplitude(m) | '
            'fragment_count |',
            '| --- | --- | --- | --- | --- | --- | --- | --- | --- |',
        ]
        for event in events:
            lines.append(
                f'| {event["event_index"]} | {event["direction"]} | '
                f'{event["start_plot_time_sec"]:.6f} | '
                f'{event["mid_plot_time_sec"]:.6f} | '
                f'{event["end_plot_time_sec"]:.6f} | '
                f'{event["start_smoothed_value"]:.6f} | '
                f'{event["end_smoothed_value"]:.6f} | '
                f'{event["abs_amplitude"]:.6f} | '
                f'{event["fragment_count"]} |'
            )
        return lines

    def _matches_table_lines(
        self,
        matches: Sequence[Dict[str, object]],
    ) -> List[str]:
        """把大动作边沿匹配列表转成 Markdown 表格."""
        if not matches:
            return ['- 无有效的大动作边沿匹配。']

        lines = [
            '| visual_event | reference_event | visual_direction | '
            'reference_direction | visual_mid_t(s) | '
            'reference_mid_t(s) | visual_abs_amplitude(m) | '
            'reference_abs_amplitude(m) | delay_sec |',
            '| --- | --- | --- | --- | --- | --- | --- | --- | --- |',
        ]
        for match in matches:
            lines.append(
                f'| {match["visual_event_index"]} | '
                f'{match["reference_event_index"]} | '
                f'{match["visual_direction"]} | '
                f'{match["reference_direction"]} | '
                f'{match["visual_mid_plot_time_sec"]:.6f} | '
                f'{match["reference_mid_plot_time_sec"]:.6f} | '
                f'{match["visual_abs_amplitude"]:.6f} | '
                f'{match["reference_abs_amplitude"]:.6f} | '
                f'{match["delay_sec"]:.6f} |'
            )
        return lines


def main(args=None):
    """节点入口，确保正常退出时一定触发分析."""
    rclpy.init(args=args)
    node = PoseDelayAnalyzerNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        previous_sigint_handler = signal.getsignal(signal.SIGINT)
        previous_sigterm_handler = signal.getsignal(signal.SIGTERM)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            node.finalize_analysis()
        finally:
            signal.signal(signal.SIGINT, previous_sigint_handler)
            signal.signal(signal.SIGTERM, previous_sigterm_handler)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
