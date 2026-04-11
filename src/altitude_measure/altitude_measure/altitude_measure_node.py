#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime
from typing import Callable

import rclpy
from mavros_msgs.msg import Altitude
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, Range


class AltitudeMeasureNode(Node):
    """记录 distance sensor / GPS / barometer 三路高度到独立 CSV。"""

    def __init__(self) -> None:
        super().__init__('altitude_measure_node')

        # ===== 输入话题参数 =====
        # distance_sensor: 通常来自 /mavros/distance_sensor/*
        self.declare_parameter('distance_sensor_topic', '/mavros/hrlv_ez4_pub')
        # gps: 通常来自 /mavros/global_position/global，使用 NavSatFix.altitude
        self.declare_parameter('gps_topic', '/mavros/global_position/global')
        # barometer: 通常来自 /mavros/altitude
        self.declare_parameter('barometer_topic', '/mavros/altitude')

        # ===== 输出参数 =====
        self.declare_parameter('output_dir', '/home/zjh/project/rasip_pi_ws/log/altitude_measure')
        self.declare_parameter('file_prefix', 'altitude_measure')
        # 定期 flush 的周期，避免频繁落盘导致 I/O 压力过大。
        self.declare_parameter('flush_interval_sec', 1.0)

        # ===== 气压高度字段选择 =====
        # 可选: monotonic / amsl / local / relative / terrain / bottom_clearance
        self.declare_parameter('barometer_field', 'amsl')

        self.distance_sensor_topic = self.get_parameter('distance_sensor_topic').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.barometer_topic = self.get_parameter('barometer_topic').value

        self.output_dir = self.get_parameter('output_dir').value
        self.file_prefix = self.get_parameter('file_prefix').value
        self.flush_interval_sec = float(self.get_parameter('flush_interval_sec').value)

        requested_baro_field = str(self.get_parameter('barometer_field').value)
        self.barometer_field = self._resolve_barometer_field(requested_baro_field)

        # ===== 准备输出文件 =====
        os.makedirs(self.output_dir, exist_ok=True)
        now_text = datetime.now().strftime('%Y%m%d_%H%M%S')
        # 每次启动创建单独运行目录，把本次三路 CSV 放在一起，便于归档管理。
        self.run_output_dir = os.path.join(self.output_dir, f'{self.file_prefix}_{now_text}')
        os.makedirs(self.run_output_dir, exist_ok=True)

        self.distance_csv_path = os.path.join(
            self.run_output_dir,
            'distance_sensor.csv',
        )
        self.gps_csv_path = os.path.join(
            self.run_output_dir,
            'gps_altitude.csv',
        )
        self.barometer_csv_path = os.path.join(
            self.run_output_dir,
            f'barometer_{self.barometer_field}.csv',
        )

        # 强制 LF 换行，避免跨平台打开 CSV 时出现空行问题。
        self.distance_csv_file = open(self.distance_csv_path, 'w', newline='', encoding='utf-8')
        self.gps_csv_file = open(self.gps_csv_path, 'w', newline='', encoding='utf-8')
        self.barometer_csv_file = open(self.barometer_csv_path, 'w', newline='', encoding='utf-8')

        self.distance_writer = csv.writer(self.distance_csv_file, lineterminator='\n')
        self.gps_writer = csv.writer(self.gps_csv_file, lineterminator='\n')
        self.barometer_writer = csv.writer(self.barometer_csv_file, lineterminator='\n')

        # 每个 CSV 仅保留“时间戳 + 高度值”两列，方便后续直接画图比较。
        self.distance_writer.writerow(['timestamp_sec', 'distance_m'])
        self.gps_writer.writerow(['timestamp_sec', 'gps_altitude_m'])
        self.barometer_writer.writerow(['timestamp_sec', f'barometer_{self.barometer_field}_m'])

        self.last_flush_time = self.get_clock().now()

        # MAVROS 多数传感话题是 BEST_EFFORT，订阅端必须匹配，避免“收不到数据”。
        mavros_best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            Range,
            self.distance_sensor_topic,
            self._on_distance_sensor,
            mavros_best_effort_qos,
        )
        self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self._on_gps,
            mavros_best_effort_qos,
        )
        self.create_subscription(
            Altitude,
            self.barometer_topic,
            self._on_barometer,
            mavros_best_effort_qos,
        )

        # 定时刷盘，降低异常退出时数据丢失风险。
        self.create_timer(0.5, self._periodic_flush)

        self.get_logger().info('altitude_measure_node 已启动')
        self.get_logger().info(f'distance_sensor_topic: {self.distance_sensor_topic}')
        self.get_logger().info(f'gps_topic: {self.gps_topic}')
        self.get_logger().info(f'barometer_topic: {self.barometer_topic}')
        self.get_logger().info(f'barometer_field: {self.barometer_field}')
        self.get_logger().info(f'run_output_dir: {self.run_output_dir}')
        self.get_logger().info(f'distance csv: {self.distance_csv_path}')
        self.get_logger().info(f'gps csv: {self.gps_csv_path}')
        self.get_logger().info(f'barometer csv: {self.barometer_csv_path}')

    def _resolve_barometer_field(self, requested_field: str) -> str:
        """检查气压高度字段是否合法；非法时回退到 amsl 并告警。"""
        valid_fields = {
            'monotonic',
            'amsl',
            'local',
            'relative',
            'terrain',
            'bottom_clearance',
        }
        if requested_field in valid_fields:
            return requested_field

        self.get_logger().warn(
            f'barometer_field={requested_field} 非法，自动回退为 amsl；'
            '可选值: monotonic/amsl/local/relative/terrain/bottom_clearance'
        )
        return 'amsl'

    def _stamp_to_sec(self, sec: int, nanosec: int) -> float:
        """将 ROS 时间戳 sec+nanosec 转为秒（浮点）。"""
        return float(sec) + float(nanosec) * 1e-9

    def _now_sec(self) -> float:
        """当前 ROS 时间秒值，用于消息无有效 header 时兜底。"""
        now_msg = self.get_clock().now().to_msg()
        return self._stamp_to_sec(now_msg.sec, now_msg.nanosec)

    def _write_row(self, writer: csv.writer, timestamp_sec: float, altitude_m: float) -> None:
        """统一写 CSV 行，集中处理格式与异常值过滤。"""
        if not math.isfinite(timestamp_sec) or not math.isfinite(altitude_m):
            return
        writer.writerow([timestamp_sec, altitude_m])

    def _periodic_flush(self) -> None:
        """按配置周期执行刷盘，平衡数据安全与写盘负载。"""
        elapsed_sec = (self.get_clock().now() - self.last_flush_time).nanoseconds / 1e9
        if elapsed_sec < self.flush_interval_sec:
            return

        self.distance_csv_file.flush()
        self.gps_csv_file.flush()
        self.barometer_csv_file.flush()
        self.last_flush_time = self.get_clock().now()

    def _on_distance_sensor(self, msg: Range) -> None:
        """记录 distance sensor 当前测距值（米）。"""
        # distance_sensor 带 header，优先使用消息自带时间戳，保证和传感器采样对齐。
        timestamp_sec = self._stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if timestamp_sec <= 0.0:
            timestamp_sec = self._now_sec()

        distance_m = float(msg.range)
        self._write_row(self.distance_writer, timestamp_sec, distance_m)

    def _on_gps(self, msg: NavSatFix) -> None:
        """记录 GPS 高度（NavSatFix.altitude，单位米）。"""
        timestamp_sec = self._stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if timestamp_sec <= 0.0:
            timestamp_sec = self._now_sec()

        gps_altitude_m = float(msg.altitude)
        self._write_row(self.gps_writer, timestamp_sec, gps_altitude_m)

    def _on_barometer(self, msg: Altitude) -> None:
        """记录气压计高度（字段由 barometer_field 参数选择）。"""
        timestamp_sec = self._stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if timestamp_sec <= 0.0:
            timestamp_sec = self._now_sec()

        # 通过 getattr 动态选择字段，便于同一节点快速切换对比指标。
        altitude_getter: Callable[[Altitude], float] = lambda m: float(getattr(m, self.barometer_field))
        barometer_altitude_m = altitude_getter(msg)
        self._write_row(self.barometer_writer, timestamp_sec, barometer_altitude_m)

    def destroy_node(self) -> bool:
        """节点退出前确保缓冲区落盘并关闭文件句柄。"""
        try:
            self.distance_csv_file.flush()
            self.gps_csv_file.flush()
            self.barometer_csv_file.flush()
            self.distance_csv_file.close()
            self.gps_csv_file.close()
            self.barometer_csv_file.close()
        except Exception as exc:
            self.get_logger().warn(f'关闭 CSV 文件时异常: {exc}')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AltitudeMeasureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
