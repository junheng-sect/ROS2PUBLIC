#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime
from typing import Optional

import rclpy
from debug_interface.msg import ArucoBasePose
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class ArucoTrackingCsvLoggerNode(Node):
    """为 aruco_tracking_minimal 记录调参数据到 CSV。"""

    def __init__(self):
        super().__init__('aruco_tracking_csv_logger_node')

        # ===== 可配置参数 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('local_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel')
        self.declare_parameter('output_dir', '/home/zjh/project/zjh_ws/log/tracking_csv')
        self.declare_parameter('file_prefix', 'aruco_tracking_minimal')
        self.declare_parameter('sample_rate_hz', 30.0)
        self.declare_parameter('stale_timeout_sec', 0.5)
        self.declare_parameter('flush_interval_sec', 1.0)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.local_pose_topic = self.get_parameter('local_pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.file_prefix = self.get_parameter('file_prefix').value
        self.sample_rate_hz = float(self.get_parameter('sample_rate_hz').value)
        self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
        self.flush_interval_sec = float(self.get_parameter('flush_interval_sec').value)

        # ===== 最新数据缓存 =====
        self.state_msg: Optional[State] = None
        self.pose_msg: Optional[ArucoBasePose] = None
        self.local_pose_msg: Optional[PoseStamped] = None
        self.cmd_msg: Optional[TwistStamped] = None

        self.pose_rx_time = None
        self.local_pose_rx_time = None
        self.cmd_rx_time = None
        self.last_flush_time = self.get_clock().now()

        # ===== 创建 CSV 文件 =====
        os.makedirs(self.output_dir, exist_ok=True)
        now_text = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.output_dir, f'{self.file_prefix}_{now_text}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
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
            'local_pose_fresh',
            'local_pose_age_sec',
            'local_x',
            'local_y',
            'local_z',
            'local_yaw_rad',
            'cmd_fresh',
            'cmd_age_sec',
            'cmd_vx',
            'cmd_vy',
            'cmd_vz',
            'cmd_wz',
        ])
        self.csv_file.flush()

        # ===== QoS 配置 =====
        mavros_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ===== 订阅 =====
        self.create_subscription(State, self.state_topic, self._on_state, 10)
        self.create_subscription(ArucoBasePose, self.pose_topic, self._on_pose, 10)
        self.create_subscription(PoseStamped, self.local_pose_topic, self._on_local_pose, mavros_pose_qos)
        self.create_subscription(TwistStamped, self.cmd_vel_topic, self._on_cmd, 10)

        # ===== 定时写盘 =====
        timer_period = 1.0 / max(self.sample_rate_hz, 1.0)
        self.create_timer(timer_period, self._write_row)

        self.get_logger().info(f'CSV 记录已启动，输出文件: {self.csv_path}')

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """四元数转 yaw（弧度）。"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _age_sec(self, stamp_time) -> float:
        """计算“当前时刻 - 消息接收时刻”的秒数。"""
        if stamp_time is None:
            return 1e9
        return (self.get_clock().now() - stamp_time).nanoseconds / 1e9

    def _is_fresh(self, stamp_time) -> bool:
        """按统一超时阈值判断数据是否新鲜。"""
        return self._age_sec(stamp_time) <= self.stale_timeout_sec

    def _on_state(self, msg: State):
        self.state_msg = msg

    def _on_pose(self, msg: ArucoBasePose):
        self.pose_msg = msg
        self.pose_rx_time = self.get_clock().now()

    def _on_local_pose(self, msg: PoseStamped):
        self.local_pose_msg = msg
        self.local_pose_rx_time = self.get_clock().now()

    def _on_cmd(self, msg: TwistStamped):
        self.cmd_msg = msg
        self.cmd_rx_time = self.get_clock().now()

    def _write_row(self):
        """按采样频率写入一行 CSV，便于后续离线调参分析。"""
        now_msg = self.get_clock().now().to_msg()
        ros_time_sec = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9

        mode = self.state_msg.mode if self.state_msg is not None else ''
        armed = int(self.state_msg.armed) if self.state_msg is not None else -1
        connected = int(self.state_msg.connected) if self.state_msg is not None else -1

        aruco_fresh = int(self._is_fresh(self.pose_rx_time))
        aruco_age = self._age_sec(self.pose_rx_time)
        aruco_x = float(self.pose_msg.x) if self.pose_msg is not None else float('nan')
        aruco_y = float(self.pose_msg.y) if self.pose_msg is not None else float('nan')
        aruco_z = float(self.pose_msg.z) if self.pose_msg is not None else float('nan')
        aruco_yaw = float(self.pose_msg.yaw) if self.pose_msg is not None else float('nan')

        local_fresh = int(self._is_fresh(self.local_pose_rx_time))
        local_age = self._age_sec(self.local_pose_rx_time)
        if self.local_pose_msg is not None:
            lp = self.local_pose_msg.pose
            local_x = float(lp.position.x)
            local_y = float(lp.position.y)
            local_z = float(lp.position.z)
            local_yaw = self._quat_to_yaw(
                lp.orientation.x,
                lp.orientation.y,
                lp.orientation.z,
                lp.orientation.w,
            )
        else:
            local_x = float('nan')
            local_y = float('nan')
            local_z = float('nan')
            local_yaw = float('nan')

        cmd_fresh = int(self._is_fresh(self.cmd_rx_time))
        cmd_age = self._age_sec(self.cmd_rx_time)
        if self.cmd_msg is not None:
            cmd_vx = float(self.cmd_msg.twist.linear.x)
            cmd_vy = float(self.cmd_msg.twist.linear.y)
            cmd_vz = float(self.cmd_msg.twist.linear.z)
            cmd_wz = float(self.cmd_msg.twist.angular.z)
        else:
            cmd_vx = float('nan')
            cmd_vy = float('nan')
            cmd_vz = float('nan')
            cmd_wz = float('nan')

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
            local_fresh,
            local_age,
            local_x,
            local_y,
            local_z,
            local_yaw,
            cmd_fresh,
            cmd_age,
            cmd_vx,
            cmd_vy,
            cmd_vz,
            cmd_wz,
        ])

        if self._age_sec(self.last_flush_time) >= self.flush_interval_sec:
            self.csv_file.flush()
            self.last_flush_time = self.get_clock().now()

    def destroy_node(self):
        """节点退出前确保文件完整落盘并关闭。"""
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackingCsvLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
