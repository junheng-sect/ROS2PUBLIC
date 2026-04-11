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


class XYTrackingCsvLoggerNode(Node):
    """为 xy_tracking 记录运行 CSV，并在退出时输出 summary。"""

    def __init__(self):
        super().__init__('xy_tracking_csv_logger_node')

        # ===== 输入话题 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('local_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel')

        # ===== 输出路径 =====
        self.declare_parameter('output_dir', '/home/zjh/project/rasip_pi_ws/log/tracking_csv')
        self.declare_parameter('file_prefix', 'xy_tracking')
        self.declare_parameter(
            'summary_csv_path',
            '/home/zjh/project/rasip_pi_ws/log/tracking_csv/xy_tracking_summary.csv',
        )

        # ===== 采样参数 =====
        self.declare_parameter('sample_rate_hz', 30.0)
        self.declare_parameter('stale_timeout_sec', 0.5)
        self.declare_parameter('flush_interval_sec', 1.0)

        # ===== 参数快照 =====
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 2.5)
        self.declare_parameter('kp_xy', 0.6)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.08)
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.06)
        self.declare_parameter('vxy_limit', 0.8)
        self.declare_parameter('vz_limit', 0.5)
        self.declare_parameter('velocity_deadband', 0.03)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.local_pose_topic = self.get_parameter('local_pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

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
            'kp_xy': float(self.get_parameter('kp_xy').value),
            'ki_xy': float(self.get_parameter('ki_xy').value),
            'kd_xy': float(self.get_parameter('kd_xy').value),
            'kp_z': float(self.get_parameter('kp_z').value),
            'ki_z': float(self.get_parameter('ki_z').value),
            'kd_z': float(self.get_parameter('kd_z').value),
            'vxy_limit': float(self.get_parameter('vxy_limit').value),
            'vz_limit': float(self.get_parameter('vz_limit').value),
            'velocity_deadband': float(self.get_parameter('velocity_deadband').value),
        }

        # ===== 最新数据缓存 =====
        self.state_msg: Optional[State] = None
        self.pose_msg: Optional[ArucoBasePose] = None
        self.local_pose_msg: Optional[PoseStamped] = None
        self.cmd_msg: Optional[TwistStamped] = None

        self.pose_rx_time = None
        self.local_pose_rx_time = None
        self.cmd_rx_time = None
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
            'target_x',
            'target_y',
            'target_z',
            'kp_xy',
            'ki_xy',
            'kd_xy',
            'kp_z',
            'ki_z',
            'kd_z',
            'vxy_limit',
            'vz_limit',
            'velocity_deadband',
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
        self.create_subscription(PoseStamped, self.local_pose_topic, self._on_local_pose, mavros_pose_qos)
        self.create_subscription(TwistStamped, self.cmd_vel_topic, self._on_cmd, 10)

        timer_period = 1.0 / max(self.sample_rate_hz, 1.0)
        self.create_timer(timer_period, self._write_row)

        self.get_logger().info(f'xy_tracking CSV记录已启动，输出文件: {self.csv_path}')

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """四元数转 yaw（弧度）。"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _rmse(vals):
        if not vals:
            return float('nan')
        return math.sqrt(sum(v * v for v in vals) / len(vals))

    @staticmethod
    def _std(vals):
        if len(vals) < 2:
            return 0.0
        mean_val = sum(vals) / len(vals)
        return math.sqrt(sum((v - mean_val) * (v - mean_val) for v in vals) / len(vals))

    @staticmethod
    def _quantile(vals, p):
        if not vals:
            return float('nan')
        sorted_vals = sorted(vals)
        idx = (len(sorted_vals) - 1) * p
        lo = int(math.floor(idx))
        hi = int(math.ceil(idx))
        if lo == hi:
            return sorted_vals[lo]
        return sorted_vals[lo] * (hi - idx) + sorted_vals[hi] * (idx - lo)

    def _age_sec(self, stamp_time) -> float:
        """计算当前时刻与最近接收时刻的间隔秒数。"""
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
        """按固定频率写一行运行数据。"""
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
            local_pose = self.local_pose_msg.pose
            local_x = float(local_pose.position.x)
            local_y = float(local_pose.position.y)
            local_z = float(local_pose.position.z)
            local_yaw = self._quat_to_yaw(
                local_pose.orientation.x,
                local_pose.orientation.y,
                local_pose.orientation.z,
                local_pose.orientation.w,
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
            self.param_snapshot['target_x'],
            self.param_snapshot['target_y'],
            self.param_snapshot['target_z'],
            self.param_snapshot['kp_xy'],
            self.param_snapshot['ki_xy'],
            self.param_snapshot['kd_xy'],
            self.param_snapshot['kp_z'],
            self.param_snapshot['ki_z'],
            self.param_snapshot['kd_z'],
            self.param_snapshot['vxy_limit'],
            self.param_snapshot['vz_limit'],
            self.param_snapshot['velocity_deadband'],
        ])

        self.metric_rows.append({
            't': ros_time_sec,
            'mode': mode,
            'aruco_fresh': aruco_fresh,
            'aruco_x': aruco_x,
            'aruco_y': aruco_y,
            'aruco_z': aruco_z,
            'local_z': local_z,
            'cmd_vx': cmd_vx,
            'cmd_vy': cmd_vy,
            'cmd_vz': cmd_vz,
            'cmd_wz': cmd_wz,
        })

        if self._age_sec(self.last_flush_time) >= self.flush_interval_sec:
            self.csv_file.flush()
            self.last_flush_time = self.get_clock().now()

    def _compute_metrics(self):
        """统计 OFFBOARD 且视觉新鲜样本上的误差与抖动指标。"""
        off_rows = [row for row in self.metric_rows if row['mode'] == 'OFFBOARD']
        eval_rows = [row for row in off_rows if row['aruco_fresh'] == 1]

        result = {
            'offboard_rows': len(off_rows),
            'eval_rows': len(eval_rows),
            'status': 'ok' if len(eval_rows) >= 1 else 'insufficient_data',
            'fresh_ratio': float('nan'),
            'max_stale_s': float('nan'),
        }

        if len(off_rows) == 0:
            return result

        result['fresh_ratio'] = len(eval_rows) / len(off_rows)

        dt_vals = []
        for idx in range(1, len(off_rows)):
            dt_vals.append(max(0.0, off_rows[idx]['t'] - off_rows[idx - 1]['t']))
        mean_dt = sum(dt_vals) / len(dt_vals) if dt_vals else 0.0

        stale_run = 0
        max_stale_run = 0
        for row in off_rows:
            if row['aruco_fresh'] == 0:
                stale_run += 1
                max_stale_run = max(max_stale_run, stale_run)
            else:
                stale_run = 0
        result['max_stale_s'] = max_stale_run * mean_dt

        if len(eval_rows) == 0:
            return result

        ex = [self.param_snapshot['target_x'] - row['aruco_x'] for row in eval_rows]
        ey = [self.param_snapshot['target_y'] - row['aruco_y'] for row in eval_rows]
        # z 误差按当前控制器真实闭环对象计算：target_z - aruco_pose.z。
        ez = [self.param_snapshot['target_z'] - row['aruco_z'] for row in eval_rows]
        exy = [math.hypot(ex[idx], ey[idx]) for idx in range(len(ex))]

        ex_abs = [abs(val) for val in ex]
        ey_abs = [abs(val) for val in ey]
        ez_abs = [abs(val) for val in ez]

        cmd_vx = [row['cmd_vx'] for row in eval_rows if math.isfinite(row['cmd_vx'])]
        cmd_vy = [row['cmd_vy'] for row in eval_rows if math.isfinite(row['cmd_vy'])]
        cmd_vz = [row['cmd_vz'] for row in eval_rows if math.isfinite(row['cmd_vz'])]
        cmd_wz = [row['cmd_wz'] for row in eval_rows if math.isfinite(row['cmd_wz'])]

        d_vx = [cmd_vx[idx] - cmd_vx[idx - 1] for idx in range(1, len(cmd_vx))]
        d_vy = [cmd_vy[idx] - cmd_vy[idx - 1] for idx in range(1, len(cmd_vy))]
        d_vz = [cmd_vz[idx] - cmd_vz[idx - 1] for idx in range(1, len(cmd_vz))]
        d_wz = [cmd_wz[idx] - cmd_wz[idx - 1] for idx in range(1, len(cmd_wz))]

        cmd_jitter_x = self._std(d_vx)
        cmd_jitter_y = self._std(d_vy)
        cmd_jitter_z = self._std(d_vz)

        result.update({
            'rmse_x': self._rmse(ex),
            'rmse_y': self._rmse(ey),
            'rmse_z': self._rmse(ez),
            'rmse_xy': self._rmse(exy),
            'p95_abs_x': self._quantile(ex_abs, 0.95),
            'p95_abs_y': self._quantile(ey_abs, 0.95),
            'p95_abs_z': self._quantile(ez_abs, 0.95),
            'p95_xy': self._quantile(exy, 0.95),
            'cmd_jitter_x': cmd_jitter_x,
            'cmd_jitter_y': cmd_jitter_y,
            'cmd_jitter_z': self._std(d_vz),
            'cmd_jitter_xy': math.hypot(cmd_jitter_x, cmd_jitter_y),
            'cmd_jitter_yaw': self._std(d_wz),
        })
        return result

    def _append_summary_csv(self, metrics):
        """将本次运行摘要追加到 summary CSV。"""
        summary_path = self.summary_csv_path
        os.makedirs(os.path.dirname(summary_path), exist_ok=True)
        self._repair_summary_csv_if_needed(summary_path)
        file_exists = os.path.exists(summary_path)

        row = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'run_csv': self.csv_path,
            'file_prefix': self.file_prefix,
            'status': metrics.get('status', 'unknown'),
            'offboard_rows': metrics.get('offboard_rows', 0),
            'eval_rows': metrics.get('eval_rows', 0),
            'fresh_ratio': metrics.get('fresh_ratio', float('nan')),
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
            'cmd_jitter_z': metrics.get('cmd_jitter_z', float('nan')),
            'cmd_jitter_xy': metrics.get('cmd_jitter_xy', float('nan')),
            'cmd_jitter_yaw': metrics.get('cmd_jitter_yaw', float('nan')),
            **self.param_snapshot,
        }

        fieldnames = list(row.keys())
        with open(summary_path, 'a', newline='', encoding='utf-8') as summary_file:
            writer = csv.DictWriter(summary_file, fieldnames=fieldnames, lineterminator='\n')
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)

    def _repair_summary_csv_if_needed(self, summary_path: str):
        """统一 summary 文件换行，并去掉历史重复表头。"""
        if not os.path.exists(summary_path):
            return
        try:
            with open(summary_path, 'r', encoding='utf-8', newline='') as summary_file:
                raw_text = summary_file.read()
            if raw_text == '':
                return

            normalized = raw_text.replace('\r\n', '\n').replace('\r', '\n')
            lines = [line for line in normalized.split('\n') if line.strip() != '']
            if not lines:
                return

            header = lines[0]
            cleaned_lines = [header]
            for line in lines[1:]:
                if line == header:
                    continue
                cleaned_lines.append(line)

            repaired = '\n'.join(cleaned_lines) + '\n'
            if repaired != raw_text:
                with open(summary_path, 'w', encoding='utf-8', newline='') as summary_file:
                    summary_file.write(repaired)
                self.get_logger().info(f'已修复 summary 格式: {summary_path}')
        except Exception as exc:
            self.get_logger().warn(f'summary 格式修复跳过: {exc}')

    def _print_summary(self, metrics):
        """在节点退出时输出本次运行摘要。"""
        self.get_logger().info('===== xy_tracking 指标汇总（OFFBOARD+fresh） =====')
        self.get_logger().info(
            f"status={metrics.get('status')} | offboard_rows={metrics.get('offboard_rows')} | "
            f"eval_rows={metrics.get('eval_rows')} | fresh_ratio={metrics.get('fresh_ratio', float('nan')):.4f} | "
            f"max_stale_s={metrics.get('max_stale_s', float('nan')):.3f}"
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
                f"z={metrics['cmd_jitter_z']:.5f}, xy={metrics['cmd_jitter_xy']:.5f}, "
                f"yaw={metrics['cmd_jitter_yaw']:.5f}"
            )

    def destroy_node(self):
        """退出前确保 CSV 落盘，并生成 summary。"""
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except BaseException:
            pass

        metrics = self._compute_metrics()
        self._print_summary(metrics)
        try:
            self._append_summary_csv(metrics)
            self.get_logger().info(f'汇总CSV已追加: {self.summary_csv_path}')
        except Exception as exc:
            self.get_logger().error(f'写入汇总CSV失败: {exc}')

        try:
            super().destroy_node()
        except BaseException:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = XYTrackingCsvLoggerNode()
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
