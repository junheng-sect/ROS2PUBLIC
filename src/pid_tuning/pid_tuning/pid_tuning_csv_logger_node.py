#!/usr/bin/env python3

import csv
import math
import os
import shutil
import subprocess
from datetime import datetime
from typing import Optional

import rclpy
from debug_interface.msg import ArucoBasePose
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class PidTuningCsvLoggerNode(Node):
    """PID 调参记录节点：写运行 CSV，并在结束时输出指标与汇总。"""

    def __init__(self):
        super().__init__('pid_tuning_csv_logger_node')

        # ===== 输入话题 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('local_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('setpoint_raw_topic', '/mavros/setpoint_raw/local')

        # ===== 输出路径 =====
        self.declare_parameter('output_dir', '/home/zjh/project/rasip_pi_ws/log/tracking_csv')
        self.declare_parameter('file_prefix', 'pid_tuning')
        self.declare_parameter('summary_csv_path', '/home/zjh/project/rasip_pi_ws/log/tracking_csv/pid_tuning_summary.csv')
        # ===== 结束后自动同步 =====
        # 说明：
        # 1) 本地同步：将本次运行 CSV 和 summary CSV 复制到目标目录（默认桌面 trackingcsv）。
        # 2) 自定义命令同步：可用 sync_cmd 触发 rsync/scp 等外部命令。
        self.declare_parameter('sync_on_shutdown', True)
        self.declare_parameter('sync_target_dir', '/home/zjh/桌面/trackingcsv')
        self.declare_parameter('sync_copy_run_csv', True)
        self.declare_parameter('sync_copy_summary_csv', True)
        self.declare_parameter('sync_cmd', '')

        # ===== 采样参数 =====
        self.declare_parameter('sample_rate_hz', 30.0)
        self.declare_parameter('stale_timeout_sec', 0.5)
        self.declare_parameter('flush_interval_sec', 1.0)

        # ===== 目标参数快照 =====
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('target_z', 2.5)

        # ===== PID/限制参数快照（用于汇总） =====
        self.declare_parameter('kp_xy', 0.5)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.08)

        self.declare_parameter('kp_x', float('nan'))
        self.declare_parameter('ki_x', float('nan'))
        self.declare_parameter('kd_x', float('nan'))
        self.declare_parameter('kp_y', float('nan'))
        self.declare_parameter('ki_y', float('nan'))
        self.declare_parameter('kd_y', float('nan'))

        self.declare_parameter('kp_yaw', 1.2)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.08)
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.06)

        self.declare_parameter('vxy_limit', 0.8)
        self.declare_parameter('vz_limit', 0.5)
        self.declare_parameter('wz_limit', 1.0)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('yaw_rate_deadband', 0.02)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.local_pose_topic = self.get_parameter('local_pose_topic').value
        self.setpoint_raw_topic = self.get_parameter('setpoint_raw_topic').value

        self.output_dir = self.get_parameter('output_dir').value
        self.file_prefix = self.get_parameter('file_prefix').value
        self.summary_csv_path = self.get_parameter('summary_csv_path').value
        self.sync_on_shutdown = bool(self.get_parameter('sync_on_shutdown').value)
        self.sync_target_dir = self.get_parameter('sync_target_dir').value
        self.sync_copy_run_csv = bool(self.get_parameter('sync_copy_run_csv').value)
        self.sync_copy_summary_csv = bool(self.get_parameter('sync_copy_summary_csv').value)
        self.sync_cmd = str(self.get_parameter('sync_cmd').value)

        self.sample_rate_hz = float(self.get_parameter('sample_rate_hz').value)
        self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
        self.flush_interval_sec = float(self.get_parameter('flush_interval_sec').value)

        # 参数快照，用于单次 CSV 与汇总 CSV。
        self.param_snapshot = {
            'target_x': float(self.get_parameter('target_x').value),
            'target_y': float(self.get_parameter('target_y').value),
            'target_yaw': float(self.get_parameter('target_yaw').value),
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
            'kp_yaw': float(self.get_parameter('kp_yaw').value),
            'ki_yaw': float(self.get_parameter('ki_yaw').value),
            'kd_yaw': float(self.get_parameter('kd_yaw').value),
            'kp_z': float(self.get_parameter('kp_z').value),
            'ki_z': float(self.get_parameter('ki_z').value),
            'kd_z': float(self.get_parameter('kd_z').value),
            'vxy_limit': float(self.get_parameter('vxy_limit').value),
            'vz_limit': float(self.get_parameter('vz_limit').value),
            'wz_limit': float(self.get_parameter('wz_limit').value),
            'velocity_deadband': float(self.get_parameter('velocity_deadband').value),
            'yaw_rate_deadband': float(self.get_parameter('yaw_rate_deadband').value),
        }

        # ===== 最新数据缓存 =====
        self.state_msg: Optional[State] = None
        self.pose_msg: Optional[ArucoBasePose] = None
        self.local_pose_msg: Optional[PoseStamped] = None
        self.setpoint_msg: Optional[PositionTarget] = None

        self.pose_rx_time = None
        self.local_pose_rx_time = None
        self.setpoint_rx_time = None
        self.last_flush_time = self.get_clock().now()

        # ===== 指标缓存（逐样本） =====
        self.metric_rows = []

        # ===== 创建单次 CSV =====
        os.makedirs(self.output_dir, exist_ok=True)
        now_text = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.output_dir, f'{self.file_prefix}_{now_text}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        # 强制使用 LF 行结尾，避免在 Linux 下出现 CRLF 导致“格式炸行/显示异常”。
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
            'setpoint_fresh',
            'setpoint_age_sec',
            'sp_frame',
            'sp_type_mask',
            'sp_vx',
            'sp_vy',
            'sp_vz',
            'sp_yaw_rate',
            # 参数快照（用于回放快速识别本次配置）
            'target_x', 'target_y', 'target_yaw', 'target_z',
            'kp_xy', 'ki_xy', 'kd_xy',
            'kp_x', 'ki_x', 'kd_x',
            'kp_y', 'ki_y', 'kd_y',
            'kp_yaw', 'ki_yaw', 'kd_yaw',
            'kp_z', 'ki_z', 'kd_z',
            'vxy_limit', 'vz_limit', 'wz_limit',
            'velocity_deadband', 'yaw_rate_deadband',
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
        self.create_subscription(PositionTarget, self.setpoint_raw_topic, self._on_setpoint, 10)

        timer_period = 1.0 / max(self.sample_rate_hz, 1.0)
        self.create_timer(timer_period, self._write_row)

        self.get_logger().info(f'PID调参CSV记录已启动，输出文件: {self.csv_path}')

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_to_pi(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def _age_sec(self, stamp_time) -> float:
        if stamp_time is None:
            return 1e9
        return (self.get_clock().now() - stamp_time).nanoseconds / 1e9

    def _is_fresh(self, stamp_time) -> bool:
        return self._age_sec(stamp_time) <= self.stale_timeout_sec

    def _on_state(self, msg: State):
        self.state_msg = msg

    def _on_pose(self, msg: ArucoBasePose):
        self.pose_msg = msg
        self.pose_rx_time = self.get_clock().now()

    def _on_local_pose(self, msg: PoseStamped):
        self.local_pose_msg = msg
        self.local_pose_rx_time = self.get_clock().now()

    def _on_setpoint(self, msg: PositionTarget):
        self.setpoint_msg = msg
        self.setpoint_rx_time = self.get_clock().now()

    def _write_row(self):
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

        sp_fresh = int(self._is_fresh(self.setpoint_rx_time))
        sp_age = self._age_sec(self.setpoint_rx_time)
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
            aruco_yaw,
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
            self.param_snapshot['target_yaw'],
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
            self.param_snapshot['kp_yaw'],
            self.param_snapshot['ki_yaw'],
            self.param_snapshot['kd_yaw'],
            self.param_snapshot['kp_z'],
            self.param_snapshot['ki_z'],
            self.param_snapshot['kd_z'],
            self.param_snapshot['vxy_limit'],
            self.param_snapshot['vz_limit'],
            self.param_snapshot['wz_limit'],
            self.param_snapshot['velocity_deadband'],
            self.param_snapshot['yaw_rate_deadband'],
        ])

        # 指标缓存：仅在写行时收集，结束时按 OFFBOARD+fresh 过滤统计。
        self.metric_rows.append({
            't': ros_time_sec,
            'mode': mode,
            'aruco_fresh': aruco_fresh,
            'aruco_x': aruco_x,
            'aruco_y': aruco_y,
            'aruco_z': aruco_z,
            'aruco_yaw': aruco_yaw,
            'sp_vx': sp_vx,
            'sp_vy': sp_vy,
            'sp_yaw_rate': sp_yaw_rate,
            'sp_fresh': sp_fresh,
        })

        if self._age_sec(self.last_flush_time) >= self.flush_interval_sec:
            self.csv_file.flush()
            self.last_flush_time = self.get_clock().now()

    @staticmethod
    def _quantile(vals, p):
        if not vals:
            return float('nan')
        s = sorted(vals)
        idx = (len(s) - 1) * p
        lo = int(math.floor(idx))
        hi = int(math.ceil(idx))
        if lo == hi:
            return s[lo]
        return s[lo] * (hi - idx) + s[hi] * (idx - lo)

    @staticmethod
    def _rmse(vals):
        if not vals:
            return float('nan')
        return math.sqrt(sum(v * v for v in vals) / len(vals))

    @staticmethod
    def _std(vals):
        if len(vals) < 2:
            return 0.0
        m = sum(vals) / len(vals)
        return math.sqrt(sum((v - m) * (v - m) for v in vals) / len(vals))

    def _compute_metrics(self):
        # 仅统计 OFFBOARD 且 aruco_fresh=1 的窗口。
        off_rows = [r for r in self.metric_rows if r['mode'] == 'OFFBOARD']
        eval_rows = [r for r in off_rows if r['aruco_fresh'] == 1]

        result = {
            'eval_rows': len(eval_rows),
            'offboard_rows': len(off_rows),
            'status': 'ok' if len(eval_rows) >= 1 else 'insufficient_data',
        }

        if len(off_rows) == 0:
            # 没有进入 OFFBOARD，直接返回最小结果。
            result.update({
                'fresh_ratio': float('nan'),
                'max_stale_s': float('nan'),
            })
            return result

        fresh_ratio = len(eval_rows) / len(off_rows)

        # OFFBOARD 内最长连续 stale 时长。
        stale_run = 0
        max_stale_run = 0
        dt_vals = []
        for i in range(1, len(off_rows)):
            dt_vals.append(max(0.0, off_rows[i]['t'] - off_rows[i - 1]['t']))
        mean_dt = sum(dt_vals) / len(dt_vals) if dt_vals else 0.0
        for r in off_rows:
            if r['aruco_fresh'] == 0:
                stale_run += 1
                max_stale_run = max(max_stale_run, stale_run)
            else:
                stale_run = 0
        max_stale_s = max_stale_run * mean_dt

        result.update({
            'fresh_ratio': fresh_ratio,
            'max_stale_s': max_stale_s,
        })

        if len(eval_rows) == 0:
            return result

        tx = self.param_snapshot['target_x']
        ty = self.param_snapshot['target_y']
        tyaw = self.param_snapshot['target_yaw']
        tz = self.param_snapshot['target_z']

        ex = [tx - r['aruco_x'] for r in eval_rows]
        ey = [ty - r['aruco_y'] for r in eval_rows]
        ez = [tz - r['aruco_z'] for r in eval_rows]
        eyaw = [self._wrap_to_pi(tyaw - r['aruco_yaw']) for r in eval_rows]

        ex_abs = [abs(v) for v in ex]
        ey_abs = [abs(v) for v in ey]
        ez_abs = [abs(v) for v in ez]
        eyaw_abs = [abs(v) for v in eyaw]
        exy = [math.hypot(ex[i], ey[i]) for i in range(len(ex))]

        sp_vx = [r['sp_vx'] for r in eval_rows if math.isfinite(r['sp_vx'])]
        sp_vy = [r['sp_vy'] for r in eval_rows if math.isfinite(r['sp_vy'])]
        sp_wz = [r['sp_yaw_rate'] for r in eval_rows if math.isfinite(r['sp_yaw_rate'])]

        d_vx = [sp_vx[i] - sp_vx[i - 1] for i in range(1, len(sp_vx))]
        d_vy = [sp_vy[i] - sp_vy[i - 1] for i in range(1, len(sp_vy))]
        d_wz = [sp_wz[i] - sp_wz[i - 1] for i in range(1, len(sp_wz))]

        cmd_jitter_x = self._std(d_vx)
        cmd_jitter_y = self._std(d_vy)
        cmd_jitter_xy = math.hypot(cmd_jitter_x, cmd_jitter_y)
        cmd_jitter_yaw = self._std(d_wz)

        result.update({
            'rmse_x': self._rmse(ex),
            'rmse_y': self._rmse(ey),
            'rmse_z': self._rmse(ez),
            'rmse_yaw': self._rmse(eyaw),
            'rmse_xy': self._rmse(exy),
            'p95_abs_x': self._quantile(ex_abs, 0.95),
            'p95_abs_y': self._quantile(ey_abs, 0.95),
            'p95_abs_z': self._quantile(ez_abs, 0.95),
            'p95_abs_yaw': self._quantile(eyaw_abs, 0.95),
            'p95_xy': self._quantile(exy, 0.95),
            'cmd_jitter_x': cmd_jitter_x,
            'cmd_jitter_y': cmd_jitter_y,
            'cmd_jitter_xy': cmd_jitter_xy,
            'cmd_jitter_yaw': cmd_jitter_yaw,
        })
        return result

    def _append_summary_csv(self, metrics):
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
            'rmse_yaw': metrics.get('rmse_yaw', float('nan')),
            'rmse_xy': metrics.get('rmse_xy', float('nan')),
            'p95_abs_x': metrics.get('p95_abs_x', float('nan')),
            'p95_abs_y': metrics.get('p95_abs_y', float('nan')),
            'p95_abs_z': metrics.get('p95_abs_z', float('nan')),
            'p95_abs_yaw': metrics.get('p95_abs_yaw', float('nan')),
            'p95_xy': metrics.get('p95_xy', float('nan')),
            'cmd_jitter_x': metrics.get('cmd_jitter_x', float('nan')),
            'cmd_jitter_y': metrics.get('cmd_jitter_y', float('nan')),
            'cmd_jitter_xy': metrics.get('cmd_jitter_xy', float('nan')),
            'cmd_jitter_yaw': metrics.get('cmd_jitter_yaw', float('nan')),
            **self.param_snapshot,
        }

        fieldnames = list(row.keys())
        with open(summary_path, 'a', newline='', encoding='utf-8') as f:
            # 强制使用 LF 行结尾，避免 summary 出现 ^M 或空行异常。
            w = csv.DictWriter(f, fieldnames=fieldnames, lineterminator='\n')
            if not file_exists:
                w.writeheader()
            w.writerow(row)

    def _repair_summary_csv_if_needed(self, summary_path: str):
        """修复历史 summary 文件格式：统一 LF，并去除重复表头。"""
        if not os.path.exists(summary_path):
            return
        try:
            with open(summary_path, 'r', encoding='utf-8', newline='') as f:
                raw = f.read()
            if raw == '':
                return

            # 统一行结束符，避免 CRLF 导致显示异常。
            normalized = raw.replace('\r\n', '\n').replace('\r', '\n')
            lines = [ln for ln in normalized.split('\n') if ln.strip() != '']
            if not lines:
                return

            header = lines[0]
            cleaned = [header]
            # 清理历史中重复写入的表头行。
            for ln in lines[1:]:
                if ln == header:
                    continue
                cleaned.append(ln)

            repaired = '\n'.join(cleaned) + '\n'
            if repaired != raw:
                with open(summary_path, 'w', encoding='utf-8', newline='') as f:
                    f.write(repaired)
                self.get_logger().info(f'已修复 summary 格式: {summary_path}')
        except Exception as exc:
            self.get_logger().warn(f'summary 格式修复跳过: {exc}')

    def _print_summary(self, metrics):
        self.get_logger().info('===== PID调参指标汇总（OFFBOARD+fresh） =====')
        self.get_logger().info(
            f"status={metrics.get('status')} | offboard_rows={metrics.get('offboard_rows')} | "
            f"eval_rows={metrics.get('eval_rows')} | fresh_ratio={metrics.get('fresh_ratio', float('nan')):.4f} | "
            f"max_stale_s={metrics.get('max_stale_s', float('nan')):.3f}"
        )
        if metrics.get('status') == 'ok':
            self.get_logger().info(
                f"RMSE: x={metrics['rmse_x']:.4f}, y={metrics['rmse_y']:.4f}, z={metrics['rmse_z']:.4f}, "
                f"yaw={metrics['rmse_yaw']:.4f}, xy={metrics['rmse_xy']:.4f}"
            )
            self.get_logger().info(
                f"P95: |x|={metrics['p95_abs_x']:.4f}, |y|={metrics['p95_abs_y']:.4f}, "
                f"|z|={metrics['p95_abs_z']:.4f}, |yaw|={metrics['p95_abs_yaw']:.4f}, xy={metrics['p95_xy']:.4f}"
            )
            self.get_logger().info(
                f"CmdJitter: x={metrics['cmd_jitter_x']:.5f}, y={metrics['cmd_jitter_y']:.5f}, "
                f"xy={metrics['cmd_jitter_xy']:.5f}, yaw={metrics['cmd_jitter_yaw']:.5f}"
            )

    def _sync_outputs(self):
        """将本次日志同步到目标目录，并可执行自定义同步命令。"""
        if not self.sync_on_shutdown:
            return

        copied_files = []
        try:
            os.makedirs(self.sync_target_dir, exist_ok=True)
            if self.sync_copy_run_csv and os.path.exists(self.csv_path):
                dst = os.path.join(self.sync_target_dir, os.path.basename(self.csv_path))
                shutil.copy2(self.csv_path, dst)
                copied_files.append(dst)
            if self.sync_copy_summary_csv and os.path.exists(self.summary_csv_path):
                dst = os.path.join(self.sync_target_dir, os.path.basename(self.summary_csv_path))
                shutil.copy2(self.summary_csv_path, dst)
                copied_files.append(dst)
            if copied_files:
                self.get_logger().info(
                    '自动同步完成（本地复制）: ' + ', '.join(copied_files)
                )
            else:
                self.get_logger().warn('自动同步未复制文件：源文件不存在或同步项已关闭。')
        except Exception as exc:
            self.get_logger().error(f'自动同步（本地复制）失败: {exc}')

        # 支持可选自定义命令，便于从树莓派直接推送到 laptop（如 rsync/scp）。
        if self.sync_cmd.strip():
            cmd = self.sync_cmd.format(
                run_csv=self.csv_path,
                summary_csv=self.summary_csv_path,
                target_dir=self.sync_target_dir,
            )
            try:
                subprocess.run(cmd, shell=True, check=True)
                self.get_logger().info(f'自定义同步命令执行成功: {cmd}')
            except Exception as exc:
                self.get_logger().error(f'自定义同步命令执行失败: {exc} | cmd={cmd}')

    def destroy_node(self):
        """节点退出前落盘、计算指标、输出汇总。"""
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
            self._sync_outputs()
        except Exception as exc:
            self.get_logger().error(f'结束同步失败: {exc}')

        try:
            super().destroy_node()
        except BaseException:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PidTuningCsvLoggerNode()
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
