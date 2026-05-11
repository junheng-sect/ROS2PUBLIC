#!/usr/bin/env python3

import json
import math
import time
from collections import deque
from typing import Optional

import rclpy
from debug_interface.msg import ArucoBasePose
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from mavros_msgs.srv import CommandBool
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, String


class PIDController:
    """简单 PID 控制器，带积分限幅和可选输出限幅。"""

    def __init__(self, kp=0.5, ki=0.0, kd=0.05, out_limit=0.8, i_limit=1.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        if out_limit is None:
            self.out_limit = None
        else:
            out_limit_value = float(out_limit)
            self.out_limit = (
                abs(out_limit_value) if math.isfinite(out_limit_value) else None
            )
        self.i_limit = abs(float(i_limit))

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.initialized = False

    def reset(self):
        """在阶段切换或数据超时时清空内部状态，避免历史量残留。"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.initialized = False

    def update(self, error: float) -> float:
        """根据当前误差输出控制量。"""
        now = time.time()
        if not self.initialized or self.prev_time is None:
            self.initialized = True
            self.prev_error = error
            self.prev_time = now
            output = self.kp * error
            if self.out_limit is not None:
                output = max(-self.out_limit, min(self.out_limit, output))
            return output

        dt = now - self.prev_time
        if dt <= 1e-6 or dt > 1.0 or not math.isfinite(dt):
            # dt 异常时只输出比例项，避免微分尖峰影响飞控指令。
            self.prev_error = error
            self.prev_time = now
            output = self.kp * error
            if self.out_limit is not None:
                output = max(-self.out_limit, min(self.out_limit, output))
            return output

        p_term = self.kp * error

        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        i_term = self.ki * self.integral

        d_term = self.kd * (error - self.prev_error) / dt

        output = p_term + i_term + d_term
        if self.out_limit is not None:
            output = max(-self.out_limit, min(self.out_limit, output))

        self.prev_error = error
        self.prev_time = now
        return output


class AngleLowPassFilter:
    """一阶角度低通滤波器，用最短角距离避免 +/-pi 附近跳变。"""

    def __init__(self):
        self.reset()

    def reset(self):
        """清空角度滤波状态，下一帧视觉数据直接作为滤波初值。"""
        self.initialized = False
        self.value = 0.0
        self.prev_time = None

    def update(self, raw_angle: float, tau_sec: float) -> float:
        """按 dt/(tau+dt) 平滑角度，输出始终归一化到 [-pi, pi]。"""
        now = time.time()
        raw_angle = LandV2Node.wrap_to_pi(float(raw_angle))
        tau_sec = float(tau_sec)
        if not self.initialized or self.prev_time is None:
            self.initialized = True
            self.value = raw_angle
            self.prev_time = now
            return self.value

        dt = now - self.prev_time
        self.prev_time = now
        if dt <= 1e-6 or tau_sec <= 0.0 or not math.isfinite(tau_sec):
            self.value = raw_angle
            return self.value

        # 先把新角度与当前滤波值的差包裹到 [-pi, pi]，再沿最短角距离逼近。
        angle_error = LandV2Node.wrap_to_pi(raw_angle - self.value)
        alpha = dt / (tau_sec + dt)
        self.value = LandV2Node.wrap_to_pi(self.value + alpha * angle_error)
        return self.value


class SlewRateLimiter:
    """输出斜率限制器，用于限制 yaw_rate 每秒最大变化量。"""

    def __init__(self, initial_value: float = 0.0):
        self.reset(initial_value)

    def reset(self, value: float = 0.0):
        """把上一输出重置到指定值，避免重新接管时沿旧指令续接。"""
        self.prev_output = float(value)
        self.prev_time = None

    def update(self, target_value: float, rate_limit: float) -> float:
        """按 rate_limit*dt 裁剪本周期允许变化量。"""
        now = time.time()
        target_value = float(target_value)
        rate_limit = abs(float(rate_limit))
        if self.prev_time is None:
            self.prev_output = target_value
            self.prev_time = now
            return self.prev_output

        dt = now - self.prev_time
        self.prev_time = now
        if dt <= 1e-6 or rate_limit <= 0.0 or not math.isfinite(rate_limit):
            self.prev_output = target_value
            return self.prev_output

        max_delta = rate_limit * dt
        delta = target_value - self.prev_output
        delta = max(-max_delta, min(max_delta, delta))
        self.prev_output += delta
        return self.prev_output


class LandV2Node(Node):
    """land_v2 主控制节点：先在 2.5m 对准，再持续跟踪下降，最终接地 disarm。"""

    PHASE_WAIT_FOR_OFFBOARD = 'WAIT_FOR_OFFBOARD'
    PHASE_ALIGN_AT_TARGET_HEIGHT = 'ALIGN_AT_TARGET_HEIGHT'
    PHASE_DESCEND_WITH_TRACK = 'DESCEND_WITH_TRACK'
    PHASE_DESCEND_RANGE_ONLY = 'DESCEND_RANGE_ONLY'
    PHASE_TOUCHDOWN_DISARM = 'TOUCHDOWN_DISARM'
    PHASE_DONE = 'DONE'

    ALIGN_MODE_SLIDING_WINDOW = 'sliding_window'

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        """将角度归一化到 [-pi, pi]。"""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def apply_deadband(value: float, deadband: float) -> float:
        """把过小的控制量压到零，降低悬停抖动。"""
        return 0.0 if abs(value) < deadband else value

    @staticmethod
    def parameter_as_bool(value) -> bool:
        """兼容 launch 字符串和原生 bool，避免 'false' 被 bool() 判真。"""
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    @staticmethod
    def limit_horizontal_velocity(
        vx_raw: float,
        vy_raw: float,
        v_limit: float,
    ) -> tuple[float, float, float, float]:
        """
        对水平速度做矢量幅值限幅，并尽量保持原始速度方向。

        `land_v2` 使用 `v_limit` 作为主限幅：先计算水平速度幅值，
        超限时对 X/Y 同比例缩放，避免单轴先饱和导致跟踪方向偏转。
        """
        v_norm = math.hypot(vx_raw, vy_raw)
        if v_norm > max(v_limit, 0.0) and v_norm > 1e-6:
            scale = v_limit / v_norm
            return vx_raw * scale, vy_raw * scale, v_norm, scale
        return vx_raw, vy_raw, v_norm, 1.0

    @staticmethod
    def rotate_marker_error_to_body(
        ex_marker: float,
        ey_marker: float,
        yaw_rel_corrected: float,
    ) -> tuple[float, float]:
        """
        将 marker 平面误差旋转到机体系误差。

        严格保持与 `pid_tuning_v4` 一致：
        e_body = R(-yaw_rel_corrected) * e_marker
        """
        cos_yaw = math.cos(yaw_rel_corrected)
        sin_yaw = math.sin(yaw_rel_corrected)
        ex_body = cos_yaw * ex_marker + sin_yaw * ey_marker
        ey_body = -sin_yaw * ex_marker + cos_yaw * ey_marker
        return ex_body, ey_body

    def __init__(self):
        super().__init__('land_v2_node')

        # ===== 输入/输出话题 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('extended_state_topic', '/mavros/extended_state')
        self.declare_parameter('distance_sensor_topic', '/mavros/hrlv_ez4_pub')
        self.declare_parameter('rel_alt_topic', '/mavros/global_position/rel_alt')
        self.declare_parameter('vel_local_topic', '/mavros/local_position/velocity_local')
        self.declare_parameter('setpoint_raw_topic', '/mavros/setpoint_raw/local')
        self.declare_parameter('arming_service', '/mavros/cmd/arming')
        self.declare_parameter('status_topic', '/land_v2/status')

        # ===== 控制目标 =====
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 2.5)
        self.declare_parameter('target_yaw', 0.0)

        # ===== XY PID：兼容粗调入口 =====
        self.declare_parameter('kp_xy', 0.5)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.08)

        # ===== XY PID：分轴精调入口（NaN 表示沿用 kp_xy） =====
        self.declare_parameter('kp_x', float('nan'))
        self.declare_parameter('ki_x', float('nan'))
        self.declare_parameter('kd_x', float('nan'))
        self.declare_parameter('kp_y', float('nan'))
        self.declare_parameter('ki_y', float('nan'))
        self.declare_parameter('kd_y', float('nan'))

        # ===== Z PID / Yaw PID 与相机 yaw 修正 =====
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.06)
        self.declare_parameter('kp_yaw', 0.4)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.03)
        self.declare_parameter('camera_yaw_compensation_deg', 0.0)
        self.declare_parameter('invert_yaw_rate_output', True)

        # ===== 对准阶段：方案 B 滑动窗口稳定型 =====
        self.declare_parameter('align_mode', self.ALIGN_MODE_SLIDING_WINDOW)
        self.declare_parameter('align_window_sec', 1.0)
        self.declare_parameter('align_window_min_ratio', 0.75)
        self.declare_parameter('align_window_min_samples', 12)
        self.declare_parameter('align_xy_tolerance_m', 0.18)
        self.declare_parameter('align_xy_mean_tolerance_m', 0.12)
        self.declare_parameter('align_z_tolerance_m', 0.15)
        self.declare_parameter('align_z_mean_tolerance_m', 0.10)
        self.declare_parameter('align_yaw_tolerance_rad', 0.12)
        self.declare_parameter('align_yaw_mean_tolerance_rad', 0.08)

        # ===== 降落 / 丢码 / 落地 =====
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

        # ===== 安全参数 =====
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
        self.declare_parameter('stop_cmd_after_disarm', True)

        # ===== Yaw 平滑参数 =====
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
        self.arming_service = self.get_parameter('arming_service').value
        self.status_topic = self.get_parameter('status_topic').value

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_z = float(self.get_parameter('target_z').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)

        kp_xy = float(self.get_parameter('kp_xy').value)
        ki_xy = float(self.get_parameter('ki_xy').value)
        kd_xy = float(self.get_parameter('kd_xy').value)

        kp_x_raw = float(self.get_parameter('kp_x').value)
        ki_x_raw = float(self.get_parameter('ki_x').value)
        kd_x_raw = float(self.get_parameter('kd_x').value)
        kp_y_raw = float(self.get_parameter('kp_y').value)
        ki_y_raw = float(self.get_parameter('ki_y').value)
        kd_y_raw = float(self.get_parameter('kd_y').value)

        self.x_use_split = (
            math.isfinite(kp_x_raw)
            or math.isfinite(ki_x_raw)
            or math.isfinite(kd_x_raw)
        )
        self.y_use_split = (
            math.isfinite(kp_y_raw)
            or math.isfinite(ki_y_raw)
            or math.isfinite(kd_y_raw)
        )

        self.kp_x = kp_x_raw if math.isfinite(kp_x_raw) else kp_xy
        self.ki_x = ki_x_raw if math.isfinite(ki_x_raw) else ki_xy
        self.kd_x = kd_x_raw if math.isfinite(kd_x_raw) else kd_xy
        self.kp_y = kp_y_raw if math.isfinite(kp_y_raw) else kp_xy
        self.ki_y = ki_y_raw if math.isfinite(ki_y_raw) else ki_xy
        self.kd_y = kd_y_raw if math.isfinite(kd_y_raw) else kd_xy

        self.kp_z = float(self.get_parameter('kp_z').value)
        self.ki_z = float(self.get_parameter('ki_z').value)
        self.kd_z = float(self.get_parameter('kd_z').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.ki_yaw = float(self.get_parameter('ki_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)

        self.camera_yaw_compensation_deg = float(
            self.get_parameter('camera_yaw_compensation_deg').value
        )
        self.camera_yaw_compensation_rad = math.radians(
            self.camera_yaw_compensation_deg
        )
        self.invert_yaw_rate_output = self.parameter_as_bool(
            self.get_parameter('invert_yaw_rate_output').value
        )

        self.align_mode = str(self.get_parameter('align_mode').value)
        self.align_window_sec = float(self.get_parameter('align_window_sec').value)
        self.align_window_min_ratio = float(
            self.get_parameter('align_window_min_ratio').value
        )
        self.align_window_min_samples = int(
            self.get_parameter('align_window_min_samples').value
        )
        self.align_xy_tolerance_m = float(
            self.get_parameter('align_xy_tolerance_m').value
        )
        self.align_xy_mean_tolerance_m = float(
            self.get_parameter('align_xy_mean_tolerance_m').value
        )
        self.align_z_tolerance_m = float(
            self.get_parameter('align_z_tolerance_m').value
        )
        self.align_z_mean_tolerance_m = float(
            self.get_parameter('align_z_mean_tolerance_m').value
        )
        self.align_yaw_tolerance_rad = float(
            self.get_parameter('align_yaw_tolerance_rad').value
        )
        self.align_yaw_mean_tolerance_rad = float(
            self.get_parameter('align_yaw_mean_tolerance_rad').value
        )

        self.descent_speed_mps = abs(
            float(self.get_parameter('descent_speed_mps').value)
        )
        self.marker_loss_hold_sec = float(
            self.get_parameter('marker_loss_hold_sec').value
        )
        self.touchdown_range_threshold_m = float(
            self.get_parameter('touchdown_range_threshold_m').value
        )
        self.touchdown_rel_alt_threshold_m = float(
            self.get_parameter('touchdown_rel_alt_threshold_m').value
        )
        self.land_vz_abs_max_mps = float(
            self.get_parameter('land_vz_abs_max_mps').value
        )
        self.land_vxy_abs_max_mps = float(
            self.get_parameter('land_vxy_abs_max_mps').value
        )
        self.land_detect_hold_sec = float(
            self.get_parameter('land_detect_hold_sec').value
        )
        self.heuristic_disarm_hold_sec = float(
            self.get_parameter('heuristic_disarm_hold_sec').value
        )
        self.min_throttle_descent_speed_mps = abs(
            float(self.get_parameter('min_throttle_descent_speed_mps').value)
        )
        self.min_throttle_disarm_duration_sec = float(
            self.get_parameter('min_throttle_disarm_duration_sec').value
        )
        self.disarm_retry_interval_sec = float(
            self.get_parameter('disarm_retry_interval_sec').value
        )

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
        self.distance_sensor_timeout_sec = float(
            self.get_parameter('distance_sensor_timeout_sec').value
        )
        self.rel_alt_timeout_sec = float(
            self.get_parameter('rel_alt_timeout_sec').value
        )
        self.local_velocity_timeout_sec = float(
            self.get_parameter('local_velocity_timeout_sec').value
        )
        self.v_limit = abs(float(self.get_parameter('v_limit').value))
        self.vxy_limit = float(self.get_parameter('vxy_limit').value)
        vx_limit_raw = float(self.get_parameter('vx_limit').value)
        vy_limit_raw = float(self.get_parameter('vy_limit').value)
        self.vx_limit = vx_limit_raw if math.isfinite(vx_limit_raw) else None
        self.vy_limit = vy_limit_raw if math.isfinite(vy_limit_raw) else None
        self.vz_limit = abs(float(self.get_parameter('vz_limit').value))
        self.yaw_rate_limit = abs(
            float(self.get_parameter('yaw_rate_limit').value)
        )
        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)
        self.require_offboard = self.parameter_as_bool(
            self.get_parameter('require_offboard').value
        )
        self.stop_cmd_after_disarm = self.parameter_as_bool(
            self.get_parameter('stop_cmd_after_disarm').value
        )
        self.enable_z_yaw_smoothing = self.parameter_as_bool(
            self.get_parameter('enable_z_yaw_smoothing').value
        )
        self.yaw_lpf_tau_sec = float(self.get_parameter('yaw_lpf_tau_sec').value)
        self.yaw_error_deadband = abs(
            float(self.get_parameter('yaw_error_deadband').value)
        )
        self.yaw_rate_slew_rate_limit = abs(
            float(self.get_parameter('yaw_rate_slew_rate_limit').value)
        )

        # ===== PID 控制器 =====
        self.pid_x = PIDController(
            self.kp_x,
            self.ki_x,
            self.kd_x,
            out_limit=None,
        )
        self.pid_y = PIDController(
            self.kp_y,
            self.ki_y,
            self.kd_y,
            out_limit=None,
        )
        self.pid_z = PIDController(
            self.kp_z,
            self.ki_z,
            self.kd_z,
            out_limit=self.vz_limit,
        )
        self.pid_yaw = PIDController(
            self.kp_yaw,
            self.ki_yaw,
            self.kd_yaw,
            out_limit=self.yaw_rate_limit,
        )

        # ===== yaw 平滑器 =====
        self.yaw_lpf = AngleLowPassFilter()
        self.yaw_rate_slew_limiter = SlewRateLimiter()

        # ===== 运行状态缓存 =====
        self.current_state = State()
        self.extended_state = ExtendedState()
        self.has_extended_state = False

        self.pose = ArucoBasePose()
        self.has_pose = False
        self.last_pose_time = None

        self.distance_sensor_msg: Optional[Range] = None
        self.last_distance_sensor_time = None

        self.rel_alt_m = float('nan')
        self.last_rel_alt_time = None

        self.vx_local = 0.0
        self.vy_local = 0.0
        self.vz_local = 0.0
        self.last_velocity_time = None

        self.phase = self.PHASE_WAIT_FOR_OFFBOARD
        self.align_window = deque()
        self.align_ok = False
        self.align_window_ratio = 0.0
        self.align_window_sample_count = 0
        self.align_window_mean_xy = float('nan')
        self.align_window_mean_z = float('nan')
        self.align_window_mean_yaw = float('nan')

        self.tracking_active = False
        self.range_only_active = False
        self.marker_loss_active = False
        self.marker_loss_start_time = None
        self.last_marker_loss_duration_sec = 0.0
        self.marker_lost_total_count = 0
        self.last_valid_tracking_vx = 0.0
        self.last_valid_tracking_vy = 0.0
        self.last_valid_tracking_time = None
        self.range_only_hold_active = False

        self.touchdown_candidate_active = False
        self.touchdown_candidate_ready = False
        self.touchdown_candidate_start_time = None
        self.heuristic_landed_start_time = None
        self.landed_by_extended_state = False
        self.landed_by_heuristic = False
        self.touchdown_height_source = 'none'
        self.touchdown_height_value = float('nan')

        self.min_throttle_start_time = None
        self.last_disarm_request_time = None
        self.disarm_requested = False
        self.disarm_in_flight = False

        self.last_ex_marker = float('nan')
        self.last_ey_marker = float('nan')
        self.last_ex_body = float('nan')
        self.last_ey_body = float('nan')
        self.last_yaw_rel_raw = float('nan')
        self.last_yaw_rel_corrected = float('nan')
        self.last_yaw_rel_corrected_ctrl = float('nan')
        self.last_eyaw = float('nan')
        self.last_eyaw_ctrl = float('nan')
        self.last_yaw_rate = 0.0

        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_vz = 0.0
        self.last_cmd_yaw_rate = 0.0
        self.latest_status = '等待进入 OFFBOARD'

        mavros_best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.state_sub = self.create_subscription(
            State,
            self.state_topic,
            self.state_callback,
            10,
        )
        self.extended_state_sub = self.create_subscription(
            ExtendedState,
            self.extended_state_topic,
            self.extended_state_callback,
            mavros_best_effort_qos,
        )
        self.pose_sub = self.create_subscription(
            ArucoBasePose,
            self.pose_topic,
            self.pose_callback,
            10,
        )
        self.distance_sensor_sub = self.create_subscription(
            Range,
            self.distance_sensor_topic,
            self.distance_sensor_callback,
            mavros_best_effort_qos,
        )
        self.rel_alt_sub = self.create_subscription(
            Float64,
            self.rel_alt_topic,
            self.rel_alt_callback,
            mavros_best_effort_qos,
        )
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            self.vel_local_topic,
            self.velocity_callback,
            mavros_best_effort_qos,
        )

        self.setpoint_pub = self.create_publisher(
            PositionTarget,
            self.setpoint_raw_topic,
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10,
        )

        self.arming_client = self.create_client(CommandBool, self.arming_service)

        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1.0),
            self.control_loop,
        )
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'land_v2_node 已启动 | '
            f'pose={self.pose_topic} | state={self.state_topic} | '
            f'extended_state={self.extended_state_topic} | '
            f'distance_sensor={self.distance_sensor_topic} | '
            f'rel_alt={self.rel_alt_topic} | vel_local={self.vel_local_topic} | '
            f'setpoint_raw={self.setpoint_raw_topic}'
        )
        self.get_logger().info(
            'PID参数 | '
            f'X=({self.kp_x:.3f},{self.ki_x:.3f},{self.kd_x:.3f}) '
            f'Y=({self.kp_y:.3f},{self.ki_y:.3f},{self.kd_y:.3f}) '
            f'Z=({self.kp_z:.3f},{self.ki_z:.3f},{self.kd_z:.3f}) '
            f'Yaw=({self.kp_yaw:.3f},{self.ki_yaw:.3f},{self.kd_yaw:.3f})'
        )
        self.get_logger().info(
            '速度限幅 | '
            f'v_limit={self.v_limit:.3f} | vz_limit={self.vz_limit:.3f} | '
            f'yaw_rate_limit={self.yaw_rate_limit:.3f} | '
            f'vx_limit={self.vx_limit if self.vx_limit is not None else "none"} | '
            f'vy_limit={self.vy_limit if self.vy_limit is not None else "none"}'
        )
        self.get_logger().info(
            '对准模式 | '
            f'align_mode={self.align_mode} | '
            f'window={self.align_window_sec:.2f}s | '
            f'min_ratio={self.align_window_min_ratio:.2f} | '
            f'xy_tol={self.align_xy_tolerance_m:.2f}m | '
            f'z_tol={self.align_z_tolerance_m:.2f}m | '
            f'yaw_tol={self.align_yaw_tolerance_rad:.3f}rad'
        )
        self.get_logger().info(
            'Yaw控制 | '
            f'target_yaw={self.target_yaw:.3f}rad | '
            f'enable_smoothing={self.enable_z_yaw_smoothing} | '
            f'yaw_lpf_tau={self.yaw_lpf_tau_sec:.3f}s | '
            f'yaw_deadband={self.yaw_error_deadband:.3f}rad | '
            f'yaw_rate_slew={self.yaw_rate_slew_rate_limit:.3f}rad/s^2 | '
            f'invert_yaw_rate_output={self.invert_yaw_rate_output}'
        )
        self.get_logger().info(
            '下降策略 | '
            f'descent_speed={self.descent_speed_mps:.2f}m/s | '
            f'marker_loss_hold={self.marker_loss_hold_sec:.2f}s | '
            f'min_throttle={self.min_throttle_descent_speed_mps:.2f}m/s'
        )

    def reset_xy_pids(self):
        """仅重置水平 PID，用于视觉超时或丢码恢复场景。"""
        self.pid_x.reset()
        self.pid_y.reset()

    def reset_yaw_control(self):
        """重置 yaw PID、低通和斜率限制器，避免旧视觉角度残留。"""
        self.pid_yaw.reset()
        self.yaw_lpf.reset()
        self.yaw_rate_slew_limiter.reset(0.0)
        self.last_yaw_rate = 0.0

    def reset_all_pids(self):
        """统一重置全部 PID。"""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()
        self.yaw_lpf.reset()
        self.yaw_rate_slew_limiter.reset(0.0)

    def clear_align_window(self):
        """视觉或测距条件不满足时清空滑动窗口。"""
        self.align_window.clear()
        self.align_ok = False
        self.align_window_ratio = 0.0
        self.align_window_sample_count = 0
        self.align_window_mean_xy = float('nan')
        self.align_window_mean_z = float('nan')
        self.align_window_mean_yaw = float('nan')

    def set_phase(self, new_phase: str, reason: str):
        """统一处理阶段切换日志，避免漏记切换原因。"""
        if self.phase == new_phase:
            return
        old_phase = self.phase
        self.phase = new_phase
        self.get_logger().info(f'阶段切换：{old_phase} -> {new_phase} | {reason}')

    def now_sec(self) -> float:
        """返回当前 ROS 时钟秒数，便于做持续时间判定。"""
        return self.get_clock().now().nanoseconds / 1e9

    def state_callback(self, msg: State):
        """缓存飞控状态，并在 OFFBOARD 上升沿启动任务。"""
        was_offboard = (self.current_state.mode == 'OFFBOARD')
        self.current_state = msg
        is_offboard = (msg.mode == 'OFFBOARD')

        if is_offboard and not was_offboard:
            self.handle_offboard_entry()

    def handle_offboard_entry(self):
        """进入 OFFBOARD 后重新初始化本次降落任务。"""
        self.reset_all_pids()
        self.clear_align_window()

        self.tracking_active = False
        self.range_only_active = False
        self.range_only_hold_active = False
        self.marker_loss_active = False
        self.marker_loss_start_time = None
        self.last_valid_tracking_time = None

        self.touchdown_candidate_active = False
        self.touchdown_candidate_ready = False
        self.touchdown_candidate_start_time = None
        self.heuristic_landed_start_time = None
        self.landed_by_extended_state = False
        self.landed_by_heuristic = False
        self.min_throttle_start_time = None

        self.last_disarm_request_time = None
        self.disarm_requested = False
        self.disarm_in_flight = False

        self.set_phase(
            self.PHASE_ALIGN_AT_TARGET_HEIGHT,
            '检测到 OFFBOARD 上升沿，开始在目标高度执行 XY/Z/Yaw 对准',
        )

    def extended_state_callback(self, msg: ExtendedState):
        self.extended_state = msg
        self.has_extended_state = True

    def pose_callback(self, msg: ArucoBasePose):
        self.pose = msg
        self.has_pose = True
        self.last_pose_time = self.get_clock().now()

    def distance_sensor_callback(self, msg: Range):
        self.distance_sensor_msg = msg
        self.last_distance_sensor_time = self.get_clock().now()

    def rel_alt_callback(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self.rel_alt_m = float(msg.data)
        self.last_rel_alt_time = self.get_clock().now()

    def velocity_callback(self, msg: TwistStamped):
        self.vx_local = float(msg.twist.linear.x)
        self.vy_local = float(msg.twist.linear.y)
        self.vz_local = float(msg.twist.linear.z)
        self.last_velocity_time = self.get_clock().now()

    def get_age_sec(self, stamp_time) -> float:
        """计算样本年龄；从未收到数据时返回极大值。"""
        if stamp_time is None:
            return 1e9
        return (self.get_clock().now() - stamp_time).nanoseconds / 1e9

    def is_pose_fresh(self) -> bool:
        """判断视觉数据是否仍在时效窗口内。"""
        if not self.has_pose or self.last_pose_time is None:
            return False
        return self.get_age_sec(self.last_pose_time) <= self.pose_timeout_sec

    def evaluate_distance_sensor(self) -> tuple[bool, float, float, str]:
        """检查测距仪数据是否可用。"""
        age_sec = self.get_age_sec(self.last_distance_sensor_time)
        if self.distance_sensor_msg is None:
            return False, float('nan'), age_sec, '距离传感器无效：从未收到数据'

        distance_sensor_z = float(self.distance_sensor_msg.range)
        min_range = float(self.distance_sensor_msg.min_range)
        max_range = float(self.distance_sensor_msg.max_range)

        if age_sec > self.distance_sensor_timeout_sec:
            return (
                False,
                distance_sensor_z,
                age_sec,
                f'距离传感器超时：age={age_sec:.3f}s',
            )

        if not math.isfinite(distance_sensor_z):
            return False, distance_sensor_z, age_sec, '距离传感器无效：Range.range 非有限数'

        lower_ok = (not math.isfinite(min_range)) or (distance_sensor_z >= min_range)
        upper_ok = (not math.isfinite(max_range)) or (distance_sensor_z <= max_range)
        if not (lower_ok and upper_ok):
            return (
                False,
                distance_sensor_z,
                age_sec,
                '距离值越界：'
                f'range={distance_sensor_z:.3f}m, '
                f'min={min_range:.3f}m, max={max_range:.3f}m',
            )

        return True, distance_sensor_z, age_sec, '距离传感器有效'

    def evaluate_rel_alt(self) -> tuple[bool, float, float, str]:
        """检查相对高度数据是否可用。"""
        age_sec = self.get_age_sec(self.last_rel_alt_time)
        if self.last_rel_alt_time is None:
            return False, float('nan'), age_sec, 'rel_alt 无效：从未收到数据'
        if age_sec > self.rel_alt_timeout_sec:
            return False, self.rel_alt_m, age_sec, f'rel_alt 超时：age={age_sec:.3f}s'
        if not math.isfinite(self.rel_alt_m):
            return False, self.rel_alt_m, age_sec, 'rel_alt 无效：非有限数'
        return True, self.rel_alt_m, age_sec, 'rel_alt 有效'

    def evaluate_local_velocity(self) -> tuple[bool, float, float, float, str]:
        """检查本地速度是否可用于接地启发式。"""
        age_sec = self.get_age_sec(self.last_velocity_time)
        if self.last_velocity_time is None:
            return False, float('nan'), float('nan'), age_sec, '本地速度无效：从未收到数据'
        if age_sec > self.local_velocity_timeout_sec:
            return False, float('nan'), float('nan'), age_sec, f'本地速度超时：age={age_sec:.3f}s'

        vxy = math.hypot(self.vx_local, self.vy_local)
        if not (
            math.isfinite(vxy)
            and math.isfinite(self.vx_local)
            and math.isfinite(self.vy_local)
            and math.isfinite(self.vz_local)
        ):
            return False, float('nan'), float('nan'), age_sec, '本地速度无效：存在非有限数'

        return True, vxy, self.vz_local, age_sec, '本地速度有效'

    def compute_tracking_error(self):
        """
        计算视觉误差，并严格保持 `pid_tuning_v4` 的误差定义与符号约定。
        """
        ex_marker = self.target_x - float(self.pose.x)
        ey_marker = float(self.pose.y) - self.target_y
        yaw_rel_raw = self.wrap_to_pi(-float(self.pose.yaw))
        yaw_rel_corrected = self.wrap_to_pi(
            yaw_rel_raw + self.camera_yaw_compensation_rad
        )
        ex_body, ey_body = self.rotate_marker_error_to_body(
            ex_marker,
            ey_marker,
            yaw_rel_corrected,
        )

        self.last_ex_marker = ex_marker
        self.last_ey_marker = ey_marker
        self.last_ex_body = ex_body
        self.last_ey_body = ey_body
        self.last_yaw_rel_raw = yaw_rel_raw
        self.last_yaw_rel_corrected = yaw_rel_corrected
        return ex_marker, ey_marker, yaw_rel_raw, yaw_rel_corrected, ex_body, ey_body

    def apply_horizontal_limits(self, vx_raw: float, vy_raw: float) -> tuple[float, float]:
        """先做 `v_limit` 矢量限幅，再按需执行分轴限幅和死区。"""
        vx_limited, vy_limited, _v_norm, _scale = self.limit_horizontal_velocity(
            vx_raw,
            vy_raw,
            self.v_limit,
        )
        if self.vx_limit is not None:
            vx_limited = max(-self.vx_limit, min(self.vx_limit, vx_limited))
        if self.vy_limit is not None:
            vy_limited = max(-self.vy_limit, min(self.vy_limit, vy_limited))

        vx = self.apply_deadband(vx_limited, self.velocity_deadband)
        vy = self.apply_deadband(vy_limited, self.velocity_deadband)
        return vx, vy

    def compute_yaw_command(self) -> tuple[float, float, float]:
        """
        计算 yaw_rate 指令。

        `last_yaw_rel_corrected` 来自当前视觉帧，XY 旋转仍使用未滤波值；
        yaw 闭环可选使用角度低通和斜率限制，避免接近目标时来回抖动。
        """
        yaw_rel_corrected_raw = self.last_yaw_rel_corrected
        if not math.isfinite(yaw_rel_corrected_raw):
            self.reset_yaw_control()
            self.last_yaw_rel_corrected_ctrl = float('nan')
            self.last_eyaw = float('nan')
            self.last_eyaw_ctrl = float('nan')
            return 0.0, float('nan'), float('nan')

        yaw_rel_corrected_ctrl = yaw_rel_corrected_raw
        if self.enable_z_yaw_smoothing:
            yaw_rel_corrected_ctrl = self.yaw_lpf.update(
                yaw_rel_corrected_raw,
                self.yaw_lpf_tau_sec,
            )
        else:
            self.yaw_lpf.reset()

        eyaw = self.wrap_to_pi(self.target_yaw - yaw_rel_corrected_raw)
        eyaw_ctrl = self.wrap_to_pi(self.target_yaw - yaw_rel_corrected_ctrl)
        if self.enable_z_yaw_smoothing:
            # 误差死区在 PID 前处理，小角度残差直接视为已对准。
            eyaw_ctrl = self.apply_deadband(eyaw_ctrl, self.yaw_error_deadband)

        yaw_rate = self.pid_yaw.update(eyaw_ctrl)
        if self.invert_yaw_rate_output:
            yaw_rate = -yaw_rate

        if self.enable_z_yaw_smoothing:
            yaw_rate = self.yaw_rate_slew_limiter.update(
                yaw_rate,
                self.yaw_rate_slew_rate_limit,
            )
        else:
            self.yaw_rate_slew_limiter.reset(yaw_rate)

        self.last_yaw_rel_corrected_ctrl = yaw_rel_corrected_ctrl
        self.last_eyaw = eyaw
        self.last_eyaw_ctrl = eyaw_ctrl
        self.last_yaw_rate = yaw_rate
        return yaw_rate, eyaw, eyaw_ctrl

    def update_align_window(
        self,
        now_sec: float,
        xy_error_m: float,
        z_error_m: float,
        yaw_error_rad: float,
    ):
        """把当前 XY/Z/Yaw 对准样本压入滑动窗口。"""
        within_threshold = (
            xy_error_m <= self.align_xy_tolerance_m
            and z_error_m <= self.align_z_tolerance_m
            and yaw_error_rad <= self.align_yaw_tolerance_rad
        )
        self.align_window.append(
            (now_sec, xy_error_m, z_error_m, yaw_error_rad, within_threshold)
        )

        window_begin = now_sec - self.align_window_sec
        while self.align_window and self.align_window[0][0] < window_begin:
            self.align_window.popleft()

        sample_count = len(self.align_window)
        if sample_count == 0:
            self.align_ok = False
            self.align_window_ratio = 0.0
            self.align_window_sample_count = 0
            self.align_window_mean_xy = float('nan')
            self.align_window_mean_z = float('nan')
            self.align_window_mean_yaw = float('nan')
            return

        pass_count = sum(1 for sample in self.align_window if sample[4])
        xy_sum = sum(sample[1] for sample in self.align_window)
        z_sum = sum(sample[2] for sample in self.align_window)
        yaw_sum = sum(sample[3] for sample in self.align_window)
        ratio = pass_count / float(sample_count)
        mean_xy = xy_sum / float(sample_count)
        mean_z = z_sum / float(sample_count)
        mean_yaw = yaw_sum / float(sample_count)

        self.align_window_ratio = ratio
        self.align_window_sample_count = sample_count
        self.align_window_mean_xy = mean_xy
        self.align_window_mean_z = mean_z
        self.align_window_mean_yaw = mean_yaw
        self.align_ok = (
            sample_count >= self.align_window_min_samples
            and ratio >= self.align_window_min_ratio
            and mean_xy <= self.align_xy_mean_tolerance_m
            and mean_z <= self.align_z_mean_tolerance_m
            and mean_yaw <= self.align_yaw_mean_tolerance_rad
        )

    def publish_body_velocity(
        self,
        vx_body: float,
        vy_body: float,
        vz_body_flu: float,
        yaw_rate: float,
    ):
        """
        发布 BODY_NED 速度指令。

        语义继承 `pid_tuning_v4` 的 BODY_NED 速度输出：
        - 输出话题 `/mavros/setpoint_raw/local`
        - `FRAME_BODY_NED`
        - `msg.velocity.x = vx_body`
        - `msg.velocity.y = -vy_body`
        - `msg.velocity.z = vz_body_flu`
        - 视觉新鲜的对准/下降阶段允许发布 yaw_rate 闭环指令
        """
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
        )
        msg.velocity.x = float(vx_body)
        msg.velocity.y = float(-vy_body)
        msg.velocity.z = float(vz_body_flu)
        msg.yaw_rate = float(yaw_rate)
        self.setpoint_pub.publish(msg)

        self.last_cmd_vx = float(vx_body)
        self.last_cmd_vy = float(vy_body)
        self.last_cmd_vz = float(vz_body_flu)
        self.last_cmd_yaw_rate = float(yaw_rate)
        self.last_yaw_rate = float(yaw_rate)

    def publish_status(self):
        """发布结构化运行状态，供独立 logger 节点记录 CSV。"""
        status_msg = String()
        status_msg.data = json.dumps({
            'phase': self.phase,
            'align_mode': self.align_mode,
            'align_ok': self.align_ok,
            'align_window_ratio': self.align_window_ratio,
            'align_window_sample_count': self.align_window_sample_count,
            'align_window_mean_xy': self.align_window_mean_xy,
            'align_window_mean_z': self.align_window_mean_z,
            'align_window_mean_yaw': self.align_window_mean_yaw,
            'target_yaw': self.target_yaw,
            'yaw_rel_raw': self.last_yaw_rel_raw,
            'yaw_rel_corrected': self.last_yaw_rel_corrected,
            'yaw_rel_corrected_ctrl': self.last_yaw_rel_corrected_ctrl,
            'eyaw': self.last_eyaw,
            'eyaw_ctrl': self.last_eyaw_ctrl,
            'yaw_rate': self.last_yaw_rate,
            'tracking_active': self.tracking_active,
            'range_only_active': self.range_only_active,
            'range_only_hold_active': self.range_only_hold_active,
            'marker_loss_active': self.marker_loss_active,
            'marker_lost_total_count': self.marker_lost_total_count,
            'last_marker_loss_duration_sec': self.last_marker_loss_duration_sec,
            'touchdown_candidate_active': self.touchdown_candidate_active,
            'touchdown_candidate_ready': self.touchdown_candidate_ready,
            'landed_by_extended_state': self.landed_by_extended_state,
            'landed_by_heuristic': self.landed_by_heuristic,
            'touchdown_height_source': self.touchdown_height_source,
            'touchdown_height_value': self.touchdown_height_value,
            'disarm_requested': self.disarm_requested,
            'latest_status': self.latest_status,
        })
        self.status_pub.publish(status_msg)

    def handle_marker_loss_entry(self, now_sec: float):
        """进入丢码续降阶段：先保留上一次速度，再逐步转为纯测距下降。"""
        if self.marker_loss_active:
            return

        self.marker_loss_active = True
        self.marker_loss_start_time = now_sec
        self.marker_lost_total_count += 1
        self.range_only_hold_active = True
        self.tracking_active = False
        self.range_only_active = True
        self.reset_xy_pids()
        self.reset_yaw_control()
        self.set_phase(
            self.PHASE_DESCEND_RANGE_ONLY,
            '下降中视觉超时，进入丢码保速 0.4s + 纯测距下降模式',
        )

    def handle_marker_reacquired(self, now_sec: float):
        """重新看到新鲜 ArUco 后恢复 XY 跟踪。"""
        if not self.marker_loss_active:
            return

        if self.marker_loss_start_time is not None:
            self.last_marker_loss_duration_sec = now_sec - self.marker_loss_start_time

        self.marker_loss_active = False
        self.marker_loss_start_time = None
        self.range_only_hold_active = False
        self.range_only_active = False
        self.tracking_active = True
        self.reset_xy_pids()
        self.reset_yaw_control()
        self.set_phase(
            self.PHASE_DESCEND_WITH_TRACK,
            '重新识别到新鲜 ArUco 数据，恢复 XY/Yaw 跟踪下降',
        )

    def is_landed_by_extended_state(self) -> bool:
        """优先使用 PX4/MAVROS 的 landed_state=ON_GROUND。"""
        if not self.has_extended_state:
            return False
        return self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def update_touchdown_flags(self, now_sec: float):
        """更新接地判定相关状态。"""
        self.landed_by_extended_state = self.is_landed_by_extended_state()

        distance_valid, distance_sensor_z, _, _ = self.evaluate_distance_sensor()
        rel_alt_valid, rel_alt_m, _, _ = self.evaluate_rel_alt()
        vel_valid, vxy_local, vz_local, _, _ = self.evaluate_local_velocity()

        if distance_valid:
            height_ok = (distance_sensor_z <= self.touchdown_range_threshold_m)
            self.touchdown_height_source = 'distance_sensor'
            self.touchdown_height_value = distance_sensor_z
        elif rel_alt_valid:
            height_ok = (rel_alt_m <= self.touchdown_rel_alt_threshold_m)
            self.touchdown_height_source = 'rel_alt'
            self.touchdown_height_value = rel_alt_m
        else:
            height_ok = False
            self.touchdown_height_source = 'none'
            self.touchdown_height_value = float('nan')

        velocity_ok = (
            vel_valid
            and abs(vz_local) <= self.land_vz_abs_max_mps
            and vxy_local <= self.land_vxy_abs_max_mps
        )
        candidate = height_ok and velocity_ok
        self.touchdown_candidate_active = candidate

        if candidate:
            if self.touchdown_candidate_start_time is None:
                self.touchdown_candidate_start_time = now_sec
            if self.heuristic_landed_start_time is None:
                self.heuristic_landed_start_time = now_sec
        else:
            self.touchdown_candidate_start_time = None
            self.heuristic_landed_start_time = None

        self.touchdown_candidate_ready = (
            candidate
            and self.touchdown_candidate_start_time is not None
            and (now_sec - self.touchdown_candidate_start_time) >= self.land_detect_hold_sec
        )
        self.landed_by_heuristic = (
            candidate
            and self.heuristic_landed_start_time is not None
            and (now_sec - self.heuristic_landed_start_time)
            >= self.heuristic_disarm_hold_sec
        )

    def can_continue_descent(self) -> tuple[bool, str]:
        """判断下降阶段是否具备基本高度监视能力。"""
        distance_valid, _, _, _ = self.evaluate_distance_sensor()
        rel_alt_valid, _, _, _ = self.evaluate_rel_alt()

        if distance_valid:
            return True, '测距仪有效，允许继续下降'
        if rel_alt_valid:
            return True, '测距仪超时，但 rel_alt 有效，允许继续下降'
        return False, '测距仪与 rel_alt 同时失效，暂停盲降'

    def try_send_disarm(self):
        """调用 MAVROS arming 服务发送 disarm 请求（带重试节流）。"""
        now_sec = self.now_sec()
        if self.disarm_in_flight:
            return
        if self.last_disarm_request_time is not None:
            if (now_sec - self.last_disarm_request_time) < self.disarm_retry_interval_sec:
                return

        if not self.arming_client.wait_for_service(timeout_sec=0.05):
            self.latest_status = '已进入 TOUCHDOWN_DISARM，但 arming 服务暂不可用，等待重试'
            return

        req = CommandBool.Request()
        req.value = False
        future = self.arming_client.call_async(req)
        self.disarm_requested = True
        self.disarm_in_flight = True
        self.last_disarm_request_time = now_sec

        def _done_cb(done_future):
            self.disarm_in_flight = False
            try:
                res = done_future.result()
            except Exception as exc:
                self.get_logger().warn(f'disarm 调用异常: {exc}')
                return

            if res.success:
                self.get_logger().info('disarm 请求已发送，飞控返回 success')
            else:
                self.get_logger().warn('disarm 请求返回失败，将按重试间隔继续尝试')

        future.add_done_callback(_done_cb)

    def compute_align_command(
        self,
        distance_sensor_z: float,
    ) -> tuple[float, float, float, float, float, float, float, float]:
        """计算对准阶段的 XY 跟踪、测距仪 Z 保持和 yaw 对准控制量。"""
        (
            ex_marker,
            ey_marker,
            _yaw_rel_raw,
            _yaw_rel_corrected,
            ex_body,
            ey_body,
        ) = self.compute_tracking_error()

        vx_raw = self.pid_x.update(ex_body)
        vy_raw = self.pid_y.update(ey_body)
        vx, vy = self.apply_horizontal_limits(vx_raw, vy_raw)
        ez = self.target_z - distance_sensor_z
        vz = self.apply_deadband(
            self.pid_z.update(ez),
            self.velocity_deadband,
        )
        yaw_rate, eyaw, _eyaw_ctrl = self.compute_yaw_command()
        return vx, vy, vz, yaw_rate, ex_marker, ey_marker, ez, eyaw

    def compute_xy_yaw_tracking_command(self) -> tuple[float, float, float, float]:
        """计算下降阶段的 XY 跟踪速度和 yaw_rate。"""
        (
            _ex_marker,
            _ey_marker,
            _yaw_rel_raw,
            _yaw_rel_corrected,
            ex_body,
            ey_body,
        ) = self.compute_tracking_error()

        vx_raw = self.pid_x.update(ex_body)
        vy_raw = self.pid_y.update(ey_body)
        vx, vy = self.apply_horizontal_limits(vx_raw, vy_raw)
        yaw_rate, eyaw, _eyaw_ctrl = self.compute_yaw_command()
        return vx, vy, yaw_rate, eyaw

    def hold_zero_and_reset(self, status_text: str):
        """统一的零输出保护。"""
        self.reset_all_pids()
        self.clear_align_window()
        self.tracking_active = False
        self.range_only_active = False
        self.range_only_hold_active = False
        self.marker_loss_active = False
        self.touchdown_candidate_active = False
        self.touchdown_candidate_ready = False
        self.landed_by_extended_state = False
        self.landed_by_heuristic = False
        self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
        self.latest_status = status_text
        self.publish_status()

    def control_loop(self):
        """主控制循环。"""
        now_sec = self.now_sec()
        self.update_touchdown_flags(now_sec)

        if not self.current_state.connected:
            self.set_phase(
                self.PHASE_WAIT_FOR_OFFBOARD,
                '飞控尚未连接，保持零输出保护',
            )
            self.hold_zero_and_reset('飞控未连接，输出零速并重置 PID')
            return

        if self.require_offboard and self.current_state.mode != 'OFFBOARD':
            self.set_phase(
                self.PHASE_WAIT_FOR_OFFBOARD,
                '未处于 OFFBOARD，保持零输出保护',
            )
            self.hold_zero_and_reset(
                f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速并重置 PID'
            )
            return

        if self.phase == self.PHASE_WAIT_FOR_OFFBOARD:
            self.handle_offboard_entry()

        if not self.current_state.armed and self.phase in (
            self.PHASE_DESCEND_WITH_TRACK,
            self.PHASE_DESCEND_RANGE_ONLY,
            self.PHASE_TOUCHDOWN_DISARM,
        ):
            self.set_phase(self.PHASE_DONE, '检测到 armed=false，降落流程完成')

        if self.phase == self.PHASE_DONE:
            vz_done = 0.0 if self.stop_cmd_after_disarm else -self.descent_speed_mps
            self.publish_body_velocity(0.0, 0.0, vz_done, 0.0)
            self.tracking_active = False
            self.range_only_active = False
            self.range_only_hold_active = False
            self.latest_status = '阶段=DONE | 已 disarm，保持最终安全输出'
            self.publish_status()
            return

        distance_valid, distance_sensor_z, _distance_age, distance_status = (
            self.evaluate_distance_sensor()
        )
        _rel_alt_valid, _rel_alt_m, _rel_alt_age, rel_alt_status = self.evaluate_rel_alt()

        if self.phase == self.PHASE_ALIGN_AT_TARGET_HEIGHT:
            self.tracking_active = False
            self.range_only_active = False
            self.range_only_hold_active = False

            if not distance_valid:
                self.reset_xy_pids()
                self.reset_yaw_control()
                self.pid_z.reset()
                self.clear_align_window()
                self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
                self.latest_status = (
                    '阶段=ALIGN_AT_TARGET_HEIGHT | '
                    f'{distance_status}，对准阶段暂停，等待测距恢复'
                )
                self.publish_status()
                return

            if not self.is_pose_fresh():
                self.reset_xy_pids()
                self.reset_yaw_control()
                self.clear_align_window()
                ez = self.target_z - distance_sensor_z
                vz = self.apply_deadband(
                    self.pid_z.update(ez),
                    self.velocity_deadband,
                )
                self.publish_body_velocity(0.0, 0.0, vz, 0.0)
                self.latest_status = (
                    '阶段=ALIGN_AT_TARGET_HEIGHT | '
                    f'视觉超时，保持 2.5m 高度等待重获 | '
                    f'distance_sensor_z={distance_sensor_z:.3f}m, ez={ez:.3f}, vz={vz:.3f}'
                )
                self.publish_status()
                return

            vx, vy, vz, yaw_rate, _ex_marker, _ey_marker, ez, eyaw = (
                self.compute_align_command(distance_sensor_z)
            )
            xy_error_norm = math.hypot(self.last_ex_body, self.last_ey_body)
            yaw_error_abs = abs(eyaw) if math.isfinite(eyaw) else float('inf')
            self.update_align_window(now_sec, xy_error_norm, abs(ez), yaw_error_abs)
            self.publish_body_velocity(vx, vy, vz, yaw_rate)

            self.latest_status = (
                '阶段=ALIGN_AT_TARGET_HEIGHT | '
                f'ex_body={self.last_ex_body:.3f}, ey_body={self.last_ey_body:.3f}, '
                f'xy={xy_error_norm:.3f}m | '
                f'eyaw={eyaw:.3f}rad, yaw_rate={yaw_rate:.3f} | '
                f'distance_sensor_z={distance_sensor_z:.3f}m, ez={ez:.3f} | '
                f'window_ratio={self.align_window_ratio:.3f}, '
                f'mean_xy={self.align_window_mean_xy:.3f}, '
                f'mean_z={self.align_window_mean_z:.3f}, '
                f'mean_yaw={self.align_window_mean_yaw:.3f} | '
                f'vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}'
            )

            if self.align_ok:
                self.pid_z.reset()
                self.reset_yaw_control()
                self.set_phase(
                    self.PHASE_DESCEND_WITH_TRACK,
                    '滑动窗口判定 XY/Z/Yaw 对准成功，开始持续跟踪下降',
                )

            self.publish_status()
            return

        if self.phase in (
            self.PHASE_DESCEND_WITH_TRACK,
            self.PHASE_DESCEND_RANGE_ONLY,
        ) and (self.landed_by_extended_state or self.landed_by_heuristic):
            self.reset_xy_pids()
            self.reset_yaw_control()
            self.pid_z.reset()
            self.min_throttle_start_time = now_sec
            reason = (
                '飞控 landed_state=ON_GROUND'
                if self.landed_by_extended_state
                else '启发式落地确认成立'
            )
            self.set_phase(self.PHASE_TOUCHDOWN_DISARM, reason)

        if self.phase == self.PHASE_TOUCHDOWN_DISARM:
            if not self.current_state.armed:
                self.set_phase(self.PHASE_DONE, 'TOUCHDOWN_DISARM 阶段检测到 armed=false')
                self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
                self.latest_status = '阶段=DONE | 已 disarm，停止发送下压指令'
                self.publish_status()
                return

            if self.min_throttle_start_time is None:
                self.min_throttle_start_time = now_sec

            hold_dt = now_sec - self.min_throttle_start_time
            self.tracking_active = False
            self.range_only_active = False
            self.range_only_hold_active = False
            self.publish_body_velocity(
                0.0,
                0.0,
                -self.min_throttle_descent_speed_mps,
                0.0,
            )
            self.try_send_disarm()
            self.latest_status = (
                '阶段=TOUCHDOWN_DISARM | '
                f'vz=-{self.min_throttle_descent_speed_mps:.2f}m/s | '
                f'hold={hold_dt:.2f}/{self.min_throttle_disarm_duration_sec:.2f}s | '
                f'ext_landed={int(self.landed_by_extended_state)} | '
                f'heuristic_landed={int(self.landed_by_heuristic)}'
            )
            self.publish_status()
            return

        descent_allowed, descent_status = self.can_continue_descent()
        descent_vz = -self.descent_speed_mps if descent_allowed else 0.0

        if self.phase == self.PHASE_DESCEND_WITH_TRACK and not self.is_pose_fresh():
            self.handle_marker_loss_entry(now_sec)

        if self.phase == self.PHASE_DESCEND_RANGE_ONLY and self.is_pose_fresh():
            self.handle_marker_reacquired(now_sec)

        if self.phase == self.PHASE_DESCEND_WITH_TRACK:
            self.tracking_active = True
            self.range_only_active = False
            self.range_only_hold_active = False

            vx, vy, yaw_rate, eyaw = self.compute_xy_yaw_tracking_command()
            self.last_valid_tracking_vx = vx
            self.last_valid_tracking_vy = vy
            self.last_valid_tracking_time = now_sec
            self.publish_body_velocity(vx, vy, descent_vz, yaw_rate)
            self.latest_status = (
                '阶段=DESCEND_WITH_TRACK | '
                f'ex_body={self.last_ex_body:.3f}, ey_body={self.last_ey_body:.3f} | '
                f'eyaw={eyaw:.3f}rad, yaw_rate={yaw_rate:.3f} | '
                f'vx={vx:.3f}, vy={vy:.3f}, vz={descent_vz:.3f} | '
                f'{descent_status}'
            )
            self.publish_status()
            return

        if self.phase == self.PHASE_DESCEND_RANGE_ONLY:
            self.tracking_active = False
            self.range_only_active = True

            loss_dt = (
                0.0
                if self.marker_loss_start_time is None
                else (now_sec - self.marker_loss_start_time)
            )
            if (
                self.marker_loss_start_time is not None
                and loss_dt <= self.marker_loss_hold_sec
            ):
                vx = self.last_valid_tracking_vx
                vy = self.last_valid_tracking_vy
                self.range_only_hold_active = True
            else:
                vx = 0.0
                vy = 0.0
                self.range_only_hold_active = False

            self.publish_body_velocity(vx, vy, descent_vz, 0.0)
            self.latest_status = (
                '阶段=DESCEND_RANGE_ONLY | '
                f'loss_dt={loss_dt:.3f}s | '
                f'hold_last_vxy={int(self.range_only_hold_active)} | '
                f'vx={vx:.3f}, vy={vy:.3f}, vz={descent_vz:.3f} | '
                f'{descent_status} | '
                f'{distance_status} | {rel_alt_status}'
            )
            self.publish_status()
            return

        self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
        self.latest_status = '未知阶段，回退到零输出保护'
        self.publish_status()

    def log_callback(self):
        """按 1Hz 输出状态摘要，便于现场观察阶段切换与安全保护。"""
        self.get_logger().info(self.latest_status)


def main(args=None):
    rclpy.init(args=args)
    node = LandV2Node()
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
