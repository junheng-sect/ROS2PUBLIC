#!/usr/bin/env python3

import math
import time

import rclpy
from debug_interface.msg import ArucoBasePose
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, HomePosition, State
from mavros_msgs.srv import CommandBool
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


class PIDController:
    """简单 PID 控制器（带积分限幅和输出限幅）。"""

    def __init__(self, kp=0.5, ki=0.0, kd=0.05, out_limit=1.0, i_limit=2.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_limit = abs(float(out_limit))
        self.i_limit = abs(float(i_limit))

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def update(self, error: float) -> float:
        now = time.time()
        dt = now - self.prev_time
        if dt <= 1e-6:
            dt = 1e-2

        # P 项：直接按当前误差线性输出。
        p_term = self.kp * error

        # I 项：累计历史误差并限幅，避免积分过冲。
        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        i_term = self.ki * self.integral

        # D 项：使用误差变化率抑制振荡。
        d_term = self.kd * (error - self.prev_error) / dt

        out = p_term + i_term + d_term
        out = max(-self.out_limit, min(self.out_limit, out))

        self.prev_error = error
        self.prev_time = now
        return out


class TotalNode(Node):
    """总任务控制：返航 -> ArUco 对正 -> 悬停1s -> 降落到 disarm。"""

    PHASE_ASCEND = 'ASCEND'
    PHASE_RETURN = 'RETURN'
    PHASE_TRACK = 'TRACK'
    PHASE_HOVER_BEFORE_LAND = 'HOVER_BEFORE_LAND'
    PHASE_LAND = 'LAND'
    PHASE_TOUCHDOWN_DISARM = 'TOUCHDOWN_DISARM'
    PHASE_DONE = 'DONE'

    def __init__(self):
        super().__init__('total_node')

        # ===== 话题与服务 =====
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('extended_state_topic', '/mavros/extended_state')
        self.declare_parameter('global_topic', '/mavros/global_position/global')
        self.declare_parameter('home_topic', '/mavros/home_position/home')
        self.declare_parameter('rel_alt_topic', '/mavros/global_position/rel_alt')
        self.declare_parameter('aruco_pose_topic', '/debug/aruco_pose')
        self.declare_parameter('base_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('vel_local_topic', '/mavros/local_position/velocity_local')
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel')
        self.declare_parameter('arming_service', '/mavros/cmd/arming')

        # ===== 任务目标 =====
        self.declare_parameter('target_alt_m', 3.0)
        self.declare_parameter('alt_tolerance_m', 0.15)
        self.declare_parameter('home_tolerance_m', 0.50)

        self.declare_parameter('track_target_x', 0.0)
        self.declare_parameter('track_target_y', 0.0)
        self.declare_parameter('track_target_yaw', 0.0)
        self.declare_parameter('xy_align_tolerance_m', 0.20)
        self.declare_parameter('yaw_align_tolerance_deg', 8.0)
        self.declare_parameter('track_align_hold_sec', 0.5)
        self.declare_parameter('hover_after_align_sec', 1.0)

        # ===== PID =====
        self.declare_parameter('kp_alt', 0.8)
        self.declare_parameter('ki_alt', 0.0)
        self.declare_parameter('kd_alt', 0.05)

        self.declare_parameter('kp_return_xy', 0.35)
        self.declare_parameter('ki_return_xy', 0.0)
        self.declare_parameter('kd_return_xy', 0.08)

        self.declare_parameter('kp_track_xy', 0.5)
        self.declare_parameter('ki_track_xy', 0.0)
        self.declare_parameter('kd_track_xy', 0.08)

        self.declare_parameter('kp_yaw', 1.2)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.08)

        # ===== 降落参数（继承 landing 逻辑） =====
        self.declare_parameter('descent_speed_mps', 0.5)
        self.declare_parameter('touchdown_descent_speed_mps', 0.15)
        self.declare_parameter('land_rel_alt_threshold_m', 0.15)
        self.declare_parameter('land_vz_abs_max_mps', 0.20)
        self.declare_parameter('land_vxy_abs_max_mps', 0.25)
        self.declare_parameter('heuristic_disarm_hold_sec', 3.0)
        self.declare_parameter('min_throttle_descent_speed_mps', 0.35)
        self.declare_parameter('min_throttle_disarm_duration_sec', 5.0)

        # ===== 通用安全参数 =====
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('base_pose_timeout_sec', 0.5)
        self.declare_parameter('max_vxy', 1.0)
        self.declare_parameter('max_vz', 0.8)
        self.declare_parameter('max_wz', 1.0)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('yaw_rate_deadband', 0.02)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('start_on_offboard_entry', True)
        self.declare_parameter('use_dynamic_marker_yaw', True)
        self.declare_parameter('fallback_marker_in_map_yaw_deg', 90.0)
        self.declare_parameter('disarm_retry_interval_sec', 1.0)

        self.state_topic = self.get_parameter('state_topic').value
        self.extended_state_topic = self.get_parameter('extended_state_topic').value
        self.global_topic = self.get_parameter('global_topic').value
        self.home_topic = self.get_parameter('home_topic').value
        self.rel_alt_topic = self.get_parameter('rel_alt_topic').value
        self.aruco_pose_topic = self.get_parameter('aruco_pose_topic').value
        self.base_pose_topic = self.get_parameter('base_pose_topic').value
        self.vel_local_topic = self.get_parameter('vel_local_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.arming_service = self.get_parameter('arming_service').value

        self.target_alt_m = float(self.get_parameter('target_alt_m').value)
        self.alt_tolerance_m = float(self.get_parameter('alt_tolerance_m').value)
        self.home_tolerance_m = float(self.get_parameter('home_tolerance_m').value)
        self.track_target_x = float(self.get_parameter('track_target_x').value)
        self.track_target_y = float(self.get_parameter('track_target_y').value)
        self.track_target_yaw = float(self.get_parameter('track_target_yaw').value)
        self.xy_align_tolerance_m = float(self.get_parameter('xy_align_tolerance_m').value)
        self.yaw_align_tolerance_deg = float(self.get_parameter('yaw_align_tolerance_deg').value)
        self.track_align_hold_sec = float(self.get_parameter('track_align_hold_sec').value)
        self.hover_after_align_sec = float(self.get_parameter('hover_after_align_sec').value)

        kp_alt = float(self.get_parameter('kp_alt').value)
        ki_alt = float(self.get_parameter('ki_alt').value)
        kd_alt = float(self.get_parameter('kd_alt').value)
        kp_return_xy = float(self.get_parameter('kp_return_xy').value)
        ki_return_xy = float(self.get_parameter('ki_return_xy').value)
        kd_return_xy = float(self.get_parameter('kd_return_xy').value)
        kp_track_xy = float(self.get_parameter('kp_track_xy').value)
        ki_track_xy = float(self.get_parameter('ki_track_xy').value)
        kd_track_xy = float(self.get_parameter('kd_track_xy').value)
        kp_yaw = float(self.get_parameter('kp_yaw').value)
        ki_yaw = float(self.get_parameter('ki_yaw').value)
        kd_yaw = float(self.get_parameter('kd_yaw').value)

        self.descent_speed_mps = abs(float(self.get_parameter('descent_speed_mps').value))
        self.touchdown_descent_speed_mps = abs(float(self.get_parameter('touchdown_descent_speed_mps').value))
        self.land_rel_alt_threshold_m = float(self.get_parameter('land_rel_alt_threshold_m').value)
        self.land_vz_abs_max_mps = float(self.get_parameter('land_vz_abs_max_mps').value)
        self.land_vxy_abs_max_mps = float(self.get_parameter('land_vxy_abs_max_mps').value)
        self.heuristic_disarm_hold_sec = float(self.get_parameter('heuristic_disarm_hold_sec').value)
        self.min_throttle_descent_speed_mps = abs(float(self.get_parameter('min_throttle_descent_speed_mps').value))
        self.min_throttle_disarm_duration_sec = float(self.get_parameter('min_throttle_disarm_duration_sec').value)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
        self.base_pose_timeout_sec = float(self.get_parameter('base_pose_timeout_sec').value)
        self.max_vxy = float(self.get_parameter('max_vxy').value)
        self.max_vz = float(self.get_parameter('max_vz').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)
        self.yaw_rate_deadband = float(self.get_parameter('yaw_rate_deadband').value)
        self.require_offboard = bool(self.get_parameter('require_offboard').value)
        self.start_on_offboard_entry = bool(self.get_parameter('start_on_offboard_entry').value)
        self.use_dynamic_marker_yaw = bool(self.get_parameter('use_dynamic_marker_yaw').value)
        self.fallback_marker_in_map_yaw_deg = float(self.get_parameter('fallback_marker_in_map_yaw_deg').value)
        self.disarm_retry_interval_sec = float(self.get_parameter('disarm_retry_interval_sec').value)

        # ===== PID =====
        self.pid_alt = PIDController(kp_alt, ki_alt, kd_alt, out_limit=self.max_vz)
        self.pid_return_e = PIDController(kp_return_xy, ki_return_xy, kd_return_xy, out_limit=self.max_vxy)
        self.pid_return_n = PIDController(kp_return_xy, ki_return_xy, kd_return_xy, out_limit=self.max_vxy)
        self.pid_track_x = PIDController(kp_track_xy, ki_track_xy, kd_track_xy, out_limit=self.max_vxy)
        self.pid_track_y = PIDController(kp_track_xy, ki_track_xy, kd_track_xy, out_limit=self.max_vxy)
        self.pid_yaw = PIDController(kp_yaw, ki_yaw, kd_yaw, out_limit=self.max_wz)

        # ===== 状态 =====
        self.current_state = State()
        self.extended_state = ExtendedState()
        self.has_extended_state = False
        self.prev_offboard = False

        self.curr_lat = 0.0
        self.curr_lon = 0.0
        self.has_gps = False
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.has_home = False

        self.rel_alt_m = float('nan')
        self.has_rel_alt = False

        self.vx_local = 0.0
        self.vy_local = 0.0
        self.vz_local = 0.0
        self.has_velocity = False

        self.aruco_pose = ArucoBasePose()
        self.has_aruco_pose = False
        self.last_aruco_time = None

        self.base_yaw = 0.0
        self.has_base_pose = False
        self.last_base_pose_time = None

        self.phase = self.PHASE_ASCEND
        self.mission_started = (not self.start_on_offboard_entry)
        self.pending_start_after_data = False

        self.track_aligned_start_time = None
        self.hover_before_land_start_time = None

        self.heuristic_landed_start_time = None
        self.min_throttle_start_time = None
        self.last_disarm_request_time = None
        self.disarm_in_flight = False

        self.latest_status = '等待任务启动'

        # ===== QoS =====
        mavros_best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        home_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ===== 通信 =====
        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.ext_state_sub = self.create_subscription(
            ExtendedState,
            self.extended_state_topic,
            self.ext_state_callback,
            mavros_best_effort_qos,
        )
        self.gps_sub = self.create_subscription(NavSatFix, self.global_topic, self.gps_callback, mavros_best_effort_qos)
        self.home_sub = self.create_subscription(HomePosition, self.home_topic, self.home_callback, home_qos)
        self.rel_alt_sub = self.create_subscription(Float64, self.rel_alt_topic, self.rel_alt_callback, mavros_best_effort_qos)
        self.vel_sub = self.create_subscription(TwistStamped, self.vel_local_topic, self.vel_callback, mavros_best_effort_qos)
        self.aruco_sub = self.create_subscription(ArucoBasePose, self.aruco_pose_topic, self.aruco_callback, 10)
        self.base_pose_sub = self.create_subscription(PoseStamped, self.base_pose_topic, self.base_pose_callback, mavros_best_effort_qos)

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.arming_client = self.create_client(CommandBool, self.arming_service)

        self.control_timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'total_node 已启动 | '
            f'state={self.state_topic} | ext={self.extended_state_topic} | '
            f'global={self.global_topic} | home={self.home_topic} | rel_alt={self.rel_alt_topic} | '
            f'aruco={self.aruco_pose_topic} | base_pose={self.base_pose_topic} | '
            f'vel={self.vel_local_topic} | cmd={self.cmd_vel_topic}'
        )

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def rotate_xy(x: float, y: float, yaw_rad: float):
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)
        return c * x - s * y, s * x + c * y

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def latlon_error_to_ne_m(curr_lat, curr_lon, target_lat, target_lon):
        earth_radius_m = 6378137.0
        d_lat = math.radians(target_lat - curr_lat)
        d_lon = math.radians(target_lon - curr_lon)
        lat_mid = math.radians((curr_lat + target_lat) * 0.5)
        north_m = d_lat * earth_radius_m
        east_m = d_lon * earth_radius_m * math.cos(lat_mid)
        return north_m, east_m

    def apply_deadband(self, value: float, deadband: float) -> float:
        return 0.0 if abs(value) < deadband else value

    def state_callback(self, msg: State):
        is_offboard = (msg.mode == 'OFFBOARD')
        offboard_rising = (is_offboard and not self.prev_offboard)
        self.prev_offboard = is_offboard
        self.current_state = msg

        if self.start_on_offboard_entry and offboard_rising:
            self.phase = self.PHASE_ASCEND
            self.mission_started = False
            self.pending_start_after_data = True
            self.track_aligned_start_time = None
            self.hover_before_land_start_time = None
            self.heuristic_landed_start_time = None
            self.min_throttle_start_time = None
            self.last_disarm_request_time = None
            self.disarm_in_flight = False
            self.reset_all_pids()
            self.try_start_mission()

    def ext_state_callback(self, msg: ExtendedState):
        self.extended_state = msg
        self.has_extended_state = True

    def gps_callback(self, msg: NavSatFix):
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        if msg.status.status < 0:
            return
        self.curr_lat = float(msg.latitude)
        self.curr_lon = float(msg.longitude)
        self.has_gps = True
        self.try_start_mission()

    def home_callback(self, msg: HomePosition):
        lat = float(msg.geo.latitude)
        lon = float(msg.geo.longitude)
        if not math.isfinite(lat) or not math.isfinite(lon):
            return
        self.home_lat = lat
        self.home_lon = lon
        self.has_home = True
        self.try_start_mission()

    def rel_alt_callback(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self.rel_alt_m = float(msg.data)
        self.has_rel_alt = True
        self.try_start_mission()

    def vel_callback(self, msg: TwistStamped):
        self.vx_local = float(msg.twist.linear.x)
        self.vy_local = float(msg.twist.linear.y)
        self.vz_local = float(msg.twist.linear.z)
        self.has_velocity = True

    def aruco_callback(self, msg: ArucoBasePose):
        self.aruco_pose = msg
        self.has_aruco_pose = True
        self.last_aruco_time = self.get_clock().now()

    def base_pose_callback(self, msg: PoseStamped):
        q = msg.pose.orientation
        self.base_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.has_base_pose = True
        self.last_base_pose_time = self.get_clock().now()

    def is_aruco_fresh(self) -> bool:
        if not self.has_aruco_pose or self.last_aruco_time is None:
            return False
        dt = (self.get_clock().now() - self.last_aruco_time).nanoseconds / 1e9
        return dt <= self.pose_timeout_sec

    def is_base_pose_fresh(self) -> bool:
        if not self.has_base_pose or self.last_base_pose_time is None:
            return False
        dt = (self.get_clock().now() - self.last_base_pose_time).nanoseconds / 1e9
        return dt <= self.base_pose_timeout_sec

    def is_landed_by_extended_state(self) -> bool:
        if not self.has_extended_state:
            return False
        return self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def is_landing_candidate_by_heuristic(self) -> bool:
        if not self.has_rel_alt or not self.has_velocity:
            return False
        vxy = math.hypot(self.vx_local, self.vy_local)
        return (
            self.rel_alt_m <= self.land_rel_alt_threshold_m
            and abs(self.vz_local) <= self.land_vz_abs_max_mps
            and vxy <= self.land_vxy_abs_max_mps
        )

    def is_heuristic_landed_confirmed(self) -> bool:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.is_landing_candidate_by_heuristic():
            if self.heuristic_landed_start_time is None:
                self.heuristic_landed_start_time = now_sec
            return (now_sec - self.heuristic_landed_start_time) >= self.heuristic_disarm_hold_sec
        self.heuristic_landed_start_time = None
        return False

    def reset_all_pids(self):
        self.pid_alt.reset()
        self.pid_return_e.reset()
        self.pid_return_n.reset()
        self.pid_track_x.reset()
        self.pid_track_y.reset()
        self.pid_yaw.reset()

    def try_start_mission(self):
        if not self.pending_start_after_data:
            return
        if self.current_state.mode != 'OFFBOARD':
            return
        if not (self.has_gps and self.has_home and self.has_rel_alt):
            return
        self.mission_started = True
        self.pending_start_after_data = False
        self.get_logger().info('OFFBOARD 已进入且返航数据齐全，开始执行总任务')

    def try_send_disarm(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.disarm_in_flight:
            return
        if self.last_disarm_request_time is not None and (now_sec - self.last_disarm_request_time) < self.disarm_retry_interval_sec:
            return

        if not self.arming_client.wait_for_service(timeout_sec=0.05):
            return

        req = CommandBool.Request()
        req.value = False
        future = self.arming_client.call_async(req)
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
                self.get_logger().info('disarm 请求成功')
            else:
                self.get_logger().warn('disarm 请求失败，将继续按间隔重试')

        future.add_done_callback(_done_cb)

    def publish_cmd(self, vx: float, vy: float, vz: float, wz: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def control_loop(self):
        # 任务完成后保持零速。
        if self.phase == self.PHASE_DONE:
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.latest_status = '阶段=DONE | 已 disarm，任务完成'
            return

        # 非 OFFBOARD 直接零速。
        if self.require_offboard and self.current_state.mode != 'OFFBOARD':
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.reset_all_pids()
            self.latest_status = f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        if self.start_on_offboard_entry and not self.mission_started:
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.reset_all_pids()
            self.latest_status = '已进入 OFFBOARD，等待总任务启动条件（GPS/home/rel_alt）'
            return

        # armed=false 说明降落已完成。
        if not self.current_state.armed and self.phase in (self.PHASE_LAND, self.PHASE_TOUCHDOWN_DISARM):
            self.phase = self.PHASE_DONE
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.latest_status = '检测到 armed=false，降落完成'
            return

        # 全流程共享高度控制（除最终降落阶段外）。
        alt_err = self.target_alt_m - self.rel_alt_m if self.has_rel_alt else 0.0
        vz_hold = self.apply_deadband(self.pid_alt.update(alt_err), self.velocity_deadband)

        if self.phase == self.PHASE_ASCEND:
            self.publish_cmd(0.0, 0.0, vz_hold, 0.0)
            self.latest_status = f'阶段=ASCEND | alt={self.rel_alt_m:.2f}m, alt_err={alt_err:.2f}m'
            if self.has_rel_alt and abs(alt_err) <= self.alt_tolerance_m:
                self.phase = self.PHASE_RETURN
                self.pid_return_e.reset()
                self.pid_return_n.reset()
                self.get_logger().info('ASCEND 完成，切换 RETURN')
            return

        if self.phase == self.PHASE_RETURN:
            if not (self.has_gps and self.has_home):
                self.publish_cmd(0.0, 0.0, vz_hold, 0.0)
                self.latest_status = '阶段=RETURN | 等待 GPS/home 数据'
                return
            err_n, err_e = self.latlon_error_to_ne_m(self.curr_lat, self.curr_lon, self.home_lat, self.home_lon)
            dist = math.hypot(err_n, err_e)
            vx = self.apply_deadband(self.pid_return_e.update(err_e), self.velocity_deadband)
            vy = self.apply_deadband(self.pid_return_n.update(err_n), self.velocity_deadband)
            self.publish_cmd(vx, vy, vz_hold, 0.0)
            self.latest_status = f'阶段=RETURN | home_dist={dist:.2f}m, err_n={err_n:.2f}, err_e={err_e:.2f}'
            if dist <= self.home_tolerance_m:
                self.phase = self.PHASE_TRACK
                self.track_aligned_start_time = None
                self.pid_track_x.reset()
                self.pid_track_y.reset()
                self.pid_yaw.reset()
                self.get_logger().info('RETURN 完成，切换 TRACK')
            return

        if self.phase == self.PHASE_TRACK:
            if not self.is_aruco_fresh():
                self.publish_cmd(0.0, 0.0, vz_hold, 0.0)
                self.latest_status = '阶段=TRACK | ArUco 数据超时，等待'
                self.track_aligned_start_time = None
                return

            ex_m = self.track_target_x - float(self.aruco_pose.x)
            ey_m = self.track_target_y - float(self.aruco_pose.y)
            eyaw = self.wrap_to_pi(self.track_target_yaw - float(self.aruco_pose.yaw))

            if self.use_dynamic_marker_yaw:
                if not self.is_base_pose_fresh():
                    self.publish_cmd(0.0, 0.0, vz_hold, 0.0)
                    self.latest_status = '阶段=TRACK | base_pose 超时，等待'
                    self.track_aligned_start_time = None
                    return
                yaw_map_marker = self.wrap_to_pi(self.base_yaw - float(self.aruco_pose.yaw))
            else:
                yaw_map_marker = math.radians(self.fallback_marker_in_map_yaw_deg)

            ex_cmd, ey_cmd = self.rotate_xy(ex_m, ey_m, yaw_map_marker)
            vx = self.apply_deadband(self.pid_track_x.update(ex_cmd), self.velocity_deadband)
            vy = self.apply_deadband(self.pid_track_y.update(ey_cmd), self.velocity_deadband)
            wz = self.apply_deadband(self.pid_yaw.update(eyaw), self.yaw_rate_deadband)
            self.publish_cmd(vx, vy, vz_hold, wz)

            xy_err = math.hypot(ex_m, ey_m)
            yaw_tol = math.radians(self.yaw_align_tolerance_deg)
            aligned = (xy_err <= self.xy_align_tolerance_m and abs(eyaw) <= yaw_tol)
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if aligned:
                if self.track_aligned_start_time is None:
                    self.track_aligned_start_time = now_sec
                if (now_sec - self.track_aligned_start_time) >= self.track_align_hold_sec:
                    self.phase = self.PHASE_HOVER_BEFORE_LAND
                    self.hover_before_land_start_time = now_sec
                    self.get_logger().info('TRACK 对正成功，切换 HOVER_BEFORE_LAND')
            else:
                self.track_aligned_start_time = None

            self.latest_status = f'阶段=TRACK | ex={ex_m:.2f}, ey={ey_m:.2f}, eyaw={math.degrees(eyaw):.1f}deg'
            return

        if self.phase == self.PHASE_HOVER_BEFORE_LAND:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            hold = 0.0 if self.hover_before_land_start_time is None else (now_sec - self.hover_before_land_start_time)
            self.latest_status = f'阶段=HOVER_BEFORE_LAND | 悬停中 {hold:.2f}/{self.hover_after_align_sec:.2f}s'
            if self.hover_before_land_start_time is not None and hold >= self.hover_after_align_sec:
                self.phase = self.PHASE_LAND
                self.heuristic_landed_start_time = None
                self.min_throttle_start_time = None
                self.get_logger().info('悬停完成，切换 LAND')
            return

        if self.phase == self.PHASE_LAND:
            if self.is_landed_by_extended_state():
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)
                self.latest_status = '阶段=LAND | ON_GROUND，申请 disarm'
                self.try_send_disarm()
                return

            if self.is_heuristic_landed_confirmed():
                self.phase = self.PHASE_TOUCHDOWN_DISARM
                self.min_throttle_start_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info('启发式落地确认，切换 TOUCHDOWN_DISARM')
                return

            self.publish_cmd(0.0, 0.0, -self.descent_speed_mps, 0.0)
            self.latest_status = f'阶段=LAND | 降落中 vz_cmd=-{self.descent_speed_mps:.2f}m/s'
            return

        # TOUCHDOWN_DISARM：最低油门下压 5s，并按 1s 周期申请 disarm。
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.min_throttle_start_time is None:
            self.min_throttle_start_time = now_sec
        hold_dt = now_sec - self.min_throttle_start_time

        self.publish_cmd(0.0, 0.0, -self.min_throttle_descent_speed_mps, 0.0)
        self.try_send_disarm()

        self.latest_status = (
            f'阶段=TOUCHDOWN_DISARM | vz_cmd=-{self.min_throttle_descent_speed_mps:.2f}m/s, '
            f'hold={hold_dt:.2f}/{self.min_throttle_disarm_duration_sec:.2f}s, 每秒disarm'
        )

    def log_callback(self):
        self.get_logger().info(self.latest_status)


def main(args=None):
    rclpy.init(args=args)
    node = TotalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
