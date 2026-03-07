#!/usr/bin/env python3

import math
import time

import rclpy
from debug_interface.msg import ArucoBasePose
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import HomePosition, State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


class PIDController:
    """简单 PID 控制器（带积分限幅与输出限幅）。"""

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

        # P 项：按当前误差直接输出。
        p_term = self.kp * error

        # I 项：积分累加并限幅，防止积分饱和。
        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        i_term = self.ki * self.integral

        # D 项：利用误差变化率抑制超调。
        d_term = self.kd * (error - self.prev_error) / dt

        out = p_term + i_term + d_term
        out = max(-self.out_limit, min(self.out_limit, out))

        self.prev_error = error
        self.prev_time = now
        return out


class ReturnTrackHoverNode(Node):
    """OFFBOARD 后先返航，再进行 ArUco 对正并悬停。"""

    PHASE_ASCEND = 'ASCEND'
    PHASE_RETURN = 'RETURN'
    PHASE_TRACK = 'TRACK'
    PHASE_HOVER = 'HOVER'

    def __init__(self):
        super().__init__('return_track_hover_node')

        # ===== 输入/输出话题 =====
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('global_topic', '/mavros/global_position/global')
        self.declare_parameter('home_topic', '/mavros/home_position/home')
        self.declare_parameter('rel_alt_topic', '/mavros/global_position/rel_alt')
        self.declare_parameter('aruco_pose_topic', '/debug/aruco_pose')
        self.declare_parameter('base_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel')

        # ===== 任务目标与阈值 =====
        self.declare_parameter('target_alt_m', 3.0)
        self.declare_parameter('alt_tolerance_m', 0.15)
        self.declare_parameter('home_tolerance_m', 0.50)

        self.declare_parameter('track_target_x', 0.0)
        self.declare_parameter('track_target_y', 0.0)
        self.declare_parameter('track_target_yaw', 0.0)
        self.declare_parameter('xy_align_tolerance_m', 0.20)
        self.declare_parameter('yaw_align_tolerance_deg', 8.0)
        self.declare_parameter('align_hold_sec', 1.0)

        # ===== PID 参数 =====
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

        # ===== 限幅/频率/安全 =====
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

        self.state_topic = self.get_parameter('state_topic').value
        self.global_topic = self.get_parameter('global_topic').value
        self.home_topic = self.get_parameter('home_topic').value
        self.rel_alt_topic = self.get_parameter('rel_alt_topic').value
        self.aruco_pose_topic = self.get_parameter('aruco_pose_topic').value
        self.base_pose_topic = self.get_parameter('base_pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.target_alt_m = float(self.get_parameter('target_alt_m').value)
        self.alt_tolerance_m = float(self.get_parameter('alt_tolerance_m').value)
        self.home_tolerance_m = float(self.get_parameter('home_tolerance_m').value)

        self.track_target_x = float(self.get_parameter('track_target_x').value)
        self.track_target_y = float(self.get_parameter('track_target_y').value)
        self.track_target_yaw = float(self.get_parameter('track_target_yaw').value)
        self.xy_align_tolerance_m = float(self.get_parameter('xy_align_tolerance_m').value)
        self.yaw_align_tolerance_deg = float(self.get_parameter('yaw_align_tolerance_deg').value)
        self.align_hold_sec = float(self.get_parameter('align_hold_sec').value)

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

        # ===== PID 初始化 =====
        self.pid_alt = PIDController(kp_alt, ki_alt, kd_alt, out_limit=self.max_vz)
        self.pid_return_e = PIDController(kp_return_xy, ki_return_xy, kd_return_xy, out_limit=self.max_vxy)
        self.pid_return_n = PIDController(kp_return_xy, ki_return_xy, kd_return_xy, out_limit=self.max_vxy)
        self.pid_track_x = PIDController(kp_track_xy, ki_track_xy, kd_track_xy, out_limit=self.max_vxy)
        self.pid_track_y = PIDController(kp_track_xy, ki_track_xy, kd_track_xy, out_limit=self.max_vxy)
        self.pid_yaw = PIDController(kp_yaw, ki_yaw, kd_yaw, out_limit=self.max_wz)

        # ===== 状态量 =====
        self.current_state = State()
        self.prev_offboard = False

        self.curr_lat = 0.0
        self.curr_lon = 0.0
        self.curr_rel_alt = 0.0
        self.has_gps = False
        self.has_rel_alt = False

        self.home_lat = 0.0
        self.home_lon = 0.0
        self.has_home = False

        self.aruco_pose = ArucoBasePose()
        self.has_aruco_pose = False
        self.last_aruco_time = None

        self.base_yaw = 0.0
        self.has_base_pose = False
        self.last_base_pose_time = None

        self.phase = self.PHASE_ASCEND
        self.mission_started = (not self.start_on_offboard_entry)
        self.pending_start_after_data = False
        self.align_start_time = None

        self.latest_status = '等待任务启动'

        # ===== QoS（匹配 MAVROS 端） =====
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
        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.global_topic,
            self.gps_callback,
            mavros_best_effort_qos,
        )
        self.rel_alt_sub = self.create_subscription(
            Float64,
            self.rel_alt_topic,
            self.rel_alt_callback,
            mavros_best_effort_qos,
        )
        self.home_sub = self.create_subscription(HomePosition, self.home_topic, self.home_callback, home_qos)
        self.aruco_sub = self.create_subscription(ArucoBasePose, self.aruco_pose_topic, self.aruco_callback, 10)
        self.base_pose_sub = self.create_subscription(
            PoseStamped,
            self.base_pose_topic,
            self.base_pose_callback,
            mavros_best_effort_qos,
        )
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        self.control_timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'return_track_hover_node 已启动 | '
            f'state={self.state_topic} | global={self.global_topic} | home={self.home_topic} | '
            f'rel_alt={self.rel_alt_topic} | aruco={self.aruco_pose_topic} | '
            f'base_pose={self.base_pose_topic} | cmd={self.cmd_vel_topic}'
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

    def state_callback(self, msg: State):
        is_offboard = (msg.mode == 'OFFBOARD')
        offboard_rising = (is_offboard and not self.prev_offboard)
        self.prev_offboard = is_offboard
        self.current_state = msg

        # 进入 OFFBOARD 上升沿时重置任务阶段，按流程重新执行。
        if self.start_on_offboard_entry and offboard_rising:
            self.phase = self.PHASE_ASCEND
            self.mission_started = False
            self.pending_start_after_data = True
            self.align_start_time = None
            self.reset_all_pids()
            self.try_start_mission()

    def gps_callback(self, msg: NavSatFix):
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        if msg.status.status < 0:
            return
        self.curr_lat = float(msg.latitude)
        self.curr_lon = float(msg.longitude)
        self.has_gps = True
        self.try_start_mission()

    def rel_alt_callback(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self.curr_rel_alt = float(msg.data)
        self.has_rel_alt = True
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

    def aruco_callback(self, msg: ArucoBasePose):
        self.aruco_pose = msg
        self.has_aruco_pose = True
        self.last_aruco_time = self.get_clock().now()

    def base_pose_callback(self, msg: PoseStamped):
        q = msg.pose.orientation
        self.base_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.has_base_pose = True
        self.last_base_pose_time = self.get_clock().now()

    def is_aruco_pose_fresh(self) -> bool:
        if not self.has_aruco_pose or self.last_aruco_time is None:
            return False
        dt = (self.get_clock().now() - self.last_aruco_time).nanoseconds / 1e9
        return dt <= self.pose_timeout_sec

    def is_base_pose_fresh(self) -> bool:
        if not self.has_base_pose or self.last_base_pose_time is None:
            return False
        dt = (self.get_clock().now() - self.last_base_pose_time).nanoseconds / 1e9
        return dt <= self.base_pose_timeout_sec

    def reset_all_pids(self):
        self.pid_alt.reset()
        self.pid_return_e.reset()
        self.pid_return_n.reset()
        self.pid_track_x.reset()
        self.pid_track_y.reset()
        self.pid_yaw.reset()

    def try_start_mission(self):
        """OFFBOARD 上升沿后，关键数据齐全时才允许任务开始。"""
        if not self.pending_start_after_data:
            return
        if self.current_state.mode != 'OFFBOARD':
            return
        if not (self.has_gps and self.has_rel_alt and self.has_home):
            return

        self.mission_started = True
        self.pending_start_after_data = False
        self.get_logger().info('OFFBOARD 已进入且返航数据齐全，开始执行 ASCEND->RETURN->TRACK')

    def apply_deadband(self, value: float, deadband: float) -> float:
        return 0.0 if abs(value) < deadband else value

    def publish_cmd(self, vx: float, vy: float, vz: float, wz: float):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(vx)
        cmd.twist.linear.y = float(vy)
        cmd.twist.linear.z = float(vz)
        cmd.twist.angular.z = float(wz)
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        # 未进入 OFFBOARD 时不执行任务。
        if self.require_offboard and self.current_state.mode != 'OFFBOARD':
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.reset_all_pids()
            self.align_start_time = None
            self.latest_status = f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        # 需要 OFFBOARD 触发启动时，未满足启动条件前保持零速。
        if self.start_on_offboard_entry and not self.mission_started:
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.reset_all_pids()
            self.latest_status = '已进入 OFFBOARD，等待返航启动条件（GPS/rel_alt/home）'
            return

        if not (self.has_gps and self.has_rel_alt and self.has_home):
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.latest_status = '返航数据未就绪，输出零速'
            return

        # 全流程都保持目标高度 3m。
        alt_err = self.target_alt_m - self.curr_rel_alt
        vz_cmd = self.pid_alt.update(alt_err)
        vz_cmd = self.apply_deadband(vz_cmd, self.velocity_deadband)

        # 阶段1：先升到 3m。
        if self.phase == self.PHASE_ASCEND:
            self.publish_cmd(0.0, 0.0, vz_cmd, 0.0)
            self.latest_status = (
                f'阶段=ASCEND | alt={self.curr_rel_alt:.2f}m, '
                f'alt_err={alt_err:.2f}m, vz={vz_cmd:.2f}'
            )
            if abs(alt_err) <= self.alt_tolerance_m:
                self.phase = self.PHASE_RETURN
                self.pid_return_e.reset()
                self.pid_return_n.reset()
                self.get_logger().info('ASCEND 完成，切换到 RETURN 阶段')
            return

        # 阶段2：回到 home 水平位置。
        if self.phase == self.PHASE_RETURN:
            err_north_m, err_east_m = self.latlon_error_to_ne_m(
                self.curr_lat,
                self.curr_lon,
                self.home_lat,
                self.home_lon,
            )
            home_dist = math.hypot(err_north_m, err_east_m)

            vx_cmd = self.pid_return_e.update(err_east_m)
            vy_cmd = self.pid_return_n.update(err_north_m)
            vx_cmd = self.apply_deadband(vx_cmd, self.velocity_deadband)
            vy_cmd = self.apply_deadband(vy_cmd, self.velocity_deadband)

            self.publish_cmd(vx_cmd, vy_cmd, vz_cmd, 0.0)
            self.latest_status = (
                f'阶段=RETURN | home_dist={home_dist:.2f}m, '
                f'err_n={err_north_m:.2f}, err_e={err_east_m:.2f}, vz={vz_cmd:.2f}'
            )

            if home_dist <= self.home_tolerance_m:
                self.phase = self.PHASE_TRACK
                self.pid_track_x.reset()
                self.pid_track_y.reset()
                self.pid_yaw.reset()
                self.align_start_time = None
                self.get_logger().info('RETURN 完成，切换到 TRACK 阶段')
            return

        # 阶段3：执行 ArUco 跟踪，直到“正对 + 对准”达到保持时间。
        if self.phase == self.PHASE_TRACK:
            if not self.is_aruco_pose_fresh():
                self.publish_cmd(0.0, 0.0, vz_cmd, 0.0)
                self.latest_status = '阶段=TRACK | ArUco 数据超时，保持高度并零速等待'
                self.align_start_time = None
                return

            # marker 坐标系误差。
            ex_m = self.track_target_x - float(self.aruco_pose.x)
            ey_m = self.track_target_y - float(self.aruco_pose.y)
            eyaw = self.wrap_to_pi(self.track_target_yaw - float(self.aruco_pose.yaw))

            # 将 marker 坐标误差转换到速度命令坐标（map/ENU）。
            if self.use_dynamic_marker_yaw:
                if not self.is_base_pose_fresh():
                    self.publish_cmd(0.0, 0.0, vz_cmd, 0.0)
                    self.latest_status = '阶段=TRACK | 本地位姿超时，保持高度并零速等待'
                    self.align_start_time = None
                    return
                yaw_map_marker = self.wrap_to_pi(self.base_yaw - float(self.aruco_pose.yaw))
            else:
                yaw_map_marker = math.radians(self.fallback_marker_in_map_yaw_deg)

            ex_cmd, ey_cmd = self.rotate_xy(ex_m, ey_m, yaw_map_marker)

            vx_cmd = self.pid_track_x.update(ex_cmd)
            vy_cmd = self.pid_track_y.update(ey_cmd)
            wz_cmd = self.pid_yaw.update(eyaw)

            vx_cmd = self.apply_deadband(vx_cmd, self.velocity_deadband)
            vy_cmd = self.apply_deadband(vy_cmd, self.velocity_deadband)
            wz_cmd = self.apply_deadband(wz_cmd, self.yaw_rate_deadband)

            self.publish_cmd(vx_cmd, vy_cmd, vz_cmd, wz_cmd)

            yaw_tol_rad = math.radians(self.yaw_align_tolerance_deg)
            xy_err = math.hypot(ex_m, ey_m)
            aligned_now = (abs(eyaw) <= yaw_tol_rad and xy_err <= self.xy_align_tolerance_m)

            if aligned_now:
                if self.align_start_time is None:
                    self.align_start_time = self.get_clock().now()
                hold_dt = (self.get_clock().now() - self.align_start_time).nanoseconds / 1e9
                if hold_dt >= self.align_hold_sec:
                    self.phase = self.PHASE_HOVER
                    self.get_logger().info('TRACK 完成（正对 ArUco 且稳定），切换到 HOVER 阶段')
            else:
                self.align_start_time = None

            self.latest_status = (
                f'阶段=TRACK | ex_m={ex_m:.2f}, ey_m={ey_m:.2f}, '
                f'eyaw={math.degrees(eyaw):.1f}deg, vz={vz_cmd:.2f}'
            )
            return

        # 阶段4：悬停。
        self.publish_cmd(0.0, 0.0, 0.0, 0.0)
        self.latest_status = '阶段=HOVER | 任务完成，悬停中'

    def log_callback(self):
        self.get_logger().info(self.latest_status)


def main(args=None):
    rclpy.init(args=args)
    node = ReturnTrackHoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
