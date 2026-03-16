#!/usr/bin/env python3

import math
import time

import rclpy
from debug_interface.msg import ArucoBasePose
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64


class PIDController:
    """简单 PID 控制器（积分限幅 + 输出限幅）。"""

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

        p_term = self.kp * error
        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.prev_error) / dt

        out = p_term + i_term + d_term
        out = max(-self.out_limit, min(self.out_limit, out))

        self.prev_error = error
        self.prev_time = now
        return out


class LandWithTrackingV2Node(Node):
    """land_with_tracking_v2 主节点：先对齐，再下降，再下压 disarm。"""

    PHASE_ALIGN = 'ALIGN'
    PHASE_HOVER_BEFORE_LAND = 'HOVER_BEFORE_LAND'
    PHASE_DESCEND_WITH_TRACK = 'DESCEND_WITH_TRACK'
    PHASE_TOUCHDOWN_DISARM = 'TOUCHDOWN_DISARM'
    PHASE_DONE = 'DONE'

    ALIGN_SUBPHASE_YAW_ONLY = 'YAW_ONLY'
    ALIGN_SUBPHASE_XYZ_ALIGN = 'XYZ_ALIGN'

    def __init__(self):
        super().__init__('land_with_tracking_v2_node')

        # ===== 话题与服务 =====
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('extended_state_topic', '/mavros/extended_state')
        self.declare_parameter('rel_alt_topic', '/mavros/global_position/rel_alt')
        self.declare_parameter('vel_local_topic', '/mavros/local_position/velocity_local')
        self.declare_parameter('aruco_pose_topic', '/debug/aruco_pose')
        self.declare_parameter('base_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel')
        self.declare_parameter('arming_service', '/mavros/cmd/arming')

        # ===== 任务目标 =====
        self.declare_parameter('track_target_x', 0.0)
        self.declare_parameter('track_target_y', 0.0)
        self.declare_parameter('track_target_yaw', 0.0)
        self.declare_parameter('track_target_z', 2.5)

        # ===== 对齐阈值 =====
        self.declare_parameter('xy_align_tolerance_m', 0.10)
        self.declare_parameter('z_align_tolerance_m', 0.10)
        self.declare_parameter('yaw_align_tolerance_deg', 5.0)
        self.declare_parameter('align_hold_sec', 1.0)
        self.declare_parameter('hover_after_align_sec', 1.0)

        # ===== PID 参数（按需求默认值） =====
        self.declare_parameter('kp_x', 0.6)
        self.declare_parameter('ki_x', 0.0)
        self.declare_parameter('kd_x', 0.02)
        self.declare_parameter('kp_y', 0.6)
        self.declare_parameter('ki_y', 0.0)
        self.declare_parameter('kd_y', 0.02)
        self.declare_parameter('kp_yaw', 0.60)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.02)
        self.declare_parameter('kp_z', 0.60)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.06)

        # ===== 速度限制与死区 =====
        self.declare_parameter('vx_limit', 1.0)
        self.declare_parameter('vy_limit', 1.0)
        self.declare_parameter('max_vz', 0.8)
        self.declare_parameter('max_wz', 1.0)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('yaw_rate_deadband', 0.03)

        # ===== 下降与落地参数 =====
        self.declare_parameter('descent_speed_mps', 0.5)
        self.declare_parameter('land_rel_alt_threshold_m', 0.15)
        self.declare_parameter('land_vz_abs_max_mps', 0.20)
        self.declare_parameter('land_vxy_abs_max_mps', 0.25)
        self.declare_parameter('heuristic_disarm_hold_sec', 3.0)
        self.declare_parameter('min_throttle_descent_speed_mps', 0.35)
        self.declare_parameter('min_throttle_disarm_duration_sec', 5.0)
        self.declare_parameter('disarm_retry_interval_sec', 1.0)

        # ===== 通用安全参数 =====
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('base_pose_timeout_sec', 0.5)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('start_on_offboard_entry', True)
        self.declare_parameter('use_dynamic_marker_yaw', True)
        self.declare_parameter('fallback_marker_in_map_yaw_deg', 90.0)

        self.state_topic = self.get_parameter('state_topic').value
        self.extended_state_topic = self.get_parameter('extended_state_topic').value
        self.rel_alt_topic = self.get_parameter('rel_alt_topic').value
        self.vel_local_topic = self.get_parameter('vel_local_topic').value
        self.aruco_pose_topic = self.get_parameter('aruco_pose_topic').value
        self.base_pose_topic = self.get_parameter('base_pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.arming_service = self.get_parameter('arming_service').value

        self.track_target_x = float(self.get_parameter('track_target_x').value)
        self.track_target_y = float(self.get_parameter('track_target_y').value)
        self.track_target_yaw = float(self.get_parameter('track_target_yaw').value)
        self.track_target_z = float(self.get_parameter('track_target_z').value)

        self.xy_align_tolerance_m = float(self.get_parameter('xy_align_tolerance_m').value)
        self.z_align_tolerance_m = float(self.get_parameter('z_align_tolerance_m').value)
        self.yaw_align_tolerance_deg = float(self.get_parameter('yaw_align_tolerance_deg').value)
        self.align_hold_sec = float(self.get_parameter('align_hold_sec').value)
        self.hover_after_align_sec = float(self.get_parameter('hover_after_align_sec').value)

        kp_x = float(self.get_parameter('kp_x').value)
        ki_x = float(self.get_parameter('ki_x').value)
        kd_x = float(self.get_parameter('kd_x').value)
        kp_y = float(self.get_parameter('kp_y').value)
        ki_y = float(self.get_parameter('ki_y').value)
        kd_y = float(self.get_parameter('kd_y').value)
        kp_yaw = float(self.get_parameter('kp_yaw').value)
        ki_yaw = float(self.get_parameter('ki_yaw').value)
        kd_yaw = float(self.get_parameter('kd_yaw').value)
        kp_z = float(self.get_parameter('kp_z').value)
        ki_z = float(self.get_parameter('ki_z').value)
        kd_z = float(self.get_parameter('kd_z').value)

        self.vx_limit = float(self.get_parameter('vx_limit').value)
        self.vy_limit = float(self.get_parameter('vy_limit').value)
        self.max_vz = float(self.get_parameter('max_vz').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)
        self.yaw_rate_deadband = float(self.get_parameter('yaw_rate_deadband').value)

        self.descent_speed_mps = abs(float(self.get_parameter('descent_speed_mps').value))
        self.land_rel_alt_threshold_m = float(self.get_parameter('land_rel_alt_threshold_m').value)
        self.land_vz_abs_max_mps = float(self.get_parameter('land_vz_abs_max_mps').value)
        self.land_vxy_abs_max_mps = float(self.get_parameter('land_vxy_abs_max_mps').value)
        self.heuristic_disarm_hold_sec = float(self.get_parameter('heuristic_disarm_hold_sec').value)
        self.min_throttle_descent_speed_mps = abs(float(self.get_parameter('min_throttle_descent_speed_mps').value))
        self.min_throttle_disarm_duration_sec = float(self.get_parameter('min_throttle_disarm_duration_sec').value)
        self.disarm_retry_interval_sec = float(self.get_parameter('disarm_retry_interval_sec').value)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
        self.base_pose_timeout_sec = float(self.get_parameter('base_pose_timeout_sec').value)
        self.require_offboard = bool(self.get_parameter('require_offboard').value)
        self.start_on_offboard_entry = bool(self.get_parameter('start_on_offboard_entry').value)
        self.use_dynamic_marker_yaw = bool(self.get_parameter('use_dynamic_marker_yaw').value)
        self.fallback_marker_in_map_yaw_deg = float(self.get_parameter('fallback_marker_in_map_yaw_deg').value)

        # ===== PID =====
        self.pid_x = PIDController(kp_x, ki_x, kd_x, out_limit=self.vx_limit)
        self.pid_y = PIDController(kp_y, ki_y, kd_y, out_limit=self.vy_limit)
        self.pid_yaw = PIDController(kp_yaw, ki_yaw, kd_yaw, out_limit=self.max_wz)
        self.pid_z = PIDController(kp_z, ki_z, kd_z, out_limit=self.max_vz)

        # ===== 运行状态 =====
        self.current_state = State()
        self.extended_state = ExtendedState()
        self.has_extended_state = False
        self.prev_offboard = False

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

        self.phase = self.PHASE_ALIGN
        self.align_subphase = self.ALIGN_SUBPHASE_YAW_ONLY
        self.mission_started = (not self.start_on_offboard_entry)
        self.align_hold_start_time = None
        self.hover_start_time = None

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

        # ===== 通信 =====
        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.ext_state_sub = self.create_subscription(
            ExtendedState, self.extended_state_topic, self.extended_state_callback, mavros_best_effort_qos
        )
        self.rel_alt_sub = self.create_subscription(
            Float64, self.rel_alt_topic, self.rel_alt_callback, mavros_best_effort_qos
        )
        self.vel_sub = self.create_subscription(
            TwistStamped, self.vel_local_topic, self.velocity_callback, mavros_best_effort_qos
        )
        self.aruco_sub = self.create_subscription(ArucoBasePose, self.aruco_pose_topic, self.aruco_callback, 10)
        self.base_pose_sub = self.create_subscription(
            PoseStamped, self.base_pose_topic, self.base_pose_callback, mavros_best_effort_qos
        )

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.arming_client = self.create_client(CommandBool, self.arming_service)

        self.control_timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'land_with_tracking_v2_node 已启动 | '
            f'aruco={self.aruco_pose_topic} | base_pose={self.base_pose_topic} | '
            f'cmd={self.cmd_vel_topic} | arming_srv={self.arming_service}'
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
    def apply_deadband(value: float, deadband: float) -> float:
        return 0.0 if abs(value) < deadband else value

    def state_callback(self, msg: State):
        is_offboard = (msg.mode == 'OFFBOARD')
        offboard_rising = (is_offboard and not self.prev_offboard)
        self.prev_offboard = is_offboard
        self.current_state = msg

        if self.start_on_offboard_entry and offboard_rising:
            # OFFBOARD 上升沿启动完整任务流程。
            self.phase = self.PHASE_ALIGN
            self.align_subphase = self.ALIGN_SUBPHASE_YAW_ONLY
            self.mission_started = True
            self.align_hold_start_time = None
            self.hover_start_time = None
            self.heuristic_landed_start_time = None
            self.min_throttle_start_time = None
            self.last_disarm_request_time = None
            self.disarm_in_flight = False
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_yaw.reset()
            self.pid_z.reset()
            self.get_logger().info('检测到 OFFBOARD 上升沿，开始执行 land_with_tracking_v2 任务')

    def extended_state_callback(self, msg: ExtendedState):
        self.extended_state = msg
        self.has_extended_state = True

    def rel_alt_callback(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self.rel_alt_m = float(msg.data)
        self.has_rel_alt = True

    def velocity_callback(self, msg: TwistStamped):
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

    def publish_cmd(self, vx: float, vy: float, vz: float, wz: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def try_send_disarm(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.disarm_in_flight:
            return
        if self.last_disarm_request_time is not None:
            if (now_sec - self.last_disarm_request_time) < self.disarm_retry_interval_sec:
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

    def compute_track_cmd(self):
        """
        计算 xy/yaw/z 误差与控制量。

        说明：
        - 按机体误差语义直接闭环（对齐 yaw_then_xy_tracking 思路）：
          ex = target_x - pose.x，ey = pose.y - target_y；
        - 去掉动态 yaw 旋转链路，不再依赖 base_pose。
        - yaw/z 误差分别独立 PID。
        """
        ex_m = self.track_target_x - float(self.aruco_pose.x)
        ey_m = float(self.aruco_pose.y) - self.track_target_y
        ez = self.track_target_z - float(self.aruco_pose.z)
        eyaw = self.wrap_to_pi(self.track_target_yaw - float(self.aruco_pose.yaw))

        # 直接机体误差闭环，不做动态 yaw 旋转。
        ex_cmd, ey_cmd = ex_m, ey_m

        # 实机验证结论：
        # 前后/左右通道在输出端存在互换，因此这里做 x-y 交换。
        # 当前版本不做额外符号反向，避免两轴同向发散。
        vx_raw = self.pid_x.update(ex_cmd)
        vy_raw = self.pid_y.update(ey_cmd)
        vx = self.apply_deadband(vy_raw, self.velocity_deadband)
        vy = self.apply_deadband(vx_raw, self.velocity_deadband)
        vz = self.apply_deadband(self.pid_z.update(ez), self.velocity_deadband)
        wz = self.apply_deadband(self.pid_yaw.update(eyaw), self.yaw_rate_deadband)
        return vx, vy, vz, wz, ex_m, ey_m, ez, eyaw

    def control_loop(self):
        if self.require_offboard and self.current_state.mode != 'OFFBOARD':
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.pid_yaw.reset()
            self.latest_status = f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        if self.start_on_offboard_entry and not self.mission_started:
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.latest_status = '已进入 OFFBOARD，等待任务启动'
            return

        if not self.current_state.armed and self.phase in (
            self.PHASE_DESCEND_WITH_TRACK,
            self.PHASE_TOUCHDOWN_DISARM,
        ):
            self.phase = self.PHASE_DONE

        if self.phase == self.PHASE_DONE:
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.latest_status = '阶段=DONE | 已 disarm，任务完成'
            return

        if self.phase in (self.PHASE_ALIGN, self.PHASE_DESCEND_WITH_TRACK):
            if not self.is_aruco_fresh():
                self.publish_cmd(0.0, 0.0, 0.0, 0.0)
                self.align_hold_start_time = None
                self.latest_status = f'阶段={self.phase} | ArUco 数据超时，输出零速等待'
                return

            track_cmd = self.compute_track_cmd()
            vx, vy, vz, wz, ex_m, ey_m, ez, eyaw = track_cmd
            yaw_tol = math.radians(self.yaw_align_tolerance_deg)
            now_sec = self.get_clock().now().nanoseconds / 1e9

            if self.phase == self.PHASE_ALIGN:
                # 子阶段1：仅 yaw 控制，x/y/z 不动。
                if self.align_subphase == self.ALIGN_SUBPHASE_YAW_ONLY:
                    self.publish_cmd(0.0, 0.0, 0.0, wz)
                    yaw_ok = (abs(eyaw) <= yaw_tol)
                    if yaw_ok:
                        if self.align_hold_start_time is None:
                            self.align_hold_start_time = now_sec
                        hold_dt = now_sec - self.align_hold_start_time
                        if hold_dt >= self.align_hold_sec:
                            self.align_subphase = self.ALIGN_SUBPHASE_XYZ_ALIGN
                            self.align_hold_start_time = None
                            self.pid_x.reset()
                            self.pid_y.reset()
                            self.pid_z.reset()
                            self.get_logger().info('ALIGN 子阶段切换：YAW_ONLY -> XYZ_ALIGN')
                    else:
                        self.align_hold_start_time = None

                    self.latest_status = (
                        f'阶段=ALIGN/YAW_ONLY | eyaw={math.degrees(eyaw):.1f}deg | '
                        f'vx=0.00, vy=0.00, vz=0.00, wz={wz:.2f}'
                    )
                    return

                # 子阶段2：调整 xyz，并保持 yaw 闭环。
                self.publish_cmd(vx, vy, vz, wz)
                xy_err = math.hypot(ex_m, ey_m)
                aligned = (
                    xy_err <= self.xy_align_tolerance_m
                    and abs(ez) <= self.z_align_tolerance_m
                    and abs(eyaw) <= yaw_tol
                )
                if aligned:
                    if self.align_hold_start_time is None:
                        self.align_hold_start_time = now_sec
                    hold_dt = now_sec - self.align_hold_start_time
                    if hold_dt >= self.align_hold_sec:
                        self.phase = self.PHASE_HOVER_BEFORE_LAND
                        self.hover_start_time = now_sec
                        self.get_logger().info('ALIGN 完成（xyz+yaw），切换 HOVER_BEFORE_LAND')
                else:
                    self.align_hold_start_time = None

                self.latest_status = (
                    f'阶段=ALIGN/XYZ_ALIGN | ex={ex_m:.2f}, ey={ey_m:.2f}, ez={ez:.2f}, '
                    f'eyaw={math.degrees(eyaw):.1f}deg | vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, wz={wz:.2f}'
                )
                return

            # 下降阶段：保持 xy+yaw 跟踪，垂向恒速下降。
            self.publish_cmd(vx, vy, -self.descent_speed_mps, wz)
            self.latest_status = (
                f'阶段=DESCEND_WITH_TRACK | vz=-{self.descent_speed_mps:.2f}, '
                f'ex={ex_m:.2f}, ey={ey_m:.2f}, eyaw={math.degrees(eyaw):.1f}deg'
            )

            if self.is_landed_by_extended_state() or self.is_heuristic_landed_confirmed():
                self.phase = self.PHASE_TOUCHDOWN_DISARM
                self.min_throttle_start_time = now_sec
                self.get_logger().info('检测到接地条件，切换 TOUCHDOWN_DISARM')
            return

        if self.phase == self.PHASE_HOVER_BEFORE_LAND:
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            now_sec = self.get_clock().now().nanoseconds / 1e9
            hold_dt = 0.0 if self.hover_start_time is None else (now_sec - self.hover_start_time)
            self.latest_status = (
                f'阶段=HOVER_BEFORE_LAND | 悬停 {hold_dt:.2f}/{self.hover_after_align_sec:.2f}s'
            )
            if self.hover_start_time is not None and hold_dt >= self.hover_after_align_sec:
                self.phase = self.PHASE_DESCEND_WITH_TRACK
                self.get_logger().info('悬停完成，切换 DESCEND_WITH_TRACK')
            return

        # TOUCHDOWN_DISARM：最低油门下压，并周期 disarm。
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.min_throttle_start_time is None:
            self.min_throttle_start_time = now_sec
        hold_dt = now_sec - self.min_throttle_start_time

        self.publish_cmd(0.0, 0.0, -self.min_throttle_descent_speed_mps, 0.0)
        self.try_send_disarm()
        self.latest_status = (
            f'阶段=TOUCHDOWN_DISARM | vz=-{self.min_throttle_descent_speed_mps:.2f}, '
            f'hold={hold_dt:.2f}/{self.min_throttle_disarm_duration_sec:.2f}s, 每秒disarm'
        )

    def log_callback(self):
        self.get_logger().info(self.latest_status)


def main(args=None):
    rclpy.init(args=args)
    node = LandWithTrackingV2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
