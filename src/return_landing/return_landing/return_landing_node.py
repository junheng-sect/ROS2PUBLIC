#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import ExtendedState, HomePosition, State
from mavros_msgs.srv import CommandBool
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
        """在阶段切换时重置 PID 内部状态，避免控制突变。"""
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


class ReturnLandingNode(Node):
    """任务状态机：先返航，再降落到 disarm。"""

    PHASE_ASCEND = 'ASCEND'
    PHASE_RETURN = 'RETURN'
    PHASE_LAND = 'LAND'
    PHASE_TOUCHDOWN_DISARM = 'TOUCHDOWN_DISARM'
    PHASE_DONE = 'DONE'

    def __init__(self):
        super().__init__('return_landing_node')

        # ===== 话题与服务参数 =====
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('extended_state_topic', '/mavros/extended_state')
        self.declare_parameter('global_topic', '/mavros/global_position/global')
        self.declare_parameter('home_topic', '/mavros/home_position/home')
        self.declare_parameter('rel_alt_topic', '/mavros/global_position/rel_alt')
        self.declare_parameter('vel_local_topic', '/mavros/local_position/velocity_local')
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel')
        self.declare_parameter('arming_service', '/mavros/cmd/arming')

        # ===== 返航参数 =====
        self.declare_parameter('target_alt_m', 3.0)
        self.declare_parameter('alt_tolerance_m', 0.15)
        self.declare_parameter('home_tolerance_m', 0.50)

        self.declare_parameter('kp_alt', 0.8)
        self.declare_parameter('ki_alt', 0.0)
        self.declare_parameter('kd_alt', 0.05)

        self.declare_parameter('kp_xy', 0.35)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.08)

        # ===== 降落参数（沿用 landing 语义） =====
        self.declare_parameter('descent_speed_mps', 0.5)
        self.declare_parameter('touchdown_descent_speed_mps', 0.15)
        self.declare_parameter('land_rel_alt_threshold_m', 0.15)
        self.declare_parameter('land_vz_abs_max_mps', 0.20)
        self.declare_parameter('land_vxy_abs_max_mps', 0.25)
        self.declare_parameter('land_detect_hold_sec', 1.0)
        self.declare_parameter('allow_heuristic_disarm_fallback', True)
        self.declare_parameter('heuristic_disarm_hold_sec', 3.0)
        self.declare_parameter('min_throttle_descent_speed_mps', 0.35)
        self.declare_parameter('min_throttle_disarm_duration_sec', 5.0)
        self.declare_parameter('disarm_retry_interval_sec', 1.0)

        # ===== 通用控制参数 =====
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('max_vz', 0.8)
        self.declare_parameter('max_vxy', 1.0)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('start_on_offboard_entry', True)

        # 参数读取。
        self.state_topic = self.get_parameter('state_topic').value
        self.extended_state_topic = self.get_parameter('extended_state_topic').value
        self.global_topic = self.get_parameter('global_topic').value
        self.home_topic = self.get_parameter('home_topic').value
        self.rel_alt_topic = self.get_parameter('rel_alt_topic').value
        self.vel_local_topic = self.get_parameter('vel_local_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.arming_service = self.get_parameter('arming_service').value

        self.target_alt_m = float(self.get_parameter('target_alt_m').value)
        self.alt_tolerance_m = float(self.get_parameter('alt_tolerance_m').value)
        self.home_tolerance_m = float(self.get_parameter('home_tolerance_m').value)

        kp_alt = float(self.get_parameter('kp_alt').value)
        ki_alt = float(self.get_parameter('ki_alt').value)
        kd_alt = float(self.get_parameter('kd_alt').value)
        kp_xy = float(self.get_parameter('kp_xy').value)
        ki_xy = float(self.get_parameter('ki_xy').value)
        kd_xy = float(self.get_parameter('kd_xy').value)

        self.descent_speed_mps = abs(float(self.get_parameter('descent_speed_mps').value))
        self.touchdown_descent_speed_mps = abs(float(self.get_parameter('touchdown_descent_speed_mps').value))
        self.land_rel_alt_threshold_m = float(self.get_parameter('land_rel_alt_threshold_m').value)
        self.land_vz_abs_max_mps = float(self.get_parameter('land_vz_abs_max_mps').value)
        self.land_vxy_abs_max_mps = float(self.get_parameter('land_vxy_abs_max_mps').value)
        self.land_detect_hold_sec = float(self.get_parameter('land_detect_hold_sec').value)
        self.allow_heuristic_disarm_fallback = bool(
            self.get_parameter('allow_heuristic_disarm_fallback').value
        )
        self.heuristic_disarm_hold_sec = float(self.get_parameter('heuristic_disarm_hold_sec').value)
        self.min_throttle_descent_speed_mps = abs(
            float(self.get_parameter('min_throttle_descent_speed_mps').value)
        )
        self.min_throttle_disarm_duration_sec = float(
            self.get_parameter('min_throttle_disarm_duration_sec').value
        )
        self.disarm_retry_interval_sec = float(self.get_parameter('disarm_retry_interval_sec').value)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.max_vz = float(self.get_parameter('max_vz').value)
        self.max_vxy = float(self.get_parameter('max_vxy').value)
        self.require_offboard = bool(self.get_parameter('require_offboard').value)
        self.start_on_offboard_entry = bool(self.get_parameter('start_on_offboard_entry').value)

        # PID：返航阶段同时控制高度和水平速度。
        self.pid_alt = PIDController(kp_alt, ki_alt, kd_alt, out_limit=self.max_vz)
        self.pid_east = PIDController(kp_xy, ki_xy, kd_xy, out_limit=self.max_vxy)
        self.pid_north = PIDController(kp_xy, ki_xy, kd_xy, out_limit=self.max_vxy)

        # ===== 状态变量 =====
        self.current_state = State()
        self.extended_state = ExtendedState()
        self.has_extended_state = False
        self.prev_offboard = False

        self.curr_lat = 0.0
        self.curr_lon = 0.0
        self.has_gps = False

        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0
        self.has_home = False

        self.rel_alt_m = float('nan')
        self.has_rel_alt = False

        self.vx_local = 0.0
        self.vy_local = 0.0
        self.vz_local = 0.0
        self.has_velocity = False

        self.phase = self.PHASE_ASCEND
        self.mission_started = (not self.start_on_offboard_entry)
        self.pending_start_after_data = False

        self.land_candidate_start_time = None
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

        # ===== 通信对象 =====
        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.ext_sub = self.create_subscription(
            ExtendedState,
            self.extended_state_topic,
            self.extended_state_callback,
            mavros_best_effort_qos,
        )
        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.global_topic,
            self.gps_callback,
            mavros_best_effort_qos,
        )
        self.home_sub = self.create_subscription(
            HomePosition,
            self.home_topic,
            self.home_callback,
            home_qos,
        )
        self.rel_alt_sub = self.create_subscription(
            Float64,
            self.rel_alt_topic,
            self.rel_alt_callback,
            mavros_best_effort_qos,
        )
        self.vel_sub = self.create_subscription(
            TwistStamped,
            self.vel_local_topic,
            self.vel_callback,
            mavros_best_effort_qos,
        )

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.arming_client = self.create_client(CommandBool, self.arming_service)

        self.control_timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'return_landing_node 已启动 | '
            f'state={self.state_topic} | ext={self.extended_state_topic} | '
            f'global={self.global_topic} | home={self.home_topic} | rel_alt={self.rel_alt_topic} | '
            f'vel={self.vel_local_topic} | cmd={self.cmd_vel_topic} | arming_srv={self.arming_service}'
        )

    @staticmethod
    def latlon_error_to_ne_m(curr_lat, curr_lon, target_lat, target_lon):
        """将经纬度差换算为北/东向米误差（目标点相对当前位置）。"""
        earth_radius_m = 6378137.0
        d_lat = math.radians(target_lat - curr_lat)
        d_lon = math.radians(target_lon - curr_lon)
        lat_mid = math.radians((curr_lat + target_lat) * 0.5)
        north_m = d_lat * earth_radius_m
        east_m = d_lon * earth_radius_m * math.cos(lat_mid)
        return north_m, east_m

    def reset_return_pid(self):
        self.pid_alt.reset()
        self.pid_east.reset()
        self.pid_north.reset()

    def state_callback(self, msg: State):
        is_offboard = (msg.mode == 'OFFBOARD')
        offboard_rising = (is_offboard and not self.prev_offboard)
        self.prev_offboard = is_offboard
        self.current_state = msg

        if self.start_on_offboard_entry and offboard_rising:
            # OFFBOARD 上升沿启动总任务。
            self.phase = self.PHASE_ASCEND
            self.reset_return_pid()
            self.mission_started = False
            self.pending_start_after_data = True
            self.land_candidate_start_time = None
            self.heuristic_landed_start_time = None
            self.min_throttle_start_time = None
            self.last_disarm_request_time = None
            self.disarm_in_flight = False

            if self.has_gps and self.has_rel_alt and self.has_home:
                self.mission_started = True
                self.pending_start_after_data = False
                self.get_logger().info('OFFBOARD 进入，开始执行 RETURN+LANDING')

    def extended_state_callback(self, msg: ExtendedState):
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
        self.try_start_after_data_ready()

    def home_callback(self, msg: HomePosition):
        lat = float(msg.geo.latitude)
        lon = float(msg.geo.longitude)
        alt = float(msg.geo.altitude)
        if not math.isfinite(lat) or not math.isfinite(lon) or not math.isfinite(alt):
            return

        self.home_lat = lat
        self.home_lon = lon
        self.home_alt = alt
        self.has_home = True
        self.try_start_after_data_ready()

    def rel_alt_callback(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self.rel_alt_m = float(msg.data)
        self.has_rel_alt = True
        self.try_start_after_data_ready()

    def vel_callback(self, msg: TwistStamped):
        self.vx_local = float(msg.twist.linear.x)
        self.vy_local = float(msg.twist.linear.y)
        self.vz_local = float(msg.twist.linear.z)
        self.has_velocity = True

    def try_start_after_data_ready(self):
        if not self.pending_start_after_data:
            return
        if self.has_gps and self.has_rel_alt and self.has_home:
            self.mission_started = True
            self.pending_start_after_data = False
            self.get_logger().info('关键数据就绪，任务正式开始')

    def publish_cmd(self, vx: float, vy: float, vz: float):
        """发布速度控制指令（ENU）。"""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(vx)
        cmd.twist.linear.y = float(vy)
        cmd.twist.linear.z = float(vz)
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def is_landed_by_extended_state(self) -> bool:
        """优先使用飞控 landed_state。"""
        if not self.has_extended_state:
            return False
        return self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def is_landing_candidate_by_heuristic(self) -> bool:
        """启发式接地判据：低高度 + 低速度。"""
        if not self.has_rel_alt or not self.has_velocity:
            return False
        vxy = math.hypot(self.vx_local, self.vy_local)
        cond_alt = (self.rel_alt_m <= self.land_rel_alt_threshold_m)
        cond_vz = (abs(self.vz_local) <= self.land_vz_abs_max_mps)
        cond_vxy = (vxy <= self.land_vxy_abs_max_mps)
        return cond_alt and cond_vz and cond_vxy

    def is_touchdown_ready(self) -> bool:
        """启发式连续满足一定时间后，进入接地准备。"""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.is_landing_candidate_by_heuristic():
            if self.land_candidate_start_time is None:
                self.land_candidate_start_time = now_sec
            return (now_sec - self.land_candidate_start_time) >= self.land_detect_hold_sec

        self.land_candidate_start_time = None
        return False

    def is_heuristic_landed_confirmed(self) -> bool:
        """当 landed_state 长时间不置位时，启发式兜底判定已落地。"""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.is_landing_candidate_by_heuristic():
            if self.heuristic_landed_start_time is None:
                self.heuristic_landed_start_time = now_sec
            return (now_sec - self.heuristic_landed_start_time) >= self.heuristic_disarm_hold_sec

        self.heuristic_landed_start_time = None
        return False

    def try_send_disarm(self):
        """通过 MAVROS arming 服务发送 disarm（带节流重试）。"""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.disarm_in_flight:
            return
        if self.last_disarm_request_time is not None:
            if (now_sec - self.last_disarm_request_time) < self.disarm_retry_interval_sec:
                return

        if not self.arming_client.wait_for_service(timeout_sec=0.05):
            self.latest_status = '已判定落地，但 arming 服务不可用，等待重试'
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
                self.get_logger().warn('disarm 请求失败，稍后重试')

        future.add_done_callback(_done_cb)

    def control_loop(self):
        # DONE：任务结束后持续输出零速。
        if self.phase == self.PHASE_DONE:
            self.publish_cmd(0.0, 0.0, 0.0)
            self.latest_status = '任务完成（已 disarm）'
            return

        # 安全门控：非 OFFBOARD 时不执行任务。
        if self.require_offboard and self.current_state.mode != 'OFFBOARD':
            self.publish_cmd(0.0, 0.0, 0.0)
            self.latest_status = f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        # 等待 OFFBOARD 上升沿触发 + 数据就绪。
        if self.start_on_offboard_entry and not self.mission_started:
            self.publish_cmd(0.0, 0.0, 0.0)
            self.latest_status = '等待任务启动条件（OFFBOARD 上升沿后关键数据就绪）'
            return

        # 全局条件：失锁后视为完成。
        if not self.current_state.armed and self.phase in (self.PHASE_LAND, self.PHASE_TOUCHDOWN_DISARM):
            self.phase = self.PHASE_DONE
            self.publish_cmd(0.0, 0.0, 0.0)
            self.latest_status = '检测到 armed=false，任务完成'
            return

        if self.phase == self.PHASE_ASCEND:
            if not self.has_rel_alt:
                self.publish_cmd(0.0, 0.0, 0.0)
                self.latest_status = 'ASCEND：等待 rel_alt 数据'
                return

            err_alt = self.target_alt_m - self.rel_alt_m
            vz_cmd = self.pid_alt.update(err_alt)
            self.publish_cmd(0.0, 0.0, vz_cmd)
            self.latest_status = f'ASCEND：err_alt={err_alt:.2f}m, vz={vz_cmd:.2f}m/s'

            if abs(err_alt) <= self.alt_tolerance_m:
                self.phase = self.PHASE_RETURN
                self.pid_east.reset()
                self.pid_north.reset()
                self.get_logger().info('到达目标高度，切换到 RETURN 阶段')
            return

        if self.phase == self.PHASE_RETURN:
            if not (self.has_gps and self.has_home and self.has_rel_alt):
                self.publish_cmd(0.0, 0.0, 0.0)
                self.latest_status = 'RETURN：等待 GPS/Home/rel_alt 数据'
                return

            err_north, err_east = self.latlon_error_to_ne_m(
                self.curr_lat,
                self.curr_lon,
                self.home_lat,
                self.home_lon,
            )
            dist = math.hypot(err_north, err_east)

            vx_cmd = self.pid_east.update(err_east)
            vy_cmd = self.pid_north.update(err_north)
            vz_cmd = self.pid_alt.update(self.target_alt_m - self.rel_alt_m)

            self.publish_cmd(vx_cmd, vy_cmd, vz_cmd)
            self.latest_status = (
                f'RETURN：dist={dist:.2f}m, err_e={err_east:.2f}, '
                f'err_n={err_north:.2f}, vz={vz_cmd:.2f}'
            )

            if dist <= self.home_tolerance_m:
                self.phase = self.PHASE_LAND
                self.land_candidate_start_time = None
                self.heuristic_landed_start_time = None
                self.min_throttle_start_time = None
                self.get_logger().info('已到 Home 点上方，切换到 LAND 阶段')
            return

        if self.phase == self.PHASE_LAND:
            # 与 landing 功能包保持一致：
            # 1) landed_state=ON_GROUND 时直接发 disarm；
            # 2) landed_state 长时间不置位时，启发式触发“最低油门+周期 disarm”兜底。
            if self.is_landed_by_extended_state():
                self.publish_cmd(0.0, 0.0, 0.0)
                self.latest_status = 'LAND：飞控已判定 ON_GROUND，发送 disarm 请求中'
                self.try_send_disarm()
                return

            # landed_state 长时间不置位时，按 landing 逻辑进入兜底 disarm。
            if self.allow_heuristic_disarm_fallback and self.is_heuristic_landed_confirmed():
                now_sec = self.get_clock().now().nanoseconds / 1e9
                if self.min_throttle_start_time is None:
                    self.min_throttle_start_time = now_sec

                hold_dt = now_sec - self.min_throttle_start_time
                self.publish_cmd(0.0, 0.0, -self.min_throttle_descent_speed_mps)

                if hold_dt < self.min_throttle_disarm_duration_sec:
                    self.try_send_disarm()
                    self.latest_status = (
                        f'LAND：启发式落地已满足，最低油门+周期disarm中 | '
                        f'vz_cmd=-{self.min_throttle_descent_speed_mps:.2f}m/s, '
                        f'hold={hold_dt:.2f}/{self.min_throttle_disarm_duration_sec:.2f}s'
                    )
                    return

                self.latest_status = (
                    f'LAND：最低油门阶段超时（{self.min_throttle_disarm_duration_sec:.1f}s），'
                    '等待飞控状态更新'
                )
                return
            else:
                self.min_throttle_start_time = None

            # 启发式接近落地时，低速下压等待 landed_state 置位。
            touchdown_ready = self.is_touchdown_ready()
            if touchdown_ready:
                self.publish_cmd(0.0, 0.0, -self.touchdown_descent_speed_mps)
                rel_alt_text = f'{self.rel_alt_m:.2f}m' if self.has_rel_alt else 'nan'
                vxy = math.hypot(self.vx_local, self.vy_local) if self.has_velocity else float('nan')
                self.latest_status = (
                    f'LAND：接地准备中 | vz_cmd=-{self.touchdown_descent_speed_mps:.2f}m/s, '
                    f'rel_alt={rel_alt_text}, vxy={vxy:.2f}, vz={self.vz_local:.2f}, '
                    '等待 landed_state=ON_GROUND'
                )
                return

            # 未接近落地前，按固定速度下降。
            self.publish_cmd(0.0, 0.0, -self.descent_speed_mps)
            rel_alt_text = f'{self.rel_alt_m:.2f}m' if self.has_rel_alt else 'nan'
            vxy = math.hypot(self.vx_local, self.vy_local) if self.has_velocity else float('nan')
            self.latest_status = (
                f'LAND：下降中 | vz_cmd=-{self.descent_speed_mps:.2f}m/s, '
                f'rel_alt={rel_alt_text}, vxy={vxy:.2f}, vz={self.vz_local:.2f}'
            )
            return

        if self.phase == self.PHASE_TOUCHDOWN_DISARM:
            if not self.current_state.armed:
                self.phase = self.PHASE_DONE
                self.publish_cmd(0.0, 0.0, 0.0)
                self.latest_status = 'TOUCHDOWN_DISARM：已 disarm，任务完成'
                return

            # 优先等待 landed_state；若长时间无 landed_state，则启发式兜底。
            landed_confirmed = self.is_landed_by_extended_state()
            if (not landed_confirmed) and self.allow_heuristic_disarm_fallback:
                landed_confirmed = self.is_heuristic_landed_confirmed()

            if landed_confirmed:
                if self.min_throttle_start_time is None:
                    self.min_throttle_start_time = self.get_clock().now().nanoseconds / 1e9

                elapsed = self.get_clock().now().nanoseconds / 1e9 - self.min_throttle_start_time
                self.publish_cmd(0.0, 0.0, -self.min_throttle_descent_speed_mps)
                self.try_send_disarm()

                remain = max(0.0, self.min_throttle_disarm_duration_sec - elapsed)
                self.latest_status = f'TOUCHDOWN_DISARM：最低油门+disarm 重试，剩余{remain:.1f}s'
                if elapsed >= self.min_throttle_disarm_duration_sec:
                    self.publish_cmd(0.0, 0.0, 0.0)
            else:
                # 尚未确认落地前保持轻微下压。
                self.publish_cmd(0.0, 0.0, -self.touchdown_descent_speed_mps)
                self.latest_status = 'TOUCHDOWN_DISARM：等待落地确认中'
            return

    def log_callback(self):
        mode_text = self.current_state.mode if self.current_state.mode else 'UNKNOWN'
        rel_alt_text = f'{self.rel_alt_m:.2f}' if self.has_rel_alt else 'nan'
        self.get_logger().info(
            f'[return_landing] phase={self.phase} mode={mode_text} armed={self.current_state.armed} '
            f'rel_alt={rel_alt_text} | {self.latest_status}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ReturnLandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
