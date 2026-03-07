#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import HomePosition, State
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
        """重置积分项和微分历史，避免模式切换时突变。"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def update(self, error: float) -> float:
        now = time.time()
        dt = now - self.prev_time
        if dt <= 1e-6:
            dt = 1e-2

        # P 项：当前误差的直接比例输出。
        p_term = self.kp * error

        # I 项：对误差积分，使用限幅防止积分饱和。
        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        i_term = self.ki * self.integral

        # D 项：抑制误差变化过快导致的过冲。
        d_term = self.kd * (error - self.prev_error) / dt

        out = p_term + i_term + d_term
        out = max(-self.out_limit, min(self.out_limit, out))

        self.prev_error = error
        self.prev_time = now
        return out


class ReturnHomeNode(Node):
    """进入 OFFBOARD 后先升到 3m，再按 GPS 导航回 home 点并悬停。"""

    PHASE_ASCEND = 'ASCEND'
    PHASE_RETURN = 'RETURN'
    PHASE_HOVER = 'HOVER'

    def __init__(self):
        super().__init__('return_home_node')

        # 话题参数。
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('global_topic', '/mavros/global_position/global')
        self.declare_parameter('home_topic', '/mavros/home_position/home')
        self.declare_parameter('rel_alt_topic', '/mavros/global_position/rel_alt')
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel')

        # 控制目标与阈值。
        self.declare_parameter('target_alt_m', 3.0)
        self.declare_parameter('alt_tolerance_m', 0.15)
        self.declare_parameter('horizontal_tolerance_m', 0.50)

        # PID 参数。
        self.declare_parameter('kp_alt', 0.8)
        self.declare_parameter('ki_alt', 0.0)
        self.declare_parameter('kd_alt', 0.05)

        self.declare_parameter('kp_xy', 0.35)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.08)

        # 限幅与频率。
        self.declare_parameter('max_vz', 0.8)
        self.declare_parameter('max_vxy', 1.0)
        self.declare_parameter('control_rate_hz', 20.0)

        # 安全门控：默认仅在 OFFBOARD 下输出控制速度。
        self.declare_parameter('require_offboard', True)
        # 返航任务仅在 OFFBOARD 上升沿启动。
        self.declare_parameter('start_return_on_offboard_entry', True)

        self.state_topic = self.get_parameter('state_topic').value
        self.global_topic = self.get_parameter('global_topic').value
        self.home_topic = self.get_parameter('home_topic').value
        self.rel_alt_topic = self.get_parameter('rel_alt_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.target_alt_m = float(self.get_parameter('target_alt_m').value)
        self.alt_tolerance_m = float(self.get_parameter('alt_tolerance_m').value)
        self.horizontal_tolerance_m = float(self.get_parameter('horizontal_tolerance_m').value)

        kp_alt = float(self.get_parameter('kp_alt').value)
        ki_alt = float(self.get_parameter('ki_alt').value)
        kd_alt = float(self.get_parameter('kd_alt').value)

        kp_xy = float(self.get_parameter('kp_xy').value)
        ki_xy = float(self.get_parameter('ki_xy').value)
        kd_xy = float(self.get_parameter('kd_xy').value)

        self.max_vz = float(self.get_parameter('max_vz').value)
        self.max_vxy = float(self.get_parameter('max_vxy').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.require_offboard = bool(self.get_parameter('require_offboard').value)
        self.start_return_on_offboard_entry = bool(
            self.get_parameter('start_return_on_offboard_entry').value
        )

        # 控制器：z 使用单独 PID，水平 east/north 各一个 PID。
        self.pid_alt = PIDController(kp_alt, ki_alt, kd_alt, out_limit=self.max_vz)
        self.pid_east = PIDController(kp_xy, ki_xy, kd_xy, out_limit=self.max_vxy)
        self.pid_north = PIDController(kp_xy, ki_xy, kd_xy, out_limit=self.max_vxy)

        # 状态量。
        self.current_state = State()
        self.has_gps = False
        self.has_rel_alt = False
        self.has_home_position = False

        self.curr_lat = 0.0
        self.curr_lon = 0.0
        self.curr_rel_alt = 0.0

        self.home_set = False
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0

        self.phase = self.PHASE_ASCEND
        self.return_started = (not self.start_return_on_offboard_entry)
        self.prev_offboard = False
        self.pending_start_after_data = False
        self.latest_status = '等待数据'

        # 通信对象。
        # MAVROS 的 global/rel_alt 发布端为 BEST_EFFORT，若订阅端用默认 RELIABLE 会不兼容收不到数据。
        mavros_best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # home_position 发布端为 RELIABLE + TRANSIENT_LOCAL，订阅端使用同等 QoS 可拿到历史 home 消息。
        home_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
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
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        # 控制循环和日志循环。
        self.control_timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'return_home_node 已启动 | '
            f'state={self.state_topic} | global={self.global_topic} | '
            f'home={self.home_topic} | rel_alt={self.rel_alt_topic} | cmd={self.cmd_vel_topic}'
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

    def state_callback(self, msg: State):
        is_offboard = (msg.mode == 'OFFBOARD')
        offboard_rising = (is_offboard and not self.prev_offboard)
        self.prev_offboard = is_offboard
        self.current_state = msg

        # 仅在进入 OFFBOARD 的上升沿启动返航任务。
        if self.start_return_on_offboard_entry and offboard_rising:
            self.phase = self.PHASE_ASCEND
            self.reset_pids()
            self.return_started = False
            self.pending_start_after_data = True

            # 若关键数据已就绪则立即开始返航；否则等待后续回调触发启动。
            if self.has_gps and self.has_rel_alt and self.has_home_position and self.home_set:
                self.return_started = True
                self.pending_start_after_data = False
                self.get_logger().info('OFFBOARD 进入，返航任务开始执行')

    def gps_callback(self, msg: NavSatFix):
        # 无效定位直接忽略，避免错误控制。
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        if msg.status.status < 0:
            return

        self.curr_lat = float(msg.latitude)
        self.curr_lon = float(msg.longitude)
        self.has_gps = True

        self.try_start_after_data_ready()

    def home_callback(self, msg: HomePosition):
        """读取 MAVROS home 点（地理坐标）作为返航目标。"""
        lat = float(msg.geo.latitude)
        lon = float(msg.geo.longitude)
        alt = float(msg.geo.altitude)

        if not math.isfinite(lat) or not math.isfinite(lon) or not math.isfinite(alt):
            return

        self.home_lat = lat
        self.home_lon = lon
        self.home_alt = alt
        self.home_set = True
        self.has_home_position = True

        self.get_logger().info(
            f'收到 home 点: lat={self.home_lat:.8f}, lon={self.home_lon:.8f}, alt={self.home_alt:.3f}'
        )

        self.try_start_after_data_ready()

    def rel_alt_callback(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self.curr_rel_alt = float(msg.data)
        self.has_rel_alt = True

        self.try_start_after_data_ready()

    def try_start_after_data_ready(self):
        """OFFBOARD 上升沿后，数据齐全时启动返航任务。"""
        if not self.pending_start_after_data:
            return
        if self.current_state.mode != 'OFFBOARD':
            return
        if not (self.has_gps and self.has_rel_alt and self.has_home_position and self.home_set):
            return

        self.return_started = True
        self.pending_start_after_data = False
        self.get_logger().info('OFFBOARD 进入后数据已就绪，返航任务开始执行')

    def publish_cmd(self, vx: float, vy: float, vz: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        # /mavros/setpoint_velocity/cmd_vel 使用局部 ENU：x=East, y=North, z=Up。
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)

        # 本返航任务只要求位置返航，角速度设为 0。
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def reset_pids(self):
        self.pid_alt.reset()
        self.pid_east.reset()
        self.pid_north.reset()

    def control_loop(self):
        # 未进 OFFBOARD 时输出零速并等待。
        if self.require_offboard and self.current_state.mode != 'OFFBOARD':
            self.publish_cmd(0.0, 0.0, 0.0)
            self.reset_pids()
            self.latest_status = f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        # 要求 OFFBOARD 触发启动时，未触发前保持零速等待。
        if self.start_return_on_offboard_entry and not self.return_started:
            self.publish_cmd(0.0, 0.0, 0.0)
            self.reset_pids()
            self.latest_status = '已进入 OFFBOARD，等待返航启动条件满足，输出零速'
            return

        # 数据未就绪时不执行控制。
        if not self.has_gps or not self.has_rel_alt or not self.home_set or not self.has_home_position:
            self.publish_cmd(0.0, 0.0, 0.0)
            self.reset_pids()
            self.latest_status = '等待 GPS/相对高度/home_position 数据，输出零速'
            return

        alt_err = self.target_alt_m - self.curr_rel_alt
        vz_cmd = self.pid_alt.update(alt_err)

        # 阶段1：先调高到 3m（容差内切入返航）。
        if self.phase == self.PHASE_ASCEND:
            self.publish_cmd(0.0, 0.0, vz_cmd)
            self.latest_status = (
                f'阶段=ASCEND | alt={self.curr_rel_alt:.2f}m, '
                f'alt_err={alt_err:.2f}m, vz={vz_cmd:.2f}m/s'
            )

            if abs(alt_err) <= self.alt_tolerance_m:
                self.phase = self.PHASE_RETURN
                self.pid_east.reset()
                self.pid_north.reset()
                self.get_logger().info('已达到目标高度，切换到 RETURN 阶段')
            return

        # 阶段2：保持目标高度的同时，飞回 home 的水平位置。
        if self.phase == self.PHASE_RETURN:
            err_north_m, err_east_m = self.latlon_error_to_ne_m(
                self.curr_lat,
                self.curr_lon,
                self.home_lat,
                self.home_lon,
            )
            horiz_dist = math.hypot(err_north_m, err_east_m)

            # ENU 速度映射：vx 对应 East，vy 对应 North。
            vx_cmd = self.pid_east.update(err_east_m)
            vy_cmd = self.pid_north.update(err_north_m)

            self.publish_cmd(vx_cmd, vy_cmd, vz_cmd)
            self.latest_status = (
                f'阶段=RETURN | dist={horiz_dist:.2f}m, '
                f'err_n={err_north_m:.2f}m, err_e={err_east_m:.2f}m, '
                f'vz={vz_cmd:.2f}m/s'
            )

            if horiz_dist <= self.horizontal_tolerance_m:
                self.phase = self.PHASE_HOVER
                self.pid_east.reset()
                self.pid_north.reset()
                self.get_logger().info('已回到 home 水平位置，切换到 HOVER 阶段')
            return

        # 阶段3：水平悬停（xy 置零），维持 3m 高度。
        self.publish_cmd(0.0, 0.0, vz_cmd)
        self.latest_status = (
            f'阶段=HOVER | alt={self.curr_rel_alt:.2f}m, '
            f'alt_err={alt_err:.2f}m, vz={vz_cmd:.2f}m/s'
        )

    def log_callback(self):
        self.get_logger().info(self.latest_status)


def main(args=None):
    rclpy.init(args=args)
    node = ReturnHomeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
