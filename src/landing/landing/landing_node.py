#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64


class LandingNode(Node):
    """OFFBOARD 后恒速下降，检测落地后自动 disarm 的降落节点。"""

    def __init__(self):
        super().__init__('landing_node')

        # ===== 话题与服务参数 =====
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('extended_state_topic', '/mavros/extended_state')
        self.declare_parameter('rel_alt_topic', '/mavros/global_position/rel_alt')
        self.declare_parameter('vel_local_topic', '/mavros/local_position/velocity_local')
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel')
        self.declare_parameter('arming_service', '/mavros/cmd/arming')

        # ===== 控制参数 =====
        self.declare_parameter('descent_speed_mps', 0.5)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('start_on_offboard_entry', True)

        # ===== 落地判据（参考 PX4 落地检测思想：低高度+低速度+持续时间） =====
        self.declare_parameter('land_rel_alt_threshold_m', 0.15)
        self.declare_parameter('land_vz_abs_max_mps', 0.20)
        self.declare_parameter('land_vxy_abs_max_mps', 0.25)
        self.declare_parameter('land_detect_hold_sec', 1.0)
        self.declare_parameter('touchdown_descent_speed_mps', 0.15)
        self.declare_parameter('allow_heuristic_disarm_fallback', True)
        self.declare_parameter('heuristic_disarm_hold_sec', 3.0)
        # 启发式落地后，先下压“最低油门”等待一段时间，再申请 disarm。
        self.declare_parameter('min_throttle_descent_speed_mps', 0.35)
        self.declare_parameter('min_throttle_disarm_duration_sec', 5.0)

        # ===== Disarm 策略 =====
        self.declare_parameter('disarm_retry_interval_sec', 1.0)
        self.declare_parameter('stop_cmd_after_disarm', True)

        self.state_topic = self.get_parameter('state_topic').value
        self.extended_state_topic = self.get_parameter('extended_state_topic').value
        self.rel_alt_topic = self.get_parameter('rel_alt_topic').value
        self.vel_local_topic = self.get_parameter('vel_local_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.arming_service = self.get_parameter('arming_service').value

        self.descent_speed_mps = abs(float(self.get_parameter('descent_speed_mps').value))
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.require_offboard = bool(self.get_parameter('require_offboard').value)
        self.start_on_offboard_entry = bool(self.get_parameter('start_on_offboard_entry').value)

        self.land_rel_alt_threshold_m = float(self.get_parameter('land_rel_alt_threshold_m').value)
        self.land_vz_abs_max_mps = float(self.get_parameter('land_vz_abs_max_mps').value)
        self.land_vxy_abs_max_mps = float(self.get_parameter('land_vxy_abs_max_mps').value)
        self.land_detect_hold_sec = float(self.get_parameter('land_detect_hold_sec').value)
        self.touchdown_descent_speed_mps = abs(float(self.get_parameter('touchdown_descent_speed_mps').value))
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
        self.stop_cmd_after_disarm = bool(self.get_parameter('stop_cmd_after_disarm').value)

        # ===== 运行状态 =====
        self.current_state = State()
        self.extended_state = ExtendedState()
        self.has_extended_state = False

        self.rel_alt_m = float('nan')
        self.has_rel_alt = False

        self.vx_local = 0.0
        self.vy_local = 0.0
        self.vz_local = 0.0
        self.has_velocity = False

        self.prev_offboard = False
        self.mission_started = (not self.start_on_offboard_entry)
        self.mission_finished = False

        self.land_candidate_start_time = None
        self.touchdown_ready = False
        self.heuristic_landed_start_time = None
        self.min_throttle_start_time = None
        self.last_disarm_request_time = None
        self.disarm_in_flight = False

        self.latest_status = '等待进入 OFFBOARD'

        # ===== QoS（MAVROS 常用 BEST_EFFORT） =====
        mavros_best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ===== 通信 =====
        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.ext_state_sub = self.create_subscription(
            ExtendedState,
            self.extended_state_topic,
            self.extended_state_callback,
            mavros_best_effort_qos,
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
            self.velocity_callback,
            mavros_best_effort_qos,
        )
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        self.arming_client = self.create_client(CommandBool, self.arming_service)

        self.control_timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'landing_node 已启动 | '
            f'state={self.state_topic} | ext={self.extended_state_topic} | '
            f'rel_alt={self.rel_alt_topic} | vel={self.vel_local_topic} | '
            f'cmd={self.cmd_vel_topic} | arming_srv={self.arming_service}'
        )

    def state_callback(self, msg: State):
        is_offboard = (msg.mode == 'OFFBOARD')
        offboard_rising = (is_offboard and not self.prev_offboard)
        self.prev_offboard = is_offboard
        self.current_state = msg

        # OFFBOARD 上升沿触发降落任务启动。
        if self.start_on_offboard_entry and offboard_rising:
            self.mission_started = True
            self.mission_finished = False
            self.land_candidate_start_time = None
            self.heuristic_landed_start_time = None
            self.min_throttle_start_time = None
            self.last_disarm_request_time = None
            self.disarm_in_flight = False
            self.get_logger().info('检测到 OFFBOARD 上升沿，开始执行降落任务')

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

    def publish_cmd(self, vz_cmd: float):
        """发布下降速度指令，水平速度与角速度保持 0。"""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = float(vz_cmd)
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def is_landed_by_extended_state(self) -> bool:
        """优先使用 PX4/MAVROS 的 landed_state 判定。"""
        if not self.has_extended_state:
            return False
        # mavros_msgs/ExtendedState 常量：LANDED_STATE_ON_GROUND=1
        return self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def is_landing_candidate_by_heuristic(self) -> bool:
        """PX4 风格启发式：低高度 + 低垂速 + 低水平速度。"""
        if not self.has_rel_alt or not self.has_velocity:
            return False

        vxy = math.hypot(self.vx_local, self.vy_local)
        cond_alt = (self.rel_alt_m <= self.land_rel_alt_threshold_m)
        cond_vz = (abs(self.vz_local) <= self.land_vz_abs_max_mps)
        cond_vxy = (vxy <= self.land_vxy_abs_max_mps)
        return cond_alt and cond_vz and cond_vxy

    def is_landed(self) -> bool:
        """融合 landed_state 与启发式并加入持续时间门槛，降低误判。"""
        now_sec = self.get_clock().now().nanoseconds / 1e9

        # landed_state 明确给出 ON_GROUND 时直接判定落地。
        if self.is_landed_by_extended_state():
            return True

        # 启发式满足时开始计时，需要持续一段时间才判定落地。
        if self.is_landing_candidate_by_heuristic():
            if self.land_candidate_start_time is None:
                self.land_candidate_start_time = now_sec
            return (now_sec - self.land_candidate_start_time) >= self.land_detect_hold_sec

        self.land_candidate_start_time = None
        return False

    def is_touchdown_ready(self) -> bool:
        """启发式接地准备判定：仅用于降速/等待，不直接触发 disarm。"""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.is_landing_candidate_by_heuristic():
            if self.land_candidate_start_time is None:
                self.land_candidate_start_time = now_sec
            return (now_sec - self.land_candidate_start_time) >= self.land_detect_hold_sec

        self.land_candidate_start_time = None
        return False

    def is_heuristic_landed_confirmed(self) -> bool:
        """启发式确认落地：仅用于 landed_state 长时间不置位时的兜底 disarm 条件。"""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.is_landing_candidate_by_heuristic():
            if self.heuristic_landed_start_time is None:
                self.heuristic_landed_start_time = now_sec
            return (now_sec - self.heuristic_landed_start_time) >= self.heuristic_disarm_hold_sec

        self.heuristic_landed_start_time = None
        return False

    def try_send_disarm(self):
        """调用 MAVROS arming 服务请求 disarm（带重试节流）。"""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.disarm_in_flight:
            return
        if self.last_disarm_request_time is not None:
            if (now_sec - self.last_disarm_request_time) < self.disarm_retry_interval_sec:
                return

        if not self.arming_client.wait_for_service(timeout_sec=0.05):
            self.latest_status = '已判定落地，但 arming 服务暂不可用，等待重试'
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
                self.get_logger().info('已发送 disarm，飞控返回 success')
            else:
                self.get_logger().warn('disarm 请求返回失败，将按重试间隔继续尝试')

        future.add_done_callback(_done_cb)

    def control_loop(self):
        # 任务结束后保持零速。
        if self.mission_finished:
            self.publish_cmd(0.0)
            self.latest_status = '降落流程已完成（已 disarm），保持零速'
            return

        # 未进入 OFFBOARD 时不执行降落。
        if self.require_offboard and self.current_state.mode != 'OFFBOARD':
            self.publish_cmd(0.0)
            self.latest_status = f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        # 需要 OFFBOARD 触发启动时，未启动前保持零速。
        if self.start_on_offboard_entry and not self.mission_started:
            self.publish_cmd(0.0)
            self.latest_status = '已进入 OFFBOARD，等待降落任务启动条件'
            return

        # 已经解锁关闭（armed=false）则判定任务完成。
        if not self.current_state.armed:
            self.mission_finished = True
            self.publish_cmd(0.0 if self.stop_cmd_after_disarm else -self.descent_speed_mps)
            self.latest_status = '检测到 armed=false，降落完成'
            return

        # 若飞控明确判定已落地，再发送 disarm，避免“not landed”拒绝。
        if self.is_landed_by_extended_state():
            self.publish_cmd(0.0)
            self.latest_status = '飞控已判定 ON_GROUND，发送 disarm 请求中'
            self.try_send_disarm()
            return

        # landed_state 长时间不置位时，进入最低油门阶段。
        # 在该阶段持续下压 5s（默认）并按重试间隔周期发送 disarm。
        if self.allow_heuristic_disarm_fallback and self.is_heuristic_landed_confirmed():
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if self.min_throttle_start_time is None:
                self.min_throttle_start_time = now_sec

            hold_dt = now_sec - self.min_throttle_start_time
            self.publish_cmd(-self.min_throttle_descent_speed_mps)

            if hold_dt < self.min_throttle_disarm_duration_sec:
                # 该阶段按 disarm_retry_interval_sec 节流发送 disarm。
                # 一旦飞控已 disarm，会在上方 armed=false 分支中结束任务并停止发送。
                self.try_send_disarm()
                self.latest_status = (
                    f'启发式落地已满足，最低油门+周期disarm中 | '
                    f'vz_cmd=-{self.min_throttle_descent_speed_mps:.2f}m/s, '
                    f'hold={hold_dt:.2f}/{self.min_throttle_disarm_duration_sec:.2f}s'
                )
                return

            self.latest_status = (
                f'最低油门阶段超时（{self.min_throttle_disarm_duration_sec:.1f}s），'
                '等待飞控状态更新'
            )
            return
        else:
            self.min_throttle_start_time = None

        # 启发式接近落地时，改为小速度下压，等待飞控 landed_state 置位。
        self.touchdown_ready = self.is_touchdown_ready()
        if self.touchdown_ready:
            self.publish_cmd(-self.touchdown_descent_speed_mps)
            rel_alt_text = f'{self.rel_alt_m:.2f}m' if self.has_rel_alt else 'nan'
            vxy = math.hypot(self.vx_local, self.vy_local) if self.has_velocity else float('nan')
            self.latest_status = (
                f'接地准备中 | vz_cmd=-{self.touchdown_descent_speed_mps:.2f}m/s, '
                f'rel_alt={rel_alt_text}, vxy={vxy:.2f}, vz={self.vz_local:.2f}, '
                '等待 landed_state=ON_GROUND'
            )
            return

        # 未接近落地前，按固定下降速度下压。
        self.publish_cmd(-self.descent_speed_mps)
        rel_alt_text = f'{self.rel_alt_m:.2f}m' if self.has_rel_alt else 'nan'
        vxy = math.hypot(self.vx_local, self.vy_local) if self.has_velocity else float('nan')
        self.latest_status = (
            f'下降中 | vz_cmd=-{self.descent_speed_mps:.2f}m/s, '
            f'rel_alt={rel_alt_text}, vxy={vxy:.2f}, vz={self.vz_local:.2f}'
        )
        return


    def log_callback(self):
        self.get_logger().info(self.latest_status)


def main(args=None):
    rclpy.init(args=args)
    node = LandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
