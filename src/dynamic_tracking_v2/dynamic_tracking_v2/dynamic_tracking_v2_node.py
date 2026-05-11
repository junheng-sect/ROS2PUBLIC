#!/usr/bin/env python3

import math
import time

import rclpy
from debug_interface.msg import ArucoBasePose, PipelineTiming
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node


class PIDController:
    """简单 PID 控制器，带积分限幅和输出限幅."""

    def __init__(self, kp=0.5, ki=0.0, kd=0.05, out_limit=None, i_limit=1.0):
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
        self.prev_time = time.time()

    def reset(self):
        """在模式切换或数据超时时清空状态，避免历史积分带来突跳."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def update(self, error: float) -> float:
        """根据当前误差计算控制输出."""
        now = time.time()
        dt = now - self.prev_time
        if dt <= 1e-6:
            dt = 1e-2

        # 比例项直接反映当前误差。
        p_term = self.kp * error

        # 积分项用于消除稳态误差，并做限幅防止积分饱和。
        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        i_term = self.ki * self.integral

        # 微分项利用误差变化率抑制过冲。
        d_term = self.kd * (error - self.prev_error) / dt

        out = p_term + i_term + d_term
        if self.out_limit is not None:
            out = max(-self.out_limit, min(self.out_limit, out))

        self.prev_error = error
        self.prev_time = now
        return out


class DynamicTrackingV2Node(Node):
    """BODY_NED 控制节点：保留 XY 主链，新增视觉 Z 与 yaw PID 跟踪."""

    def __init__(self):
        super().__init__('dynamic_tracking_v2_node')

        # ===== 输入/输出话题 =====
        self.declare_parameter('pose_topic', '/debug/aruco_pose')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('setpoint_raw_topic', '/mavros/setpoint_raw/local')

        # ===== 控制目标 =====
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 2.5)
        self.declare_parameter('target_yaw', 0.0)

        # ===== XY PID：兼容粗调入口 =====
        self.declare_parameter('kp_xy', 0.5)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.08)

        # ===== XY PID：分轴精调入口（NaN 表示未单独配置） =====
        self.declare_parameter('kp_x', float('nan'))
        self.declare_parameter('ki_x', float('nan'))
        self.declare_parameter('kd_x', float('nan'))
        self.declare_parameter('kp_y', float('nan'))
        self.declare_parameter('ki_y', float('nan'))
        self.declare_parameter('kd_y', float('nan'))

        # ===== Z / yaw PID 与 yaw 修正参数 =====
        self.declare_parameter('kp_z', 0.5)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.03)
        self.declare_parameter('kp_yaw', 0.4)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.03)
        self.declare_parameter('camera_yaw_compensation_deg', 0.0)

        # ===== 安全参数 =====
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('v_limit', 0.8)
        self.declare_parameter('vz_limit', 0.3)
        self.declare_parameter('yaw_rate_limit', 0.4)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('require_offboard', True)
        self.declare_parameter('enable_z_hold', True)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.setpoint_raw_topic = self.get_parameter('setpoint_raw_topic').value

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

        # 分轴参数优先于粗调参数，方便从统一调参逐步切到分轴精调。
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

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
        self.v_limit = abs(float(self.get_parameter('v_limit').value))
        self.vz_limit = abs(float(self.get_parameter('vz_limit').value))
        self.yaw_rate_limit = abs(
            float(self.get_parameter('yaw_rate_limit').value)
        )
        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)
        self.require_offboard = bool(self.get_parameter('require_offboard').value)
        self.enable_z_hold = bool(self.get_parameter('enable_z_hold').value)

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

        # ===== 状态缓存 =====
        self.current_state = State()
        self.pose = ArucoBasePose()
        self.has_pose = False
        self.last_pose_time = None
        self.latest_status = '等待视觉与飞控状态数据'

        self.state_sub = self.create_subscription(
            State,
            self.state_topic,
            self.state_callback,
            10,
        )
        self.pose_sub = self.create_subscription(
            ArucoBasePose,
            self.pose_topic,
            self.pose_callback,
            10,
        )
        self.setpoint_pub = self.create_publisher(
            PositionTarget,
            self.setpoint_raw_topic,
            10,
        )
        # 发布完整视觉链 timing，供 CSV logger 直接记录真实节点内部时间点。
        self.pipeline_timing_pub = self.create_publisher(
            PipelineTiming,
            '/debug/pipeline_timing',
            10,
        )

        # 当前最新视觉样本在第 4 级真正进入回调的时刻。
        self.latest_ctrl_cb_start_stamp = None

        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1.0),
            self.control_loop,
        )
        self.log_timer = self.create_timer(1.0, self.log_callback)

        self.get_logger().info(
            'dynamic_tracking_v2_node 已启动 | '
            f'pose={self.pose_topic} | state={self.state_topic} | '
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
            f'v_limit={self.v_limit:.3f} | '
            f'vz_limit={self.vz_limit:.3f} | '
            f'yaw_rate_limit={self.yaw_rate_limit:.3f}'
        )
        self.get_logger().info(
            '参数来源 | '
            f'X轴={"split(kp_x/ki_x/kd_x)" if self.x_use_split else "legacy(kp_xy/ki_xy/kd_xy)"} | '
            f'Y轴={"split(kp_y/ki_y/kd_y)" if self.y_use_split else "legacy(kp_xy/ki_xy/kd_xy)"}'
        )
        self.get_logger().info(
            'yaw 修正 | '
            f'camera_yaw_compensation_deg={self.camera_yaw_compensation_deg:.3f} '
            '(俯视逆时针为正；当前视觉链 yaw 在控制前先取反，再叠加固定补偿)'
        )
        self.get_logger().info(
            '控制优先级 | XY 主控沿用 pid_tuning_v6；'
            'Z 与 yaw 使用更保守的默认 PID 与限幅'
        )
        self.get_logger().info(
            'Z 高度来源使用视觉 aruco_z，yaw 误差闭环到 '
            'yaw_rel_corrected 并输出 yaw_rate'
        )

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        """将角度归一化到 [-pi, pi]."""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def apply_deadband(value: float, deadband: float) -> float:
        """把很小的速度指令压到零，减少悬停抖动."""
        return 0.0 if abs(value) < deadband else value

    @staticmethod
    def limit_horizontal_velocity(
        vx_raw: float,
        vy_raw: float,
        v_limit: float,
    ) -> tuple[float, float, float, float]:
        """
        对水平速度做向量幅值限幅，并保持速度方向不变.

        这里不再按 X/Y 分轴分别裁剪，而是先计算速度幅值：
        v_norm = sqrt(vx_raw^2 + vy_raw^2)

        当 v_norm 超过 v_limit 时，对 vx_raw 和 vy_raw 乘同一个缩放系数：
        scale = v_limit / v_norm

        由于两个分量使用同一个比例缩放，因此 vx:vy 的方向比值保持不变，
        最终速度仍然指向 target，不会因为单轴先饱和而把方向“折弯”成绕圈轨迹。
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
        将 marker 平面误差旋转到机体平面误差.

        这里严格采用：
        e_body = R(-yaw_rel_corrected) * e_marker

        其中：
        - ex_marker = target_x - pose.x
        - ey_marker = pose.y - target_y
        - yaw_rel_corrected = 控制使用的相对 yaw + 固定补偿角
        """
        cos_yaw = math.cos(yaw_rel_corrected)
        sin_yaw = math.sin(yaw_rel_corrected)
        ex_body = cos_yaw * ex_marker + sin_yaw * ey_marker
        ey_body = -sin_yaw * ex_marker + cos_yaw * ey_marker
        return ex_body, ey_body

    def reset_all_pids(self):
        """统一重置所有控制器，避免状态残留影响下一次接管."""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()

    def state_callback(self, msg: State):
        self.current_state = msg

    def pose_callback(self, msg: ArucoBasePose):
        self.pose = msg
        self.has_pose = True
        self.last_pose_time = self.get_clock().now()
        # 第 4 级回调一进入就记录，后续用于区分控制节点排队与计算耗时。
        self.latest_ctrl_cb_start_stamp = self.last_pose_time.to_msg()

    def is_pose_fresh(self) -> bool:
        """判断视觉位姿是否在允许时限内."""
        if not self.has_pose or self.last_pose_time is None:
            return False
        dt = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        return dt <= self.pose_timeout_sec

    def publish_body_velocity(
        self,
        vx_body: float,
        vy_body: float,
        vz_body_flu: float,
        yaw_rate: float,
        *,
        timing_pose_msg: ArucoBasePose | None = None,
        ctrl_cb_start_stamp=None,
    ):
        """
        发布 BODY_NED 速度指令.

        约定与 `pid_tuning_v6` 保持一致：
        - 内部控制按 FLU 机体系理解：x前、y左、z上
        - 发布到 `FRAME_BODY_NED` 时：`msg.velocity.y = -vy_body`
        - `msg.velocity.z = vz_body_flu`
        - yaw 闭环输出 `yaw_rate`，不发布绝对 yaw 角
        """
        msg = PositionTarget()
        # 速度指令 header.stamp 与 timing 消息中的 setpoint_pub_stamp 必须严格一致。
        setpoint_pub_stamp = self.get_clock().now().to_msg()
        msg.header.stamp = setpoint_pub_stamp
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

        if timing_pose_msg is None or ctrl_cb_start_stamp is None:
            return

        timing_msg = PipelineTiming()
        timing_msg.header = timing_pose_msg.header
        timing_msg.tvec_cb_start_stamp = timing_pose_msg.tvec_cb_start_stamp
        timing_msg.tvec_cv_bridge_done_stamp = (
            timing_pose_msg.tvec_cv_bridge_done_stamp
        )
        timing_msg.tvec_detect_done_stamp = timing_pose_msg.tvec_detect_done_stamp
        timing_msg.tvec_pose_done_stamp = timing_pose_msg.tvec_pose_done_stamp
        timing_msg.tvec_pub_stamp = timing_pose_msg.tvec_pub_stamp
        timing_msg.tf_cb_start_stamp = timing_pose_msg.tf_cb_start_stamp
        timing_msg.tf_pub_stamp = timing_pose_msg.tf_pub_stamp
        timing_msg.ctrl_cb_start_stamp = ctrl_cb_start_stamp
        timing_msg.setpoint_pub_stamp = setpoint_pub_stamp
        self.pipeline_timing_pub.publish(timing_msg)

    def control_loop(self):
        """主控制循环：XY 保持主链，Z/yaw 使用视觉闭环."""
        if self.require_offboard and self.current_state.mode != 'OFFBOARD':
            self.reset_all_pids()
            self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
            self.latest_status = f'模式={self.current_state.mode}，未进入 OFFBOARD，输出零速'
            return

        if not self.is_pose_fresh():
            self.reset_all_pids()
            self.publish_body_velocity(0.0, 0.0, 0.0, 0.0)
            self.latest_status = '视觉位姿超时，输出零速并重置 PID'
            return

        # 先在 marker 平面计算误差，符号定义严格沿用 v6。
        ex_marker = self.target_x - float(self.pose.x)
        ey_marker = float(self.pose.y) - self.target_y

        # 当前 `/debug/aruco_pose.yaw` 与 BODY_NED 控制所需旋转方向相反，
        # 因此控制前先取反，再叠加固定相机安装补偿角。
        yaw_rel_raw = self.wrap_to_pi(-float(self.pose.yaw))
        yaw_rel_corrected = self.wrap_to_pi(
            yaw_rel_raw + self.camera_yaw_compensation_rad
        )
        ex_body, ey_body = self.rotate_marker_error_to_body(
            ex_marker,
            ey_marker,
            yaw_rel_corrected,
        )

        # X/Y PID 先输出原始速度，再统一做一次水平速度向量限幅，
        # 避免旧版“分轴限幅”把速度方向从 target 方向拉偏。
        vx_raw = self.pid_x.update(ex_body)
        vy_raw = self.pid_y.update(ey_body)
        vx_limited, vy_limited, horizontal_speed_raw, horizontal_scale = (
            self.limit_horizontal_velocity(vx_raw, vy_raw, self.v_limit)
        )
        vx = self.apply_deadband(vx_limited, self.velocity_deadband)
        vy = self.apply_deadband(vy_limited, self.velocity_deadband)

        aruco_z = float(self.pose.z)
        ez = self.target_z - aruco_z
        if self.enable_z_hold:
            vz = self.apply_deadband(
                self.pid_z.update(ez),
                self.velocity_deadband,
            )
            z_status = f'aruco_z={aruco_z:.3f}, ez={ez:.3f}'
        else:
            self.pid_z.reset()
            vz = 0.0
            z_status = 'enable_z_hold=false，Z 保持关闭'

        eyaw = self.wrap_to_pi(self.target_yaw - yaw_rel_corrected)

        # 实机日志表明：若直接发布 PID 输出，`yaw_rel_corrected` 会持续远离 target，
        # 说明当前视觉 yaw 口径与 `FRAME_BODY_NED` 的 yaw_rate 正方向之间仍差一个符号。
        # 因此这里保持误差定义 `target_yaw - yaw_rel_corrected` 不变，
        # 只在最终发布前对 yaw_rate 做一次符号转换。
        yaw_rate = -self.pid_yaw.update(eyaw)

        self.publish_body_velocity(
            vx,
            vy,
            vz,
            yaw_rate,
            timing_pose_msg=self.pose,
            ctrl_cb_start_stamp=self.latest_ctrl_cb_start_stamp,
        )
        self.latest_status = (
            '闭环控制 | '
            f'ex_marker={ex_marker:.3f}, ey_marker={ey_marker:.3f} | '
            f'yaw_rel_raw={yaw_rel_raw:.3f} rad, '
            f'yaw_rel_corrected={yaw_rel_corrected:.3f} rad | '
            f'ex_body={ex_body:.3f}, ey_body={ey_body:.3f} | '
            f'{z_status} | '
            f'target_yaw={self.target_yaw:.3f}, eyaw={eyaw:.3f}, '
            f'yaw_rate={yaw_rate:.3f} | '
            f'vx_raw={vx_raw:.3f}, vy_raw={vy_raw:.3f}, '
            f'horizontal_speed_raw={horizontal_speed_raw:.3f}, '
            f'horizontal_scale={horizontal_scale:.3f} | '
            f'vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}'
        )

    def log_callback(self):
        """按 1Hz 输出状态摘要，便于在线调参."""
        self.get_logger().info(self.latest_status)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicTrackingV2Node()
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
