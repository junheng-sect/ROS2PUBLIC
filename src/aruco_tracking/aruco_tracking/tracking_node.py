#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ArUco 跟踪控制节点

功能：
- 融合 ArUco 视觉位姿信息
- 通过 PID 控制实现无人机水平位置自动对齐
- 通过 PID 控制实现无人机 yaw 角度对齐（使 yaw 趋向于 0）
- 目标：使 vision_pose 与 map 坐标系 XY 平面对齐，且 yaw=0

TF Tree:
    map ├── base_link (MAVROS)
        └── vision_pose (ArUco 视觉估计)

控制逻辑：
- 订阅 /aruco_pose 获取视觉位姿
- 目标位置：map 坐标系原点 (0, 0)
- 目标 yaw：0 弧度（机头朝北）
- 通过 PID 计算速度和角速度指令
- 发布到 /mavros/setpoint_velocity/cmd_vel
- 进入 Offboard 模式后自动启用对齐控制
- 未检测到 ArUco 时保持悬停
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# 消息类型导入
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from std_msgs.msg import Header
from aruco_interfaces.msg import ArucoPose
from std_srvs.srv import SetBool

import time
import math


class PIDController:
    """
    简单的 PID 控制器类，用于计算位置/角度误差对应的速度/角速度指令
    """
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, limit=5.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit  # 输出限幅，防止速度过大

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def update(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt <= 0:
            dt = 0.01  # 防止除零

        # P
        p_term = self.kp * error
        # I
        self.integral += error * dt
        # 积分抗饱和
        self.integral = max(-self.limit, min(self.limit, self.integral))
        i_term = self.ki * self.integral
        # D
        d_term = self.kd * (error - self.prev_error) / dt

        output = p_term + i_term + d_term

        # 输出限幅
        output = max(-self.limit, min(self.limit, output))

        self.prev_error = error
        self.prev_time = current_time
        return output


class ArucoTrackingNode(Node):
    def __init__(self):
        super().__init__('aruco_tracking')

        # ========== 参数配置 ==========
        # PID 参数（水平方向）
        self.declare_parameter('kp_xy', 0.5)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.1)

        # PID 参数（Z 轴，通常更敏感）
        self.declare_parameter('kp_z', 1.2)
        self.declare_parameter('ki_z', 0.01)
        self.declare_parameter('kd_z', 0.05)

        # PID 参数（Yaw 轴）- 新增
        self.declare_parameter('kp_yaw', 1.0)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.05)

        # 控制频率
        self.declare_parameter('control_frequency', 50.0)  # Hz

        # 目标位置（相对于 map 坐标系）
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z_offset', 0.0)  # Z 轴偏移

        # 目标 yaw 角度（弧度）- 新增
        self.declare_parameter('target_yaw', 0.0)  # 0 弧度 = 机头朝北

        # 对齐阈值
        self.declare_parameter('align_threshold_xy', 0.1)  # 米
        self.declare_parameter('align_threshold_z', 0.05)  # 米
        self.declare_parameter('align_threshold_yaw', 0.1)  # 弧度 (~5.7 度) - 新增

        # 是否启用 Z 轴控制
        self.declare_parameter('enable_z_control', False)

        # 是否启用 yaw 控制 - 新增
        self.declare_parameter('enable_yaw_control', True)

        # 是否启用视觉对齐（否则使用位置控制）
        self.declare_parameter('use_vision_alignment', True)

        # 进入 Offboard 后是否自动启用控制
        self.declare_parameter('auto_enable_offboard', True)

        # 速度死区（m/s）- 小于此值的速度输出为 0，防止漂移
        self.declare_parameter('velocity_deadband', 0.05)  # 5cm/s

        # 角速度死区（rad/s）- 新增
        self.declare_parameter('angular_velocity_deadband', 0.02)  # ~1.1 度/s

        # 获取参数
        kp_xy = self.get_parameter('kp_xy').value
        ki_xy = self.get_parameter('ki_xy').value
        kd_xy = self.get_parameter('kd_xy').value
        kp_z = self.get_parameter('kp_z').value
        ki_z = self.get_parameter('ki_z').value
        kd_z = self.get_parameter('kd_z').value
        kp_yaw = self.get_parameter('kp_yaw').value
        ki_yaw = self.get_parameter('ki_yaw').value
        kd_yaw = self.get_parameter('kd_yaw').value
        freq = self.get_parameter('control_frequency').value

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_z_offset = self.get_parameter('target_z_offset').value
        self.target_yaw = self.get_parameter('target_yaw').value
        self.align_threshold_xy = self.get_parameter('align_threshold_xy').value
        self.align_threshold_z = self.get_parameter('align_threshold_z').value
        self.align_threshold_yaw = self.get_parameter('align_threshold_yaw').value
        self.enable_z_control = self.get_parameter('enable_z_control').value
        self.enable_yaw_control = self.get_parameter('enable_yaw_control').value
        self.use_vision_alignment = self.get_parameter('use_vision_alignment').value
        self.auto_enable_offboard = self.get_parameter('auto_enable_offboard').value
        self.velocity_deadband = self.get_parameter('velocity_deadband').value
        self.angular_velocity_deadband = self.get_parameter('angular_velocity_deadband').value

        # ArUco 检测超时时间（秒）
        self.declare_parameter('aruco_timeout', 0.5)  # 0.5 秒未检测到视为丢失
        self.aruco_timeout = self.get_parameter('aruco_timeout').value

        # 初始悬停时间（秒）- 进入 Offboard 后先悬停一段时间，等数据稳定
        self.declare_parameter('initial_hover_time', 2.0)
        self.initial_hover_time = self.get_parameter('initial_hover_time').value

        # ========== 初始化 PID 控制器 ==========
        self.pid_x = PIDController(kp=kp_xy, ki=ki_xy, kd=kd_xy, limit=1.0)  # 限幅 1m/s
        self.pid_y = PIDController(kp=kp_xy, ki=ki_xy, kd=kd_xy, limit=1.0)
        self.pid_z = PIDController(kp=kp_z, ki=ki_z, kd=kd_z, limit=1.0)  # Z 轴限幅 1m/s
        self.pid_yaw = PIDController(kp=kp_yaw, ki=ki_yaw, kd=kd_yaw, limit=1.0)  # yaw 限幅 1rad/s

        # ========== 状态变量 ==========
        # vision_pose 位置（来自 ArUco 检测）
        self.vision_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        # 目标位置
        self.target_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}

        # 状态标志
        self.drone_state = State()
        self.has_vision_pos = False  # 是否有 ArUco 视觉数据（经过超时判断）
        self.is_aligned = False  # 是否已对齐
        self.control_enabled = False  # 是否启用控制
        self.prev_offboard_mode = False  # 记录上一次是否为 Offboard 模式

        # ArUco 检测时间戳（用于超时判断）
        self.last_aruco_time = None  # 最后一次检测到 ArUco 的时间

        # 进入 Offboard 的时间（用于初始悬停）
        self.offboard_enter_time = None  # 进入 Offboard 的时刻

        # ========== QoS 配置 ==========
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ========== 订阅者 ==========
        # 无人机状态
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10
        )
        # ArUco 位姿（vision_pose 位置）
        self.vision_sub = self.create_subscription(
            ArucoPose, '/aruco_pose', self.vision_callback, qos_profile
        )

        # ========== 发布者 ==========
        # 速度控制指令
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10
        )

        # ========== 服务端 ==========
        # 启用/禁用对齐控制
        self.enable_service = self.create_service(
            SetBool, 'enable_alignment', self.enable_callback
        )
        # 重置 PID
        self.reset_service = self.create_service(
            SetBool, 'reset_pid', self.reset_callback
        )

        # ========== 定时器 ==========
        # 控制循环（50Hz）
        self.timer = self.create_timer(1.0/freq, self.timer_callback)

        # 状态日志（2Hz）
        self.log_timer = self.create_timer(0.5, self.log_callback)
        self.latest_log = ""

        # ========== 日志 ==========
        self.get_logger().info('ArUco Tracking Node Initialized')
        self.get_logger().info(f'  - KP(xy)={kp_xy}, KI(xy)={ki_xy}, KD(xy)={kd_xy}')
        self.get_logger().info(f'  - KP(z)={kp_z}, KI(z)={ki_z}, KD(z)={kd_z}')
        self.get_logger().info(f'  - KP(yaw)={kp_yaw}, KI(yaw)={ki_yaw}, KD(yaw)={kd_yaw}')
        self.get_logger().info(f'  - Target: [{self.target_x}, {self.target_y}, {self.target_z_offset}], Yaw={self.target_yaw:.2f}rad')
        self.get_logger().info('Waiting for FCU connection...')

    def state_callback(self, msg):
        """
        无人机状态回调
        进入 Offboard 模式时自动启用控制
        """
        self.drone_state = msg

        # 检测是否进入 Offboard 模式
        if msg.mode == "OFFBOARD" and not self.prev_offboard_mode:
            if self.auto_enable_offboard:
                self.control_enabled = True
                # 记录进入 Offboard 的时间
                self.offboard_enter_time = self.get_clock().now()
                # 重置 PID
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_z.reset()
                self.pid_yaw.reset()
                self.get_logger().info(f'Offboard detected, will hover for {self.initial_hover_time}s before alignment')

        self.prev_offboard_mode = (msg.mode == "OFFBOARD")

    def vision_callback(self, msg: ArucoPose):
        """
        接收 ArUco 检测的 vision_pose 位置
        vision_pose 表示基于 ArUco 视觉估计的无人机在 map 坐标系中的位姿
        """
        # 更新 ArUco 检测时间戳
        self.last_aruco_time = self.get_clock().now()

        self.vision_pos['x'] = msg.x
        self.vision_pos['y'] = msg.y
        self.vision_pos['z'] = msg.z
        self.vision_pos['yaw'] = msg.yaw
        # has_vision_pos 在 timer 中根据超时判断更新

    def enable_callback(self, request, response):
        """
        启用/禁用对齐控制
        """
        self.control_enabled = request.data
        if request.data:
            # 启用时，重置 PID
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.pid_yaw.reset()
            response.success = True
            response.message = "Alignment control enabled"
            self.get_logger().info('Alignment Control ENABLED')
        else:
            response.success = True
            response.message = "Alignment control disabled"
            self.get_logger().info('Alignment Control DISABLED')
        return response

    def reset_callback(self, request, response):
        """
        重置 PID 积分
        """
        if request.data:
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.pid_yaw.reset()
            response.success = True
            response.message = "PID reset"
            self.get_logger().info('PID Reset')
        else:
            response.success = True
            response.message = "PID not reset"
        return response

    def _normalize_angle(self, angle):
        """
        将角度归一化到 [-π, π] 范围
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def timer_callback(self):
        """
        主控制循环（50Hz）
        """
        # ========== 安全检查 ==========
        if not self.drone_state.connected:
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)
            return

        if self.drone_state.mode != "OFFBOARD":
            # 非 Offboard 模式，发布零速度保持心跳
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)
            return

        if not self.control_enabled:
            # 控制未启用，发布零速度
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)
            return

        # ========== 初始悬停检查 ==========
        if self.offboard_enter_time is not None:
            elapsed = (self.get_clock().now() - self.offboard_enter_time).nanoseconds / 1e9
            if elapsed < self.initial_hover_time:
                # 初始悬停阶段，直接悬停
                self.publish_velocity(0.0, 0.0, 0.0, 0.0)
                if int(elapsed * 10) % 50 == 0:  # 每 0.1 秒打印一次
                    self.get_logger().info(f'Initial hover: {self.initial_hover_time - elapsed:.1f}s remaining...')
                return
            else:
                # 悬停结束，重置 PID（清除悬停期间的积分累积）
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_z.reset()
                self.pid_yaw.reset()
                self.offboard_enter_time = None
                self.get_logger().info('Initial hover done, starting alignment control')

        # ========== 检查 ArUco 检测状态（超时判断） ==========
        current_time = self.get_clock().now()
        if self.last_aruco_time is not None:
            elapsed = (current_time - self.last_aruco_time).nanoseconds / 1e9
            if elapsed > self.aruco_timeout:
                # 超过超时时间未检测到 ArUco
                self.has_vision_pos = False
            else:
                # 在超时时间内，认为有有效数据
                self.has_vision_pos = True
        else:
            # 从未检测到 ArUco
            self.has_vision_pos = False

        # ========== 计算控制量 ==========
        vx, vy, vz, wz = 0.0, 0.0, 0.0, 0.0

        # 使用 vision_pose 进行对齐
        if self.has_vision_pos:
            # 目标：vision_pose 与 map 原点重合 (0, 0)，yaw=0
            err_x = self.target_x - self.vision_pos['x']
            err_y = self.target_y - self.vision_pos['y']

            # Z 轴：保持当前高度（不控制）
            if self.enable_z_control:
                err_z = self.target_z_offset - self.vision_pos['z']
            else:
                err_z = 0.0

            # Yaw 轴：计算角度误差（需要归一化到 [-π, π]）
            if self.enable_yaw_control:
                err_yaw = self._normalize_angle(self.target_yaw - self.vision_pos['yaw'])
            else:
                err_yaw = 0.0

            # PID 计算
            vx = self.pid_x.update(err_x)
            vy = self.pid_y.update(err_y)
            vz = self.pid_z.update(err_z) if self.enable_z_control else 0.0
            wz = self.pid_yaw.update(err_yaw) if self.enable_yaw_control else 0.0

            # ========== 速度死区处理 ==========
            # 当速度小于死区值时，输出为 0，防止小误差导致的漂移
            if abs(vx) < self.velocity_deadband:
                vx = 0.0
            if abs(vy) < self.velocity_deadband:
                vy = 0.0
            if self.enable_z_control and abs(vz) < self.velocity_deadband:
                vz = 0.0
            # 角速度死区处理
            if self.enable_yaw_control and abs(wz) < self.angular_velocity_deadband:
                wz = 0.0

            # 检查是否对齐
            xy_aligned = abs(err_x) < self.align_threshold_xy and abs(err_y) < self.align_threshold_xy
            yaw_aligned = abs(err_yaw) < self.align_threshold_yaw if self.enable_yaw_control else True
            
            if xy_aligned and yaw_aligned:
                self.is_aligned = True
                vx, vy, wz = 0.0, 0.0, 0.0  # 对齐后停止
            else:
                self.is_aligned = False

            self.latest_log = (
                f'Vision: X={self.vision_pos["x"]:.3f} Y={self.vision_pos["y"]:.3f} Yaw={math.degrees(self.vision_pos["yaw"]):.1f}° | '
                f'Error: X={err_x:.3f} Y={err_y:.3f} YawErr={math.degrees(err_yaw):.1f}° | '
                f'Cmd: Vx={vx:.2f} Vy={vy:.2f} Wz={math.degrees(wz):.1f}°/s'
            )
        else:
            # 没有视觉数据，保持悬停
            vx, vy, vz, wz = 0.0, 0.0, 0.0, 0.0
            self.latest_log = 'No ArUco detected, hovering...'

        # ========== 发布速度指令 ==========
        self.publish_velocity(vx, vy, vz, wz)

    def publish_velocity(self, vx, vy, vz, wz):
        """
        发布速度指令到 MAVROS

        坐标系说明：
        - vision_pose 基于 ArUco 检测，使用 ENU 坐标系
        - MAVROS /mavros/setpoint_velocity/cmd_vel 使用 NED 坐标系
        - 需要对 X 轴速度取反
        - yaw 角速度：ENU 和 NED 都是绕 Z 轴旋转，方向相同（不需要取反）
        """
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # ========== 坐标系变换：ENU → NED ==========
        msg.twist.linear.x = -vx  # X 取反
        msg.twist.linear.y = vy   # Y 不变
        msg.twist.linear.z = vz   # Z 不变

        # 角速度：绕 Z 轴旋转，ENU 和 NED 方向相同
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = wz  # yaw 角速度不取反

        self.vel_pub.publish(msg)

    def log_callback(self):
        """2Hz 状态日志"""
        if self.latest_log:
            self.get_logger().info(self.latest_log)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
