#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ArUco 自动降落控制节点

功能：
- 融合 ArUco 视觉位姿信息
- 通过 PID 控制实现无人机水平位置自动对齐
- 通过 PID 控制实现无人机 yaw 角度对齐（使 yaw 趋向于 0）
- 当 XY 和 yaw 都进入控制死区后，等待 2 秒，随后以 0.5m/s 的速度降落
- 降落后等待 3 秒，自动调用 Disarm 服务

TF Tree:
    map ├── base_link (MAVROS)
        └── vision_pose (ArUco 视觉估计)

控制逻辑：
- 阶段 1：对齐阶段 - vision_pose 与 map 坐标系 XY 平面对齐，yaw=0
- 阶段 2：等待阶段 - 对齐完成后等待 2 秒
- 阶段 3：降落阶段 - 以 0.5m/s 的速度下降
- 阶段 4：已降落 - 等待 3 秒
- 阶段 5：Disarm - 调用 Disarm 服务
- 进入 Offboard 模式后自动启用控制
- 未检测到 ArUco 时保持悬停

状态机：
    IDLE → ALIGNING → WAITING → LANDING → LANDED → WAITING_DISARM → DISARMING
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum

# 消息类型导入
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from std_msgs.msg import Header
from aruco_interfaces.msg import ArucoPose
from std_srvs.srv import SetBool
from px4_interfaces.srv import Disarm

import time
import math


class LandingState(Enum):
    """降落状态机"""
    IDLE = 0          # 空闲
    ALIGNING = 1      # 对齐中
    WAITING = 2       # 等待中（对齐完成后等待 2 秒）
    LANDING = 3       # 降落中
    LANDED = 4        # 已降落
    WAITING_DISARM = 5  # 等待 disarm（降落后等待 3 秒）
    DISARMING = 6     # Disarm 中


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


class ArucoLandingNode(Node):
    def __init__(self):
        super().__init__('aruco_landing')

        # ========== 参数配置 ==========
        # PID 参数（水平方向）
        self.declare_parameter('kp_xy', 0.5)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.1)

        # PID 参数（Z 轴，通常更敏感）
        self.declare_parameter('kp_z', 1.2)
        self.declare_parameter('ki_z', 0.01)
        self.declare_parameter('kd_z', 0.05)

        # PID 参数（Yaw 轴）
        self.declare_parameter('kp_yaw', 1.0)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.05)

        # 控制频率
        self.declare_parameter('control_frequency', 50.0)  # Hz

        # 目标位置（相对于 map 坐标系）
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)

        # 目标 yaw 角度（弧度）
        self.declare_parameter('target_yaw', 0.0)  # 0 弧度 = 机头朝北

        # 对齐阈值（控制死区）
        self.declare_parameter('align_threshold_xy', 0.1)  # 米 (10cm)
        self.declare_parameter('align_threshold_yaw', 0.1)  # 弧度 (~5.7 度)

        # 等待时间（对齐完成后等待多久开始降落）
        self.declare_parameter('wait_time', 2.0)  # 秒

        # 降落速度
        self.declare_parameter('landing_speed', 0.5)  # m/s

        # 降落后等待 disarm 时间
        self.declare_parameter('wait_disarm_time', 3.0)  # 秒

        # 是否启用 yaw 控制
        self.declare_parameter('enable_yaw_control', True)

        # 是否启用自动降落
        self.declare_parameter('enable_auto_landing', True)

        # 是否启用自动 disarm
        self.declare_parameter('enable_auto_disarm', True)

        # 速度死区（m/s）
        self.declare_parameter('velocity_deadband', 0.05)  # 5cm/s

        # 角速度死区（rad/s）
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
        self.target_yaw = self.get_parameter('target_yaw').value
        self.align_threshold_xy = self.get_parameter('align_threshold_xy').value
        self.align_threshold_yaw = self.get_parameter('align_threshold_yaw').value
        self.wait_time = self.get_parameter('wait_time').value
        self.landing_speed = self.get_parameter('landing_speed').value
        self.wait_disarm_time = self.get_parameter('wait_disarm_time').value
        self.enable_yaw_control = self.get_parameter('enable_yaw_control').value
        self.enable_auto_landing = self.get_parameter('enable_auto_landing').value
        self.enable_auto_disarm = self.get_parameter('enable_auto_disarm').value
        self.velocity_deadband = self.get_parameter('velocity_deadband').value
        self.angular_velocity_deadband = self.get_parameter('angular_velocity_deadband').value

        # ArUco 检测超时时间（秒）
        self.declare_parameter('aruco_timeout', 0.5)
        self.aruco_timeout = self.get_parameter('aruco_timeout').value

        # 初始悬停时间（秒）
        self.declare_parameter('initial_hover_time', 2.0)
        self.initial_hover_time = self.get_parameter('initial_hover_time').value

        # 最低高度（降落停止高度）
        self.declare_parameter('min_height', 0.2)  # 米，离地 20cm 停止
        self.min_height = self.get_parameter('min_height').value

        # ========== 初始化 PID 控制器 ==========
        self.pid_x = PIDController(kp=kp_xy, ki=ki_xy, kd=kd_xy, limit=1.0)
        self.pid_y = PIDController(kp=kp_xy, ki=ki_xy, kd=kd_xy, limit=1.0)
        self.pid_z = PIDController(kp=kp_z, ki=ki_z, kd=kd_z, limit=1.0)
        self.pid_yaw = PIDController(kp=kp_yaw, ki=ki_yaw, kd=kd_yaw, limit=1.0)

        # ========== 状态变量 ==========
        # vision_pose 位置（来自 ArUco 检测）
        self.vision_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}

        # 状态标志
        self.drone_state = State()
        self.has_vision_pos = False
        self.control_enabled = False
        self.prev_offboard_mode = False

        # ArUco 检测时间戳
        self.last_aruco_time = None

        # 进入 Offboard 的时间
        self.offboard_enter_time = None

        # ========== 降落状态机 ==========
        self.landing_state = LandingState.IDLE
        self.wait_start_time = None  # 等待阶段开始时间
        self.landing_start_z = None  # 降落开始时的高度
        self.landed_time = None  # 降落完成时间
        self.disarm_sent = False  # 是否已发送 disarm 命令

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
        # ArUco 位姿
        self.vision_sub = self.create_subscription(
            ArucoPose, '/aruco_pose', self.vision_callback, qos_profile
        )

        # ========== 发布者 ==========
        # 速度控制指令
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10
        )

        # ========== 客户端 ==========
        # Disarm 服务客户端
        self.disarm_client = self.create_client(
            Disarm, '/disarm'
        )
        while not self.disarm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /disarm service...')

        # ========== 服务端 ==========
        # 启用/禁用控制
        self.enable_service = self.create_service(
            SetBool, 'enable_control', self.enable_callback
        )
        # 重置 PID
        self.reset_service = self.create_service(
            SetBool, 'reset_pid', self.reset_callback
        )
        # 强制开始降落
        self.start_landing_service = self.create_service(
            SetBool, 'start_landing', self.start_landing_callback
        )
        # 中止降落/悬停
        self.abort_landing_service = self.create_service(
            SetBool, 'abort_landing', self.abort_landing_callback
        )

        # ========== 定时器 ==========
        # 控制循环（50Hz）
        self.timer = self.create_timer(1.0/freq, self.timer_callback)

        # 状态日志（2Hz）
        self.log_timer = self.create_timer(0.5, self.log_callback)
        self.latest_log = ""

        # ========== 日志 ==========
        self.get_logger().info('ArUco Landing Node Initialized')
        self.get_logger().info(f'  - KP(xy)={kp_xy}, KI(xy)={ki_xy}, KD(xy)={kd_xy}')
        self.get_logger().info(f'  - KP(yaw)={kp_yaw}, KI(yaw)={ki_yaw}, KD(yaw)={kd_yaw}')
        self.get_logger().info(f'  - Target: [{self.target_x}, {self.target_y}], Yaw={self.target_yaw:.2f}rad')
        self.get_logger().info(f'  - Wait Time: {self.wait_time}s, Landing Speed: {self.landing_speed}m/s')
        self.get_logger().info(f'  - Wait Disarm Time: {self.wait_disarm_time}s')
        self.get_logger().info(f'  - Align Threshold: XY={self.align_threshold_xy}m, Yaw={math.degrees(self.align_threshold_yaw):.1f}°')
        self.get_logger().info('Waiting for FCU connection...')

    def state_callback(self, msg):
        """无人机状态回调"""
        self.drone_state = msg
        # Offboard 检测已移到 timer_callback 中处理

    def vision_callback(self, msg: ArucoPose):
        """接收 ArUco 检测的 vision_pose 位置"""
        self.last_aruco_time = self.get_clock().now()
        self.vision_pos['x'] = msg.x
        self.vision_pos['y'] = msg.y
        self.vision_pos['z'] = msg.z
        self.vision_pos['yaw'] = msg.yaw

    def enable_callback(self, request, response):
        """启用/禁用控制"""
        self.control_enabled = request.data
        if request.data:
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.pid_yaw.reset()
            self.landing_state = LandingState.ALIGNING
            self.wait_start_time = None
            response.success = True
            response.message = "Control enabled"
            self.get_logger().info('Control ENABLED')
        else:
            response.success = True
            response.message = "Control disabled"
            self.get_logger().info('Control DISABLED')
        return response

    def reset_callback(self, request, response):
        """重置 PID 积分"""
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

    def start_landing_callback(self, request, response):
        """强制开始降落"""
        if request.data:
            if self.has_vision_pos:
                self.landing_state = LandingState.LANDING
                self.landing_start_z = self.vision_pos['z']
                response.success = True
                response.message = f"Landing started from Z={self.vision_pos['z']:.2f}m"
                self.get_logger().info(f'Forced landing started from Z={self.vision_pos["z"]:.2f}m')
            else:
                response.success = False
                response.message = "No vision position available"
                self.get_logger().warn('Cannot start landing: no vision position')
        else:
            response.success = True
            response.message = "Landing not started"
        return response

    def abort_landing_callback(self, request, response):
        """中止降落/悬停"""
        if request.data:
            self.landing_state = LandingState.ALIGNING
            self.wait_start_time = None
            self.landing_start_z = None
            self.landed_time = None
            self.disarm_sent = False
            response.success = True
            response.message = "Landing aborted, hovering"
            self.get_logger().info('Landing aborted, back to aligning')
        else:
            response.success = True
            response.message = "Landing not aborted"
        return response

    def disarm_callback(self, future):
        """Disarm 命令回调"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Disarm SUCCESS!')
            else:
                self.get_logger().warn('Disarm FAILED!')
        except Exception as e:
            self.get_logger().error(f'Disarm service call failed: {e}')

    def _normalize_angle(self, angle):
        """将角度归一化到 [-π, π] 范围"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def timer_callback(self):
        """主控制循环（50Hz）"""
        # ========== 始终发送 setpoint（即使未进入 OFFBOARD）==========
        # MAVROS 要求：进入 OFFBOARD 前需要持续发送 setpoint（>2Hz，持续 1 秒）
        if not self.drone_state.connected:
            # 未连接 FCU，发送零速度保持心跳
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)
            return

        # 检查是否进入 Offboard 模式
        if self.drone_state.mode == "OFFBOARD" and not self.prev_offboard_mode:
            self.control_enabled = True
            self.offboard_enter_time = self.get_clock().now()
            # 重置 PID
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.pid_yaw.reset()
            # 重置状态机
            self.landing_state = LandingState.ALIGNING
            self.wait_start_time = None
            self.landing_start_z = None
            self.landed_time = None
            self.disarm_sent = False
            self.get_logger().info(f'Offboard detected, initial hover for {self.initial_hover_time}s')

        self.prev_offboard_mode = (self.drone_state.mode == "OFFBOARD")

        # 非 OFFBOARD 模式时，发送零速度保持心跳（允许切换模式）
        if self.drone_state.mode != "OFFBOARD":
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)
            self.landing_state = LandingState.IDLE
            return

        if not self.control_enabled:
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)
            return

        # ========== 初始悬停检查 ==========
        if self.offboard_enter_time is not None:
            elapsed = (self.get_clock().now() - self.offboard_enter_time).nanoseconds / 1e9
            if elapsed < self.initial_hover_time:
                self.publish_velocity(0.0, 0.0, 0.0, 0.0)
                if int(elapsed * 10) % 50 == 0:
                    self.get_logger().info(f'Initial hover: {self.initial_hover_time - elapsed:.1f}s remaining...')
                return
            else:
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_z.reset()
                self.pid_yaw.reset()
                self.offboard_enter_time = None
                self.landing_state = LandingState.ALIGNING
                self.get_logger().info('Initial hover done, starting alignment')

        # ========== 检查 ArUco 检测状态 ==========
        current_time = self.get_clock().now()
        if self.last_aruco_time is not None:
            elapsed = (current_time - self.last_aruco_time).nanoseconds / 1e9
            self.has_vision_pos = (elapsed <= self.aruco_timeout)
        else:
            self.has_vision_pos = False

        # ========== 状态机处理 ==========
        vx, vy, vz, wz = 0.0, 0.0, 0.0, 0.0

        if not self.has_vision_pos:
            # 没有视觉数据，保持悬停
            self.landing_state = LandingState.ALIGNING
            self.wait_start_time = None
            self.latest_log = 'No ArUco detected, hovering...'
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)
            return

        if self.landing_state == LandingState.ALIGNING:
            # ===== 阶段 1：对齐 =====
            err_x = self.target_x - self.vision_pos['x']
            err_y = self.target_y - self.vision_pos['y']
            err_yaw = self._normalize_angle(self.target_yaw - self.vision_pos['yaw'])

            # PID 计算
            vx = self.pid_x.update(err_x)
            vy = self.pid_y.update(err_y)
            wz = self.pid_yaw.update(err_yaw) if self.enable_yaw_control else 0.0

            # 死区处理
            if abs(vx) < self.velocity_deadband:
                vx = 0.0
            if abs(vy) < self.velocity_deadband:
                vy = 0.0
            if self.enable_yaw_control and abs(wz) < self.angular_velocity_deadband:
                wz = 0.0

            # 检查是否对齐
            xy_aligned = abs(err_x) < self.align_threshold_xy and abs(err_y) < self.align_threshold_xy
            yaw_aligned = abs(err_yaw) < self.align_threshold_yaw if self.enable_yaw_control else True

            if xy_aligned and yaw_aligned:
                # 对齐完成，进入等待阶段
                self.landing_state = LandingState.WAITING
                self.wait_start_time = self.get_clock().now()
                self.get_logger().info(f'Aligned! Waiting {self.wait_time}s before landing...')
                vx, vy, wz = 0.0, 0.0, 0.0
            else:
                self.latest_log = (
                    f'ALIGN: X={err_x:.3f} Y={err_y:.3f} YawErr={math.degrees(err_yaw):.1f}° | '
                    f'Cmd: Vx={vx:.2f} Vy={vy:.2f} Wz={math.degrees(wz):.1f}°/s'
                )

        elif self.landing_state == LandingState.WAITING:
            # ===== 阶段 2：等待 =====
            elapsed = (current_time - self.wait_start_time).nanoseconds / 1e9
            remaining = self.wait_time - elapsed

            if remaining <= 0:
                # 等待完成，开始降落
                self.landing_state = LandingState.LANDING
                self.landing_start_z = self.vision_pos['z']
                self.get_logger().info(f'Waiting done! Starting landing from Z={self.landing_start_z:.2f}m')
                # 重置 Z 轴 PID，防止积分累积
                self.pid_z.reset()
            else:
                # 继续等待，保持悬停
                if int(elapsed * 10) % 25 == 0:  # 每 0.25 秒打印一次
                    self.get_logger().info(f'Waiting for landing: {remaining:.1f}s remaining...')
                self.latest_log = f'WAITING: {remaining:.1f}s remaining...'

        elif self.landing_state == LandingState.LANDING:
            # ===== 阶段 3：降落 =====
            # 检查当前高度
            current_z = self.vision_pos['z']

            if current_z <= self.min_height:
                # 到达最低高度，降落完成
                self.landing_state = LandingState.LANDED
                self.landed_time = self.get_clock().now()
                self.disarm_sent = False
                self.get_logger().info(f'Landed at Z={current_z:.2f}m, waiting {self.wait_disarm_time}s before disarm...')
                self.latest_log = f'LANDED at Z={current_z:.2f}m'
                vx, vy, vz, wz = 0.0, 0.0, 0.0, 0.0
            else:
                # 继续下降，以恒定速度降落
                vz = -self.landing_speed  # 负值表示下降

                # 同时保持 XY 和 yaw 对齐
                err_x = self.target_x - self.vision_pos['x']
                err_y = self.target_y - self.vision_pos['y']
                err_yaw = self._normalize_angle(self.target_yaw - self.vision_pos['yaw'])

                vx = self.pid_x.update(err_x)
                vy = self.pid_y.update(err_y)
                wz = self.pid_yaw.update(err_yaw) if self.enable_yaw_control else 0.0

                # 死区处理
                if abs(vx) < self.velocity_deadband:
                    vx = 0.0
                if abs(vy) < self.velocity_deadband:
                    vy = 0.0
                if self.enable_yaw_control and abs(wz) < self.angular_velocity_deadband:
                    wz = 0.0

                self.latest_log = (
                    f'LANDING: Z={current_z:.2f}m Vz={vz:.2f}m/s | '
                    f'XY: X={err_x:.3f} Y={err_y:.3f} | '
                    f'Cmd: Vx={vx:.2f} Vy={vy:.2f} Vz={vz:.2f}'
                )

        elif self.landing_state == LandingState.LANDED:
            # ===== 阶段 4：已降落，等待 disarm =====
            elapsed = (current_time - self.landed_time).nanoseconds / 1e9
            remaining = self.wait_disarm_time - elapsed

            if remaining <= 0:
                # 等待完成，进入 disarm 状态
                self.landing_state = LandingState.WAITING_DISARM
                self.get_logger().info('Disarm wait done, sending disarm command...')
            else:
                # 继续等待
                if int(elapsed * 2) % 50 == 0:  # 每 0.5 秒打印一次
                    self.get_logger().info(f'Waiting for disarm: {remaining:.1f}s remaining...')
                self.latest_log = f'LANDED: waiting disarm {remaining:.1f}s...'
                vx, vy, vz, wz = 0.0, 0.0, 0.0, 0.0

        elif self.landing_state == LandingState.WAITING_DISARM:
            # ===== 阶段 5：发送 disarm 命令 =====
            if not self.disarm_sent and self.enable_auto_disarm:
                # 发送 disarm 命令
                req = Disarm.Request()
                future = self.disarm_client.call_async(req)
                future.add_done_callback(self.disarm_callback)
                self.disarm_sent = True
                self.landing_state = LandingState.DISARMING
                self.get_logger().info('Disarm command sent!')
            vx, vy, vz, wz = 0.0, 0.0, 0.0, 0.0
            self.latest_log = 'DISARMING...'

        elif self.landing_state == LandingState.DISARMING:
            # ===== 阶段 6：Disarm 完成 =====
            vx, vy, vz, wz = 0.0, 0.0, 0.0, 0.0
            self.latest_log = 'DISARMED'

        # ========== 发布速度指令 ==========
        self.publish_velocity(vx, vy, vz, wz)

    def publish_velocity(self, vx, vy, vz, wz):
        """
        发布速度指令到 MAVROS

        坐标系说明：
        - vision_pose 基于 ArUco 检测，使用 ENU 坐标系
        - MAVROS /mavros/setpoint_velocity/cmd_vel 使用 NED 坐标系
        - X 轴速度取反，yaw 角速度不取反
        """
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # ========== 坐标系变换：ENU → NED ==========
        msg.twist.linear.x = -vx  # X 取反
        msg.twist.linear.y = vy   # Y 不变
        msg.twist.linear.z = vz   # Z 不变（下降速度为负）

        # 角速度
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = wz  # yaw 角速度不取反

        self.vel_pub.publish(msg)

    def log_callback(self):
        """2Hz 状态日志"""
        if self.latest_log:
            state_str = self.landing_state.name
            self.get_logger().info(f'[{state_str}] {self.latest_log}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
