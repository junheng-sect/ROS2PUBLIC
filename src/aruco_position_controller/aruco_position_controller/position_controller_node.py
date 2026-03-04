#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ArUco 位置控制节点

功能：
- 融合 ArUco 视觉位姿信息
- 通过 PID 控制实现无人机水平位置自动对齐
- 目标：使 vision_pose 与 map 坐标系 XY 平面对齐（即 ArUco 标记在 XY=0 位置）

TF Tree:
    map ├── base_link (MAVROS)
        └── vision_pose (ArUco 视觉估计)

控制逻辑：
- 查询 TF (map -> vision_pose) 获取视觉位姿
- 目标位置：map 坐标系原点 (0, 0)
- 通过 PID 计算速度指令，发布到 /mavros/setpoint_velocity/cmd_vel
- 进入 Offboard 模式后自动启用对齐控制
- 未检测到 ArUco 时保持悬停
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# 消息类型导入
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from std_msgs.msg import Header
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration

import time


class PIDController:
    """
    简单的 PID 控制器类，用于计算位置误差对应的速度指令
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


class ArucoPositionControlNode(Node):
    def __init__(self):
        super().__init__('aruco_position_controller')

        # ========== 参数配置 ==========
        # PID 参数（水平方向）
        self.declare_parameter('kp_xy', 1.5)
        self.declare_parameter('ki_xy', 0.02)
        self.declare_parameter('kd_xy', 0.1)
        
        # PID 参数（Z 轴，通常更敏感）
        self.declare_parameter('kp_z', 1.2)
        self.declare_parameter('ki_z', 0.01)
        self.declare_parameter('kd_z', 0.05)
        
        # 控制频率
        self.declare_parameter('control_frequency', 50.0)  # Hz
        
        # 目标位置（相对于 map 坐标系）
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z_offset', 0.0)  # Z 轴偏移
        
        # 对齐阈值
        self.declare_parameter('align_threshold_xy', 0.1)  # 米
        self.declare_parameter('align_threshold_z', 0.05)  # 米
        
        # 是否启用 Z 轴控制
        self.declare_parameter('enable_z_control', False)
        
        # 是否启用视觉对齐（否则使用位置控制）
        self.declare_parameter('use_vision_alignment', True)
        # TF 坐标系参数（视觉位姿）
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('vision_frame', 'vision_pose')
        self.declare_parameter('tf_lookup_timeout', 0.05)
        
        # 进入 Offboard 后是否自动启用控制
        self.declare_parameter('auto_enable_offboard', True)
        
        # 速度死区（m/s）- 小于此值的速度输出为 0，防止漂移
        self.declare_parameter('velocity_deadband', 0.05)  # 5cm/s

        # 获取参数
        kp_xy = self.get_parameter('kp_xy').value
        ki_xy = self.get_parameter('ki_xy').value
        kd_xy = self.get_parameter('kd_xy').value
        kp_z = self.get_parameter('kp_z').value
        ki_z = self.get_parameter('ki_z').value
        kd_z = self.get_parameter('kd_z').value
        freq = self.get_parameter('control_frequency').value
        
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_z_offset = self.get_parameter('target_z_offset').value
        self.align_threshold_xy = self.get_parameter('align_threshold_xy').value
        self.align_threshold_z = self.get_parameter('align_threshold_z').value
        self.enable_z_control = self.get_parameter('enable_z_control').value
        self.use_vision_alignment = self.get_parameter('use_vision_alignment').value
        self.world_frame = self.get_parameter('world_frame').value
        self.vision_frame = self.get_parameter('vision_frame').value
        self.tf_lookup_timeout = self.get_parameter('tf_lookup_timeout').value
        self.auto_enable_offboard = self.get_parameter('auto_enable_offboard').value
        self.velocity_deadband = self.get_parameter('velocity_deadband').value
        
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

        # ========== 状态变量 ==========
        # vision_pose 位置（来自 ArUco 检测）
        self.vision_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        # 目标位置
        self.target_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # 状态标志
        self.drone_state = State()
        self.has_vision_pos = False  # 是否有 ArUco 视觉数据（经过超时判断）
        self.is_aligned = False  # 是否已对齐
        self.control_enabled = False  # 是否启用控制
        self.prev_offboard_mode = False  # 记录上一次是否为 Offboard 模式
        
        # 视觉 TF 时间戳（用于超时判断）
        self.last_vision_time = None  # 最后一次成功查询 map->vision_pose 的时间
        
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
        # TF 监听器：读取 map->vision_pose（与 aruco_tf_vision 结果保持一致）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        self.get_logger().info('ArUco Position Control Node Initialized')
        self.get_logger().info(f'  - KP(xy)={kp_xy}, KI(xy)={ki_xy}, KD(xy)={kd_xy}')
        self.get_logger().info(f'  - KP(z)={kp_z}, KI(z)={ki_z}, KD(z)={kd_z}')
        self.get_logger().info(f'  - Target: [{self.target_x}, {self.target_y}, {self.target_z_offset}]')
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
                self.get_logger().info(f'Offboard detected, will hover for {self.initial_hover_time}s before alignment')
        
        self.prev_offboard_mode = (msg.mode == "OFFBOARD")

    def update_vision_from_tf(self):
        """
        从 TF 查询 map->vision_pose，并更新 vision_pos。
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.vision_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

        self.vision_pos['x'] = float(transform.transform.translation.x)
        self.vision_pos['y'] = float(transform.transform.translation.y)
        self.vision_pos['z'] = float(transform.transform.translation.z)
        self.last_vision_time = self.get_clock().now()
        return True

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
            response.success = True
            response.message = "PID reset"
            self.get_logger().info('PID Reset')
        else:
            response.success = True
            response.message = "PID not reset"
        return response

    def timer_callback(self):
        """
        主控制循环（50Hz）
        """
        # ========== 安全检查 ==========
        if not self.drone_state.connected:
            self.publish_velocity(0.0, 0.0, 0.0)
            return

        if self.drone_state.mode != "OFFBOARD":
            # 非 Offboard 模式，发布零速度保持心跳
            self.publish_velocity(0.0, 0.0, 0.0)
            return

        if not self.control_enabled:
            # 控制未启用，发布零速度
            self.publish_velocity(0.0, 0.0, 0.0)
            return

        # ========== 初始悬停检查 ==========
        if self.offboard_enter_time is not None:
            elapsed = (self.get_clock().now() - self.offboard_enter_time).nanoseconds / 1e9
            if elapsed < self.initial_hover_time:
                # 初始悬停阶段，直接悬停
                self.publish_velocity(0.0, 0.0, 0.0)
                if int(elapsed * 10) % 50 == 0:  # 每 0.1 秒打印一次
                    self.get_logger().info(f'Initial hover: {self.initial_hover_time - elapsed:.1f}s remaining...')
                return
            else:
                # 悬停结束，重置 PID（清除悬停期间的积分累积）
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_z.reset()
                self.offboard_enter_time = None
                self.get_logger().info('Initial hover done, starting alignment control')

        # ========== 检查视觉 TF 状态（超时判断） ==========
        self.update_vision_from_tf()
        current_time = self.get_clock().now()
        if self.last_vision_time is not None:
            elapsed = (current_time - self.last_vision_time).nanoseconds / 1e9
            if elapsed > self.aruco_timeout:
                # 超过超时时间未查询到有效 vision_pose
                self.has_vision_pos = False
            else:
                # 在超时时间内，认为有有效数据
                self.has_vision_pos = True
        else:
            # 从未查询到有效 vision_pose
            self.has_vision_pos = False

        # ========== 计算控制量 ==========
        vx, vy, vz = 0.0, 0.0, 0.0

        # 使用 vision_pose 进行对齐
        if self.has_vision_pos:
            # 目标：vision_pose 与 map 原点重合 (0, 0)
            err_x = self.target_x - self.vision_pos['x']
            err_y = self.target_y - self.vision_pos['y']
            
            # Z 轴：保持当前高度（不控制）
            if self.enable_z_control:
                err_z = self.target_z_offset - self.vision_pos['z']
            else:
                err_z = 0.0
            
            # PID 计算
            vx = self.pid_x.update(err_x)
            vy = self.pid_y.update(err_y)
            vz = self.pid_z.update(err_z) if self.enable_z_control else 0.0
            
            # ========== 速度死区处理 ==========
            # 当速度小于死区值时，输出为 0，防止小误差导致的漂移
            if abs(vx) < self.velocity_deadband:
                vx = 0.0
            if abs(vy) < self.velocity_deadband:
                vy = 0.0
            if self.enable_z_control and abs(vz) < self.velocity_deadband:
                vz = 0.0
            
            # 检查是否对齐
            if abs(err_x) < self.align_threshold_xy and abs(err_y) < self.align_threshold_xy:
                self.is_aligned = True
                vx, vy = 0.0, 0.0  # 对齐后停止
            else:
                self.is_aligned = False
                
            self.latest_log = (
                f'Vision: X={self.vision_pos["x"]:.3f} Y={self.vision_pos["y"]:.3f} | '
                f'Error: X={err_x:.3f} Y={err_y:.3f} | '
                f'Cmd: Vx={vx:.2f} Vy={vy:.2f}'
            )
        else:
            # 没有视觉数据，保持悬停
            vx, vy, vz = 0.0, 0.0, 0.0
            self.latest_log = (
                f'No vision TF ({self.world_frame}->{self.vision_frame}), hovering...'
            )

        # ========== 发布速度指令 ==========
        self.publish_velocity(vx, vy, vz)

    def publish_velocity(self, vx, vy, vz):
        """
        发布速度指令到 MAVROS
        
        坐标系说明：
        - vision_pose 基于 ArUco 检测，使用 ENU 坐标系
        - MAVROS /mavros/setpoint_velocity/cmd_vel 使用 NED 坐标系
        - 当前实测中 X/Y 控制方向均需取反修正
        """
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # ========== 速度映射（按当前实测方向） ==========
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz   # Z 不变
        
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.vel_pub.publish(msg)

    def log_callback(self):
        """2Hz 状态日志"""
        if self.latest_log:
            self.get_logger().info(self.latest_log)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPositionControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
