#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# 消息类型导入
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Header
# 导入自定义服务接口
from px4_interfaces.srv import SetTargetPosition

import math
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
        if dt <= 0: dt = 0.01 # 防止除零

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

class PositionControlNode(Node):
    def __init__(self):
        super().__init__('px4_position_controller')

        # --- 参数配置 ---
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('ki', 0.05)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('control_frequency', 50.0) # Hz
        
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        freq = self.get_parameter('control_frequency').value

        # 初始化三个轴的 PID 控制器
        self.pid_x = PIDController(kp=kp, ki=ki, kd=kd, limit=3.0) # 限幅 3m/s
        self.pid_y = PIDController(kp=kp, ki=ki, kd=kd, limit=3.0)
        self.pid_z = PIDController(kp=kp*0.8, ki=ki, kd=kd, limit=2.0) # Z 轴通常更敏感

        # --- 状态变量 ---
        self.current_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.target_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.is_target_set = False
        self.drone_state = State()
        self.has_local_pos = False

        # --- QoS 配置 (用于本地位置订阅，保证最新数据) ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- 订阅者 ---
        # 订阅无人机状态 (连接、解锁、模式)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        # 订阅本地位置 (ENU 坐标系)
        self.pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pos_callback, qos_profile)

        # --- 发布者 ---
        # 发布速度控制指令 (Offboard 模式核心)
        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # --- 服务端 ---
        # 接收目标位置服务
        self.service = self.create_service(SetTargetPosition, 'set_target_position', self.set_target_callback)

        # --- 定时器 ---
        # 50Hz 控制循环，同时满足 Offboard 心跳要求 (>2Hz)
        self.timer = self.create_timer(1.0/freq, self.timer_callback)

        self.get_logger().info('Position Control Node Initialized. Waiting for FCU connection...')

    def state_callback(self, msg):
        self.drone_state = msg
        # 简单检查连接状态
        if msg.connected and not self.has_local_pos:
            self.get_logger().warn('FCU Connected, but waiting for Local Position...')

    def pos_callback(self, msg):
        self.current_pos['x'] = msg.pose.position.x
        self.current_pos['y'] = msg.pose.position.y
        self.current_pos['z'] = msg.pose.position.z
        self.has_local_pos = True

    def set_target_callback(self, request, response):
        """
        服务回调：更新目标位置并重置 PID 积分
        """
        self.target_pos['x'] = request.x
        self.target_pos['y'] = request.y
        self.target_pos['z'] = request.z
        self.is_target_set = True
        
        # 重置 PID，防止切换目标时的积分累积导致超调
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()

        response.success = True
        response.message = f"Target set to [{request.x}, {request.y}, {request.z}]"
        self.get_logger().info(f"Target Updated: {response.message}")
        return response

    def timer_callback(self):
        """
        主控制循环
        1. 检查安全状态
        2. 计算 PID
        3. 发布速度指令
        """
        # 1. 安全检查
        if not self.drone_state.connected:
            self.get_logger().warn('FCU not connected!')
            return
        
        # 注意：虽然用户通过遥控器进入 Offboard，但代码最好确认一下模式
        # MAVROS State 中的 mode 字符串可能因 PX4 版本不同而异，通常是 "OFFBOARD"
        if self.drone_state.mode != "OFFBOARD":
            # 即使不是 Offboard，为了保持心跳防止报错，我们仍然发布零速度
            # 但不会进行 PID 计算，避免地面误动作
            self.publish_velocity(0.0, 0.0, 0.0)
            return

        if not self.has_local_pos:
            self.get_logger().warn('Waiting for Local Position...')
            return

        # 2. 计算控制量
        vx, vy, vz = 0.0, 0.0, 0.0

        if self.is_target_set:
            # 计算误差
            err_x = self.target_pos['x'] - self.current_pos['x']
            err_y = self.target_pos['y'] - self.current_pos['y']
            err_z = self.target_pos['z'] - self.current_pos['z']

            # PID 计算速度指令
            vx = self.pid_x.update(err_x)
            vy = self.pid_y.update(err_y)
            vz = self.pid_z.update(err_z)

            # 判断是否到达目标 (简单阈值)
            if abs(err_x) < 0.1 and abs(err_y) < 0.1 and abs(err_z) < 0.1:
                vx, vy, vz = 0.0, 0.0, 0.0 # 到达后停止输出
        else:
            # 未设置目标前，保持悬停
            vx, vy, vz = 0.0, 0.0, 0.0

        # 3. 发布指令
        self.publish_velocity(vx, vy, vz)

    def publish_velocity(self, vx, vy, vz):
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map" # 或者 "local_origin"
        
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        # 角速度通常设为 0，由遥控器或上层航向控制决定
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PositionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()