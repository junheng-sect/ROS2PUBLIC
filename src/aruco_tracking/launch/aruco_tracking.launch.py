#!/usr/bin/env python3
"""
ArUco 跟踪控制系统 Launch 文件

功能整合：
- Gazebo 仿真图像桥接
- ArUco 检测与 TF 广播
- 位置控制（基于 ArUco 视觉对齐）
- Yaw 角度控制（使 yaw 趋向于 0）

系统架构：
1. Gazebo → 图像 → ArUco 检测 → vision_pose
2. MAVROS → 位置 → 跟踪控制器
3. 跟踪控制器 → 速度指令 → MAVROS → PX4

TF Tree:
    map ├── base_link (MAVROS)
        └── vision_pose (ArUco 视觉估计)

控制逻辑：
- 目标：vision_pose 与 map 坐标系 XY 平面对齐，yaw=0
- 通过 PID 控制水平位置（X, Y）
- 通过 PID 控制 yaw 角度
- Z 轴可选控制或保持当前高度
"""
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # ========== 配置区域 ==========
    # Gazebo 图像话题
    gz_image_topic = '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
    ros_image_topic = '/camera/image_raw'

    # 相机内参（根据 SDF 文件：640x480, hfov=1.74 rad）
    camera_fx = 268.5
    camera_fy = 268.5
    camera_cx = 320.0
    camera_cy = 240.0

    # 获取包路径
    pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # ========== 节点配置 ==========

    # 1. ROS-GZ 桥接
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_image',
        output='screen',
        arguments=[
            f'{gz_image_topic}@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        remappings=[
            (gz_image_topic, ros_image_topic)
        ],
    )

    # 2. ArUco 检测节点
    aruco_detector_node = Node(
        package='aruco_tf_vision',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[{
            'image_topic': ros_image_topic,
            'use_compressed': False,
            'publish_annotated_image': True,
            'camera_fx': camera_fx,
            'camera_fy': camera_fy,
            'camera_cx': camera_cx,
            'camera_cy': camera_cy,
            'marker_size_33': 0.5,
            'marker_size_42': 0.063,
            'switch_distance': 0.5,
            'switch_hysteresis': 0.05,
            'world_frame': 'map',
            'vision_frame': 'vision_pose',
            'camera_frame': 'camera_link',
            'vision_offset_x': 0.0,
            'vision_offset_y': 0.0,
            'vision_offset_z': -0.167,  # 消除初始高度偏移
        }],
    )

    # 3. TF 广播节点（MAVROS → base_link）
    tf_broadcaster_node = Node(
        package='aruco_tf_vision',
        executable='tf_broadcaster_node',
        name='tf_broadcaster_node',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'child_frame_id': 'base_link',
            'publish_rate': 50.0,
        }],
    )

    # 4. ArUco 跟踪控制器（新增 yaw 控制）
    tracking_node = Node(
        package='aruco_tracking',
        executable='tracking_node',
        name='aruco_tracking',
        output='screen',
        parameters=[{
            # PID 参数（水平方向）
            'kp_xy': 0.5,
            'ki_xy': 0.0,   # 禁用积分，防止漂移
            'kd_xy': 0.1,
            # PID 参数（Z 轴）
            'kp_z': 1.2,
            'ki_z': 0.01,
            'kd_z': 0.05,
            # PID 参数（Yaw 轴）- 新增
            'kp_yaw': 1.0,
            'ki_yaw': 0.0,
            'kd_yaw': 0.05,
            # 控制频率
            'control_frequency': 50.0,
            # 目标位置（map 原点）
            'target_x': 0.0,
            'target_y': 0.0,
            'target_z_offset': 0.0,
            # 目标 yaw 角度（0 = 机头朝北）- 新增
            'target_yaw': 0.0,
            # 对齐阈值
            'align_threshold_xy': 0.1,  # 10cm
            'align_threshold_z': 0.05,  # 5cm
            'align_threshold_yaw': 0.1,  # ~5.7 度
            # 控制选项
            'enable_z_control': False,  # 默认不控制 Z 轴
            'enable_yaw_control': True,  # 启用 yaw 控制
            'use_vision_alignment': True,  # 使用视觉对齐
            'auto_enable_offboard': True,  # 进入 Offboard 自动启用
        }],
    )

    # 5. RViz2 可视化
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'aruco_tracking.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_aruco_tracking',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        gz_bridge_node,
        aruco_detector_node,
        tf_broadcaster_node,
        tracking_node,
        rviz_node,
    ])
