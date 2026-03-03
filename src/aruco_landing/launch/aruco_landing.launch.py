#!/usr/bin/env python3
"""
ArUco 自动降落系统 Launch 文件

功能整合：
- Gazebo 仿真图像桥接
- ArUco 检测与 TF 广播
- 位置控制 + Yaw 控制
- 自动降落（对齐后等待 2 秒，以 0.5m/s 速度降落）

系统架构：
1. Gazebo → 图像 → ArUco 检测 → vision_pose
2. MAVROS → 位置 → 降落控制器
3. 降落控制器 → 速度指令 → MAVROS → PX4

TF Tree:
    map ├── base_link (MAVROS)
        └── vision_pose (ArUco 视觉估计)

状态机：
    IDLE → ALIGNING → WAITING(2s) → LANDING → LANDED
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
            'vision_offset_z': -0.167,
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

    # 4. ArUco 自动降落控制器
    landing_node = Node(
        package='aruco_landing',
        executable='landing_node',
        name='aruco_landing',
        output='screen',
        parameters=[{
            # PID 参数（水平方向）
            'kp_xy': 0.5,
            'ki_xy': 0.0,
            'kd_xy': 0.1,
            # PID 参数（Yaw 轴）
            'kp_yaw': 1.0,
            'ki_yaw': 0.0,
            'kd_yaw': 0.05,
            # 控制频率
            'control_frequency': 50.0,
            # 目标位置（map 原点）
            'target_x': 0.0,
            'target_y': 0.0,
            # 目标 yaw 角度（0 = 机头朝北）
            'target_yaw': 0.0,
            # 对齐阈值（控制死区）
            'align_threshold_xy': 0.1,  # 10cm
            'align_threshold_yaw': 0.1,  # ~5.7 度
            # 降落参数
            'wait_time': 2.0,  # 对齐后等待 2 秒
            'landing_speed': 0.5,  # 0.5 m/s 降落速度
            'min_height': 0.2,  # 最低高度 20cm
            # 控制选项
            'enable_yaw_control': True,
            'enable_auto_landing': True,
        }],
    )

    # 5. RViz2 可视化
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'aruco_landing.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_aruco_landing',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        gz_bridge_node,
        aruco_detector_node,
        tf_broadcaster_node,
        landing_node,
        rviz_node,
    ])
