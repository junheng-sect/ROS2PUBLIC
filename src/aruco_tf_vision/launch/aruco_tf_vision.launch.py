#!/usr/bin/env python3
"""
ArUco TF Vision 系统 Launch 文件

功能整合：
- Gazebo 仿真图像桥接
- ArUco 检测与标注图像显示
- TF 广播：map → vision_pose（基于 ArUco）
- TF 广播：map → base_link（基于 MAVROS）
- RViz 可视化：显示相机图像、TF 树（map、base_link、vision_pose）

TF Tree:
    map ├── base_link (MAVROS)
        └── vision_pose (ArUco 视觉估计)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import os


def generate_launch_description():
    # ========== 配置区域 ==========
    # Gazebo 图像话题（根据仿真环境配置）
    gz_image_topic = '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
    ros_image_topic = '/camera/image_raw'

    # 相机内参（根据 SDF 文件：640x480, hfov=1.74 rad）
    # fx = fy = 640 / (2 * tan(1.74/2)) ≈ 268.5
    # cx = 320.0, cy = 240.0
    camera_fx = 268.5
    camera_fy = 268.5
    camera_cx = 320.0
    camera_cy = 240.0

    # 获取包路径
    pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'aruco_tf_vision.rviz')

    # MAVROS 启动文件路径
    mavros_launch_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(pkg_path))),
        'tf_display', 'launch', 'tf_display.launch.py'
    )
    # ================================================================

    return LaunchDescription([
        # 1. ROS-GZ 桥接（将 Gazebo 图像话题桥接到 ROS 2）
        Node(
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
        ),

        # 2. ArUco 检测节点
        # - 发布标注图像到 /aruco/image_annotated
        # - 发布 TF: map → vision_pose
        Node(
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
                'marker_size_33': 0.135,
                'marker_size_42': 0.017,
                'switch_distance': 0.5,
                'switch_hysteresis': 0.05,
                'world_frame': 'map',
                'vision_frame': 'vision_pose',
                'camera_frame': 'camera_link',
                'vision_offset_x': 0.0,
                'vision_offset_y': 0.0,
                'vision_offset_z': -0.167,  # 消除初始高度偏移
            }],
        ),

        # 3. TF 广播节点（MAVROS → base_link）
        # 发布 TF: map → base_link
        Node(
            package='aruco_tf_vision',
            executable='tf_broadcaster_node',
            name='tf_broadcaster_node',
            output='screen',
            parameters=[{
                'frame_id': 'map',
                'child_frame_id': 'base_link',
                'publish_rate': 50.0,
            }],
        ),

        # 4. RViz2 可视化
        # - 显示相机图像（/aruco/image_annotated）
        # - 显示 TF 树（map、base_link、vision_pose）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_aruco_tf',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
    ])
