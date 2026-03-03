#!/usr/bin/env python3
"""
完整 ArUco 系统 Launch 文件
一键启动：摄像头 + ArUco 检测 + 图像显示
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ========== 1. 摄像头驱动 ==========
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[{
                'image_size': {'width': 640, 'height': 480},
                'framerate': 30,
                'camera_frame_id': 'camera_link',
            }],
        ),

        # ========== 2. Camera Node（解码 + 发布）==========
        Node(
            package='camera_test',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_topic': '/image_raw',
                'use_compressed': True,
                'output_image': True,
                'log_fps': True,
            }],
        ),

        # ========== 3. ArUco 检测节点 ==========
        Node(
            package='aruco_detector',
            executable='aruco_node',
            name='aruco_detector',
            output='screen',
            parameters=[{
                'image_topic': '/camera/output',
                'publish_annotated_image': True,
            }],
        ),

        # ========== 4. RQT Image View（显示带标记的画面）==========
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
        ),
    ])