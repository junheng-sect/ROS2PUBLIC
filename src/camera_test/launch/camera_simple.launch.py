#!/usr/bin/env python3
"""
简化版摄像头 Launch 文件 (修复版)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. 摄像头驱动 (移除 pixel_format 参数)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[{
                'image_size': {'width': 640, 'height': 480},
                'framerate': 30,
                # 删除 'pixel_format': 'MJPG',  <-- 这行导致问题
                'camera_frame_id': 'camera_link',
            }],
        ),

        # 2. Camera Node
        Node(
            package='camera_test',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_topic': '/image_raw',
                'use_compressed': True,
                'output_image': True,
            }],
        ),

        # 3. RQT Image View
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
        ),
    ])