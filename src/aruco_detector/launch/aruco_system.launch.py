#!/usr/bin/env python3
"""
优化版 ArUco 系统 Launch 文件
- 使用 usb_cam 驱动（修复参数类型）
- 直接订阅压缩话题
- 降低分辨率提高性能
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ========== 1. 摄像头驱动（使用 usb_cam）==========
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 320,
                'image_height': 240,
                'pixel_format': 'yuyv2rgb',
                'framerate': 30.0,  # ✅ 改为浮点数 30.0
                'camera_frame_id': 'camera_link',
                'io_method': 'mmap',
            }],
        ),

        # ========== 2. ArUco 检测节点 ==========
        Node(
            package='aruco_detector',
            executable='aruco_node',
            name='aruco_detector',
            output='screen',
            parameters=[{
                'image_topic': '/image_raw',
                'use_compressed': True,
                'publish_annotated_image': True,
            }],
        ),

        # ========== 3. RQT Image View（指定话题）==========
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
            arguments=['--force-discover'],
            remappings=[
                ('/image_raw', '/aruco/image_annotated'),  # ✅ 重映射到带标记的图像
            ],
        ),
    ])