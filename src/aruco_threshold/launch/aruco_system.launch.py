#!/usr/bin/env python3
"""
ArUco 系统 Launch 文件 (Mid Version)
- 指向新的 aruco_threshold 包
- 相机内参主点默认已修正为中心
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 摄像头驱动
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
                'framerate': 30.0,
                'camera_frame_id': 'camera_link',
                'io_method': 'mmap',
            }],
        ),
        # 2. ArUco 检测节点 (新版)
        Node(
            package='aruco_threshold',
            executable='aruco_node',
            name='aruco_threshold',
            output='screen',
            parameters=[{
                'image_topic': '/image_raw',
                'use_compressed': True,
                'publish_annotated_image': True,
                'camera_cx': 160.0,
                'camera_cy': 120.0,
            }],
        ),
        # 3. 图像显示
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
            arguments=['--force-discover'],
            remappings=[
                ('/image_raw', '/aruco/image_annotated'),
            ],
        ),
    ])