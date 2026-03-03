#!/usr/bin/env python3
"""
ArUco 系统 Launch 文件 (Mid Version)
- 修复：将 usb_cam 改为可选启动，避免未安装时启动失败
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    start_usb_cam = LaunchConfiguration('start_usb_cam')
    start_rqt_image_view = LaunchConfiguration('start_rqt_image_view')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_usb_cam',
            default_value='false',
            description='是否启动 usb_cam 相机节点（未安装 usb_cam 时请保持 false）'
        ),
        DeclareLaunchArgument(
            'start_rqt_image_view',
            default_value='false',
            description='是否启动 rqt_image_view 图像窗口'
        ),
        # 1. 摄像头驱动
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            condition=IfCondition(start_usb_cam),
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
            package='aruco_detector_mid',
            executable='aruco_node',
            name='aruco_detector_mid',
            output='screen',
            parameters=[{
                'image_topic': '/image_raw',
                'use_compressed': False,
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
            condition=IfCondition(start_rqt_image_view),
            output='screen',
            arguments=['--force-discover'],
            remappings=[
                ('/image_raw', '/aruco/image_annotated'),
            ],
        ),
    ])
