#!/usr/bin/env python3
"""
ArUco 系统 Launch 文件 (Mid Version)
- 指向新的 aruco_csv 包
- 相机内参主点默认已修正为中心
- 【新增】CSV 日志记录配置
"""
from launch import LaunchDescription
from launch_ros.actions import Node
import os

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
            package='aruco_csv',
            executable='aruco_node',
            name='aruco_csv',
            output='screen',
            parameters=[{
                'image_topic': '/image_raw',
                'use_compressed': False,
                'publish_annotated_image': True,
                'camera_cx': 160.0,
                'camera_cy': 120.0,
                # 【新增】CSV 日志配置
                'enable_csv_log': True,  # 设置为 False 可关闭日志
                'csv_log_path': os.path.expanduser('~/project/project_ws/src/aruco_csv/aruco_log/aruco_detection_log.csv'), # 可修改为绝对路径
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