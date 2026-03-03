#!/usr/bin/env python3
"""
摄像头标定 Launch 文件
启动摄像头 + 标定节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 参数声明
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/image_raw',
        description='摄像头话题名称'
    )
    
    size_arg = DeclareLaunchArgument(
        'size',
        default_value='7x5',
        description='标定板内角点数（列 x 行）'
    )
    
    square_arg = DeclareLaunchArgument(
        'square',
        default_value='0.0216',
        description='棋盘格每个方格的边长（米）'
    )

    return LaunchDescription([
        camera_topic_arg,
        size_arg,
        square_arg,

        # 1. 摄像头驱动
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv2rgb',
                'framerate': 30.0,
                'camera_frame_id': 'camera_link',
                'camera_name': 'default_cam',  # ✅ 添加相机名称
            }],
        ),

        # 2. 标定节点
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='camera_calibrator',
            output='screen',
            parameters=[{
                'size': LaunchConfiguration('size'),
                'square': LaunchConfiguration('square'),
            }],
            remappings=[
                ('image', '/image_raw'),           # ✅ 匹配 ros2 topic list 输出
                ('camera_info', '/camera_info'),   # ✅ 显式映射 camera_info（推荐）
                ('camera', 'usb_cam'),             # ✅ 用于 set_camera_info 服务
            ],
            arguments=[
                '--size', LaunchConfiguration('size'),
                '--square', LaunchConfiguration('square'),
            ],
        ),
    ])