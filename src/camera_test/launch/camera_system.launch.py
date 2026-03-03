#!/usr/bin/env python3
"""
ROS 2 摄像头系统 Launch 文件
一键启动：摄像头驱动 + camera_node + rqt_image_view
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # ========== 声明可配置参数 ==========
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/image_raw',
        description='摄像头原始话题名称'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='图像宽度'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='图像高度'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='帧率'
    )
    
    use_compressed_arg = DeclareLaunchArgument(
        'use_compressed',
        default_value='true',
        description='是否使用压缩话题'
    )
    
    show_rqt_arg = DeclareLaunchArgument(
        'show_rqt',
        default_value='true',
        description='是否启动 rqt_image_view'
    )

    # ========== 1. 摄像头驱动节点 (v4l2_camera) ==========
    camera_test_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[{
            'image_size': {
                'width': LaunchConfiguration('image_width'),
                'height': LaunchConfiguration('image_height')
            },
            'framerate': LaunchConfiguration('framerate'),
            'pixel_format': 'MJPG',  # 使用 MJPG 减少 USB 带宽
            'camera_frame_id': 'camera_link',
        }],
        remappings=[
            ('/image_raw', LaunchConfiguration('camera_topic')),
        ],
    )

    # ========== 2. Camera Node (解码 + 发布) ==========
    camera_node = Node(
        package='rcamera_test',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'use_compressed': LaunchConfiguration('use_compressed'),
            'output_image': True,
            'log_fps': True,
        }],
    )

    # ========== 3. RQT Image View (图像查看器) ==========
    # 使用 OpaqueFunction 条件启动 rqt
    def launch_rqt(context):
        show_rqt = LaunchConfiguration('show_rqt').perform(context)
        if show_rqt.lower() == 'true':
            return [Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='rqt_image_view',
                output='screen',
                arguments=['--force-discover'],  # 强制发现话题
            )]
        else:
            return []

    rqt_node = OpaqueFunction(function=launch_rqt)

    # ========== 创建 LaunchDescription ==========
    return LaunchDescription([
        # 参数声明
        camera_topic_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        use_compressed_arg,
        show_rqt_arg,
        
        # 节点
        camera_test_node,
        camera_node,
        rqt_node,
    ])