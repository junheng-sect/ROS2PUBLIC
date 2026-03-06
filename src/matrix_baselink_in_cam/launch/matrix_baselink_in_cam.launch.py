#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 启动固定外参矩阵发布节点。
    return LaunchDescription([
        Node(
            package='matrix_baselink_in_cam',
            executable='matrix_baselink_in_cam_node',
            name='matrix_baselink_in_cam_node',
            output='screen',
            parameters=[{
                'output_topic': '/debug/matrix/baselink_in_cam',
                'output_frame_id': 'cam_optical',
                'publish_rate_hz': 10.0,
            }],
        )
    ])
