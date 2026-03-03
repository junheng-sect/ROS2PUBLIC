#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PX4 Disarm 启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_disarm',
            executable='disarm_node',
            name='px4_disarm',
            output='screen',
        ),
    ])
