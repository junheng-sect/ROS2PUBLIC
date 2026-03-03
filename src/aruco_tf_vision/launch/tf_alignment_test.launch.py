#!/usr/bin/env python3
"""
占位测试启动文件。
保留该文件用于兼容已有构建缓存中的 launch 软链接。
"""

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
