#!/usr/bin/env python3
"""
TF 显示 Launch 文件

功能：
- 启动 MAVROS 节点
- 启动 TF 广播节点
- 启动 RViz 并加载预配置

使用方式：
    ros2 launch tf_display tf_display.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    """生成启动描述"""

    # ========== 声明参数 ==========
    use_mavros_arg = DeclareLaunchArgument(
        'use_mavros',
        default_value='true',
        description='是否启动 MAVROS 节点'
    )

    # ========== MAVROS 节点（使用官方 launch 文件）==========
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'px4.launch'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('use_mavros')),
        launch_arguments={
            'fcu_url': 'udp://:14540@'
        }.items()
    )

    # ========== TF 广播节点 ==========
    tf_broadcaster_node = Node(
        package='tf_display',
        executable='tf_broadcaster_node',
        name='tf_broadcaster',
        output='screen',
        parameters=[{
            'frame_id': 'map',         # 世界坐标系（map）
            'child_frame_id': 'base_link',  # 无人机机体坐标系
            'publish_rate': 50.0,       # 发布频率 (Hz)
        }],
    )

    # ========== RViz 节点 ==========
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('tf_display'),
                'rviz',
                'tf_display.rviz'
            ])
        ],
    )

    return LaunchDescription([
        use_mavros_arg,
        mavros_launch,
        tf_broadcaster_node,
        rviz_node,
    ])
