#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    ros_image_topic = LaunchConfiguration('ros_image_topic')

    # 默认包含视觉链路，保证一键启动可直接执行任务。
    tvec_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('tvec_tf'), '/launch/tvec_tf.launch.py']),
        launch_arguments={
            'world_name': world_name,
            'model_name': model_name,
            'ros_image_topic': ros_image_topic,
        }.items(),
    )

    node = Node(
        package='land_with_tracking',
        executable='land_with_tracking_node',
        name='land_with_tracking_node',
        output='screen',
        parameters=[{
            'xy_align_tolerance_m': 0.10,
            'yaw_align_tolerance_deg': 5.0,
            'align_hold_sec': 1.0,
            'hover_after_align_sec': 1.0,
            'descent_speed_mps': 0.5,
            'min_throttle_descent_speed_mps': 0.35,
            'min_throttle_disarm_duration_sec': 5.0,
            'disarm_retry_interval_sec': 1.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='rover'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/camera/image_raw'),
        tvec_tf_launch,
        node,
    ])
