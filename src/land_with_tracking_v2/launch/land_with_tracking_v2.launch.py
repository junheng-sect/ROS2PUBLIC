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

    # 默认包含视觉链路，保证一键启动即可执行流程。
    tvec_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('tvec_tf'), '/launch/tvec_tf.launch.py']),
        launch_arguments={
            'world_name': world_name,
            'model_name': model_name,
            'ros_image_topic': ros_image_topic,
        }.items(),
    )

    node = Node(
        package='land_with_tracking_v2',
        executable='land_with_tracking_v2_node',
        name='land_with_tracking_v2_node',
        output='screen',
        parameters=[{
            'track_target_z': 2.5,
            'xy_align_tolerance_m': 0.10,
            'z_align_tolerance_m': 0.10,
            'yaw_align_tolerance_deg': 5.0,
            'align_hold_sec': 1.0,
            'hover_after_align_sec': 1.0,
            'kp_x': 0.6,
            'ki_x': 0.0,
            'kd_x': 0.02,
            'kp_y': 0.6,
            'ki_y': 0.0,
            'kd_y': 0.02,
            'kp_yaw': 0.60,
            'ki_yaw': 0.0,
            'kd_yaw': 0.02,
            'kp_z': 0.60,
            'ki_z': 0.0,
            'kd_z': 0.06,
            'vx_limit': 1.0,
            'vy_limit': 1.0,
            'velocity_deadband': 0.03,
            'yaw_rate_deadband': 0.03,
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
