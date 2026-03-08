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
    use_rqt = LaunchConfiguration('use_rqt')

    tvec_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('tvec_tf'), '/launch/tvec_tf.launch.py']
        ),
        launch_arguments={
            'world_name': world_name,
            'model_name': model_name,
            'ros_image_topic': ros_image_topic,
            'use_rqt': use_rqt,
        }.items(),
    )

    controller_node = Node(
        package='return_track_hover',
        executable='return_track_hover_node',
        name='return_track_hover_node',
        output='screen',
        parameters=[{
            'state_topic': '/mavros/state',
            'global_topic': '/mavros/global_position/global',
            'home_topic': '/mavros/home_position/home',
            'rel_alt_topic': '/mavros/global_position/rel_alt',
            'aruco_pose_topic': '/debug/aruco_pose',
            'base_pose_topic': '/mavros/local_position/pose',
            'cmd_vel_topic': '/mavros/setpoint_velocity/cmd_vel',
            'target_alt_m': 3.0,
            'alt_tolerance_m': 0.15,
            'home_tolerance_m': 0.50,
            'track_target_x': 0.0,
            'track_target_y': 0.0,
            'track_target_yaw': 0.0,
            'xy_align_tolerance_m': 0.20,
            'yaw_align_tolerance_deg': 8.0,
            'align_hold_sec': 1.0,
            'require_offboard': True,
            'start_on_offboard_entry': True,
            'use_dynamic_marker_yaw': True,
            'fallback_marker_in_map_yaw_deg': 90.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='rover'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('use_rqt', default_value='true'),
        tvec_tf_launch,
        controller_node,
    ])
