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

    # 复用现有视觉链路，避免改动原有包。
    tvec_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('tvec_tf'), '/launch/tvec_tf.launch.py']),
        launch_arguments={
            'world_name': world_name,
            'model_name': model_name,
            'ros_image_topic': ros_image_topic,
            'use_rqt': use_rqt,
        }.items(),
    )

    tracking_node = Node(
        package='body_frame_tracking',
        executable='body_frame_tracking_node',
        name='body_frame_tracking_node',
        output='screen',
        parameters=[{
            'pose_topic': '/debug/aruco_pose',
            'state_topic': '/mavros/state',
            'setpoint_raw_topic': '/mavros/setpoint_raw/local',
            'target_x': 0.0,
            'target_y': 0.0,
            'target_yaw': 0.0,
            'enable_z_hold': False,
            'require_offboard': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='rover'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('use_rqt', default_value='true'),
        tvec_tf_launch,
        tracking_node,
    ])
