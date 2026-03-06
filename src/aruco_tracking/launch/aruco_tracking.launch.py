#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 透传 tvec_tf 的场景参数。
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    ros_image_topic = LaunchConfiguration('ros_image_topic')

    # 启动视觉链路（图像桥接 + tvec + tf + rviz）。
    tvec_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('tvec_tf'), '/launch/tvec_tf.launch.py']
        ),
        launch_arguments={
            'world_name': world_name,
            'model_name': model_name,
            'ros_image_topic': ros_image_topic,
        }.items(),
    )

    # 启动 ArUco 跟踪控制节点。
    tracking_node = Node(
        package='aruco_tracking',
        executable='tracking_node',
        name='aruco_tracking_node',
        output='screen',
        parameters=[{
            'pose_topic': '/debug/aruco_pose',
            'state_topic': '/mavros/state',
            'cmd_vel_topic': '/mavros/setpoint_velocity/cmd_vel',
            'target_x': 0.0,
            'target_y': 0.0,
            'target_yaw': 0.0,
            'rotate_error_to_map': True,
            'marker_in_map_yaw_deg': 90.0,
            'enable_relative_z_hold': True,
            'reset_z_ref_on_offboard': True,
            'kp_z_hold': 0.8,
            'ki_z_hold': 0.0,
            'kd_z_hold': 0.06,
            'vz_limit': 0.5,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='rover'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/camera/image_raw'),
        tvec_tf_launch,
        tracking_node,
    ])
