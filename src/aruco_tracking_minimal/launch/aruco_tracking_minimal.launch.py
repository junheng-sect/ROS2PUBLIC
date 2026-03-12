#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    ros_image_topic = LaunchConfiguration('ros_image_topic').perform(context)
    enable_csv_logger = LaunchConfiguration('enable_csv_logger')
    csv_output_dir = LaunchConfiguration('csv_output_dir').perform(context)
    csv_prefix = LaunchConfiguration('csv_prefix').perform(context)
    csv_sample_rate_hz = float(LaunchConfiguration('csv_sample_rate_hz').perform(context))

    gz_image_topic = (
        f'/world/{world_name}/model/{model_name}/'
        'link/camera_link/sensor/imager/image'
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_image',
        output='screen',
        arguments=[f'{gz_image_topic}@sensor_msgs/msg/Image@gz.msgs.Image'],
        remappings=[(gz_image_topic, ros_image_topic)],
    )

    tvec_node = Node(
        package='tvec',
        executable='tvec_rvec_node',
        name='tvec_rvec_node',
        output='screen',
        parameters=[{
            'image_topic': ros_image_topic,
            'camera_fx': 268.5,
            'camera_fy': 268.5,
            'camera_cx': 320.0,
            'camera_cy': 240.0,
            'marker_size_33': 0.5,
            'marker_size_42': 0.063,
        }],
    )

    tvec_tf_node = Node(
        package='tvec_tf',
        executable='tvec_tf_node',
        name='tvec_tf_node',
        output='screen',
        parameters=[{
            'input_topic': '/debug/tvec',
            'parent_frame': 'arucomarker',
            'child_frame': 'vision_pose',
        }],
    )

    tracking_node = Node(
        package='aruco_tracking',
        executable='tracking_node',
        name='aruco_tracking_node',
        output='screen',
        parameters=[{
            'pose_topic': '/debug/aruco_pose',
            'base_pose_topic': '/mavros/local_position/pose',
            'state_topic': '/mavros/state',
            'cmd_vel_topic': '/mavros/setpoint_velocity/cmd_vel',
            'target_x': 0.0,
            'target_y': 0.0,
            'target_yaw': 0.0,
            'rotate_error_to_map': True,
            'use_dynamic_marker_yaw': True,
            'fallback_marker_in_map_yaw_deg': 90.0,
            'base_pose_timeout_sec': 0.5,
            'enable_relative_z_hold': True,
            'reset_z_ref_on_offboard': True,
            'kp_z_hold': 0.8,
            'ki_z_hold': 0.0,
            'kd_z_hold': 0.06,
            'vz_limit': 0.5,
        }],
    )

    csv_logger_node = Node(
        package='aruco_tracking_minimal',
        executable='csv_logger_node',
        name='aruco_tracking_csv_logger_node',
        output='screen',
        condition=IfCondition(enable_csv_logger),
        parameters=[{
            'pose_topic': '/debug/aruco_pose',
            'state_topic': '/mavros/state',
            'local_pose_topic': '/mavros/local_position/pose',
            'cmd_vel_topic': '/mavros/setpoint_velocity/cmd_vel',
            'output_dir': csv_output_dir,
            'file_prefix': csv_prefix,
            'sample_rate_hz': csv_sample_rate_hz,
        }],
    )

    return [gz_bridge_node, tvec_node, tvec_tf_node, tracking_node, csv_logger_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='rover'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument(
            'enable_csv_logger',
            default_value='true',
            description='是否启动 CSV 记录节点',
        ),
        DeclareLaunchArgument(
            'csv_output_dir',
            default_value=os.path.expanduser('~/project/zjh_ws/log/tracking_csv'),
            description='CSV 输出目录',
        ),
        DeclareLaunchArgument(
            'csv_prefix',
            default_value='aruco_tracking_minimal',
            description='CSV 文件名前缀',
        ),
        DeclareLaunchArgument(
            'csv_sample_rate_hz',
            default_value='30.0',
            description='CSV 记录频率',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
