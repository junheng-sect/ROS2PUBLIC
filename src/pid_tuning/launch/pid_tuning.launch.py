#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    ros_image_topic = LaunchConfiguration('ros_image_topic')
    use_rqt = LaunchConfiguration('use_rqt')
    use_usb_cam = LaunchConfiguration('use_usb_cam')
    video_device = LaunchConfiguration('video_device')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    pixel_format = LaunchConfiguration('pixel_format')
    framerate = LaunchConfiguration('framerate')
    annotated_image_topic = LaunchConfiguration('annotated_image_topic')
    image_qos_reliability = LaunchConfiguration('image_qos_reliability')

    csv_output_dir = LaunchConfiguration('csv_output_dir')
    csv_prefix = LaunchConfiguration('csv_prefix')
    csv_sample_rate_hz = LaunchConfiguration('csv_sample_rate_hz')
    enable_csv_logger = LaunchConfiguration('enable_csv_logger')
    summary_csv_path = LaunchConfiguration('summary_csv_path')
    sync_on_shutdown = LaunchConfiguration('sync_on_shutdown')
    sync_target_dir = LaunchConfiguration('sync_target_dir')
    sync_copy_run_csv = LaunchConfiguration('sync_copy_run_csv')
    sync_copy_summary_csv = LaunchConfiguration('sync_copy_summary_csv')
    sync_cmd = LaunchConfiguration('sync_cmd')

    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    target_yaw = LaunchConfiguration('target_yaw')
    target_z = LaunchConfiguration('target_z')

    kp_xy = LaunchConfiguration('kp_xy')
    ki_xy = LaunchConfiguration('ki_xy')
    kd_xy = LaunchConfiguration('kd_xy')
    kp_x = LaunchConfiguration('kp_x')
    ki_x = LaunchConfiguration('ki_x')
    kd_x = LaunchConfiguration('kd_x')
    kp_y = LaunchConfiguration('kp_y')
    ki_y = LaunchConfiguration('ki_y')
    kd_y = LaunchConfiguration('kd_y')

    kp_yaw = LaunchConfiguration('kp_yaw')
    ki_yaw = LaunchConfiguration('ki_yaw')
    kd_yaw = LaunchConfiguration('kd_yaw')
    kp_z = LaunchConfiguration('kp_z')
    ki_z = LaunchConfiguration('ki_z')
    kd_z = LaunchConfiguration('kd_z')

    vxy_limit = LaunchConfiguration('vxy_limit')
    vx_limit = LaunchConfiguration('vx_limit')
    vy_limit = LaunchConfiguration('vy_limit')
    vz_limit = LaunchConfiguration('vz_limit')
    wz_limit = LaunchConfiguration('wz_limit')
    velocity_deadband = LaunchConfiguration('velocity_deadband')
    yaw_rate_deadband = LaunchConfiguration('yaw_rate_deadband')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    pose_timeout_sec = LaunchConfiguration('pose_timeout_sec')
    stale_timeout_sec = LaunchConfiguration('stale_timeout_sec')
    require_offboard = LaunchConfiguration('require_offboard')
    enable_z_hold = LaunchConfiguration('enable_z_hold')

    tvec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('tvec'), '/launch/tvec.launch.py']
        ),
        launch_arguments={
            'world_name': world_name,
            'model_name': model_name,
            'ros_image_topic': ros_image_topic,
            'use_rqt': use_rqt,
            'use_usb_cam': use_usb_cam,
            'video_device': video_device,
            'image_width': image_width,
            'image_height': image_height,
            'pixel_format': pixel_format,
            'framerate': framerate,
            'annotated_image_topic': annotated_image_topic,
            'image_qos_reliability': image_qos_reliability,
        }.items(),
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

    pid_tuning_node = Node(
        package='pid_tuning',
        executable='pid_tuning_node',
        name='pid_tuning_node',
        output='screen',
        parameters=[{
            'pose_topic': '/debug/aruco_pose',
            'state_topic': '/mavros/state',
            'setpoint_raw_topic': '/mavros/setpoint_raw/local',
            'target_x': target_x,
            'target_y': target_y,
            'target_yaw': target_yaw,
            'target_z': target_z,
            'kp_xy': kp_xy,
            'ki_xy': ki_xy,
            'kd_xy': kd_xy,
            'kp_x': kp_x,
            'ki_x': ki_x,
            'kd_x': kd_x,
            'kp_y': kp_y,
            'ki_y': ki_y,
            'kd_y': kd_y,
            'kp_yaw': kp_yaw,
            'ki_yaw': ki_yaw,
            'kd_yaw': kd_yaw,
            'kp_z': kp_z,
            'ki_z': ki_z,
            'kd_z': kd_z,
            'vxy_limit': vxy_limit,
            'vx_limit': vx_limit,
            'vy_limit': vy_limit,
            'vz_limit': vz_limit,
            'wz_limit': wz_limit,
            'velocity_deadband': velocity_deadband,
            'yaw_rate_deadband': yaw_rate_deadband,
            'control_rate_hz': control_rate_hz,
            'pose_timeout_sec': pose_timeout_sec,
            'require_offboard': require_offboard,
            'enable_z_hold': enable_z_hold,
        }],
    )

    csv_logger_node = Node(
        package='pid_tuning',
        executable='csv_logger_node',
        name='pid_tuning_csv_logger_node',
        output='screen',
        condition=IfCondition(enable_csv_logger),
        parameters=[{
            'pose_topic': '/debug/aruco_pose',
            'state_topic': '/mavros/state',
            'local_pose_topic': '/mavros/local_position/pose',
            'setpoint_raw_topic': '/mavros/setpoint_raw/local',
            'output_dir': csv_output_dir,
            'file_prefix': csv_prefix,
            'sample_rate_hz': csv_sample_rate_hz,
            'stale_timeout_sec': stale_timeout_sec,
            'summary_csv_path': summary_csv_path,
            'sync_on_shutdown': sync_on_shutdown,
            'sync_target_dir': sync_target_dir,
            'sync_copy_run_csv': sync_copy_run_csv,
            'sync_copy_summary_csv': sync_copy_summary_csv,
            'sync_cmd': sync_cmd,
            'target_x': target_x,
            'target_y': target_y,
            'target_yaw': target_yaw,
            'target_z': target_z,
            'kp_xy': kp_xy,
            'ki_xy': ki_xy,
            'kd_xy': kd_xy,
            'kp_x': kp_x,
            'ki_x': ki_x,
            'kd_x': kd_x,
            'kp_y': kp_y,
            'ki_y': ki_y,
            'kd_y': kd_y,
            'kp_yaw': kp_yaw,
            'ki_yaw': ki_yaw,
            'kd_yaw': kd_yaw,
            'kp_z': kp_z,
            'ki_z': ki_z,
            'kd_z': kd_z,
            'vxy_limit': vxy_limit,
            'vz_limit': vz_limit,
            'wz_limit': wz_limit,
            'velocity_deadband': velocity_deadband,
            'yaw_rate_deadband': yaw_rate_deadband,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='rover'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('annotated_image_topic', default_value='/tvec/image_annotated'),
        DeclareLaunchArgument('use_rqt', default_value='false'),
        DeclareLaunchArgument('use_usb_cam', default_value='true'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('pixel_format', default_value='mjpeg2rgb'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument('image_qos_reliability', default_value='reliable'),
        DeclareLaunchArgument('enable_csv_logger', default_value='true'),
        DeclareLaunchArgument(
            'csv_output_dir',
            default_value=os.path.expanduser('~/project/zjh_ws/log/tracking_csv'),
        ),
        DeclareLaunchArgument('csv_prefix', default_value='pid_tuning'),
        DeclareLaunchArgument('csv_sample_rate_hz', default_value='30.0'),
        DeclareLaunchArgument(
            'summary_csv_path',
            default_value=os.path.expanduser('~/project/zjh_ws/log/tracking_csv/pid_tuning_summary.csv'),
        ),
        DeclareLaunchArgument('sync_on_shutdown', default_value='true'),
        DeclareLaunchArgument(
            'sync_target_dir',
            default_value=os.path.expanduser('~/桌面/trackingcsv'),
        ),
        DeclareLaunchArgument('sync_copy_run_csv', default_value='true'),
        DeclareLaunchArgument('sync_copy_summary_csv', default_value='true'),
        DeclareLaunchArgument('sync_cmd', default_value=''),
        DeclareLaunchArgument('target_x', default_value='0.0'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_yaw', default_value='0.0'),
        DeclareLaunchArgument('target_z', default_value='2.5'),
        DeclareLaunchArgument('kp_xy', default_value='0.5'),
        DeclareLaunchArgument('ki_xy', default_value='0.0'),
        DeclareLaunchArgument('kd_xy', default_value='0.08'),
        DeclareLaunchArgument('kp_x', default_value='nan'),
        DeclareLaunchArgument('ki_x', default_value='nan'),
        DeclareLaunchArgument('kd_x', default_value='nan'),
        DeclareLaunchArgument('kp_y', default_value='nan'),
        DeclareLaunchArgument('ki_y', default_value='nan'),
        DeclareLaunchArgument('kd_y', default_value='nan'),
        DeclareLaunchArgument('kp_yaw', default_value='1.2'),
        DeclareLaunchArgument('ki_yaw', default_value='0.0'),
        DeclareLaunchArgument('kd_yaw', default_value='0.08'),
        DeclareLaunchArgument('kp_z', default_value='0.8'),
        DeclareLaunchArgument('ki_z', default_value='0.0'),
        DeclareLaunchArgument('kd_z', default_value='0.06'),
        DeclareLaunchArgument('vxy_limit', default_value='0.8'),
        DeclareLaunchArgument('vx_limit', default_value='nan'),
        DeclareLaunchArgument('vy_limit', default_value='nan'),
        DeclareLaunchArgument('vz_limit', default_value='0.5'),
        DeclareLaunchArgument('wz_limit', default_value='1.0'),
        DeclareLaunchArgument('velocity_deadband', default_value='0.03'),
        DeclareLaunchArgument('yaw_rate_deadband', default_value='0.02'),
        DeclareLaunchArgument('control_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('pose_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('stale_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('require_offboard', default_value='true'),
        DeclareLaunchArgument('enable_z_hold', default_value='true'),
        tvec_launch,
        tvec_tf_node,
        pid_tuning_node,
        csv_logger_node,
    ])
