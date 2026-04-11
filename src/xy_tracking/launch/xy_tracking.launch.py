#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 透传世界、模型和图像话题，复用 rasip_pi_ws 当前实机视觉链。
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    ros_image_topic = LaunchConfiguration('ros_image_topic')
    use_rqt = LaunchConfiguration('use_rqt')
    image_qos_reliability = LaunchConfiguration('image_qos_reliability')
    aruco_dictionary = LaunchConfiguration('aruco_dictionary')
    enable_csv_logger = LaunchConfiguration('enable_csv_logger')
    csv_output_dir = LaunchConfiguration('csv_output_dir')
    csv_prefix = LaunchConfiguration('csv_prefix')
    csv_sample_rate_hz = LaunchConfiguration('csv_sample_rate_hz')
    csv_stale_timeout_sec = LaunchConfiguration('csv_stale_timeout_sec')
    summary_csv_path = LaunchConfiguration('summary_csv_path')

    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    target_z = LaunchConfiguration('target_z')

    kp_xy = LaunchConfiguration('kp_xy')
    ki_xy = LaunchConfiguration('ki_xy')
    kd_xy = LaunchConfiguration('kd_xy')
    kp_z = LaunchConfiguration('kp_z')
    ki_z = LaunchConfiguration('ki_z')
    kd_z = LaunchConfiguration('kd_z')

    control_rate_hz = LaunchConfiguration('control_rate_hz')
    pose_timeout_sec = LaunchConfiguration('pose_timeout_sec')
    base_pose_timeout_sec = LaunchConfiguration('base_pose_timeout_sec')
    vxy_limit = LaunchConfiguration('vxy_limit')
    vz_limit = LaunchConfiguration('vz_limit')
    velocity_deadband = LaunchConfiguration('velocity_deadband')

    tvec_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('tvec_tf'), '/launch/tvec_tf.launch.py']
        ),
        launch_arguments={
            'world_name': world_name,
            'model_name': model_name,
            'ros_image_topic': ros_image_topic,
            'use_rqt': use_rqt,
            'image_qos_reliability': image_qos_reliability,
            'aruco_dictionary': aruco_dictionary,
        }.items(),
    )

    controller_node = Node(
        package='xy_tracking',
        executable='xy_tracking_node',
        name='xy_tracking_node',
        output='screen',
        parameters=[{
            'pose_topic': '/debug/aruco_pose',
            'base_pose_topic': '/mavros/local_position/pose',
            'state_topic': '/mavros/state',
            'cmd_vel_topic': '/mavros/setpoint_velocity/cmd_vel',
            'target_x': target_x,
            'target_y': target_y,
            'target_z': target_z,
            'kp_xy': kp_xy,
            'ki_xy': ki_xy,
            'kd_xy': kd_xy,
            'kp_z': kp_z,
            'ki_z': ki_z,
            'kd_z': kd_z,
            'control_rate_hz': control_rate_hz,
            'pose_timeout_sec': pose_timeout_sec,
            'base_pose_timeout_sec': base_pose_timeout_sec,
            'vxy_limit': vxy_limit,
            'vz_limit': vz_limit,
            'velocity_deadband': velocity_deadband,
        }],
    )

    csv_logger_node = Node(
        package='xy_tracking',
        executable='xy_tracking_csv_logger_node',
        name='xy_tracking_csv_logger_node',
        output='screen',
        condition=IfCondition(enable_csv_logger),
        parameters=[{
            'pose_topic': '/debug/aruco_pose',
            'state_topic': '/mavros/state',
            'local_pose_topic': '/mavros/local_position/pose',
            'cmd_vel_topic': '/mavros/setpoint_velocity/cmd_vel',
            'output_dir': csv_output_dir,
            'file_prefix': csv_prefix,
            'summary_csv_path': summary_csv_path,
            'sample_rate_hz': csv_sample_rate_hz,
            'stale_timeout_sec': csv_stale_timeout_sec,
            'target_x': target_x,
            'target_y': target_y,
            'target_z': target_z,
            'kp_xy': kp_xy,
            'ki_xy': ki_xy,
            'kd_xy': kd_xy,
            'kp_z': kp_z,
            'ki_z': ki_z,
            'kd_z': kd_z,
            'vxy_limit': vxy_limit,
            'vz_limit': vz_limit,
            'velocity_deadband': velocity_deadband,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='rover'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('use_rqt', default_value='false'),
        DeclareLaunchArgument('image_qos_reliability', default_value='best_effort'),
        DeclareLaunchArgument('aruco_dictionary', default_value='DICT_5X5_1000'),
        DeclareLaunchArgument('enable_csv_logger', default_value='true'),
        DeclareLaunchArgument('csv_output_dir', default_value='/home/zjh/project/rasip_pi_ws/log/tracking_csv'),
        DeclareLaunchArgument('csv_prefix', default_value='xy_tracking'),
        DeclareLaunchArgument('csv_sample_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('csv_stale_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument(
            'summary_csv_path',
            default_value='/home/zjh/project/rasip_pi_ws/log/tracking_csv/xy_tracking_summary.csv',
        ),
        DeclareLaunchArgument('target_x', default_value='0.0'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_z', default_value='2.5'),
        DeclareLaunchArgument('kp_xy', default_value='0.6'),
        DeclareLaunchArgument('ki_xy', default_value='0.0'),
        DeclareLaunchArgument('kd_xy', default_value='0.08'),
        DeclareLaunchArgument('kp_z', default_value='0.8'),
        DeclareLaunchArgument('ki_z', default_value='0.0'),
        DeclareLaunchArgument('kd_z', default_value='0.06'),
        DeclareLaunchArgument('control_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('pose_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('base_pose_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('vxy_limit', default_value='0.8'),
        DeclareLaunchArgument('vz_limit', default_value='0.5'),
        DeclareLaunchArgument('velocity_deadband', default_value='0.03'),
        tvec_tf_launch,
        controller_node,
        csv_logger_node,
    ])
