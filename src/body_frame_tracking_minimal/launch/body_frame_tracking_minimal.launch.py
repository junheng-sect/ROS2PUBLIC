#!/usr/bin/env python3

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
    enable_csv_logger = LaunchConfiguration('enable_csv_logger')
    csv_output_dir = LaunchConfiguration('csv_output_dir')
    csv_prefix = LaunchConfiguration('csv_prefix')
    csv_sample_rate_hz = LaunchConfiguration('csv_sample_rate_hz')
    kp_xy = LaunchConfiguration('kp_xy')
    ki_xy = LaunchConfiguration('ki_xy')
    kd_xy = LaunchConfiguration('kd_xy')
    kp_yaw = LaunchConfiguration('kp_yaw')
    ki_yaw = LaunchConfiguration('ki_yaw')
    kd_yaw = LaunchConfiguration('kd_yaw')
    vxy_limit = LaunchConfiguration('vxy_limit')
    wz_limit = LaunchConfiguration('wz_limit')
    velocity_deadband = LaunchConfiguration('velocity_deadband')
    yaw_rate_deadband = LaunchConfiguration('yaw_rate_deadband')

    # 仅复用 tvec（USB 相机 + ArUco 检测），避免拉起不需要的静态 TF 节点。
    tvec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('tvec'), '/launch/tvec.launch.py']
        ),
        launch_arguments={
            'world_name': world_name,
            'model_name': model_name,
            'ros_image_topic': ros_image_topic,
            'use_rqt': use_rqt,
        }.items(),
    )

    # 仅启动必须的 tvec->TF 转换节点。
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
            'kp_xy': kp_xy,
            'ki_xy': ki_xy,
            'kd_xy': kd_xy,
            'kp_yaw': kp_yaw,
            'ki_yaw': ki_yaw,
            'kd_yaw': kd_yaw,
            'vxy_limit': vxy_limit,
            'wz_limit': wz_limit,
            'velocity_deadband': velocity_deadband,
            'yaw_rate_deadband': yaw_rate_deadband,
            'enable_z_hold': False,
            'require_offboard': True,
        }],
    )

    csv_logger_node = Node(
        package='body_frame_tracking_minimal',
        executable='csv_logger_node',
        name='body_frame_tracking_csv_logger_node',
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
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='rover'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('use_rqt', default_value='false'),
        DeclareLaunchArgument(
            'enable_csv_logger',
            default_value='true',
            description='是否启动 CSV 记录节点',
        ),
        DeclareLaunchArgument(
            'csv_output_dir',
            default_value='/home/zjh/project/rasip_pi_ws/log/tracking_csv',
            description='CSV 输出目录',
        ),
        DeclareLaunchArgument(
            'csv_prefix',
            default_value='body_frame_tracking_minimal',
            description='CSV 文件名前缀',
        ),
        DeclareLaunchArgument(
            'csv_sample_rate_hz',
            default_value='30.0',
            description='CSV 记录频率',
        ),
        DeclareLaunchArgument('kp_xy', default_value='0.5'),
        DeclareLaunchArgument('ki_xy', default_value='0.0'),
        DeclareLaunchArgument('kd_xy', default_value='0.08'),
        DeclareLaunchArgument('kp_yaw', default_value='1.2'),
        DeclareLaunchArgument('ki_yaw', default_value='0.0'),
        DeclareLaunchArgument('kd_yaw', default_value='0.08'),
        DeclareLaunchArgument('vxy_limit', default_value='0.8'),
        DeclareLaunchArgument('wz_limit', default_value='1.0'),
        DeclareLaunchArgument('velocity_deadband', default_value='0.03'),
        DeclareLaunchArgument('yaw_rate_deadband', default_value='0.02'),
        tvec_launch,
        tvec_tf_node,
        tracking_node,
        csv_logger_node,
    ])
