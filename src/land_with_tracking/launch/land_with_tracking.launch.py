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

    # 仅包含 tvec（USB 相机 + ArUco 检测），避免拉起静态 TF 与其他无关节点。
    tvec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('tvec'), '/launch/tvec.launch.py']),
        launch_arguments={
            'world_name': world_name,
            'model_name': model_name,
            'ros_image_topic': ros_image_topic,
            'use_rqt': use_rqt,
        }.items(),
    )

    # 仅启动任务所需的 tvec->TF 节点。
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
        DeclareLaunchArgument('ros_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('use_rqt', default_value='false'),
        tvec_launch,
        tvec_tf_node,
        node,
    ])
