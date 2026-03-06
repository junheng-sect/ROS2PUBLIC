#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 与 tvec 保持同一组参数，便于快速切换 world/model。
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    ros_image_topic = LaunchConfiguration('ros_image_topic')

    # 启动 tvec 全链路（桥接 + 检测 + RViz）。
    tvec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('tvec'), '/launch/tvec.launch.py']
        ),
        launch_arguments={
            'world_name': world_name,
            'model_name': model_name,
            'ros_image_topic': ros_image_topic,
        }.items(),
    )

    # 启动矩阵发布节点。
    matrix_node = Node(
        package='matrix_marker_in_cam',
        executable='matrix_marker_in_cam_node',
        name='matrix_marker_in_cam_node',
        output='screen',
        parameters=[{
            'input_topic': '/debug/tvec',
            'output_topic': '/debug/matrix/marker_in_cam',
            'output_frame_id': 'cam_optical',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='aruco'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/camera/image_raw'),
        tvec_launch,
        matrix_node,
    ])
