#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 统一参数接口，便于一条命令切换 world/model/topic。
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

    # 启动 marker_in_cam 节点：输出 marker 在 cam(optical) 下位姿。
    marker_in_cam_node = Node(
        package='marker_in_cam',
        executable='marker_in_cam_node',
        name='marker_in_cam_node',
        output='screen',
        parameters=[{
            'input_topic': '/debug/tvec',
            'output_topic': '/debug/marker_in_cam',
            'output_frame_id': 'cam_optical',
        }],
    )

    # 启动 matrix_marker_in_cam 节点：输出 marker->cam 4x4 矩阵。
    matrix_marker_in_cam_node = Node(
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

    # 启动 matrix_cam_in_marker 节点：对 marker->cam 做反变换，得到 cam->marker。
    matrix_cam_in_marker_node = Node(
        package='matrix_cam_in_marker',
        executable='matrix_cam_in_marker_node',
        name='matrix_cam_in_marker_node',
        output='screen',
        parameters=[{
            'input_topic': '/debug/matrix/marker_in_cam',
            'output_topic': '/debug/matrix/cam_in_marker',
            'output_frame_id': 'marker_optical',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='aruco'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/image_raw'),
        tvec_launch,
        marker_in_cam_node,
        matrix_marker_in_cam_node,
        matrix_cam_in_marker_node,
    ])
