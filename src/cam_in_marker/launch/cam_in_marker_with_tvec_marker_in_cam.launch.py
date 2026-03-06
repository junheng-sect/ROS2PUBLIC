#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 与现有 tvec/marker_in_cam 统一参数接口，便于切换 world/model。
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    ros_image_topic = LaunchConfiguration('ros_image_topic')

    # 先启动 tvec 链路（桥接+检测+RViz）。
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

    # 同时启动 marker_in_cam 节点（你要求一起启动 markerincam）。
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

    # 启动 cam_in_marker 节点，输出 cam 在 marker(FLU) 下位姿。
    cam_in_marker_node = Node(
        package='cam_in_marker',
        executable='cam_in_marker_node',
        name='cam_in_marker_node',
        output='screen',
        parameters=[{
            'input_topic': '/debug/tvec',
            'output_topic': '/debug/cam_in_marker',
            'output_frame_id': 'marker_flu',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='aruco'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/camera/image_raw'),
        tvec_launch,
        marker_in_cam_node,
        cam_in_marker_node,
    ])
