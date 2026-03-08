#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 复用 tvec 启动参数，实现同一命令切换不同 world/model。
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    ros_image_topic = LaunchConfiguration('ros_image_topic')

    # 启动 tvec 全链路（桥接 + 检测节点 + RViz）。
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

    # 启动 marker_in_cam 节点，消费 /debug/tvec，发布 /debug/marker_in_cam。
    marker_node = Node(
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

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='aruco'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/image_raw'),
        tvec_launch,
        marker_node,
    ])
