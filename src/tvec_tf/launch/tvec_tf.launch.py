#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 透传 tvec 基础链路参数，支持快速切换世界和模型。
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    ros_image_topic = LaunchConfiguration('ros_image_topic')
    use_rqt = LaunchConfiguration('use_rqt')

    # 1) 启动 tvec：桥接图像 + tvec 节点 + RViz 显示。
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

    # 2) 启动 aruco marker 静态 TF（map -> arucomarker）。
    aruco_static_tf_node = Node(
        package='tf_broadcast',
        executable='map_aruco_static_tf_node',
        name='map_aruco_static_tf_node',
        output='screen',
        parameters=[{
            'parent_frame': 'map',
            'child_frame': 'arucomarker',
            'tx': 0.0,
            'ty': 0.0,
            'tz': 0.3,
        }],
    )

    # 3) 启动 tvec->TF 广播（arucomarker -> vision_pose）。
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

    # 4) 启动 rvec 相对 yaw 日志节点（订阅 /debug/tvec，1Hz 输出）。
    rvec_yaw_node = Node(
        package='tvec',
        executable='rvec_yaw_node',
        name='rvec_yaw_node',
        output='screen',
        parameters=[{
            'input_topic': '/debug/tvec',
        }],
    )

    # 5) 启动 map->base_link 动态 TF 广播节点（来自 MAVROS 位姿）。
    map_base_link_tf_node = Node(
        package='tf_broadcast',
        executable='map_base_link_tf_node',
        name='map_base_link_tf_node',
        output='screen',
        parameters=[{
            'pose_topic': '/mavros/local_position/pose',
            'parent_frame': 'map',
            'child_frame': 'base_link',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='aruco'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('use_rqt', default_value='true'),
        tvec_launch,
        aruco_static_tf_node,
        tvec_tf_node,
        rvec_yaw_node,
        map_base_link_tf_node,
    ])
