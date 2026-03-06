#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 统一参数，传递给 tvec 的基础启动链路。
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    ros_image_topic = LaunchConfiguration('ros_image_topic')

    # 启动 tvec 全链路（桥接 + tvec_rvec_node + RViz）。
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

    # matrix_marker_in_cam：输出 marker->cam 4x4 矩阵。
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

    # matrix_cam_in_marker：输出 cam->marker 4x4 矩阵（marker->cam 的逆）。
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

    # matrix_baselink_in_cam：输出 baselink->cam 固定外参矩阵。
    matrix_baselink_in_cam_node = Node(
        package='matrix_baselink_in_cam',
        executable='matrix_baselink_in_cam_node',
        name='matrix_baselink_in_cam_node',
        output='screen',
        parameters=[{
            'output_topic': '/debug/matrix/baselink_in_cam',
            'output_frame_id': 'cam_optical',
            'publish_rate_hz': 10.0,
        }],
    )

    # matrix_baselink_in_marker：输出 marker->baselink 4x4 矩阵。
    matrix_baselink_in_marker_node = Node(
        package='matrix_baselink_in_marker',
        executable='matrix_baselink_in_marker_node',
        name='matrix_baselink_in_marker_node',
        output='screen',
        parameters=[{
            'marker_in_cam_topic': '/debug/matrix/marker_in_cam',
            'baselink_in_cam_topic': '/debug/matrix/baselink_in_cam',
            'output_topic': '/debug/matrix/baselink_in_marker',
            'output_frame_id': 'marker_optical',
        }],
    )

    # tf_broadcast：静态 TF map->aruco_marker。
    tf_broadcast_node = Node(
        package='tf_broadcast',
        executable='map_aruco_static_tf_node',
        name='map_aruco_static_tf_node',
        output='screen',
        parameters=[{
            'parent_frame': 'map',
            'child_frame': 'aruco_marker',
            'tx': 0.0,
            'ty': 0.0,
            'tz': 0.3,
        }],
    )

    # tf_broadcast：动态 TF map->base_link（由 Pose 话题驱动）。
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

    # tf_broadcast：将 cam_in_marker 矩阵转为 aruco_marker->cam TF。
    cam_tf_node = Node(
        package='tf_broadcast',
        executable='cam_tf_node',
        name='cam_tf_node',
        output='screen',
        parameters=[{
            'input_topic': '/debug/matrix/cam_in_marker',
            'parent_frame': 'aruco_marker',
            'child_frame': 'cam',
        }],
    )

    # tf_broadcast：将 baselink_in_marker 矩阵转为 aruco_marker->vision_pose TF。
    vision_pose_tf_node = Node(
        package='tf_broadcast',
        executable='vision_pose_tf_node',
        name='vision_pose_tf_node',
        output='screen',
        parameters=[{
            'input_topic': '/debug/matrix/baselink_in_marker',
            'parent_frame': 'aruco_marker',
            'child_frame': 'vision_pose',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='aruco'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/camera/image_raw'),
        tvec_launch,
        matrix_marker_in_cam_node,
        matrix_cam_in_marker_node,
        matrix_baselink_in_cam_node,
        matrix_baselink_in_marker_node,
        tf_broadcast_node,
        map_base_link_tf_node,
        cam_tf_node,
        vision_pose_tf_node,
    ])
