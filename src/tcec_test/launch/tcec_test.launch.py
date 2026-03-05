#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def _launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    ros_image_topic = LaunchConfiguration('ros_image_topic').perform(context)

    gz_image_topic = (
        f'/world/{world_name}/model/{model_name}/'
        'link/camera_link/sensor/imager/image'
    )

    pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'tcec_test.rviz')

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_image',
        output='screen',
        arguments=[f'{gz_image_topic}@sensor_msgs/msg/Image@gz.msgs.Image'],
        remappings=[(gz_image_topic, ros_image_topic)],
    )

    detector_node = Node(
        package='tcec_test',
        executable='tvec_rvec_node',
        name='tvec_rvec_node',
        output='screen',
        parameters=[{
            'image_topic': ros_image_topic,
            'camera_fx': 268.5,
            'camera_fy': 268.5,
            'camera_cx': 320.0,
            'camera_cy': 240.0,
            'marker_size_33': 0.5,
            'marker_size_42': 0.063,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_tcec_test',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return [gz_bridge_node, detector_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='aruco'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/camera/image_raw'),
        OpaqueFunction(function=_launch_setup),
    ])
