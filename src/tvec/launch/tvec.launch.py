#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def _launch_setup(context, *args, **kwargs):
    # 在启动时解析 world/model/topic 参数，便于一个 launch 快速切换场景。
    world_name = LaunchConfiguration('world_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    ros_image_topic = LaunchConfiguration('ros_image_topic').perform(context)

    # PX4 SITL 场景下 Gazebo 相机原始图像话题的标准路径模板。
    gz_image_topic = (
        f'/world/{world_name}/model/{model_name}/'
        'link/camera_link/sensor/imager/image'
    )

    pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'tvec.rviz')

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_image',
        output='screen',
        # 将选定 Gazebo 图像话题桥接为 ROS Image 话题。
        arguments=[f'{gz_image_topic}@sensor_msgs/msg/Image@gz.msgs.Image'],
        remappings=[(gz_image_topic, ros_image_topic)],
    )

    detector_node = Node(
        package='tvec',
        executable='tvec_rvec_node',
        name='tvec_rvec_node',
        output='screen',
        parameters=[{
            # 在 launch 中显式给出相机参数与 marker 尺寸，便于快速调参。
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
        name='rviz2_tvec',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return [gz_bridge_node, detector_node, rviz_node]


def generate_launch_description():
    # 由于 Gazebo 话题字符串依赖运行时参数拼接，使用 OpaqueFunction 生成节点。
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='aruco'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/camera/image_raw'),
        OpaqueFunction(function=_launch_setup),
    ])
