#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    # 保留 world/model 参数仅用于接口兼容；实机模式不再使用 GZ 图像。
    _ = LaunchConfiguration('world_name').perform(context)
    _ = LaunchConfiguration('model_name').perform(context)
    ros_image_topic = LaunchConfiguration('ros_image_topic').perform(context)

    # 实机默认使用 USB 相机发布图像。
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_usb_cam')),
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'framerate': LaunchConfiguration('framerate'),
            'camera_frame_id': 'camera_link',
            'camera_name': 'default_cam',
        }],
        remappings=[('/image_raw', ros_image_topic)],
    )

    detector_node = Node(
        package='tvec',
        executable='tvec_rvec_node',
        name='tvec_rvec_node',
        output='screen',
        parameters=[{
            'image_topic': ros_image_topic,
            # 默认实机 USB 摄像头内参（来自 ost.yaml）。
            'camera_fx': 810.78076,
            'camera_fy': 813.75141,
            'camera_cx': 346.24076,
            'camera_cy': 251.50143,
            'dist_k1': -0.410508,
            'dist_k2': 0.102062,
            'dist_p1': 0.001503,
            'dist_p2': -0.000384,
            'dist_k3': 0.0,
            'marker_size_33': 0.5,
            'marker_size_42': 0.063,
        }],
    )

    # 用 rqt_image_view 显示 ArUco 标注后的图像。
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view_tvec',
        output='screen',
        arguments=[LaunchConfiguration('annotated_image_topic')],
        condition=IfCondition(LaunchConfiguration('use_rqt')),
    )

    return [usb_cam_node, detector_node, image_view_node]


def generate_launch_description():
    return LaunchDescription([
        # 上游 launch 仍会透传这两个参数，这里保留声明避免参数不匹配报错。
        DeclareLaunchArgument('world_name', default_value='aruco'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('annotated_image_topic', default_value='/tvec/image_annotated'),
        DeclareLaunchArgument('use_usb_cam', default_value='true'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('pixel_format', default_value='mjpeg2rgb'),
        DeclareLaunchArgument('framerate', default_value='15.0'),
        DeclareLaunchArgument('use_rqt', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
