#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    ros_image_topic = LaunchConfiguration('ros_image_topic').perform(context)
    enable_csv_logger = LaunchConfiguration('enable_csv_logger')
    csv_output_dir = LaunchConfiguration('csv_output_dir').perform(context)
    csv_prefix = LaunchConfiguration('csv_prefix').perform(context)
    csv_sample_rate_hz = float(LaunchConfiguration('csv_sample_rate_hz').perform(context))

    gz_image_topic = (
        f'/world/{world_name}/model/{model_name}/'
        'link/camera_link/sensor/imager/image'
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_image',
        output='screen',
        arguments=[f'{gz_image_topic}@sensor_msgs/msg/Image@gz.msgs.Image'],
        remappings=[(gz_image_topic, ros_image_topic)],
    )

    tvec_node = Node(
        package='tvec',
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

    tracking_node = Node(
        package='body_frame_tracking',
        executable='body_frame_tracking_node',
        name='body_frame_tracking_node',
        output='screen',
        parameters=[{
            'pose_topic': '/debug/aruco_pose',
            'state_topic': '/mavros/state',
            'setpoint_raw_topic': '/mavros/setpoint_raw/local',
            'target_x': 0.0,
            'target_y': 0.0,
            'target_yaw': 0.0,
            'enable_z_hold': False,
            'require_offboard': True,
        }],
    )

    csv_logger_node = Node(
        package='body_frame_tracking_minimal',
        executable='csv_logger_node',
        name='body_frame_tracking_csv_logger_node',
        output='screen',
        condition=IfCondition(enable_csv_logger),
        parameters=[{
            'pose_topic': '/debug/aruco_pose',
            'state_topic': '/mavros/state',
            'local_pose_topic': '/mavros/local_position/pose',
            'setpoint_raw_topic': '/mavros/setpoint_raw/local',
            'output_dir': csv_output_dir,
            'file_prefix': csv_prefix,
            'sample_rate_hz': csv_sample_rate_hz,
        }],
    )

    return [gz_bridge_node, tvec_node, tvec_tf_node, tracking_node, csv_logger_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='rover'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument(
            'enable_csv_logger',
            default_value='true',
            description='是否启动 CSV 记录节点',
        ),
        DeclareLaunchArgument(
            'csv_output_dir',
            default_value='/home/zjh/project/zjh_ws/log/tracking_csv',
            description='CSV 输出目录',
        ),
        DeclareLaunchArgument(
            'csv_prefix',
            default_value='body_frame_tracking_minimal',
            description='CSV 文件名前缀',
        ),
        DeclareLaunchArgument(
            'csv_sample_rate_hz',
            default_value='30.0',
            description='CSV 记录频率',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
