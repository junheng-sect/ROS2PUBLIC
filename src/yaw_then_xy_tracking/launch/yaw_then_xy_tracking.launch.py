#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    ros_image_topic = LaunchConfiguration('ros_image_topic')
    use_rqt = LaunchConfiguration('use_rqt')
    use_usb_cam = LaunchConfiguration('use_usb_cam')
    video_device = LaunchConfiguration('video_device')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    pixel_format = LaunchConfiguration('pixel_format')
    framerate = LaunchConfiguration('framerate')
    annotated_image_topic = LaunchConfiguration('annotated_image_topic')
    image_qos_reliability = LaunchConfiguration('image_qos_reliability')
    enable_csv_logger = LaunchConfiguration('enable_csv_logger')
    csv_output_dir = LaunchConfiguration('csv_output_dir')
    csv_prefix = LaunchConfiguration('csv_prefix')
    csv_sample_rate_hz = LaunchConfiguration('csv_sample_rate_hz')
    csv_stale_timeout_sec = LaunchConfiguration('csv_stale_timeout_sec')
    summary_csv_path = LaunchConfiguration('summary_csv_path')

    node = Node(
        package='yaw_then_xy_tracking',
        executable='yaw_then_xy_tracking_node',
        name='yaw_then_xy_tracking_node',
        output='screen',
        parameters=[{
            'pose_topic': '/debug/aruco_pose',
            'state_topic': '/mavros/state',
            'setpoint_raw_topic': '/mavros/setpoint_raw/local',
            'target_x': 0.0,
            'target_y': 0.0,
            'target_yaw': 0.0,
            'target_z': 2.5,
            'kp_x': 0.6,
            'ki_x': 0.0,
            'kd_x': 0.02,
            'kp_y': 0.6,
            'ki_y': 0.0,
            'kd_y': 0.02,
            'kp_yaw': 0.60,
            'ki_yaw': 0.0,
            'kd_yaw': 0.02,
            'kp_z': 0.60,
            'ki_z': 0.0,
            'kd_z': 0.06,
            'vx_limit': 1.0,
            'vy_limit': 1.0,
            'velocity_deadband': 0.03,
            'yaw_rate_deadband': 0.03,
            'yaw_align_threshold_deg': 5.0,
            'yaw_align_hold_sec': 0.5,
        }],
    )

    tvec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('tvec'), '/launch/tvec.launch.py']
        ),
        launch_arguments={
            'world_name': world_name,
            'model_name': model_name,
            'ros_image_topic': ros_image_topic,
            'use_rqt': use_rqt,
            'use_usb_cam': use_usb_cam,
            'video_device': video_device,
            'image_width': image_width,
            'image_height': image_height,
            'pixel_format': pixel_format,
            'framerate': framerate,
            'annotated_image_topic': annotated_image_topic,
            'image_qos_reliability': image_qos_reliability,
        }.items(),
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

    csv_logger_node = Node(
        package='yaw_then_xy_tracking',
        executable='yaw_then_xy_tracking_csv_logger_node',
        name='yaw_then_xy_tracking_csv_logger_node',
        output='screen',
        condition=IfCondition(enable_csv_logger),
        parameters=[{
            'pose_topic': '/debug/aruco_pose',
            'state_topic': '/mavros/state',
            'local_pose_topic': '/mavros/local_position/pose',
            'setpoint_raw_topic': '/mavros/setpoint_raw/local',
            'output_dir': csv_output_dir,
            'file_prefix': csv_prefix,
            'summary_csv_path': summary_csv_path,
            'sample_rate_hz': csv_sample_rate_hz,
            'stale_timeout_sec': csv_stale_timeout_sec,
            'metric_window_sec': 5.0,
            # 参数快照（summary 将记录这些值）
            'target_x': 0.0,
            'target_y': 0.0,
            'target_yaw': 0.0,
            'target_z': 2.5,
            'kp_x': 0.6, 'ki_x': 0.0, 'kd_x': 0.02,
            'kp_y': 0.6, 'ki_y': 0.0, 'kd_y': 0.02,
            'kp_yaw': 0.60, 'ki_yaw': 0.0, 'kd_yaw': 0.02,
            'kp_z': 0.60, 'ki_z': 0.0, 'kd_z': 0.06,
            'vxy_limit': 1.0,
            'vz_limit': 0.5,
            'wz_limit': 1.0,
            'velocity_deadband': 0.03,
            'yaw_rate_deadband': 0.03,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='rover'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('annotated_image_topic', default_value='/tvec/image_annotated'),
        DeclareLaunchArgument('use_rqt', default_value='false'),
        DeclareLaunchArgument('use_usb_cam', default_value='true'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('pixel_format', default_value='mjpeg2rgb'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument('image_qos_reliability', default_value='reliable'),
        DeclareLaunchArgument('enable_csv_logger', default_value='true'),
        DeclareLaunchArgument('csv_output_dir', default_value='/home/zjh/project/rasip_pi_ws/log/tracking_csv'),
        DeclareLaunchArgument('csv_prefix', default_value='yaw_then_xy_tracking'),
        DeclareLaunchArgument('csv_sample_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('csv_stale_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument(
            'summary_csv_path',
            default_value='/home/zjh/project/rasip_pi_ws/log/tracking_csv/yaw_then_xy_tracking_summary.csv',
        ),
        tvec_launch,
        tvec_tf_node,
        node,
        csv_logger_node,
    ])
