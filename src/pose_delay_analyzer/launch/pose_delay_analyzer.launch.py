#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成自包含最小视觉链的 pose_delay_analyzer 启动入口."""
    tvec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('tvec'), '/launch/tvec.launch.py']
        ),
        condition=IfCondition(LaunchConfiguration('enable_visual_chain')),
        launch_arguments={
            'camera_profile': LaunchConfiguration('camera_profile'),
            'use_usb_cam': LaunchConfiguration('use_usb_cam'),
            'video_device': LaunchConfiguration('video_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'framerate': LaunchConfiguration('framerate'),
            'image_qos_reliability': LaunchConfiguration(
                'image_qos_reliability'
            ),
            'aruco_dictionary': LaunchConfiguration('aruco_dictionary'),
            'publish_annotated_image': LaunchConfiguration(
                'publish_annotated_image'
            ),
            'use_rqt': LaunchConfiguration('use_rqt'),
            'ros_image_topic': LaunchConfiguration('ros_image_topic'),
            'annotated_image_topic': LaunchConfiguration(
                'annotated_image_topic'
            ),
        }.items(),
    )

    tvec_tf_node = Node(
        package='tvec_tf',
        executable='tvec_tf_node',
        name='tvec_tf_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_visual_chain')),
        parameters=[{
            'input_topic': '/debug/tvec',
            'parent_frame': 'arucomarker',
            'child_frame': 'vision_pose',
        }],
    )

    analyzer_node = Node(
        package='pose_delay_analyzer',
        executable='pose_delay_analyzer_node',
        name='pose_delay_analyzer_node',
        output='screen',
        sigterm_timeout='20',
        sigkill_timeout='20',
        parameters=[{
            'local_pose_topic': ParameterValue(
                LaunchConfiguration('local_pose_topic'),
                value_type=str,
            ),
            'distance_sensor_topic': ParameterValue(
                LaunchConfiguration('distance_sensor_topic'),
                value_type=str,
            ),
            'visual_pose_topic': ParameterValue(
                LaunchConfiguration('visual_pose_topic'),
                value_type=str,
            ),
            'distance_sensor_timeout_sec': ParameterValue(
                LaunchConfiguration('distance_sensor_timeout_sec'),
                value_type=float,
            ),
            'output_dir': ParameterValue(
                LaunchConfiguration('output_dir'),
                value_type=str,
            ),
            'file_prefix': ParameterValue(
                LaunchConfiguration('file_prefix'),
                value_type=str,
            ),
            'analysis_axis': ParameterValue(
                LaunchConfiguration('analysis_axis'),
                value_type=str,
            ),
            'min_samples': ParameterValue(
                LaunchConfiguration('min_samples'),
                value_type=int,
            ),
            'smooth_window_sec': ParameterValue(
                LaunchConfiguration('smooth_window_sec'),
                value_type=float,
            ),
            'slope_threshold': ParameterValue(
                LaunchConfiguration('slope_threshold'),
                value_type=float,
            ),
            'merge_gap_sec': ParameterValue(
                LaunchConfiguration('merge_gap_sec'),
                value_type=float,
            ),
            'major_edge_merge_gap_sec': ParameterValue(
                LaunchConfiguration('major_edge_merge_gap_sec'),
                value_type=float,
            ),
            'major_edge_min_amplitude': ParameterValue(
                LaunchConfiguration('major_edge_min_amplitude'),
                value_type=float,
            ),
            'major_edge_max_match_offset_sec': ParameterValue(
                LaunchConfiguration('major_edge_max_match_offset_sec'),
                value_type=float,
            ),
            'plot_relative_time': ParameterValue(
                LaunchConfiguration('plot_relative_time'),
                value_type=bool,
            ),
            'auto_report': ParameterValue(
                LaunchConfiguration('auto_report'),
                value_type=bool,
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_visual_chain',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'local_pose_topic',
            default_value='/mavros/local_position/pose',
        ),
        DeclareLaunchArgument(
            'distance_sensor_topic',
            default_value='/mavros/hrlv_ez4_pub',
        ),
        DeclareLaunchArgument(
            'visual_pose_topic',
            default_value='/debug/aruco_pose',
        ),
        DeclareLaunchArgument(
            'distance_sensor_timeout_sec',
            default_value='0.5',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/home/zjh/project/rasip_pi_ws/log/pose_delay',
        ),
        DeclareLaunchArgument(
            'file_prefix',
            default_value='pose_delay',
        ),
        DeclareLaunchArgument(
            'analysis_axis',
            default_value='z',
        ),
        DeclareLaunchArgument(
            'min_samples',
            default_value='30',
        ),
        DeclareLaunchArgument(
            'smooth_window_sec',
            default_value='0.20',
        ),
        DeclareLaunchArgument(
            'slope_threshold',
            default_value='0.05',
        ),
        DeclareLaunchArgument(
            'merge_gap_sec',
            default_value='0.30',
        ),
        DeclareLaunchArgument(
            'major_edge_merge_gap_sec',
            default_value='1.0',
        ),
        DeclareLaunchArgument(
            'major_edge_min_amplitude',
            default_value='0.08',
        ),
        DeclareLaunchArgument(
            'major_edge_max_match_offset_sec',
            default_value='2.0',
        ),
        DeclareLaunchArgument(
            'plot_relative_time',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'auto_report',
            default_value='true',
        ),
        # 下面这些参数直接透传给最小视觉链，便于实机实验时单命令启动。
        DeclareLaunchArgument(
            'camera_profile',
            default_value='old_cam',
        ),
        DeclareLaunchArgument(
            'use_usb_cam',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='__profile__',
        ),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='__profile__',
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='__profile__',
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='__profile__',
        ),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='__profile__',
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value='__profile__',
        ),
        DeclareLaunchArgument(
            'image_qos_reliability',
            default_value='best_effort',
        ),
        DeclareLaunchArgument(
            'aruco_dictionary',
            default_value='DICT_5X5_1000',
        ),
        DeclareLaunchArgument(
            'publish_annotated_image',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'use_rqt',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'ros_image_topic',
            default_value='/image_raw',
        ),
        DeclareLaunchArgument(
            'annotated_image_topic',
            default_value='/tvec/image_annotated',
        ),
        tvec_launch,
        tvec_tf_node,
        analyzer_node,
    ])
