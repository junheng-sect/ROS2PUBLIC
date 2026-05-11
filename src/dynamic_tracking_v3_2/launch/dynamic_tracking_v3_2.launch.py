#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PROFILE_OVERRIDE_SENTINEL = '__profile__'


def _launch_value_as_bool(value: str) -> bool:
    """把 launch 参数字符串统一转成布尔值。"""
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _create_csv_logger_node(
    context,
    *,
    enable_csv_logger,
    image_raw_timing_topic,
    raw_tvec_topic,
    pipeline_timing_topic,
    attitude_topic,
    csv_output_dir,
    csv_prefix,
    csv_sample_rate_hz,
    summary_csv_path,
    target_x,
    target_y,
    target_z,
    target_yaw,
    kp_xy,
    ki_xy,
    kd_xy,
    kp_x,
    ki_x,
    kd_x,
    kp_y,
    ki_y,
    kd_y,
    kp_z,
    ki_z,
    kd_z,
    kp_yaw,
    ki_yaw,
    kd_yaw,
    camera_yaw_compensation_deg,
    v_limit,
    vz_limit,
    yaw_rate_limit,
    velocity_deadband,
    control_rate_hz,
    pose_timeout_sec,
    stale_timeout_sec,
    require_offboard,
    enable_z_hold,
    enable_z_yaw_smoothing,
    z_lpf_tau_sec,
    yaw_lpf_tau_sec,
    z_error_deadband,
    yaw_error_deadband,
    vz_slew_rate_limit,
    yaw_rate_slew_rate_limit,
    camera_profile,
    image_width,
    image_height,
    publish_annotated_image,
):
    """在 launch 展开阶段把元数据解析成字面量，避免 CSV logger 收到空字符串。"""
    if not _launch_value_as_bool(enable_csv_logger.perform(context)):
        return []

    return [
        Node(
            package='dynamic_tracking_v3_2',
            executable='dynamic_tracking_v3_2_csv_logger_node',
            name='dynamic_tracking_v3_2_csv_logger_node',
            output='screen',
            parameters=[{
                'pose_topic': '/debug/aruco_pose',
                'image_raw_timing_topic': image_raw_timing_topic,
                'raw_tvec_topic': raw_tvec_topic,
                'pipeline_timing_topic': pipeline_timing_topic,
                'state_topic': '/mavros/state',
                'local_pose_topic': '/mavros/local_position/pose',
                'attitude_topic': attitude_topic,
                'setpoint_raw_topic': '/mavros/setpoint_raw/local',
                'output_dir': csv_output_dir,
                'file_prefix': csv_prefix,
                'sample_rate_hz': csv_sample_rate_hz,
                'stale_timeout_sec': stale_timeout_sec,
                'summary_csv_path': summary_csv_path,
                'target_x': target_x,
                'target_y': target_y,
                'target_z': target_z,
                'target_yaw': target_yaw,
                'kp_xy': kp_xy,
                'ki_xy': ki_xy,
                'kd_xy': kd_xy,
                'kp_x': kp_x,
                'ki_x': ki_x,
                'kd_x': kd_x,
                'kp_y': kp_y,
                'ki_y': ki_y,
                'kd_y': kd_y,
                'kp_z': kp_z,
                'ki_z': ki_z,
                'kd_z': kd_z,
                'kp_yaw': kp_yaw,
                'ki_yaw': ki_yaw,
                'kd_yaw': kd_yaw,
                'camera_yaw_compensation_deg': camera_yaw_compensation_deg,
                'v_limit': v_limit,
                'vz_limit': vz_limit,
                'yaw_rate_limit': yaw_rate_limit,
                'velocity_deadband': velocity_deadband,
                'control_rate_hz': control_rate_hz,
                'pose_timeout_sec': pose_timeout_sec,
                'require_offboard': require_offboard,
                'enable_z_hold': enable_z_hold,
                'enable_z_yaw_smoothing': enable_z_yaw_smoothing,
                'z_lpf_tau_sec': z_lpf_tau_sec,
                'yaw_lpf_tau_sec': yaw_lpf_tau_sec,
                'z_error_deadband': z_error_deadband,
                'yaw_error_deadband': yaw_error_deadband,
                'vz_slew_rate_limit': vz_slew_rate_limit,
                'yaw_rate_slew_rate_limit': yaw_rate_slew_rate_limit,
                'camera_profile': camera_profile.perform(context),
                'image_width': image_width.perform(context),
                'image_height': image_height.perform(context),
                'publish_annotated_image': _launch_value_as_bool(
                    publish_annotated_image.perform(context)
                ),
            }],
        )
    ]


def generate_launch_description():
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    ros_image_topic = LaunchConfiguration('ros_image_topic')
    annotated_image_topic = LaunchConfiguration('annotated_image_topic')
    publish_annotated_image = LaunchConfiguration('publish_annotated_image')
    camera_profile = LaunchConfiguration('camera_profile')
    camera_name = LaunchConfiguration('camera_name')
    camera_info_url = LaunchConfiguration('camera_info_url')
    use_rqt = LaunchConfiguration('use_rqt')
    use_usb_cam = LaunchConfiguration('use_usb_cam')
    video_device = LaunchConfiguration('video_device')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    pixel_format = LaunchConfiguration('pixel_format')
    framerate = LaunchConfiguration('framerate')
    camera_fx = LaunchConfiguration('camera_fx')
    camera_fy = LaunchConfiguration('camera_fy')
    camera_cx = LaunchConfiguration('camera_cx')
    camera_cy = LaunchConfiguration('camera_cy')
    dist_k1 = LaunchConfiguration('dist_k1')
    dist_k2 = LaunchConfiguration('dist_k2')
    dist_p1 = LaunchConfiguration('dist_p1')
    dist_p2 = LaunchConfiguration('dist_p2')
    dist_k3 = LaunchConfiguration('dist_k3')
    image_qos_reliability = LaunchConfiguration('image_qos_reliability')
    aruco_dictionary = LaunchConfiguration('aruco_dictionary')

    enable_csv_logger = LaunchConfiguration('enable_csv_logger')
    csv_output_dir = LaunchConfiguration('csv_output_dir')
    csv_prefix = LaunchConfiguration('csv_prefix')
    csv_sample_rate_hz = LaunchConfiguration('csv_sample_rate_hz')
    summary_csv_path = LaunchConfiguration('summary_csv_path')
    image_raw_timing_topic = LaunchConfiguration('image_raw_timing_topic')
    raw_tvec_topic = LaunchConfiguration('raw_tvec_topic')
    pipeline_timing_topic = LaunchConfiguration('pipeline_timing_topic')
    attitude_topic = LaunchConfiguration('attitude_topic')

    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    target_z = LaunchConfiguration('target_z')
    target_yaw = LaunchConfiguration('target_yaw')

    kp_xy = LaunchConfiguration('kp_xy')
    ki_xy = LaunchConfiguration('ki_xy')
    kd_xy = LaunchConfiguration('kd_xy')
    kp_x = LaunchConfiguration('kp_x')
    ki_x = LaunchConfiguration('ki_x')
    kd_x = LaunchConfiguration('kd_x')
    kp_y = LaunchConfiguration('kp_y')
    ki_y = LaunchConfiguration('ki_y')
    kd_y = LaunchConfiguration('kd_y')

    kp_z = LaunchConfiguration('kp_z')
    ki_z = LaunchConfiguration('ki_z')
    kd_z = LaunchConfiguration('kd_z')
    kp_yaw = LaunchConfiguration('kp_yaw')
    ki_yaw = LaunchConfiguration('ki_yaw')
    kd_yaw = LaunchConfiguration('kd_yaw')
    camera_yaw_compensation_deg = LaunchConfiguration('camera_yaw_compensation_deg')

    v_limit = LaunchConfiguration('v_limit')
    vz_limit = LaunchConfiguration('vz_limit')
    yaw_rate_limit = LaunchConfiguration('yaw_rate_limit')
    velocity_deadband = LaunchConfiguration('velocity_deadband')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    pose_timeout_sec = LaunchConfiguration('pose_timeout_sec')
    stale_timeout_sec = LaunchConfiguration('stale_timeout_sec')
    require_offboard = LaunchConfiguration('require_offboard')
    enable_z_hold = LaunchConfiguration('enable_z_hold')
    enable_z_yaw_smoothing = LaunchConfiguration('enable_z_yaw_smoothing')
    z_lpf_tau_sec = LaunchConfiguration('z_lpf_tau_sec')
    yaw_lpf_tau_sec = LaunchConfiguration('yaw_lpf_tau_sec')
    z_error_deadband = LaunchConfiguration('z_error_deadband')
    yaw_error_deadband = LaunchConfiguration('yaw_error_deadband')
    vz_slew_rate_limit = LaunchConfiguration('vz_slew_rate_limit')
    yaw_rate_slew_rate_limit = LaunchConfiguration('yaw_rate_slew_rate_limit')
    invert_yaw_rate_output = LaunchConfiguration('invert_yaw_rate_output')

    tvec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('tvec'), '/launch/tvec.launch.py']
        ),
        launch_arguments={
            'world_name': world_name,
            'model_name': model_name,
            'ros_image_topic': ros_image_topic,
            'annotated_image_topic': annotated_image_topic,
            'publish_annotated_image': publish_annotated_image,
            'camera_profile': camera_profile,
            'camera_name': camera_name,
            'camera_info_url': camera_info_url,
            'use_rqt': use_rqt,
            'use_usb_cam': use_usb_cam,
            'video_device': video_device,
            'image_width': image_width,
            'image_height': image_height,
            'pixel_format': pixel_format,
            'framerate': framerate,
            'camera_fx': camera_fx,
            'camera_fy': camera_fy,
            'camera_cx': camera_cx,
            'camera_cy': camera_cy,
            'dist_k1': dist_k1,
            'dist_k2': dist_k2,
            'dist_p1': dist_p1,
            'dist_p2': dist_p2,
            'dist_k3': dist_k3,
            'image_qos_reliability': image_qos_reliability,
            'aruco_dictionary': aruco_dictionary,
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

    controller_node = Node(
        package='dynamic_tracking_v3_2',
        executable='dynamic_tracking_v3_2_node',
        name='dynamic_tracking_v3_2_node',
        output='screen',
        parameters=[{
            'pose_topic': '/debug/aruco_pose',
            'state_topic': '/mavros/state',
            'setpoint_raw_topic': '/mavros/setpoint_raw/local',
            'target_x': target_x,
            'target_y': target_y,
            'target_z': target_z,
            'target_yaw': target_yaw,
            'kp_xy': kp_xy,
            'ki_xy': ki_xy,
            'kd_xy': kd_xy,
            'kp_x': kp_x,
            'ki_x': ki_x,
            'kd_x': kd_x,
            'kp_y': kp_y,
            'ki_y': ki_y,
            'kd_y': kd_y,
            'kp_z': kp_z,
            'ki_z': ki_z,
            'kd_z': kd_z,
            'kp_yaw': kp_yaw,
            'ki_yaw': ki_yaw,
            'kd_yaw': kd_yaw,
            'camera_yaw_compensation_deg': camera_yaw_compensation_deg,
            'invert_yaw_rate_output': invert_yaw_rate_output,
            'v_limit': v_limit,
            'vz_limit': vz_limit,
            'yaw_rate_limit': yaw_rate_limit,
            'velocity_deadband': velocity_deadband,
            'control_rate_hz': control_rate_hz,
            'pose_timeout_sec': pose_timeout_sec,
            'require_offboard': require_offboard,
            'enable_z_hold': enable_z_hold,
            'enable_z_yaw_smoothing': enable_z_yaw_smoothing,
            'z_lpf_tau_sec': z_lpf_tau_sec,
            'yaw_lpf_tau_sec': yaw_lpf_tau_sec,
            'z_error_deadband': z_error_deadband,
            'yaw_error_deadband': yaw_error_deadband,
            'vz_slew_rate_limit': vz_slew_rate_limit,
            'yaw_rate_slew_rate_limit': yaw_rate_slew_rate_limit,
        }],
    )

    csv_logger_node = OpaqueFunction(
        function=_create_csv_logger_node,
        kwargs={
            'enable_csv_logger': enable_csv_logger,
            'image_raw_timing_topic': image_raw_timing_topic,
            'raw_tvec_topic': raw_tvec_topic,
            'pipeline_timing_topic': pipeline_timing_topic,
            'attitude_topic': attitude_topic,
            'csv_output_dir': csv_output_dir,
            'csv_prefix': csv_prefix,
            'csv_sample_rate_hz': csv_sample_rate_hz,
            'summary_csv_path': summary_csv_path,
            'target_x': target_x,
            'target_y': target_y,
            'target_z': target_z,
            'target_yaw': target_yaw,
            'kp_xy': kp_xy,
            'ki_xy': ki_xy,
            'kd_xy': kd_xy,
            'kp_x': kp_x,
            'ki_x': ki_x,
            'kd_x': kd_x,
            'kp_y': kp_y,
            'ki_y': ki_y,
            'kd_y': kd_y,
            'kp_z': kp_z,
            'ki_z': ki_z,
            'kd_z': kd_z,
            'kp_yaw': kp_yaw,
            'ki_yaw': ki_yaw,
            'kd_yaw': kd_yaw,
            'camera_yaw_compensation_deg': camera_yaw_compensation_deg,
            'v_limit': v_limit,
            'vz_limit': vz_limit,
            'yaw_rate_limit': yaw_rate_limit,
            'velocity_deadband': velocity_deadband,
            'control_rate_hz': control_rate_hz,
            'pose_timeout_sec': pose_timeout_sec,
            'stale_timeout_sec': stale_timeout_sec,
            'require_offboard': require_offboard,
            'enable_z_hold': enable_z_hold,
            'enable_z_yaw_smoothing': enable_z_yaw_smoothing,
            'z_lpf_tau_sec': z_lpf_tau_sec,
            'yaw_lpf_tau_sec': yaw_lpf_tau_sec,
            'z_error_deadband': z_error_deadband,
            'yaw_error_deadband': yaw_error_deadband,
            'vz_slew_rate_limit': vz_slew_rate_limit,
            'yaw_rate_slew_rate_limit': yaw_rate_slew_rate_limit,
            'camera_profile': camera_profile,
            'image_width': image_width,
            'image_height': image_height,
            'publish_annotated_image': publish_annotated_image,
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='rover'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument(
            'annotated_image_topic',
            default_value='/tvec/image_annotated',
        ),
        DeclareLaunchArgument('publish_annotated_image', default_value='false'),
        DeclareLaunchArgument('camera_profile', default_value='old_cam'),
        DeclareLaunchArgument('camera_name', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument(
            'camera_info_url', default_value=PROFILE_OVERRIDE_SENTINEL,
        ),
        DeclareLaunchArgument('use_rqt', default_value='false'),
        DeclareLaunchArgument('use_usb_cam', default_value='true'),
        DeclareLaunchArgument('video_device', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('image_width', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('image_height', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('pixel_format', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('framerate', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('camera_fx', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('camera_fy', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('camera_cx', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('camera_cy', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('dist_k1', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('dist_k2', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('dist_p1', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('dist_p2', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('dist_k3', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument(
            'image_qos_reliability', default_value='best_effort',
        ),
        DeclareLaunchArgument(
            'aruco_dictionary', default_value='DICT_5X5_1000',
        ),
        DeclareLaunchArgument('enable_csv_logger', default_value='true'),
        DeclareLaunchArgument(
            'csv_output_dir',
            default_value=os.path.expanduser(
                '~/project/rasip_pi_ws/log/tracking_csv'
            ),
        ),
        DeclareLaunchArgument('csv_prefix', default_value='dynamic_tracking_v3_2'),
        DeclareLaunchArgument('csv_sample_rate_hz', default_value='30.0'),
        DeclareLaunchArgument(
            'summary_csv_path',
            default_value=os.path.expanduser(
                '~/project/rasip_pi_ws/log/tracking_csv/'
                'dynamic_tracking_v3_2_summary.csv'
            ),
        ),
        DeclareLaunchArgument(
            'image_raw_timing_topic',
            default_value='/debug/image_raw_timing',
        ),
        DeclareLaunchArgument('raw_tvec_topic', default_value='/debug/tvec'),
        DeclareLaunchArgument(
            'pipeline_timing_topic',
            default_value='/debug/pipeline_timing',
        ),
        DeclareLaunchArgument('attitude_topic', default_value='/mavros/imu/data'),
        DeclareLaunchArgument('target_x', default_value='0.0'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_z', default_value='2.5'),
        DeclareLaunchArgument('target_yaw', default_value='0.0'),
        DeclareLaunchArgument('kp_xy', default_value='0.5'),
        DeclareLaunchArgument('ki_xy', default_value='0.0'),
        DeclareLaunchArgument('kd_xy', default_value='0.08'),
        DeclareLaunchArgument('kp_x', default_value='nan'),
        DeclareLaunchArgument('ki_x', default_value='nan'),
        DeclareLaunchArgument('kd_x', default_value='nan'),
        DeclareLaunchArgument('kp_y', default_value='nan'),
        DeclareLaunchArgument('ki_y', default_value='nan'),
        DeclareLaunchArgument('kd_y', default_value='nan'),
        DeclareLaunchArgument('kp_z', default_value='0.5'),
        DeclareLaunchArgument('ki_z', default_value='0.0'),
        DeclareLaunchArgument('kd_z', default_value='0.03'),
        DeclareLaunchArgument('kp_yaw', default_value='0.4'),
        DeclareLaunchArgument('ki_yaw', default_value='0.0'),
        DeclareLaunchArgument('kd_yaw', default_value='0.03'),
        DeclareLaunchArgument(
            'camera_yaw_compensation_deg', default_value='0.0',
        ),
        DeclareLaunchArgument('invert_yaw_rate_output', default_value='true'),
        DeclareLaunchArgument('v_limit', default_value='0.8'),
        DeclareLaunchArgument('vz_limit', default_value='0.3'),
        DeclareLaunchArgument('yaw_rate_limit', default_value='0.4'),
        DeclareLaunchArgument('velocity_deadband', default_value='0.03'),
        DeclareLaunchArgument('control_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('pose_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('stale_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('require_offboard', default_value='true'),
        DeclareLaunchArgument('enable_z_hold', default_value='true'),
        DeclareLaunchArgument('enable_z_yaw_smoothing', default_value='true'),
        DeclareLaunchArgument('z_lpf_tau_sec', default_value='0.35'),
        DeclareLaunchArgument('yaw_lpf_tau_sec', default_value='0.35'),
        DeclareLaunchArgument('z_error_deadband', default_value='0.04'),
        DeclareLaunchArgument('yaw_error_deadband', default_value='0.04'),
        DeclareLaunchArgument('vz_slew_rate_limit', default_value='0.30'),
        DeclareLaunchArgument('yaw_rate_slew_rate_limit', default_value='0.35'),
        tvec_launch,
        tvec_tf_node,
        controller_node,
        csv_logger_node,
    ])
