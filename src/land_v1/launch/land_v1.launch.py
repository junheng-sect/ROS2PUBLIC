#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PROFILE_OVERRIDE_SENTINEL = '__profile__'


def generate_launch_description():
    # ===== 视觉链参数 =====
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

    # ===== CSV / 运行参数 =====
    enable_csv_logger = LaunchConfiguration('enable_csv_logger')
    csv_output_dir = LaunchConfiguration('csv_output_dir')
    csv_prefix = LaunchConfiguration('csv_prefix')
    csv_sample_rate_hz = LaunchConfiguration('csv_sample_rate_hz')
    stale_timeout_sec = LaunchConfiguration('stale_timeout_sec')
    flush_interval_sec = LaunchConfiguration('flush_interval_sec')
    summary_csv_path = LaunchConfiguration('summary_csv_path')

    # ===== 话题参数 =====
    pose_topic = LaunchConfiguration('pose_topic')
    raw_tvec_topic = LaunchConfiguration('raw_tvec_topic')
    state_topic = LaunchConfiguration('state_topic')
    extended_state_topic = LaunchConfiguration('extended_state_topic')
    distance_sensor_topic = LaunchConfiguration('distance_sensor_topic')
    rel_alt_topic = LaunchConfiguration('rel_alt_topic')
    vel_local_topic = LaunchConfiguration('vel_local_topic')
    setpoint_raw_topic = LaunchConfiguration('setpoint_raw_topic')
    arming_service = LaunchConfiguration('arming_service')
    status_topic = LaunchConfiguration('status_topic')

    # ===== 控制参数 =====
    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    target_z = LaunchConfiguration('target_z')
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
    camera_yaw_compensation_deg = LaunchConfiguration(
        'camera_yaw_compensation_deg'
    )

    align_mode = LaunchConfiguration('align_mode')
    align_window_sec = LaunchConfiguration('align_window_sec')
    align_window_min_ratio = LaunchConfiguration('align_window_min_ratio')
    align_window_min_samples = LaunchConfiguration('align_window_min_samples')
    align_xy_tolerance_m = LaunchConfiguration('align_xy_tolerance_m')
    align_xy_mean_tolerance_m = LaunchConfiguration('align_xy_mean_tolerance_m')
    align_z_tolerance_m = LaunchConfiguration('align_z_tolerance_m')
    align_z_mean_tolerance_m = LaunchConfiguration('align_z_mean_tolerance_m')

    descent_speed_mps = LaunchConfiguration('descent_speed_mps')
    marker_loss_hold_sec = LaunchConfiguration('marker_loss_hold_sec')
    touchdown_range_threshold_m = LaunchConfiguration('touchdown_range_threshold_m')
    touchdown_rel_alt_threshold_m = LaunchConfiguration('touchdown_rel_alt_threshold_m')
    land_vz_abs_max_mps = LaunchConfiguration('land_vz_abs_max_mps')
    land_vxy_abs_max_mps = LaunchConfiguration('land_vxy_abs_max_mps')
    land_detect_hold_sec = LaunchConfiguration('land_detect_hold_sec')
    heuristic_disarm_hold_sec = LaunchConfiguration('heuristic_disarm_hold_sec')
    min_throttle_descent_speed_mps = LaunchConfiguration(
        'min_throttle_descent_speed_mps'
    )
    min_throttle_disarm_duration_sec = LaunchConfiguration(
        'min_throttle_disarm_duration_sec'
    )
    disarm_retry_interval_sec = LaunchConfiguration('disarm_retry_interval_sec')

    control_rate_hz = LaunchConfiguration('control_rate_hz')
    pose_timeout_sec = LaunchConfiguration('pose_timeout_sec')
    distance_sensor_timeout_sec = LaunchConfiguration('distance_sensor_timeout_sec')
    rel_alt_timeout_sec = LaunchConfiguration('rel_alt_timeout_sec')
    local_velocity_timeout_sec = LaunchConfiguration('local_velocity_timeout_sec')
    vxy_limit = LaunchConfiguration('vxy_limit')
    vx_limit = LaunchConfiguration('vx_limit')
    vy_limit = LaunchConfiguration('vy_limit')
    vz_limit = LaunchConfiguration('vz_limit')
    velocity_deadband = LaunchConfiguration('velocity_deadband')
    require_offboard = LaunchConfiguration('require_offboard')
    stop_cmd_after_disarm = LaunchConfiguration('stop_cmd_after_disarm')

    # ===== 复用现有视觉主链 =====
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
            'input_topic': raw_tvec_topic,
            'parent_frame': 'arucomarker',
            'child_frame': 'vision_pose',
        }],
    )

    controller_node = Node(
        package='land_v1',
        executable='land_v1_node',
        name='land_v1_node',
        output='screen',
        parameters=[{
            'pose_topic': pose_topic,
            'state_topic': state_topic,
            'extended_state_topic': extended_state_topic,
            'distance_sensor_topic': distance_sensor_topic,
            'rel_alt_topic': rel_alt_topic,
            'vel_local_topic': vel_local_topic,
            'setpoint_raw_topic': setpoint_raw_topic,
            'arming_service': arming_service,
            'status_topic': status_topic,
            'target_x': target_x,
            'target_y': target_y,
            'target_z': target_z,
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
            'camera_yaw_compensation_deg': camera_yaw_compensation_deg,
            'align_mode': align_mode,
            'align_window_sec': align_window_sec,
            'align_window_min_ratio': align_window_min_ratio,
            'align_window_min_samples': align_window_min_samples,
            'align_xy_tolerance_m': align_xy_tolerance_m,
            'align_xy_mean_tolerance_m': align_xy_mean_tolerance_m,
            'align_z_tolerance_m': align_z_tolerance_m,
            'align_z_mean_tolerance_m': align_z_mean_tolerance_m,
            'descent_speed_mps': descent_speed_mps,
            'marker_loss_hold_sec': marker_loss_hold_sec,
            'touchdown_range_threshold_m': touchdown_range_threshold_m,
            'touchdown_rel_alt_threshold_m': touchdown_rel_alt_threshold_m,
            'land_vz_abs_max_mps': land_vz_abs_max_mps,
            'land_vxy_abs_max_mps': land_vxy_abs_max_mps,
            'land_detect_hold_sec': land_detect_hold_sec,
            'heuristic_disarm_hold_sec': heuristic_disarm_hold_sec,
            'min_throttle_descent_speed_mps': min_throttle_descent_speed_mps,
            'min_throttle_disarm_duration_sec': min_throttle_disarm_duration_sec,
            'disarm_retry_interval_sec': disarm_retry_interval_sec,
            'control_rate_hz': control_rate_hz,
            'pose_timeout_sec': pose_timeout_sec,
            'distance_sensor_timeout_sec': distance_sensor_timeout_sec,
            'rel_alt_timeout_sec': rel_alt_timeout_sec,
            'local_velocity_timeout_sec': local_velocity_timeout_sec,
            'vxy_limit': vxy_limit,
            'vx_limit': vx_limit,
            'vy_limit': vy_limit,
            'vz_limit': vz_limit,
            'velocity_deadband': velocity_deadband,
            'require_offboard': require_offboard,
            'stop_cmd_after_disarm': stop_cmd_after_disarm,
        }],
    )

    csv_logger_node = Node(
        package='land_v1',
        executable='land_v1_csv_logger_node',
        name='land_v1_csv_logger_node',
        output='screen',
        condition=IfCondition(enable_csv_logger),
        parameters=[{
            'pose_topic': pose_topic,
            'state_topic': state_topic,
            'extended_state_topic': extended_state_topic,
            'distance_sensor_topic': distance_sensor_topic,
            'rel_alt_topic': rel_alt_topic,
            'vel_local_topic': vel_local_topic,
            'setpoint_raw_topic': setpoint_raw_topic,
            'status_topic': status_topic,
            'output_dir': csv_output_dir,
            'file_prefix': csv_prefix,
            'sample_rate_hz': csv_sample_rate_hz,
            'stale_timeout_sec': stale_timeout_sec,
            'flush_interval_sec': flush_interval_sec,
            'summary_csv_path': summary_csv_path,
            'target_x': target_x,
            'target_y': target_y,
            'target_z': target_z,
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
            'camera_yaw_compensation_deg': camera_yaw_compensation_deg,
            'align_mode': align_mode,
            'align_window_sec': align_window_sec,
            'align_window_min_ratio': align_window_min_ratio,
            'align_window_min_samples': align_window_min_samples,
            'align_xy_tolerance_m': align_xy_tolerance_m,
            'align_xy_mean_tolerance_m': align_xy_mean_tolerance_m,
            'align_z_tolerance_m': align_z_tolerance_m,
            'align_z_mean_tolerance_m': align_z_mean_tolerance_m,
            'descent_speed_mps': descent_speed_mps,
            'marker_loss_hold_sec': marker_loss_hold_sec,
            'touchdown_range_threshold_m': touchdown_range_threshold_m,
            'touchdown_rel_alt_threshold_m': touchdown_rel_alt_threshold_m,
            'land_vz_abs_max_mps': land_vz_abs_max_mps,
            'land_vxy_abs_max_mps': land_vxy_abs_max_mps,
            'land_detect_hold_sec': land_detect_hold_sec,
            'heuristic_disarm_hold_sec': heuristic_disarm_hold_sec,
            'min_throttle_descent_speed_mps': min_throttle_descent_speed_mps,
            'min_throttle_disarm_duration_sec': min_throttle_disarm_duration_sec,
            'disarm_retry_interval_sec': disarm_retry_interval_sec,
            'control_rate_hz': control_rate_hz,
            'pose_timeout_sec': pose_timeout_sec,
            'distance_sensor_timeout_sec': distance_sensor_timeout_sec,
            'rel_alt_timeout_sec': rel_alt_timeout_sec,
            'local_velocity_timeout_sec': local_velocity_timeout_sec,
            'vxy_limit': vxy_limit,
            'vx_limit': vx_limit,
            'vy_limit': vy_limit,
            'vz_limit': vz_limit,
            'velocity_deadband': velocity_deadband,
            'require_offboard': require_offboard,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='rover'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('annotated_image_topic', default_value='/tvec/image_annotated'),
        DeclareLaunchArgument('publish_annotated_image', default_value='true'),
        DeclareLaunchArgument('camera_profile', default_value='old_cam'),
        DeclareLaunchArgument('camera_name', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=PROFILE_OVERRIDE_SENTINEL,
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
        DeclareLaunchArgument('image_qos_reliability', default_value='best_effort'),
        DeclareLaunchArgument('aruco_dictionary', default_value='DICT_5X5_1000'),
        DeclareLaunchArgument('enable_csv_logger', default_value='true'),
        DeclareLaunchArgument(
            'csv_output_dir',
            default_value=os.path.expanduser('~/project/rasip_pi_ws/log/tracking_csv'),
        ),
        DeclareLaunchArgument('csv_prefix', default_value='land_v1'),
        DeclareLaunchArgument('csv_sample_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('stale_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('flush_interval_sec', default_value='1.0'),
        DeclareLaunchArgument(
            'summary_csv_path',
            default_value=os.path.expanduser(
                '~/project/rasip_pi_ws/log/tracking_csv/land_v1_summary.csv'
            ),
        ),
        DeclareLaunchArgument('pose_topic', default_value='/debug/aruco_pose'),
        DeclareLaunchArgument('raw_tvec_topic', default_value='/debug/tvec'),
        DeclareLaunchArgument('state_topic', default_value='/mavros/state'),
        DeclareLaunchArgument(
            'extended_state_topic',
            default_value='/mavros/extended_state',
        ),
        DeclareLaunchArgument(
            'distance_sensor_topic',
            default_value='/mavros/hrlv_ez4_pub',
        ),
        DeclareLaunchArgument(
            'rel_alt_topic',
            default_value='/mavros/global_position/rel_alt',
        ),
        DeclareLaunchArgument(
            'vel_local_topic',
            default_value='/mavros/local_position/velocity_local',
        ),
        DeclareLaunchArgument(
            'setpoint_raw_topic',
            default_value='/mavros/setpoint_raw/local',
        ),
        DeclareLaunchArgument('arming_service', default_value='/mavros/cmd/arming'),
        DeclareLaunchArgument('status_topic', default_value='/land_v1/status'),
        DeclareLaunchArgument('target_x', default_value='0.0'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_z', default_value='2.5'),
        DeclareLaunchArgument('kp_xy', default_value='0.5'),
        DeclareLaunchArgument('ki_xy', default_value='0.0'),
        DeclareLaunchArgument('kd_xy', default_value='0.08'),
        DeclareLaunchArgument('kp_x', default_value='nan'),
        DeclareLaunchArgument('ki_x', default_value='nan'),
        DeclareLaunchArgument('kd_x', default_value='nan'),
        DeclareLaunchArgument('kp_y', default_value='nan'),
        DeclareLaunchArgument('ki_y', default_value='nan'),
        DeclareLaunchArgument('kd_y', default_value='nan'),
        DeclareLaunchArgument('kp_z', default_value='0.8'),
        DeclareLaunchArgument('ki_z', default_value='0.0'),
        DeclareLaunchArgument('kd_z', default_value='0.06'),
        DeclareLaunchArgument('camera_yaw_compensation_deg', default_value='0.0'),
        DeclareLaunchArgument('align_mode', default_value='sliding_window'),
        DeclareLaunchArgument('align_window_sec', default_value='1.0'),
        DeclareLaunchArgument('align_window_min_ratio', default_value='0.75'),
        DeclareLaunchArgument('align_window_min_samples', default_value='12'),
        DeclareLaunchArgument('align_xy_tolerance_m', default_value='0.18'),
        DeclareLaunchArgument('align_xy_mean_tolerance_m', default_value='0.12'),
        DeclareLaunchArgument('align_z_tolerance_m', default_value='0.15'),
        DeclareLaunchArgument('align_z_mean_tolerance_m', default_value='0.10'),
        DeclareLaunchArgument('descent_speed_mps', default_value='0.2'),
        DeclareLaunchArgument('marker_loss_hold_sec', default_value='0.4'),
        DeclareLaunchArgument('touchdown_range_threshold_m', default_value='0.10'),
        DeclareLaunchArgument('touchdown_rel_alt_threshold_m', default_value='0.15'),
        DeclareLaunchArgument('land_vz_abs_max_mps', default_value='0.20'),
        DeclareLaunchArgument('land_vxy_abs_max_mps', default_value='0.25'),
        DeclareLaunchArgument('land_detect_hold_sec', default_value='1.0'),
        DeclareLaunchArgument('heuristic_disarm_hold_sec', default_value='3.0'),
        DeclareLaunchArgument('min_throttle_descent_speed_mps', default_value='0.35'),
        DeclareLaunchArgument('min_throttle_disarm_duration_sec', default_value='5.0'),
        DeclareLaunchArgument('disarm_retry_interval_sec', default_value='1.0'),
        DeclareLaunchArgument('control_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('pose_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('distance_sensor_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('rel_alt_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('local_velocity_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('vxy_limit', default_value='0.8'),
        DeclareLaunchArgument('vx_limit', default_value='nan'),
        DeclareLaunchArgument('vy_limit', default_value='nan'),
        DeclareLaunchArgument('vz_limit', default_value='0.5'),
        DeclareLaunchArgument('velocity_deadband', default_value='0.03'),
        DeclareLaunchArgument('require_offboard', default_value='true'),
        DeclareLaunchArgument('stop_cmd_after_disarm', default_value='true'),
        tvec_launch,
        tvec_tf_node,
        controller_node,
        csv_logger_node,
    ])
