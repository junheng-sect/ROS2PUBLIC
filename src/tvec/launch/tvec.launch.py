#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tvec.camera_profiles import (
    PROFILE_OVERRIDE_SENTINEL,
    get_camera_profile,
    get_default_camera_profile_name,
    list_camera_profiles,
)


def _resolve_profile_value(context, argument_name, profile_value, caster):
    """若 launch 参数未显式覆盖，则回退到 profile 中的配置值。"""
    raw_value = LaunchConfiguration(argument_name).perform(context).strip()
    if raw_value in ('', PROFILE_OVERRIDE_SENTINEL):
        return profile_value
    return caster(raw_value)


def _launch_flag_enabled(context, argument_name):
    """统一解析 launch 布尔参数，兼容 true/false、1/0 等常见写法。"""
    raw_value = LaunchConfiguration(argument_name).perform(context).strip().lower()
    return raw_value in ('1', 'true', 'yes', 'on')


def _resolve_video_device_path(video_device):
    """将 by-id 等软链接解析为真实设备路径，避免 usb_cam 误解析相对链接。"""
    device_path = Path(str(video_device).strip())
    try:
        if device_path.exists():
            return str(device_path.resolve(strict=True))
    except OSError:
        pass
    return str(video_device)


def _resolve_camera_settings(context):
    """读取选中的相机 profile，并应用 launch 级显式覆盖。"""
    camera_profile = LaunchConfiguration('camera_profile').perform(context).strip()
    if not camera_profile:
        camera_profile = get_default_camera_profile_name()

    available_profiles = list_camera_profiles()
    if camera_profile not in available_profiles:
        raise RuntimeError(
            f'未知 camera_profile={camera_profile}，可选值：{", ".join(available_profiles)}'
        )

    profile = get_camera_profile(camera_profile)
    return {
        'camera_profile': camera_profile,
        'camera_name': _resolve_profile_value(
            context, 'camera_name', profile['camera_name'], str
        ),
        'camera_info_url': _resolve_profile_value(
            context, 'camera_info_url', profile['camera_info_url'], str
        ),
        'video_device': _resolve_video_device_path(
            _resolve_profile_value(
                context, 'video_device', profile['video_device'], str
            )
        ),
        'image_width': _resolve_profile_value(
            context, 'image_width', profile['image_width'], int
        ),
        'image_height': _resolve_profile_value(
            context, 'image_height', profile['image_height'], int
        ),
        'pixel_format': _resolve_profile_value(
            context, 'pixel_format', profile['pixel_format'], str
        ),
        'framerate': _resolve_profile_value(
            context, 'framerate', profile['framerate'], float
        ),
        'camera_fx': _resolve_profile_value(
            context, 'camera_fx', profile['camera_fx'], float
        ),
        'camera_fy': _resolve_profile_value(
            context, 'camera_fy', profile['camera_fy'], float
        ),
        'camera_cx': _resolve_profile_value(
            context, 'camera_cx', profile['camera_cx'], float
        ),
        'camera_cy': _resolve_profile_value(
            context, 'camera_cy', profile['camera_cy'], float
        ),
        'dist_k1': _resolve_profile_value(
            context, 'dist_k1', profile['dist_k1'], float
        ),
        'dist_k2': _resolve_profile_value(
            context, 'dist_k2', profile['dist_k2'], float
        ),
        'dist_p1': _resolve_profile_value(
            context, 'dist_p1', profile['dist_p1'], float
        ),
        'dist_p2': _resolve_profile_value(
            context, 'dist_p2', profile['dist_p2'], float
        ),
        'dist_k3': _resolve_profile_value(
            context, 'dist_k3', profile['dist_k3'], float
        ),
    }


def _launch_setup(context, *args, **kwargs):
    # 保留 world/model 参数仅用于接口兼容；实机模式不再使用 GZ 图像。
    _ = LaunchConfiguration('world_name').perform(context)
    _ = LaunchConfiguration('model_name').perform(context)
    ros_image_topic = LaunchConfiguration('ros_image_topic').perform(context)
    camera_settings = _resolve_camera_settings(context)
    publish_annotated_image = _launch_flag_enabled(context, 'publish_annotated_image')
    use_rqt = _launch_flag_enabled(context, 'use_rqt')
    enable_image_timing_probe = _launch_flag_enabled(
        context,
        'enable_image_timing_probe',
    )

    # 实机默认使用 USB 相机发布图像。
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_usb_cam')),
        parameters=[{
            'video_device': camera_settings['video_device'],
            'image_width': camera_settings['image_width'],
            'image_height': camera_settings['image_height'],
            'pixel_format': camera_settings['pixel_format'],
            'framerate': camera_settings['framerate'],
            'camera_frame_id': 'camera_link',
            'camera_name': camera_settings['camera_name'],
            'camera_info_url': camera_settings['camera_info_url'],
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
            'image_qos_reliability': LaunchConfiguration('image_qos_reliability'),
            'aruco_dictionary': LaunchConfiguration('aruco_dictionary'),
            'camera_profile': camera_settings['camera_profile'],
            'camera_fx': camera_settings['camera_fx'],
            'camera_fy': camera_settings['camera_fy'],
            'camera_cx': camera_settings['camera_cx'],
            'camera_cy': camera_settings['camera_cy'],
            'dist_k1': camera_settings['dist_k1'],
            'dist_k2': camera_settings['dist_k2'],
            'dist_p1': camera_settings['dist_p1'],
            'dist_p2': camera_settings['dist_p2'],
            'dist_k3': camera_settings['dist_k3'],
            'marker_size_33': 0.193,
            'marker_size_42': 0.025,
            'publish_annotated_image': publish_annotated_image,
        }],
    )

    image_timing_probe_node = Node(
        package='tvec',
        executable='image_raw_timing_probe_node',
        name='image_raw_timing_probe_node',
        output='screen',
        parameters=[{
            'image_topic': ros_image_topic,
            'image_qos_reliability': LaunchConfiguration('image_qos_reliability'),
            'timing_topic': '/debug/image_raw_timing',
        }],
    )

    # 用 rqt_image_view 显示 ArUco 标注后的图像。
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view_tvec',
        output='screen',
        # rqt_image_view 默认订阅 /image，这里用 remap 固定到标注图像话题，
        # 避免仅靠命令行参数导致“窗口打开但未订阅图像”。
        remappings=[('/image', LaunchConfiguration('annotated_image_topic'))],
    )

    launch_actions = [usb_cam_node, detector_node]
    if enable_image_timing_probe:
        launch_actions.append(image_timing_probe_node)
    if use_rqt and publish_annotated_image:
        launch_actions.append(image_view_node)

    return launch_actions


def generate_launch_description():
    default_camera_profile = get_default_camera_profile_name()
    camera_profiles_text = ', '.join(list_camera_profiles())
    return LaunchDescription([
        # 上游 launch 仍会透传这两个参数，这里保留声明避免参数不匹配报错。
        DeclareLaunchArgument('world_name', default_value='aruco'),
        DeclareLaunchArgument('model_name', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('ros_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('annotated_image_topic', default_value='/tvec/image_annotated'),
        DeclareLaunchArgument('publish_annotated_image', default_value='true'),
        DeclareLaunchArgument(
            'camera_profile',
            default_value=default_camera_profile,
            description=f'相机 profile，当前可选：{camera_profiles_text}',
        ),
        DeclareLaunchArgument('use_usb_cam', default_value='true'),
        DeclareLaunchArgument('camera_name', default_value=PROFILE_OVERRIDE_SENTINEL),
        DeclareLaunchArgument('camera_info_url', default_value=PROFILE_OVERRIDE_SENTINEL),
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
        DeclareLaunchArgument('aruco_dictionary', default_value='DICT_5X5_1000'),
        # 默认使用 best_effort，避免与 usb_cam 的 QoS 不匹配导致无法收图。
        DeclareLaunchArgument('image_qos_reliability', default_value='best_effort'),
        DeclareLaunchArgument('enable_image_timing_probe', default_value='true'),
        DeclareLaunchArgument('use_rqt', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
