import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include_mavros(context):
    """根据 launch 参数选择最终使用的插件清单文件."""
    package_share = FindPackageShare('mavros_profiles').perform(context)
    mavros_share = FindPackageShare('mavros').perform(context)

    pluginlists_yaml_override = LaunchConfiguration('pluginlists_yaml').perform(context).strip()
    enable_imu = LaunchConfiguration('enable_imu').perform(context).strip().lower()

    if pluginlists_yaml_override != '':
        selected_pluginlists_yaml = pluginlists_yaml_override
    else:
        pluginlist_filename = (
            'workspace_minimal_pluginlists.yaml'
            if enable_imu not in ('false', '0')
            else 'workspace_minimal_pluginlists_no_imu.yaml'
        )
        selected_pluginlists_yaml = os.path.join(
            package_share,
            'config',
            pluginlist_filename,
        )

    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(mavros_share, 'launch', 'node.launch')
            ),
            launch_arguments={
                'fcu_url': LaunchConfiguration('fcu_url').perform(context),
                'gcs_url': LaunchConfiguration('gcs_url').perform(context),
                'tgt_system': LaunchConfiguration('tgt_system').perform(context),
                'tgt_component': LaunchConfiguration('tgt_component').perform(context),
                'pluginlists_yaml': selected_pluginlists_yaml,
                'config_yaml': LaunchConfiguration('config_yaml').perform(context),
                'log_output': LaunchConfiguration('log_output').perform(context),
                'fcu_protocol': LaunchConfiguration('fcu_protocol').perform(context),
                'respawn_mavros': LaunchConfiguration('respawn_mavros').perform(context),
                'namespace': LaunchConfiguration('namespace').perform(context),
            }.items(),
        ),
    ]


def generate_launch_description():
    mavros_share = FindPackageShare('mavros')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyS0:921600',
            description='MAVLink FCU connection URL',
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='Optional GCS bridge URL',
        ),
        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='Target system ID',
        ),
        DeclareLaunchArgument(
            'tgt_component',
            default_value='1',
            description='Target component ID',
        ),
        DeclareLaunchArgument(
            'log_output',
            default_value='screen',
            description='MAVROS log output mode',
        ),
        DeclareLaunchArgument(
            'fcu_protocol',
            default_value='v2.0',
            description='MAVLink protocol version',
        ),
        DeclareLaunchArgument(
            'respawn_mavros',
            default_value='false',
            description='Whether to respawn MAVROS after crash',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='mavros',
            description='MAVROS ROS namespace',
        ),
        DeclareLaunchArgument(
            'enable_imu',
            default_value='true',
            description='是否在最小 MAVROS 配置中启用 imu 插件',
        ),
        DeclareLaunchArgument(
            'pluginlists_yaml',
            default_value='',
            description='手动指定插件清单 YAML；非空时优先级高于 enable_imu',
        ),
        DeclareLaunchArgument(
            'config_yaml',
            default_value=PathJoinSubstitution([
                mavros_share,
                'launch',
                'px4_config.yaml',
            ]),
            description='Base MAVROS PX4 config file',
        ),
        OpaqueFunction(function=_include_mavros),
    ])
