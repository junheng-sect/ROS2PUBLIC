#!/usr/bin/env python3
"""
MAVROS PX4 启动文件

功能：
- 启动 MAVROS 节点，连接 PX4 飞控
- 默认使用 UDP 连接 SITL 仿真

使用方式：
    ros2 launch tf_display mavros_px4.launch.py
    ros2 launch tf_display mavros_px4.launch.py fcu_url:=udp://:14540@
    ros2 launch tf_display mavros_px4.launch.py fcu_url:=/dev/ttyACM0:57600
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成启动描述"""

    # ========== 声明参数 ==========
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@',
        description='FCU 连接 URL（UDP 用于 SITL 仿真，串口用于真实飞控）'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='GCS 连接 URL'
    )

    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='1',
        description='目标系统 ID'
    )

    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='1',
        description='目标组件 ID'
    )

    log_output_arg = DeclareLaunchArgument(
        'log_output',
        default_value='screen',
        description='日志输出方式'
    )

    fcu_protocol_arg = DeclareLaunchArgument(
        'fcu_protocol',
        default_value='v2.0',
        description='MAVLink 协议版本'
    )

    respawn_mavros_arg = DeclareLaunchArgument(
        'respawn_mavros',
        default_value='false',
        description='是否自动重启 MAVROS'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='mavros',
        description='节点命名空间'
    )

    # ========== 获取 MAVROS 配置路径 ==========
    mavros_share = get_package_share_directory('mavros')

    # ========== MAVROS 节点 ==========
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            # PX4 插件列表配置
            os.path.join(mavros_share, 'launch', 'px4_pluginlists.yaml'),
            # PX4 插件参数配置
            os.path.join(mavros_share, 'launch', 'px4_config.yaml'),
            # 连接参数
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'tgt_system': LaunchConfiguration('tgt_system'),
                'tgt_component': LaunchConfiguration('tgt_component'),
                'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            }
        ],
        output=LaunchConfiguration('log_output'),
        respawn=LaunchConfiguration('respawn_mavros'),
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        log_output_arg,
        fcu_protocol_arg,
        respawn_mavros_arg,
        namespace_arg,
        mavros_node,
    ])
