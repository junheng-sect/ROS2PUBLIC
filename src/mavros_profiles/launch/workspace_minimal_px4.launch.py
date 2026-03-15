from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('mavros_profiles')
    mavros_share = FindPackageShare('mavros')

    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    tgt_system = LaunchConfiguration('tgt_system')
    tgt_component = LaunchConfiguration('tgt_component')
    log_output = LaunchConfiguration('log_output')
    fcu_protocol = LaunchConfiguration('fcu_protocol')
    respawn_mavros = LaunchConfiguration('respawn_mavros')
    namespace = LaunchConfiguration('namespace')
    pluginlists_yaml = LaunchConfiguration('pluginlists_yaml')
    config_yaml = LaunchConfiguration('config_yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://:14540@127.0.0.1:14557',
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
            'pluginlists_yaml',
            default_value=PathJoinSubstitution([
                package_share,
                'config',
                'workspace_minimal_pluginlists.yaml',
            ]),
            description='Plugin allow/deny list for the minimal workspace profile',
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
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([mavros_share, 'launch', 'node.launch'])
            ),
            launch_arguments={
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                'tgt_system': tgt_system,
                'tgt_component': tgt_component,
                'pluginlists_yaml': pluginlists_yaml,
                'config_yaml': config_yaml,
                'log_output': log_output,
                'fcu_protocol': fcu_protocol,
                'respawn_mavros': respawn_mavros,
                'namespace': namespace,
            }.items(),
        ),
    ])
