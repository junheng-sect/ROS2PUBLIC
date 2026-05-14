from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 altitude_descent 启动描述."""
    state_topic = LaunchConfiguration('state_topic')
    distance_sensor_topic = LaunchConfiguration('distance_sensor_topic')
    setpoint_raw_topic = LaunchConfiguration('setpoint_raw_topic')
    status_topic = LaunchConfiguration('status_topic')
    esc_status_topic = LaunchConfiguration('esc_status_topic')
    esc_telemetry_topic = LaunchConfiguration('esc_telemetry_topic')
    rc_out_topic = LaunchConfiguration('rc_out_topic')

    target_descent_speed = LaunchConfiguration('target_descent_speed')
    kp_z = LaunchConfiguration('kp_z')
    ki_z = LaunchConfiguration('ki_z')
    kd_z = LaunchConfiguration('kd_z')
    pid_i_limit = LaunchConfiguration('pid_i_limit')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    distance_sensor_timeout_sec = LaunchConfiguration(
        'distance_sensor_timeout_sec'
    )
    motor_signal_timeout_sec = LaunchConfiguration('motor_signal_timeout_sec')
    vz_limit = LaunchConfiguration('vz_limit')
    velocity_deadband = LaunchConfiguration('velocity_deadband')
    min_height_stop = LaunchConfiguration('min_height_stop')
    require_offboard = LaunchConfiguration('require_offboard')
    motor_signal_source = LaunchConfiguration('motor_signal_source')
    rc_pwm_min = LaunchConfiguration('rc_pwm_min')
    rc_pwm_max = LaunchConfiguration('rc_pwm_max')
    rc_motor_channel_offset = LaunchConfiguration('rc_motor_channel_offset')
    rc_motor_channel_count = LaunchConfiguration('rc_motor_channel_count')

    enable_csv_logger = LaunchConfiguration('enable_csv_logger')
    csv_output_dir = LaunchConfiguration('csv_output_dir')
    csv_prefix = LaunchConfiguration('csv_prefix')
    csv_sample_rate_hz = LaunchConfiguration('csv_sample_rate_hz')
    stale_timeout_sec = LaunchConfiguration('stale_timeout_sec')
    flush_interval_sec = LaunchConfiguration('flush_interval_sec')

    shared_params = {
        'state_topic': state_topic,
        'distance_sensor_topic': distance_sensor_topic,
        'setpoint_raw_topic': setpoint_raw_topic,
        'status_topic': status_topic,
        'esc_status_topic': esc_status_topic,
        'esc_telemetry_topic': esc_telemetry_topic,
        'rc_out_topic': rc_out_topic,
        'target_descent_speed': target_descent_speed,
        'kp_z': kp_z,
        'ki_z': ki_z,
        'kd_z': kd_z,
        'pid_i_limit': pid_i_limit,
        'control_rate_hz': control_rate_hz,
        'distance_sensor_timeout_sec': distance_sensor_timeout_sec,
        'motor_signal_timeout_sec': motor_signal_timeout_sec,
        'vz_limit': vz_limit,
        'velocity_deadband': velocity_deadband,
        'min_height_stop': min_height_stop,
        'require_offboard': require_offboard,
        'motor_signal_source': motor_signal_source,
        'rc_pwm_min': rc_pwm_min,
        'rc_pwm_max': rc_pwm_max,
        'rc_motor_channel_offset': rc_motor_channel_offset,
        'rc_motor_channel_count': rc_motor_channel_count,
    }

    controller_node = Node(
        package='altitude_descent',
        executable='altitude_descent_node',
        name='altitude_descent_node',
        output='screen',
        parameters=[shared_params],
    )

    csv_logger_node = Node(
        package='altitude_descent',
        executable='altitude_descent_csv_logger_node',
        name='altitude_descent_csv_logger_node',
        output='screen',
        condition=IfCondition(enable_csv_logger),
        parameters=[{
            **shared_params,
            'output_dir': csv_output_dir,
            'file_prefix': csv_prefix,
            'sample_rate_hz': csv_sample_rate_hz,
            'stale_timeout_sec': stale_timeout_sec,
            'flush_interval_sec': flush_interval_sec,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('state_topic', default_value='/mavros/state'),
        DeclareLaunchArgument(
            'distance_sensor_topic',
            default_value='/mavros/hrlv_ez4_pub',
        ),
        DeclareLaunchArgument(
            'setpoint_raw_topic',
            default_value='/mavros/setpoint_raw/local',
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value='/altitude_descent/status',
        ),
        DeclareLaunchArgument(
            'esc_status_topic',
            default_value='/mavros/esc_status/status',
        ),
        DeclareLaunchArgument(
            'esc_telemetry_topic',
            default_value='/mavros/esc_telemetry/telemetry',
        ),
        DeclareLaunchArgument('rc_out_topic', default_value='/mavros/rc/out'),
        DeclareLaunchArgument('target_descent_speed', default_value='0.2'),
        DeclareLaunchArgument('kp_z', default_value='0.8'),
        DeclareLaunchArgument('ki_z', default_value='0.0'),
        DeclareLaunchArgument('kd_z', default_value='0.06'),
        DeclareLaunchArgument('pid_i_limit', default_value='1.0'),
        DeclareLaunchArgument('control_rate_hz', default_value='30.0'),
        DeclareLaunchArgument(
            'distance_sensor_timeout_sec',
            default_value='0.5',
        ),
        DeclareLaunchArgument(
            'motor_signal_timeout_sec',
            default_value='0.5',
        ),
        DeclareLaunchArgument('vz_limit', default_value='0.3'),
        DeclareLaunchArgument('velocity_deadband', default_value='0.03'),
        DeclareLaunchArgument('min_height_stop', default_value='0.20'),
        DeclareLaunchArgument('require_offboard', default_value='true'),
        DeclareLaunchArgument('motor_signal_source', default_value='auto'),
        DeclareLaunchArgument('rc_pwm_min', default_value='1000.0'),
        DeclareLaunchArgument('rc_pwm_max', default_value='2000.0'),
        DeclareLaunchArgument('rc_motor_channel_offset', default_value='0'),
        DeclareLaunchArgument('rc_motor_channel_count', default_value='4'),
        DeclareLaunchArgument('enable_csv_logger', default_value='true'),
        DeclareLaunchArgument(
            'csv_output_dir',
            default_value=(
                '/home/zjh/project/rasip_pi_ws/log/altitude_descent_csv'
            ),
        ),
        DeclareLaunchArgument('csv_prefix', default_value='altitude_descent'),
        DeclareLaunchArgument('csv_sample_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('stale_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('flush_interval_sec', default_value='1.0'),
        controller_node,
        csv_logger_node,
    ])
