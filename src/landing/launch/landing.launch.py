from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='landing',
            executable='landing_node',
            name='landing_node',
            output='screen',
            parameters=[{
                'state_topic': '/mavros/state',
                'extended_state_topic': '/mavros/extended_state',
                'rel_alt_topic': '/mavros/global_position/rel_alt',
                'vel_local_topic': '/mavros/local_position/velocity_local',
                'cmd_vel_topic': '/mavros/setpoint_velocity/cmd_vel',
                'arming_service': '/mavros/cmd/arming',
                'descent_speed_mps': 0.5,
                'control_rate_hz': 30.0,
                'require_offboard': True,
                'start_on_offboard_entry': True,
                'land_rel_alt_threshold_m': 0.15,
                'land_vz_abs_max_mps': 0.20,
                'land_vxy_abs_max_mps': 0.25,
                'land_detect_hold_sec': 1.0,
                'touchdown_descent_speed_mps': 0.15,
                'allow_heuristic_disarm_fallback': True,
                'heuristic_disarm_hold_sec': 3.0,
                'min_throttle_descent_speed_mps': 0.35,
                'min_throttle_disarm_duration_sec': 5.0,
                'disarm_retry_interval_sec': 1.0,
                'stop_cmd_after_disarm': True,
            }],
        ),
    ])
