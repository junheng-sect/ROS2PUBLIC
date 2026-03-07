from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='return_home',
            executable='return_home_node',
            name='return_home_node',
            output='screen',
            parameters=[{
                'state_topic': '/mavros/state',
                'global_topic': '/mavros/global_position/global',
                'home_topic': '/mavros/home_position/home',
                'rel_alt_topic': '/mavros/global_position/rel_alt',
                'cmd_vel_topic': '/mavros/setpoint_velocity/cmd_vel',
                'target_alt_m': 3.0,
                'alt_tolerance_m': 0.15,
                'horizontal_tolerance_m': 0.50,
                'kp_alt': 0.8,
                'ki_alt': 0.0,
                'kd_alt': 0.05,
                'kp_xy': 0.35,
                'ki_xy': 0.0,
                'kd_xy': 0.08,
                'max_vz': 0.8,
                'max_vxy': 1.0,
                'control_rate_hz': 20.0,
                'require_offboard': True,
                'start_return_on_offboard_entry': True,
            }],
        ),
    ])
