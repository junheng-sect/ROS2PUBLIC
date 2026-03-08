from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard_test',
            executable='offboard_test_node',
            name='offboard_test_node',
            output='screen',
            parameters=[{
                'state_topic': '/mavros/state',
                'cmd_vel_topic': '/mavros/setpoint_velocity/cmd_vel',
                'south_speed_mps': 0.5,
                'fly_duration_sec': 10.0,
                'control_rate_hz': 30.0,
                'require_offboard': True,
                'start_on_offboard_entry': True,
            }],
        ),
    ])

