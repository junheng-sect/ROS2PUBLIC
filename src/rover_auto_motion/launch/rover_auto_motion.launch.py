from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    auto_loop = Node(
        package='rover_auto_motion',
        executable='rover_auto_loop_node',
        name='rover_auto_loop',
        output='screen',
    )

    return LaunchDescription([
        auto_loop,
    ])
