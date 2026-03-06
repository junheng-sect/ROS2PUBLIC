from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rover_set_pose_bridge',
        output='screen',
        arguments=['/world/rover/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
    )

    controller = Node(
        package='rover_teleop_ackermann',
        executable='ackermann_kinematic_controller_node',
        name='ackermann_kinematic_controller',
        output='screen',
    )

    auto_loop = Node(
        package='rover_auto_motion',
        executable='rover_auto_loop_node',
        name='rover_auto_loop',
        output='screen',
    )

    return LaunchDescription([
        bridge,
        controller,
        auto_loop,
    ])
