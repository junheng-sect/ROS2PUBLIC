# rover_teleop_ackermann

ROS 2 Jazzy package for keyboard-based Ackermann-like teleoperation of the Gazebo `rover` model.

## Features

- WASD teleop (`W/A/S/D` only)
- Publishes Ackermann command on `/rover/ackermann_cmd` (`geometry_msgs/msg/Twist`)
- Kinematic bicycle model controller
- Drives Gazebo model pose through `/world/rover/set_pose` (`ros_gz_interfaces/srv/SetEntityPose`)

## Keyboard Mapping

- `W`: forward at `+v_max`
- `S`: backward at `-v_max`
- `A`: steering `+steer_max`
- `D`: steering `-steer_max`
- `SPACE`: immediate stop and steering reset
- Key release: speed and steering auto return to zero

## Parameters

### `keyboard_arrow_teleop_node`

- `cmd_topic` (default: `/rover/ackermann_cmd`)
- `publish_rate_hz` (default: `40.0`)
- `key_timeout_sec` (default: `0.90`)
- `v_max` (default: `0.6` m/s)
- `steer_max_deg` (default: `25.0` deg)

### `ackermann_kinematic_controller_node`

- `cmd_topic` (default: `/rover/ackermann_cmd`)
- `world_name` (default: `rover`)
- `model_name` (default: `rover`)
- `control_rate_hz` (default: `30.0`)
- `wheelbase` (default: `0.45` m)
- `v_max` (default: `0.6` m/s)
- `steer_max_deg` (default: `25.0` deg)
- `cmd_timeout_sec` (default: `0.3`)
- `init_x` (default: `0.0`)
- `init_y` (default: `0.0`)
- `init_yaw_deg` (default: `0.0`)
- `z_fixed` (default: `0.15`)

## Run

Start simulation first:

```bash
cd ~/PX4_Firmware
PX4_GZ_WORLD=rover PX4_GZ_MODEL_POSE="0,0,0.65,0,0,0" make px4_sitl gz_x500_mono_cam_down
```

Then launch teleop:

```bash
cd ~/project/project_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rover_teleop_ackermann rover_teleop_ackermann.launch.py
```

## Troubleshooting

- If keyboard has no effect, make sure the launch terminal has focus and is a TTY.
- If controller warns `SetEntityPose service is not ready`, check Gazebo world is running and bridge node is alive.
- If model does not move, verify target model name is `rover` and world is `rover`.
