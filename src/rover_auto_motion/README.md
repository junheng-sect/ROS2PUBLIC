# rover_auto_motion

自动运动功能包（ROS 2 Jazzy, `ament_python`）：
- 前进 `1m`
- 右转 `90deg`（转弯半径 `1m`）
- 循环执行

## 启动

先启动仿真世界 `rover`，然后：

```bash
cd ~/project/project_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rover_auto_motion rover_auto_motion.launch.py
```

说明：
- 本包通过 `gz service /world/rover/set_pose` 直接驱动 Gazebo 中的 `rover` 模型位姿。
- 不依赖 `rover_teleop_ackermann`，也不依赖 `ros_gz_bridge` 服务桥接。

## 参数

- `linear_speed`：默认 `0.30` m/s
- `straight_distance_m`：默认 `1.0` m
- `turn_radius_m`：默认 `1.0` m
- `turn_angle_deg`：默认 `90.0` deg
- `publish_rate_hz`：默认 `30.0` Hz（位姿更新频率）
- `cmd_smoothing_tau_sec`：默认 `0.18` s（速度/角速度一阶平滑时间常数）
- `entity_name`：默认 `rover`
- `set_pose_service`：默认 `/world/rover/set_pose`（Gazebo Transport 服务）
- `z_height_m`：默认 `0.0` m

说明：右转角速度由 `yaw_rate = -v / radius` 自动计算。
