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

## 参数

- `linear_speed`：默认 `0.30` m/s
- `straight_distance_m`：默认 `1.0` m
- `turn_radius_m`：默认 `1.0` m
- `turn_angle_deg`：默认 `90.0` deg
- `wheelbase_m`：默认 `0.45` m

说明：右转控制量由 `delta = -atan(wheelbase / radius)` 自动计算。
