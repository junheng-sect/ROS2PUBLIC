# offboard_test

`offboard_test` 用于做一个最小 OFFBOARD 速度控制验证：

1. 检测到进入 `OFFBOARD` 后开始计时。
2. 以 `0.5m/s` 向南飞行（ENU 下 `vy=-0.5`）。
3. 持续 `10s` 后自动切换为悬停（发布零速）。

## 话题

- 订阅：`/mavros/state`（`mavros_msgs/msg/State`）
- 发布：`/mavros/setpoint_velocity/cmd_vel`（`geometry_msgs/msg/TwistStamped`）

## 参数

- `south_speed_mps`：向南速度，默认 `0.5`
- `fly_duration_sec`：飞行时长，默认 `10.0`
- `control_rate_hz`：控制频率，默认 `30.0`
- `require_offboard`：是否仅在 OFFBOARD 下输出运动速度，默认 `True`
- `start_on_offboard_entry`：是否在 OFFBOARD 上升沿启动任务，默认 `True`

## 运行

```bash
source /opt/ros/humble/setup.bash
cd /home/zjh/project/simple_ws
source install/setup.bash
ros2 launch offboard_test offboard_test.launch.py
```

