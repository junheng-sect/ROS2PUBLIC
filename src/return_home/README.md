# return_home

`return_home` 是一个基于 GPS 的返航控制功能包，返航流程固定为：

1. 先将相对高度调整到 `3.0m`。
2. 再飞回 `home` 点的水平位置（经纬度与 home 一致）。
3. 到达后进入悬停（水平速度置零，并维持 3m 高度）。

返航任务在进入 `OFFBOARD` 模式后才会启动，`home` 点来自 `/mavros/home_position/home`。

## 话题

- 订阅：`/mavros/state`（`mavros_msgs/msg/State`）
- 订阅：`/mavros/global_position/global`（`sensor_msgs/msg/NavSatFix`）
- 订阅：`/mavros/home_position/home`（`mavros_msgs/msg/HomePosition`）
- 订阅：`/mavros/global_position/rel_alt`（`std_msgs/msg/Float64`）
- 发布：`/mavros/setpoint_velocity/cmd_vel`（`geometry_msgs/msg/TwistStamped`）

## 关键参数

- `target_alt_m`：目标相对高度，默认 `3.0`
- `alt_tolerance_m`：进入返航阶段的高度容差，默认 `0.15`
- `horizontal_tolerance_m`：判定到达 home 的水平容差，默认 `0.50`
- `require_offboard`：是否仅在 `OFFBOARD` 下控制，默认 `True`
- `start_return_on_offboard_entry`：是否在 OFFBOARD 上升沿启动返航，默认 `True`

## 运行

```bash
ros2 launch return_home return_home.launch.py
```

## 控制逻辑说明

- 节点从 `/mavros/home_position/home` 读取 `geo.latitude/longitude` 作为返航目标 `home`。
- 阶段 `ASCEND`：只输出 `vz`，将高度调到 3m。
- 阶段 `RETURN`：保持高度的同时，根据当前位置到 home 点的经纬度误差换算出北/东误差，再输出 `vx/vy` 返回。
- 阶段 `HOVER`：`vx=vy=0`，继续用高度 PID 抑制高度偏差。
