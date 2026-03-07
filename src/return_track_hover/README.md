# return_track_hover

`return_track_hover` 将返航与 ArUco 跟踪整合到同一节点，任务流程如下：

1. 进入 `OFFBOARD` 后，先将相对高度调整到 `3m`。
2. 通过 GPS 导航回 `home` 点（`/mavros/home_position/home` 的经纬度）。
3. 到达 home 水平位置后，启动 ArUco 跟踪与对正。
4. 当横向误差和 yaw 误差持续满足阈值后，进入悬停。

## 主要话题

- 订阅：`/mavros/state`（`mavros_msgs/msg/State`）
- 订阅：`/mavros/global_position/global`（`sensor_msgs/msg/NavSatFix`）
- 订阅：`/mavros/global_position/rel_alt`（`std_msgs/msg/Float64`）
- 订阅：`/mavros/home_position/home`（`mavros_msgs/msg/HomePosition`）
- 订阅：`/debug/aruco_pose`（`debug_interface/msg/ArucoBasePose`）
- 订阅：`/mavros/local_position/pose`（`geometry_msgs/msg/PoseStamped`）
- 发布：`/mavros/setpoint_velocity/cmd_vel`（`geometry_msgs/msg/TwistStamped`）

## 运行

```bash
ros2 launch return_track_hover return_track_hover.launch.py
```

## 阶段说明

- `ASCEND`：仅控制 `vz`，将相对高度调到 `target_alt_m`（默认 3m）。
- `RETURN`：通过经纬度误差换算北东米误差，控制 `vx/vy` 回到 home。
- `TRACK`：使用 `/debug/aruco_pose` 的 `x/y/yaw` 做 PID 跟踪，直到“正对且对准”稳定满足阈值。
- `HOVER`：输出零速度悬停。

## 关键参数（节选）

- `target_alt_m`：返航前目标高度，默认 `3.0`
- `home_tolerance_m`：回到 home 的水平容差，默认 `0.50`
- `xy_align_tolerance_m`：Aruco 横向对准阈值，默认 `0.20`
- `yaw_align_tolerance_deg`：Aruco 对正阈值，默认 `8.0`
- `align_hold_sec`：对准持续时间阈值，默认 `1.0`
