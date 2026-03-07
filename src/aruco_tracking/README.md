# aruco_tracking

`aruco_tracking` 在进入 `OFFBOARD` 后，读取 `/debug/aruco_pose` 的 `x/y/z/yaw`，
通过 PID 调速控制无人机飞到标志上方并正对。

## 输入与输出

- 输入位姿：`/debug/aruco_pose`（`debug_interface/msg/ArucoBasePose`）
- 飞控状态：`/mavros/state`（`mavros_msgs/msg/State`）
- 控制输出：`/mavros/setpoint_velocity/cmd_vel`（`geometry_msgs/msg/TwistStamped`）

## 控制目标

- `target_x = 0`
- `target_y = 0`
- `target_yaw = 0`

## 启动

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select aruco_tracking
source install/setup.bash
ros2 launch aruco_tracking aruco_tracking.launch.py world_name:=rover
```

## 说明

- 节点只在 `mode == OFFBOARD` 时执行 PID 控制。
- 若视觉数据超时（默认 0.5s），自动输出零速悬停。
- 默认启用动态坐标旋转：通过 `/mavros/local_position/pose` 提取 `yaw_map_base`，结合
  `/debug/aruco_pose.yaw`（`yaw_marker_base`）实时计算
  `yaw_map_marker = yaw_map_base - yaw_marker_base`，再将 marker 系 XY 误差旋转到 map 系控制。
- 若本地位姿超时（`base_pose_timeout_sec`），同样进入零速悬停；可通过
  `fallback_marker_in_map_yaw_deg` 在禁用动态模式时使用固定角回退。
- z 方向使用“相对高度保持”PID：进入 OFFBOARD 后锁定当前 `aruco_pose.z` 为 `z_ref`，随后用 `ez=z_ref-z_now` 计算 `vz`，抑制持续下沉/上浮。
- PID、速度限幅、死区参数均可通过参数调整。
- 默认开启“误差坐标旋转补偿”：将 marker 坐标系下的 XY 误差旋转到 map/ENU 后再输出速度，避免出现绕圈/圆弧轨迹。
