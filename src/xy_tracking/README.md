# xy_tracking

`xy_tracking` 用于实机环境下的 ArUco 仅 XY 对准控制：

- `XY` 目标来自 `/debug/aruco_pose`
- `Z` 目标来自视觉高度 `/debug/aruco_pose.z`
- 不做 yaw 跟踪，`angular.z` 固定为 `0.0`

## 输入与输出

- 输入位姿：`/debug/aruco_pose`（`debug_interface/msg/ArucoBasePose`）
- 输入本地位姿：`/mavros/local_position/pose`（`geometry_msgs/msg/PoseStamped`）
- 输入飞控状态：`/mavros/state`（`mavros_msgs/msg/State`）
- 控制输出：`/mavros/setpoint_velocity/cmd_vel`（`geometry_msgs/msg/TwistStamped`）

## 控制逻辑

- 节点仅在 `OFFBOARD` 下输出控制指令。
- `XY` 误差先在 marker 坐标系中计算，再利用
  `yaw_map_marker = yaw_map_base - yaw_marker_base`
  旋转到 map/ENU 坐标系后进入 PID。
- `yaw_map_base` 来自 `/mavros/local_position/pose` 的姿态四元数。
- `yaw_marker_base` 来自 `/debug/aruco_pose.yaw`。
- 高度控制使用 `/debug/aruco_pose.z` 与 `target_z`
  做相对视觉高度 PID 保持，默认目标高度为 `2.5m`。
- 若 `OFFBOARD` 未进入、视觉超时或本地位姿超时，节点会输出零速并重置 PID。

## 启动

```bash
source /opt/ros/humble/setup.bash
cd /home/zjh/project/rasip_pi_ws
colcon build --symlink-install --packages-select xy_tracking
source install/setup.bash
ros2 launch xy_tracking xy_tracking.launch.py use_rqt:=false
```

默认实机链参数：

- `world_name:=rover`
- `model_name:=x500_mono_cam_down_0`
- `ros_image_topic:=/image_raw`
- `use_rqt:=false`
- `image_qos_reliability:=best_effort`
- `aruco_dictionary:=DICT_5X5_1000`

## 调参示例

```bash
ros2 launch xy_tracking xy_tracking.launch.py \
  target_z:=2.5 \
  kp_xy:=0.60 kd_xy:=0.02 \
  kp_z:=0.6 vz_limit:=0.4 \
  vxy_limit:=0.6 velocity_deadband:=0.02 \
  use_rqt:=false
```

## CSV 记录与 Summary

- 已内置 `xy_tracking_csv_logger_node`，默认随 launch 启动（`enable_csv_logger:=true`）。
- 单次运行 CSV 默认输出到：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/`
- Summary 默认输出到：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/xy_tracking_summary.csv`
- 统计口径：默认使用全部 `OFFBOARD` 且 `aruco_fresh=1` 的样本统计
  `fresh_ratio`、`max_stale_s`、`RMSE/P95` 与命令抖动。
- 其中 `z` 误差按当前控制器真实闭环对象统计，即
  `target_z - /debug/aruco_pose.z`。

关闭 CSV 记录：

```bash
ros2 launch xy_tracking xy_tracking.launch.py \
  use_rqt:=false \
  enable_csv_logger:=false
```

## 可调参数

- `target_x`、`target_y`、`target_z`
- `kp_xy`、`ki_xy`、`kd_xy`
- `kp_z`、`ki_z`、`kd_z`
- `vxy_limit`、`vz_limit`
- `velocity_deadband`
- `control_rate_hz`
- `pose_timeout_sec`
- `base_pose_timeout_sec`
- `enable_csv_logger`
- `csv_output_dir`
- `csv_prefix`
- `csv_sample_rate_hz`
- `csv_stale_timeout_sec`
- `summary_csv_path`
