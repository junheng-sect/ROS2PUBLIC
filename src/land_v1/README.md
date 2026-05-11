# land_v1

`land_v1` 是一个独立的 `ament_python` 精准降落功能包，控制基线直接继承 `pid_tuning_v4`，落地与 `disarm` 流程参考 `landing`。

本包的核心目标是：

1. 先在 `2.5m` 高度执行 `XY` 对准。
2. 对准成功后以固定 `0.2m/s` 开始下降。
3. 下降过程中持续跟踪 ArUco。
4. 若下降中丢码，不退出 `OFFBOARD`，先保留最近一次有效 `vx/vy` 共 `0.4s`，随后将 `vx/vy` 置零。
5. 丢码期间继续依赖测距仪 / `rel_alt` 维持下降流程。
6. 判定接地后持续下压并周期请求 `disarm`，最终以 `armed=false` 作为完成条件。

## 控制语义

本包严格保持 `pid_tuning_v4` 的控制语义，不退回 `cmd_vel`：

- 输出话题：`/mavros/setpoint_raw/local`
- 坐标系：`mavros_msgs/msg/PositionTarget.FRAME_BODY_NED`
- 发布时保持：
  - `msg.velocity.x = vx_body`
  - `msg.velocity.y = -vy_body`
  - `msg.velocity.z = vz_body_flu`
  - `yaw_rate = 0.0`

`XY` 误差定义、marker 到 body 的误差旋转以及 yaw 修正公式也与 `pid_tuning_v4` 完全一致：

```text
ex_marker = target_x - pose.x
ey_marker = pose.y - target_y
yaw_rel_raw = wrap_to_pi(-pose.yaw)
yaw_rel_corrected = wrap_to_pi(yaw_rel_raw + camera_yaw_compensation_rad)
e_body = R(-yaw_rel_corrected) * e_marker
```

## 状态机

固定状态机如下：

- `WAIT_FOR_OFFBOARD`
- `ALIGN_AT_TARGET_HEIGHT`
- `DESCEND_WITH_TRACK`
- `DESCEND_RANGE_ONLY`
- `TOUCHDOWN_DISARM`
- `DONE`

各阶段说明：

- `WAIT_FOR_OFFBOARD`
  - 未进入 `OFFBOARD` 时统一输出零速，并重置 PID。
- `ALIGN_AT_TARGET_HEIGHT`
  - 使用视觉做 `XY` 对准。
  - 使用测距仪 `sensor_msgs/msg/Range` 在 `2.5m` 做高度保持。
  - 采用 `方案 B：滑动窗口稳定型` 判定“已对准，可以开始降落”。
- `DESCEND_WITH_TRACK`
  - `vz` 固定为 `-0.2m/s`。
  - `XY` 持续跟踪地标。
- `DESCEND_RANGE_ONLY`
  - 视觉超时后进入该阶段。
  - 先保留最近一次有效 `vx/vy` 共 `0.4s`，随后 `vx/vy=0`。
  - 如果测距仪与 `rel_alt` 同时失效，则暂停盲降，`vz=0`。
  - 一旦重新识别到新鲜 ArUco 数据，自动切回 `DESCEND_WITH_TRACK`。
- `TOUCHDOWN_DISARM`
  - 优先使用 `/mavros/extended_state` 的 `ON_GROUND` 判定接地。
  - 同时保留启发式兜底：高度 + 本地垂向速度 + 本地水平速度 + 持续时间。
  - 进入该阶段后持续发送最低油门下压，并按周期调用 `/mavros/cmd/arming` 请求 `disarm`。
- `DONE`
  - 检测到 `armed=false` 后进入完成态。

## 对准判据

当前默认实现使用 `方案 B：滑动窗口稳定型`，且默认阈值偏向实机放宽版：

- 最近 `1.0s` 窗口
- 至少 `75%` 样本满足阈值
- `xy` 单样本阈值：`0.18m`
- `xy` 窗口均值阈值：`0.12m`
- `z` 单样本阈值：`0.15m`
- `z` 窗口均值阈值：`0.10m`

当对准效果还不理想、存在轻微转圈或来回摆动时，可以优先调以下参数：

- `align_xy_tolerance_m`
- `align_xy_mean_tolerance_m`
- `align_window_min_ratio`
- `align_window_sec`

## 丢码策略

下降阶段的丢码恢复逻辑固定为：

1. ArUco 超时后不退出 `OFFBOARD`。
2. 进入 `DESCEND_RANGE_ONLY`。
3. 保留最近一次有效 `vx/vy` 共 `marker_loss_hold_sec`，默认 `0.4s`。
4. 超过保持时间后，水平速度置零。
5. 垂向仍尝试保持 `-0.2m/s` 下降。
6. 若测距仪与 `rel_alt` 同时失效，则暂停盲降，`vz=0.0`。
7. 重新看到新鲜 ArUco 后自动恢复 `XY` 跟踪。

## 落地判定

落地判定优先级如下：

1. `/mavros/extended_state` 的 `landed_state == ON_GROUND`
2. 启发式兜底：
   - 测距仪高度低于 `touchdown_range_threshold_m`
   - 或 `rel_alt` 低于 `touchdown_rel_alt_threshold_m`
   - `|vz_local| <= land_vz_abs_max_mps`
   - `vxy_local <= land_vxy_abs_max_mps`
   - 持续满足时间条件

## Launch

`land_v1.launch.py` 默认按树莓派实机视觉链组织，但不会重复启动 MAVROS。

默认链路：

```text
usb_cam(可选) -> /debug/tvec -> tvec_tf_node -> /debug/aruco_pose
```

默认启动命令：

```bash
ros2 launch land_v1 land_v1.launch.py
```

切换相机 profile：

```bash
ros2 launch land_v1 land_v1.launch.py \
  camera_profile:=old_cam

ros2 launch land_v1 land_v1.launch.py \
  camera_profile:=icspring_1080
```

若需要复用外部图像流、不启动 USB 相机：

```bash
ros2 launch land_v1 land_v1.launch.py \
  use_usb_cam:=false \
  ros_image_topic:=/image_raw
```

与 `pid_tuning_v4` 同风格的共享显式参数：

- `camera_profile`
- `publish_annotated_image`
- `use_usb_cam`
- `video_device`
- `ros_image_topic`
- `distance_sensor_topic`
- `csv_output_dir`
- `csv_prefix`
- `csv_sample_rate_hz`
- `stale_timeout_sec`
- `summary_csv_path`
- `target_x`
- `target_y`
- `target_z`
- `kp_xy`
- `ki_xy`
- `kd_xy`
- `kp_x`
- `ki_x`
- `kd_x`
- `kp_y`
- `ki_y`
- `kd_y`
- `kp_z`
- `ki_z`
- `kd_z`
- `camera_yaw_compensation_deg`
- `vxy_limit`
- `vx_limit`
- `vy_limit`
- `vz_limit`
- `velocity_deadband`
- `control_rate_hz`
- `pose_timeout_sec`
- `distance_sensor_timeout_sec`
- `require_offboard`

`camera_profile` 的统一配置文件位于 `src/tvec/config/camera_profiles.yaml`。后续新增相机时，优先在该文件追加 profile，而不是再分叉一套视觉代码。

`land_v1` 特有的可覆盖参数：

- `align_mode`
- `align_window_sec`
- `align_window_min_ratio`
- `align_window_min_samples`
- `align_xy_tolerance_m`
- `align_xy_mean_tolerance_m`
- `align_z_tolerance_m`
- `align_z_mean_tolerance_m`
- `descent_speed_mps`
- `marker_loss_hold_sec`
- `touchdown_range_threshold_m`
- `touchdown_rel_alt_threshold_m`
- `land_vz_abs_max_mps`
- `land_vxy_abs_max_mps`
- `land_detect_hold_sec`
- `heuristic_disarm_hold_sec`
- `min_throttle_descent_speed_mps`
- `min_throttle_disarm_duration_sec`
- `disarm_retry_interval_sec`
- `rel_alt_timeout_sec`
- `local_velocity_timeout_sec`
- `stop_cmd_after_disarm`

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
cd /home/zjh/project/rasip_pi_ws
colcon build --symlink-install --packages-select land_v1
source install/setup.bash
ros2 launch land_v1 land_v1.launch.py
```

### 最新启动命令

当前最常用的启动命令建议直接用下面这几条：

新相机实机启动：

```bash
ros2 launch land_v1 land_v1.launch.py \
  camera_profile:=icspring_1080 \
  use_rqt:=false
```

旧相机实机启动：

```bash
ros2 launch land_v1 land_v1.launch.py \
  camera_profile:=old_cam \
  use_rqt:=false
```

关闭 USB 相机、复用外部图像流：

```bash
ros2 launch land_v1 land_v1.launch.py \
  camera_profile:=icspring_1080 \
  use_usb_cam:=false \
  use_rqt:=false \
  ros_image_topic:=/image_raw
```

仅保留位姿解算，不生成标注图：

```bash
ros2 launch land_v1 land_v1.launch.py \
  camera_profile:=icspring_1080 \
  publish_annotated_image:=false \
  use_rqt:=false
```

如果外部图像来自仿真链路，通常把 `ros_image_topic` 改成 `/camera/image_raw`。

### 最小启动示例

实机默认启动：

```bash
ros2 launch land_v1 land_v1.launch.py \
  use_rqt:=false
```

关闭 USB 相机、仅复用外部图像流：

```bash
ros2 launch land_v1 land_v1.launch.py \
  use_usb_cam:=false \
  use_rqt:=false \
  ros_image_topic:=/image_raw
```

### 共享参数示例

下面这类显式传参方式与 `pid_tuning_v4` 保持一致：

```bash
ros2 launch land_v1 land_v1.launch.py \
  use_rqt:=false \
  target_z:=2.5 \
  kp_xy:=1.0 kd_xy:=0.06 \
  kp_z:=0.60 vz_limit:=0.40 \
  vxy_limit:=0.60 velocity_deadband:=0.02 \
  distance_sensor_timeout_sec:=0.5 \
  camera_yaw_compensation_deg:=0.0
```

分轴参数示例：

```bash
ros2 launch land_v1 land_v1.launch.py \
  use_rqt:=false \
  kp_x:=0.70 ki_x:=0.00 kd_x:=0.06 \
  kp_y:=0.60 ki_y:=0.00 kd_y:=0.05 \
  vx_limit:=0.70 vy_limit:=0.50 \
  distance_sensor_topic:=/mavros/hrlv_ez4_pub
```

### `land_v1` 特有参数示例

对准窗口与下降策略显式传参示例：

```bash
ros2 launch land_v1 land_v1.launch.py \
  use_rqt:=false \
  align_window_sec:=1.2 \
  align_window_min_ratio:=0.70 \
  align_xy_tolerance_m:=0.20 \
  align_xy_mean_tolerance_m:=0.14 \
  align_z_tolerance_m:=0.18 \
  align_z_mean_tolerance_m:=0.12 \
  descent_speed_mps:=0.20 \
  marker_loss_hold_sec:=0.40
```

落地与解锁策略显式传参示例：

```bash
ros2 launch land_v1 land_v1.launch.py \
  use_rqt:=false \
  touchdown_range_threshold_m:=0.10 \
  touchdown_rel_alt_threshold_m:=0.15 \
  land_vz_abs_max_mps:=0.20 \
  land_vxy_abs_max_mps:=0.25 \
  land_detect_hold_sec:=1.0 \
  heuristic_disarm_hold_sec:=3.0 \
  min_throttle_descent_speed_mps:=0.35 \
  min_throttle_disarm_duration_sec:=5.0
```

## CSV 记录

本包包含独立 logger 节点 `land_v1_csv_logger_node`，默认输出：

- 单次 CSV 目录：`/home/zjh/project/rasip_pi_ws/log/tracking_csv`
- 单次 CSV 前缀：`land_v1`
- summary CSV：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/land_v1_summary.csv`

CSV 至少记录以下关键信息：

- 时间戳、飞控 `mode/armed/connected`
- `phase`、`align_mode`、`align_ok`
- `aruco` 是否新鲜、年龄、`x/y/z/yaw`
- 测距仪高度、有效性、年龄
- `rel_alt`
- `ex_marker/ey_marker/ex_body/ey_body`
- `setpoint vx/vy/vz/yaw_rate`
- 当前是否处于 `tracking` 或 `range_only`
- 丢码累计次数、最近一次丢码时长
- 是否由 `extended_state` 判定落地
- 是否由启发式判定落地
- 是否已经发送 `disarm` 请求
- 关键 PID、限幅、滑动窗口与落地阈值参数快照

说明：

- `sp_vx/sp_vy/sp_vz` 记录的是 `PositionTarget` 消息中实际发布的字段值。
- 控制内部仍按机体系 `FLU` 理解 `vx/vy/vz`，其中发布到消息时 `y` 通道保持 `msg.velocity.y = -vy_body`。
