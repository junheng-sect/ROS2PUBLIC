# dynamic_tracking_v2

`dynamic_tracking_v2` 是基于 `src/pid_tuning_v6` 完整复制并独立演进出来的 `ament_python` 功能包，不直接修改 `pid_tuning_v6`。它保留了 `pid_tuning_v6` 的 XY 主控制链、`FRAME_BODY_NED` 发布方式、`msg.velocity.y = -vy_body` 口径，以及 `v_limit` 的水平速度向量限幅逻辑，同时把 Z 改回视觉 `aruco_z`，并新增 yaw PID 跟踪。

## 与 pid_tuning_v6 的核心差异

- XY 主控逻辑不变：继续沿用 `ex_marker / ey_marker`、`yaw_rel_raw = wrap_to_pi(-pose.yaw)`、`yaw_rel_corrected = wrap_to_pi(yaw_rel_raw + camera_yaw_compensation_rad)`、`rotate_marker_error_to_body()`、`FRAME_BODY_NED`、`msg.velocity.y = -vy_body`，以及 `v_limit` 的水平速度向量限幅。
- Z 高度来源改回视觉：不再使用测距仪，统一使用 `ez = target_z - aruco_z`，其中 `aruco_z = /debug/aruco_pose.z`。
- 新增 yaw PID 跟踪：yaw 误差统一按 `eyaw = wrap_to_pi(target_yaw - yaw_rel_corrected)` 计算，输出为 `msg.yaw_rate`，不发布绝对 yaw 角。
- 公开 yaw 限幅参数统一为 `yaw_rate_limit`。
- 控制优先级明确为“XY 主控，Z 和 yaw 次要”：XY 默认值沿用 `pid_tuning_v6`，而 Z / yaw 默认参数更保守。

## 默认参数

- XY：
  - `kp_xy=0.5`、`ki_xy=0.0`、`kd_xy=0.08`
  - `v_limit=0.8`
- Z：
  - `kp_z=0.5`、`ki_z=0.0`、`kd_z=0.03`
  - `vz_limit=0.3`
- yaw：
  - `kp_yaw=0.4`、`ki_yaw=0.0`、`kd_yaw=0.03`
  - `yaw_rate_limit=0.4`

`velocity_deadband` 继续只作用于 XY 与 Z，本版不新增公开 yaw deadband 参数。

## 输入与输出

- 输入视觉位姿：`/debug/aruco_pose`
- 输入飞控状态：`/mavros/state`
- CSV logger 额外订阅：
  - `/debug/tvec`
  - `/mavros/local_position/pose`
  - `/mavros/imu/data`
- 控制输出：`/mavros/setpoint_raw/local`

## 启动链路

`dynamic_tracking_v2.launch.py` 继续复用当前工作空间里的实机视觉主链：

```text
usb_cam(可选) -> /debug/tvec -> tvec_tf_node -> /debug/aruco_pose
```

launch 会启动：

- `tvec.launch.py`
- `tvec_tf_node`
- `dynamic_tracking_v2_node`
- `dynamic_tracking_v2_csv_logger_node`

## CSV 记录说明

CSV logger 保留以下关键观测列：

- `aruco_yaw_rad`
- `yaw_rel_raw_rad`
- `yaw_rel_corrected_rad`
- `sp_yaw_rate`
- `ez`
- `eyaw`

参数快照与对应列新增：

- `target_yaw`
- `kp_yaw`
- `ki_yaw`
- `kd_yaw`
- `yaw_rate_limit`

已删除全部测距仪相关订阅、参数快照与 CSV 列，不再保留测距仪话题、超时、高度有效性与 `z_source_used` 口径。

默认日志路径：

- 单次 CSV 默认目录：`/home/zjh/project/rasip_pi_ws/log/tracking_csv`
- summary CSV 默认路径：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/dynamic_tracking_v2_summary.csv`
- 单次 CSV 默认前缀：`dynamic_tracking_v2`

## 主要参数

- `target_x / target_y / target_z / target_yaw`
- `kp_xy / ki_xy / kd_xy`
- `kp_x / ki_x / kd_x`
- `kp_y / ki_y / kd_y`
- `kp_z / ki_z / kd_z`
- `kp_yaw / ki_yaw / kd_yaw`
- `camera_yaw_compensation_deg`
- `v_limit / vz_limit / yaw_rate_limit`
- `velocity_deadband`
- `require_offboard`
- `enable_z_hold`

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
cd /home/zjh/project/rasip_pi_ws
colcon build --symlink-install --packages-select dynamic_tracking_v2
source install/setup.bash
```

### 最新启动命令

当前最常用的启动命令建议直接用下面这几条：

新相机实机启动：

```bash
ros2 launch dynamic_tracking_v2 dynamic_tracking_v2.launch.py \
  camera_profile:=icspring_1080 \
  use_rqt:=false
```

旧相机实机启动：

```bash
ros2 launch dynamic_tracking_v2 dynamic_tracking_v2.launch.py \
  camera_profile:=old_cam \
  use_rqt:=false
```

关闭 USB 相机、复用外部图像流：

```bash
ros2 launch dynamic_tracking_v2 dynamic_tracking_v2.launch.py \
  camera_profile:=icspring_1080 \
  use_usb_cam:=false \
  use_rqt:=false \
  ros_image_topic:=/image_raw
```

如果外部图像来自仿真链路，通常把 `ros_image_topic` 改成 `/camera/image_raw`。

### 最小启动示例

```bash
ros2 launch dynamic_tracking_v2 dynamic_tracking_v2.launch.py \
  use_rqt:=false
```

### 相机 profile 切换

`dynamic_tracking_v2.launch.py` 会把 `camera_profile` 继续透传给 `tvec.launch.py`。当前可用 profile：

- `old_cam`
- `icspring_1080`

示例：

```bash
ros2 launch dynamic_tracking_v2 dynamic_tracking_v2.launch.py \
  camera_profile:=old_cam

ros2 launch dynamic_tracking_v2 dynamic_tracking_v2.launch.py \
  camera_profile:=icspring_1080
```

如果需要新增相机，请到 `src/tvec/config/camera_profiles.yaml` 增加 profile；若只想覆盖单个字段，仍可继续显式传 `video_device`、`image_width`、`image_height`、`pixel_format`、`framerate` 等参数。

### 最小调参示例

```bash
ros2 launch dynamic_tracking_v2 dynamic_tracking_v2.launch.py \
  camera_profile:=icspring_1080 \
  publish_annotated_image:=false \
  use_rqt:=false \
  target_x:=0.0 target_y:=0.0 target_z:=2.5 target_yaw:=0.0 \
  kp_xy:=1.00 ki_xy:=0.0 kd_xy:=0.06 \
  kp_z:=0.5 ki_z:=0.0 kd_z:=0.03 \
  kp_yaw:=0.4 ki_yaw:=0.0 kd_yaw:=0.03 \
  v_limit:=0.8 vz_limit:=0.2 yaw_rate_limit:=0.4 \
  camera_yaw_compensation_deg:=0.0
```
### 800*600
ros2 launch dynamic_tracking_v2 dynamic_tracking_v2.launch.py \
  camera_profile:=icspring_800x600 \
  publish_annotated_image:=true \
  use_rqt:=false \
  target_x:=0.0 target_y:=0.0 target_z:=2.5 target_yaw:=0.0 \
  kp_xy:=1.00 ki_xy:=0.0 kd_xy:=0.06 \
  kp_z:=0.5 ki_z:=0.0 kd_z:=0.03 \
  kp_yaw:=0.4 ki_yaw:=0.0 kd_yaw:=0.03 \
  v_limit:=0.8 vz_limit:=0.2 yaw_rate_limit:=0.4 \
  camera_yaw_compensation_deg:=0.0

### 640*480
ros2 launch dynamic_tracking_v2 dynamic_tracking_v2.launch.py \
  camera_profile:=icspring_640x480 \
  publish_annotated_image:=false \
  use_rqt:=false \
  target_x:=0.0 target_y:=0.0 target_z:=1.0 target_yaw:=0.0 \
  kp_xy:=1.00 ki_xy:=0.0 kd_xy:=0.06 \
  kp_z:=0.5 ki_z:=0.0 kd_z:=0.03 \
  kp_yaw:=0.4 ki_yaw:=0.0 kd_yaw:=0.03 \
  v_limit:=0.8 vz_limit:=0.2 yaw_rate_limit:=0.4 \
  camera_yaw_compensation_deg:=0.0

### 旧相机
ros2 launch dynamic_tracking_v2 dynamic_tracking_v2.launch.py \
  camera_profile:=old_cam \
  publish_annotated_image:=false \
  use_rqt:=false \
  require_offboard:=false \
  target_x:=0.0 target_y:=0.0 target_z:=2.5 target_yaw:=0.0 \
  kp_xy:=1.00 ki_xy:=0.0 kd_xy:=0.06 \
  kp_z:=0.5 ki_z:=0.0 kd_z:=0.03 \
  kp_yaw:=0.4 ki_yaw:=0.0 kd_yaw:=0.03 \
  v_limit:=0.8 vz_limit:=0.2 yaw_rate_limit:=0.4 \
  camera_yaw_compensation_deg:=0.0

### 分轴 XY 调参示例

```bash
ros2 launch dynamic_tracking_v2 dynamic_tracking_v2.launch.py \
  use_rqt:=false \
  kp_x:=0.70 ki_x:=0.00 kd_x:=0.06 \
  kp_y:=0.60 ki_y:=0.00 kd_y:=0.05 \
  kp_z:=0.5 ki_z:=0.0 kd_z:=0.03 \
  kp_yaw:=0.4 ki_yaw:=0.0 kd_yaw:=0.03 \
  target_yaw:=0.0 \
  v_limit:=0.60 vz_limit:=0.30 yaw_rate_limit:=0.40
```

### 关闭 USB 相机，仅复用外部图像流

```bash
ros2 launch dynamic_tracking_v2 dynamic_tracking_v2.launch.py \
  use_usb_cam:=false \
  use_rqt:=false \
  ros_image_topic:=/image_raw
```

## 设计说明

- XY 仍然是主要控制对象，默认保持 `pid_tuning_v6` 的控制强度与限幅。
- Z 与 yaw 默认权重更低，目的是在动态跟踪时尽量避免它们抢走 XY 主跟踪带宽。
- yaw 闭环统一作用于 `yaw_rel_corrected`，这样可以继续复用 `pid_tuning_v6` 的相机安装补偿和 XY 旋转口径。
