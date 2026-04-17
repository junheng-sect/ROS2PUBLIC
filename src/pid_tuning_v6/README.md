# pid_tuning_v6

`pid_tuning_v6` 是基于 `/home/zjh/project/rasip_pi_ws/src/pid_tuning_v4` 派生出来的独立 `ament_python` 功能包。`v6` 保留了 `v4` 的视觉 XY 对准、yaw 修正、距离传感器 Z 闭环、安全保护和 CSV 记录框架，只把横向限速策略统一改成“水平速度幅值限速”。

## v6 与 v4 的核心区别

- `v4` 的横向速度口径是旧的分轴/混合限速方式
- `v6` 删除以上三项，只保留 `v_limit`
- `v6` 不再按 X/Y 分轴分别裁剪速度，而是先计算水平速度幅值
- 当 `sqrt(vx^2 + vy^2) > v_limit` 时，按同一比例同时缩小 `vx` 和 `vy`
- 由于 `vx` 与 `vy` 使用同一个缩放系数，限幅后速度方向保持不变，仍然指向 target，可避免“单轴先锁 limit 后速度方向偏掉，最终绕圈”的情况

核心公式如下：

```text
v_norm = sqrt(vx_raw^2 + vy_raw^2)

if v_norm > v_limit:
    scale = v_limit / v_norm
    vx = vx_raw * scale
    vy = vy_raw * scale
else:
    vx = vx_raw
    vy = vy_raw
```

## 保持不变的控制行为

- 输出仍然使用 `/mavros/setpoint_raw/local`
- 仍然使用 `mavros_msgs/PositionTarget.FRAME_BODY_NED`
- 控制内部继续按 FLU 机体系理解速度
- 发布时继续保持：
  - `msg.velocity.x = vx_body`
  - `msg.velocity.y = -vy_body`
  - `msg.velocity.z = vz_body_flu`
- 不做 yaw 闭环，`yaw_rate = 0.0`
- XY 误差定义、yaw 修正逻辑、`rotate_marker_error_to_body()` 保持与 `v4` 一致
- Z 仍然来自距离传感器 `/mavros/hrlv_ez4_pub`
- OFFBOARD 未进入、视觉超时、测距失效时的安全保护语义保持不变

## 输入与输出

- 输入视觉位姿：`/debug/aruco_pose`
- 输入飞控状态：`/mavros/state`
- 输入距离传感器：`/mavros/hrlv_ez4_pub`
- CSV logger 额外订阅：`/mavros/local_position/pose`
- CSV logger 姿态订阅：`/mavros/imu/data`
- 控制输出：`/mavros/setpoint_raw/local`

距离传感器消息类型保持不变：

- `sensor_msgs/msg/Range`

## 启动链路

`pid_tuning_v6.launch.py` 继续复用当前工作空间里的实机视觉主链：

```text
usb_cam(可选) -> /debug/tvec -> tvec_tf_node -> /debug/aruco_pose
```

launch 会启动：

- `tvec.launch.py`
- `tvec_tf_node`
- `pid_tuning_v6_node`
- `pid_tuning_v6_csv_logger_node`

## 相机话题说明

默认仍按实机链路配置：

- 默认图像话题：`/image_raw`
- 默认 `use_usb_cam:=true`

如果后续切回仿真图像流，可手动覆盖：

```bash
ros2 launch pid_tuning_v6 pid_tuning_v6.launch.py \
  use_usb_cam:=false \
  ros_image_topic:=/camera/image_raw
```

## CSV 记录说明

`v6` 的 CSV 继续保留 `v4` 的主要观测列，包括：

- `aruco_z`
- `aruco_yaw_rad`
- `yaw_rel_raw_rad`
- `yaw_rel_corrected_rad`
- `ex_marker`
- `ey_marker`
- `ex_body`
- `ey_body`
- `distance_sensor_z`
- `distance_sensor_valid`
- `sp_vx / sp_vy / sp_vz`

参数快照口径调整为：

- 保留：`v_limit`、`vz_limit`
- 已移除旧的横向分轴限速参数

默认日志路径：

- 单次 CSV 默认目录：`/home/zjh/project/rasip_pi_ws/log/tracking_csv`
- summary CSV 默认路径：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/pid_tuning_v6_summary.csv`
- 单次 CSV 默认前缀：`pid_tuning_v6`

## 主要参数

- `distance_sensor_topic`：默认 `/mavros/hrlv_ez4_pub`
- `distance_sensor_timeout_sec`：默认 `0.5`
- `ros_image_topic`：默认 `/image_raw`
- `use_usb_cam`：默认 `true`
- `target_x / target_y / target_z`
- `kp_xy / ki_xy / kd_xy`
- `kp_x / ki_x / kd_x`
- `kp_y / ki_y / kd_y`
- `kp_z / ki_z / kd_z`
- `camera_yaw_compensation_deg`
- `v_limit / vz_limit`
- `velocity_deadband`
- `require_offboard`
- `enable_z_hold`

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
cd /home/zjh/project/rasip_pi_ws
colcon build --symlink-install --packages-select pid_tuning_v6
source install/setup.bash
```

### 最小启动示例

```bash
ros2 launch pid_tuning_v6 pid_tuning_v6.launch.py \
  use_rqt:=false
```

### 关闭 USB 相机，仅复用外部图像流

```bash
ros2 launch pid_tuning_v6 pid_tuning_v6.launch.py \
  use_usb_cam:=false \
  use_rqt:=false \
  ros_image_topic:=/image_raw
```

### 调参示例

```bash
ros2 launch pid_tuning_v6 pid_tuning_v6.launch.py \
  use_rqt:=false \
  target_z:=2.5 \
  kp_xy:=1.0 kd_xy:=0.06 \
  kp_z:=0.60 vz_limit:=0.40 \
  v_limit:=1.00 velocity_deadband:=0.02 \
  distance_sensor_timeout_sec:=0.5 \
  camera_yaw_compensation_deg:=0.0
```

### 分轴 PID 调参示例

这里的“分轴”只指 PID 参数可以分别设置，横向速度限幅仍然统一使用 `v_limit`：

```bash
ros2 launch pid_tuning_v6 pid_tuning_v6.launch.py \
  use_rqt:=false \
  kp_x:=0.70 ki_x:=0.00 kd_x:=0.06 \
  kp_y:=0.60 ki_y:=0.00 kd_y:=0.05 \
  v_limit:=0.60 \
  distance_sensor_topic:=/mavros/hrlv_ez4_pub
```
