# pid_tuning_v4

`pid_tuning_v4` 是基于 `/home/zjh/project/rasip_pi_ws/src/pid_tuning_v3` 派生出来的独立 `ament_python` 功能包。本包以当前工作空间里的 `pid_tuning_v3` 为唯一直接基线实现，没有回到 `simple_ws` 重写，也没有回退成旧版本逻辑。

`v4` 与 `v3` 的核心区别只有一项：

- `v3` 的 Z 来源是视觉高度 `/debug/aruco_pose.z`
- `v4` 的 Z 来源是距离传感器 `/mavros/hrlv_ez4_pub`

除了这项改动外，`v4` 继续尽量保持 `v3` 的行为一致：

- 输出仍然使用 `/mavros/setpoint_raw/local`
- 仍然使用 `mavros_msgs/PositionTarget.FRAME_BODY_NED`
- 控制内部继续按 FLU 机体系理解速度
- 发布时保持：
  - `msg.velocity.x = vx_body`
  - `msg.velocity.y = -vy_body`
  - `msg.velocity.z = vz_body_flu`
- 不做 yaw 闭环，`yaw_rate = 0.0`
- 保留 XY 对准
- 保留“固定 yaw 补偿 + 动态相对 yaw 修正”
- 保留已经修正过的 yaw 方向处理：

```text
yaw_rel_raw = wrap_to_pi(-pose.yaw)
yaw_rel_corrected = wrap_to_pi(yaw_rel_raw + camera_yaw_compensation_rad)
```

## 输入与输出

- 输入视觉位姿：`/debug/aruco_pose`
- 输入飞控状态：`/mavros/state`
- 输入距离传感器：`/mavros/hrlv_ez4_pub`
- CSV logger 额外订阅：`/mavros/local_position/pose`
- CSV logger 姿态订阅：`/mavros/imu/data`
- 控制输出：`/mavros/setpoint_raw/local`

距离传感器消息类型沿用当前工作空间已有用法：

- `sensor_msgs/msg/Range`

## 相机话题说明

这次虽然改的是 Z 来源，但 XY 和 yaw 仍然依赖视觉链，因此 launch 默认保持实机配置：

- 实机默认图像话题：`/image_raw`
- 不是仿真的 `/camera/image_raw`

因此 `pid_tuning_v4.launch.py` 默认使用：

```bash
ros_image_topic:=/image_raw
use_usb_cam:=true
```

如果后续切回仿真图像流，可以手动覆盖：

```bash
ros2 launch pid_tuning_v4 pid_tuning_v4.launch.py \
  use_usb_cam:=false \
  ros_image_topic:=/camera/image_raw
```

## 核心控制逻辑

### 1. XY 与 yaw 保持不变

`v4` 完整保留 `v3` 的 XY 误差定义、yaw 符号修正和 marker -> body 二维误差旋转逻辑：

```text
ex_marker = target_x - pose.x
ey_marker = pose.y - target_y
```

```text
yaw_rel_raw = wrap_to_pi(-pose.yaw)
yaw_rel_corrected = wrap_to_pi(yaw_rel_raw + camera_yaw_compensation_rad)
```

```text
e_body = R(-yaw_rel_corrected) * e_marker
```

### 2. Z 改为距离传感器闭环

`v4` 中真正参与 Z 控制的是距离传感器高度：

```text
distance_sensor_z = Range.range
ez = target_z - distance_sensor_z
```

视觉高度 `/debug/aruco_pose.z` 不再参与 Z 控制，但仍然保留在 CSV 中，便于后续对比视觉高度和测距仪高度的差异。

## 测距仪失效保护

当 `enable_z_hold=true` 时，`v4` 不会无条件信任测距仪数据。控制节点会依次检查：

- 是否从未收到距离传感器数据
- 是否超过 `distance_sensor_timeout_sec`
- `Range.range` 是否为有限数
- `Range.range` 是否落在 `Range.min_range ~ Range.max_range` 内

当前实现采用如下安全策略：

- 若视觉和 OFFBOARD 正常，但测距仪无效：
  - `pid_z.reset()`
  - `vz = 0.0`
  - XY 继续按 `v3` 逻辑闭环控制
- 若视觉超时或未进入 OFFBOARD：
  - 与 `v3` 一样，整机输出零速并重置 PID

运行状态日志会明确输出以下原因之一：

- `距离传感器无效：从未收到数据`
- `距离传感器超时`
- `距离传感器无效：Range.range 非有限数`
- `距离值越界`

## 启动链路

`pid_tuning_v4.launch.py` 基于 `pid_tuning_v3.launch.py` 复制修改，继续复用当前工作空间的实机视觉主链：

```text
usb_cam(可选) -> /debug/tvec -> tvec_tf_node -> /debug/aruco_pose
```

也就是说，launch 会启动：

- `tvec.launch.py`
- `tvec_tf_node`
- `pid_tuning_v4_node`
- `pid_tuning_v4_csv_logger_node`

## CSV 记录说明

`v4` 的 CSV 继续保留 `v3` 的原有关键列，尤其保留：

- `aruco_z`
- `aruco_yaw_rad`
- `yaw_rel_raw_rad`
- `yaw_rel_corrected_rad`
- `ex_marker`
- `ey_marker`
- `ex_body`
- `ey_body`

同时新增以下列，用于对比“视觉高度 vs 测距仪高度”：

- `distance_sensor_z`
- `distance_sensor_fresh`
- `distance_sensor_age_sec`
- `distance_sensor_valid`
- `z_source_used`
- `ez_from_vision`
- `ez_from_distance_sensor`

各列语义如下：

- `aruco_z`：视觉高度，来自 `/debug/aruco_pose.z`
- `distance_sensor_z`：测距仪高度，来自 `Range.range`
- `distance_sensor_fresh`：当前测距样本是否在超时窗口内
- `distance_sensor_age_sec`：当前样本距离最近一次测距消息的时间差
- `distance_sensor_valid`：是否同时满足 fresh + finite + in-range
- `z_source_used`：当前控制实际使用的 Z 来源
  - `distance_sensor`：Z 闭环正在使用测距仪
  - `invalid`：Z 闭环已启用，但当前测距样本不可用
  - `disabled`：`enable_z_hold=false`
- `ez_from_vision = target_z - aruco_z`
- `ez_from_distance_sensor = target_z - distance_sensor_z`

当前单次 CSV 还会额外记录以下原始输入，供后续离线理论复算使用：

- 原始视觉 `/debug/tvec` 的 `raw_tvec_x/raw_tvec_y/raw_tvec_z`
- 原始视觉 `/debug/tvec` 的 `raw_rvec_x/raw_rvec_y/raw_rvec_z`
- 本机姿态 `/mavros/local_position/pose` 的 `local_roll_rad/local_pitch_rad/local_yaw_rad`

其中：

- `local_x/local_y/local_z` 继续来自 `/mavros/local_position/pose`
- `local_roll_rad/local_pitch_rad/local_yaw_rad` 优先来自 `/mavros/imu/data` 的四元数；若 IMU 暂不可用，则回退到 `/mavros/local_position/pose`

这批补充数据主要用于：

- 为后续按 `T_c_m -> T_m_c -> T_m_b` 做理论复算提供原始输入
- 分析飞行中的 `roll/pitch` 与 ArUco 位姿大跳之间的关系

默认日志路径为：

- 单次 CSV 默认目录：`/home/zjh/project/rasip_pi_ws/log/tracking_csv`
- summary CSV 默认路径：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/pid_tuning_v4_summary.csv`
- 单次 CSV 默认前缀：`pid_tuning_v4`

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
- `vxy_limit / vx_limit / vy_limit / vz_limit`
- `velocity_deadband`
- `require_offboard`
- `enable_z_hold`

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
cd /home/zjh/project/rasip_pi_ws
colcon build --symlink-install --packages-select pid_tuning_v4
source install/setup.bash
```

### 最小启动示例

实机默认启动：

```bash
ros2 launch pid_tuning_v4 pid_tuning_v4.launch.py \
  use_rqt:=false
```

关闭 USB 相机、仅复用外部图像流：

```bash
ros2 launch pid_tuning_v4 pid_tuning_v4.launch.py \
  use_usb_cam:=false \
  use_rqt:=false \
  ros_image_topic:=/image_raw
```

### 调参示例

```bash
ros2 launch pid_tuning_v4 pid_tuning_v4.launch.py \
  use_rqt:=false \
  target_z:=0.8 \
  kp_xy:=1.0 kd_xy:=0.06 \
  kp_z:=0.60 vz_limit:=0.40 \
  vxy_limit:=0.60 velocity_deadband:=0.02 \
  distance_sensor_timeout_sec:=0.5 \
  camera_yaw_compensation_deg:=0.0
```

分轴参数示例：

```bash
ros2 launch pid_tuning_v4 pid_tuning_v4.launch.py \
  use_rqt:=false \
  kp_x:=0.70 ki_x:=0.00 kd_x:=0.06 \
  kp_y:=0.60 ki_y:=0.00 kd_y:=0.05 \
  vx_limit:=0.70 vy_limit:=0.50 \
  distance_sensor_topic:=/mavros/hrlv_ez4_pub
```
