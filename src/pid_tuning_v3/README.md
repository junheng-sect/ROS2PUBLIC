# pid_tuning_v3

`pid_tuning_v3` 是从 `/home/zjh/project/simple_ws/src/pid_tuning_v3` 迁移到 `rasip_pi_ws` 的独立 `ament_python` 功能包。本包以 `simple_ws` 中已经验证通过的版本为唯一基线，迁移时保留了已经验证过的控制逻辑，不回退为旧版本。

当前版本明确保留以下核心约定：

- 输出仍然使用 `/mavros/setpoint_raw/local`
- 仍然使用 `mavros_msgs/PositionTarget.FRAME_BODY_NED`
- 控制内部继续按 FLU 机体系理解速度
- 发布时保持：
  - `msg.velocity.x = vx_body`
  - `msg.velocity.y = -vy_body`
  - `msg.velocity.z = vz_body_flu`
- 不做 yaw 闭环，`yaw_rate = 0.0`
- 保留 `XY` 对准
- 保留 `Z` 保持
- 保留“固定 yaw 补偿 + 动态相对 yaw 修正”
- 保留已经修正过的 yaw 方向处理：

```text
yaw_rel_raw = wrap_to_pi(-pose.yaw)
yaw_rel_corrected = wrap_to_pi(yaw_rel_raw + camera_yaw_compensation_rad)
```

## 迁移后的视觉链

本次迁移没有修改 `simple_ws` 的 `pid_tuning_v3` 控制代码，而是让 `rasip_pi_ws` 中的新包复用当前工作空间已经在实机上使用的视觉主链：

```text
usb_cam(可选) -> /debug/tvec -> tvec_tf_node -> /debug/aruco_pose
```

也就是说，`pid_tuning_v3.launch.py` 现在会在 `rasip_pi_ws` 中启动：

- `tvec.launch.py`
- `tvec_tf_node`
- `pid_tuning_v3_node`
- `pid_tuning_v3_csv_logger_node`

## 相机话题说明

这是本次迁移最重要的区别，请务必区分：

- 仿真默认图像话题：`/camera/image_raw`
- 实机默认图像话题：以树莓派当前真实相机话题为准，`rasip_pi_ws` 现有实机链路当前统一使用 `/image_raw`

因此，迁移后的 `pid_tuning_v3.launch.py` 默认使用：

```bash
ros_image_topic:=/image_raw
use_usb_cam:=true
```

如果要复用仿真图像流，请改成：

```bash
ros_image_topic:=/camera/image_raw
use_usb_cam:=false
```

## 输入与输出

- 输入视觉位姿：`/debug/aruco_pose`
- 输入飞控状态：`/mavros/state`
- CSV logger 额外订阅：`/mavros/local_position/pose`
- 控制输出：`/mavros/setpoint_raw/local`

## 核心控制逻辑

### 1. marker 平面误差

```text
ex_marker = target_x - pose.x
ey_marker = pose.y - target_y
```

### 2. yaw 修正

当前版本保留了 `simple_ws` 中已经修正过的 yaw 方向处理，不再直接用 `pose.yaw` 进入控制：

```text
yaw_rel_raw = wrap_to_pi(-pose.yaw)
yaw_rel_corrected = wrap_to_pi(yaw_rel_raw + camera_yaw_compensation_rad)
```

其中：

- `yaw_rel_raw_rad` 代表经过符号修正后、真正用于控制计算的相对 yaw
- `yaw_rel_corrected_rad` 代表在 `yaw_rel_raw_rad` 基础上，再叠加固定安装补偿角后的控制 yaw

### 3. marker 误差转到机体系

```text
e_body = R(-yaw_rel_corrected) * e_marker
```

展开后为：

```text
ex_body = cos(yaw_rel_corrected) * ex_marker + sin(yaw_rel_corrected) * ey_marker
ey_body = -sin(yaw_rel_corrected) * ex_marker + cos(yaw_rel_corrected) * ey_marker
```

### 4. BODY_NED 发布约定

- 内部速度语义：FLU
  - `x` 前
  - `y` 左
  - `z` 上
- 发到 `FRAME_BODY_NED` 时保持：
  - `msg.velocity.x = vx_body`
  - `msg.velocity.y = -vy_body`
  - `msg.velocity.z = vz_body_flu`
- `yaw_rate` 固定为 `0.0`

## `camera_yaw_compensation_deg` 的意义

`camera_yaw_compensation_deg` 表示“相机安装方向相对机体方向”的固定补偿角，约定从俯视看逆时针为正。它是一个常数项，用来修正相机安装偏差导致的 `XY` 平面耦合。

控制里真正使用的是：

```text
camera_yaw_compensation_rad = radians(camera_yaw_compensation_deg)
```

然后把它叠加到已经做过符号修正的 `yaw_rel_raw` 上：

```text
yaw_rel_corrected = wrap_to_pi(yaw_rel_raw + camera_yaw_compensation_rad)
```

## `aruco_yaw_rad` 与 `yaw_rel_raw_rad` 的区别

- `aruco_yaw_rad`
  - 直接记录视觉链 `/debug/aruco_pose.yaw` 的原始输出
  - 方便复盘视觉链本身的姿态变化
- `yaw_rel_raw_rad`
  - 记录控制真正使用的“符号修正后 yaw”
  - 当前版本固定采用 `wrap_to_pi(-pose.yaw)`

因此，这两个值通常不会完全相同。`aruco_yaw_rad` 更偏原始观测，`yaw_rel_raw_rad` 才是控制入口。

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
cd /home/zjh/project/rasip_pi_ws
colcon build --symlink-install --packages-select pid_tuning_v3
source install/setup.bash
```

### 最小启动示例

实机默认启动：

```bash
ros2 launch pid_tuning_v3 pid_tuning_v3.launch.py \
  use_rqt:=false
```

仿真图像流复用示例：

```bash
ros2 launch pid_tuning_v3 pid_tuning_v3.launch.py \
  use_usb_cam:=false \
  use_rqt:=false \
  ros_image_topic:=/camera/image_raw
```

仅验证控制节点和 logger 的最小示例：

```bash
ros2 launch pid_tuning_v3 pid_tuning_v3.launch.py \
  use_usb_cam:=false \
  use_rqt:=false \
  enable_csv_logger:=true
```

## 常用调参示例

```bash
ros2 launch pid_tuning_v3 pid_tuning_v3.launch.py \
  use_rqt:=false \
  target_z:=2.5 \
  kp_xy:=0.80 kd_xy:=0.02 \
  kp_z:=0.60 vz_limit:=0.40 \
  vxy_limit:=0.60 velocity_deadband:=0.02 \
  camera_yaw_compensation_deg:=0.0
```

分轴参数示例：

```bash
ros2 launch pid_tuning_v3 pid_tuning_v3.launch.py \
  use_rqt:=false \
  kp_x:=0.70 ki_x:=0.00 kd_x:=0.06 \
  kp_y:=0.60 ki_y:=0.00 kd_y:=0.05 \
  vx_limit:=0.70 vy_limit:=0.50
```

## CSV 记录

- 单次 CSV 默认目录：`/home/zjh/project/rasip_pi_ws/log/tracking_csv`
- summary CSV 默认路径：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/pid_tuning_v3_summary.csv`
- 单次 CSV 默认前缀：`pid_tuning_v3`

会保留并记录以下关键观测列：

- `aruco_yaw_rad`
- `yaw_rel_raw_rad`
- `yaw_rel_corrected_rad`
- `ex_marker`
- `ey_marker`
- `ex_body`
- `ey_body`

退出时会：

- 在终端打印本次调参摘要
- 在 summary CSV 中追加一行
- 保留 `sp_yaw_rate` 观测列，用于确认 yaw 未参与闭环且保持 `0.0`
