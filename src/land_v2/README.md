# land_v2

`land_v2` 是基于 `land_v1` 新建的精准降落功能包。核心变化是把 yaw 方向纳入对准和下降闭环：

1. 先在目标高度执行 `XY/Z/Yaw` 对准。
2. 对准成功后以固定下降速度继续跟踪 ArUco。
3. 视觉新鲜时持续输出 `yaw_rate`，视觉丢失后停止 yaw 转动。
4. 接地后进入最低油门下压与周期 `disarm` 流程。

## 控制语义

- 输出话题：`/mavros/setpoint_raw/local`
- 坐标系：`mavros_msgs/msg/PositionTarget.FRAME_BODY_NED`
- 发布字段：
  - `msg.velocity.x = vx_body`
  - `msg.velocity.y = -vy_body`
  - `msg.velocity.z = vz_body_flu`
  - `msg.yaw_rate = yaw_rate`

`XY` 误差旋转继续沿用 `land_v1` / `pid_tuning_v4` 口径：

```text
ex_marker = target_x - pose.x
ey_marker = pose.y - target_y
yaw_rel_raw = wrap_to_pi(-pose.yaw)
yaw_rel_corrected = wrap_to_pi(yaw_rel_raw + camera_yaw_compensation_rad)
e_body = R(-yaw_rel_corrected) * e_marker
```

Yaw 闭环使用：

```text
eyaw = wrap_to_pi(target_yaw - yaw_rel_corrected)
yaw_rate = PID_Yaw(eyaw)
```

默认 `invert_yaw_rate_output:=true`，用于匹配当前视觉 yaw 口径与 `FRAME_BODY_NED` yaw_rate 方向。若实机观察到 yaw 方向越修越偏，优先把该参数改为 `false` 验证。

## 状态机

- `WAIT_FOR_OFFBOARD`
- `ALIGN_AT_TARGET_HEIGHT`
- `DESCEND_WITH_TRACK`
- `DESCEND_RANGE_ONLY`
- `TOUCHDOWN_DISARM`
- `DONE`

Yaw 控制只在视觉新鲜的 `ALIGN_AT_TARGET_HEIGHT` 与 `DESCEND_WITH_TRACK` 阶段启用。`DESCEND_RANGE_ONLY`、`TOUCHDOWN_DISARM`、`DONE` 阶段固定 `yaw_rate=0.0`。

## 推荐启动

```bash
source /opt/ros/humble/setup.bash
cd /home/zjh/project/rasip_pi_ws
colcon build --symlink-install --packages-select land_v2
source install/setup.bash

ros2 launch land_v2 land_v2.launch.py \
  camera_profile:=icspring_1080 \
  use_rqt:=false \
  target_x:=0.0 target_y:=0.0 target_z:=2.5 target_yaw:=0.0 \
  kp_xy:=0.5 ki_xy:=0.0 kd_xy:=0.08 \
  kp_z:=0.8 ki_z:=0.0 kd_z:=0.06 \
  kp_yaw:=0.4 ki_yaw:=0.0 kd_yaw:=0.03 \
  v_limit:=0.8 vz_limit:=0.2 yaw_rate_limit:=0.4 \
  enable_z_yaw_smoothing:=true \
  yaw_lpf_tau_sec:=0.35 \
  yaw_error_deadband:=0.04 \
  yaw_rate_slew_rate_limit:=0.35 \
  invert_yaw_rate_output:=true
```

关闭 USB 相机、复用外部图像流：

```bash
ros2 launch land_v2 land_v2.launch.py \
  use_usb_cam:=false \
  use_rqt:=false \
  ros_image_topic:=/image_raw
```

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_yaw` | `0.0` | 目标 yaw，单位 rad |
| `kp_yaw` / `ki_yaw` / `kd_yaw` | `0.4 / 0.0 / 0.03` | yaw PID 参数 |
| `yaw_rate_limit` | `0.4` | yaw_rate 输出限幅，单位 rad/s |
| `yaw_error_deadband` | `0.04` | yaw 误差死区，单位 rad |
| `yaw_lpf_tau_sec` | `0.35` | yaw 角度低通时间常数 |
| `yaw_rate_slew_rate_limit` | `0.35` | yaw_rate 斜率限制，单位 rad/s^2 |
| `align_yaw_tolerance_rad` | `0.12` | 对准窗口单样本 yaw 阈值 |
| `align_yaw_mean_tolerance_rad` | `0.08` | 对准窗口 yaw 均值阈值 |
| `invert_yaw_rate_output` | `true` | 是否反转 yaw_rate 输出符号 |
| `v_limit` | `0.8` | 水平速度矢量限幅，单位 m/s |
| `vz_limit` | `0.2` | Z PID 输出限幅，单位 m/s |

`vx_limit` / `vy_limit` 仍保留为可选分轴限幅，默认 `nan` 表示不启用分轴裁剪。

## CSV 记录

默认输出：

- 单次 CSV 目录：`/home/zjh/project/rasip_pi_ws/log/tracking_csv`
- 单次 CSV 前缀：`land_v2`
- summary CSV：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/land_v2_summary.csv`

CSV 增加记录：

- `target_yaw`
- `eyaw_rad`
- `status_eyaw_rad`
- `status_eyaw_ctrl_rad`
- `status_yaw_rate`
- `sp_yaw_rate`
- `align_window_mean_yaw`
- yaw PID、yaw 限幅和 yaw 平滑参数快照

