# dynamic_tracking_v3

`dynamic_tracking_v3` 是基于 `dynamic_tracking_v2` 新建的独立 `ament_python` 功能包，不修改 `dynamic_tracking_v2`。本包保留 v2 的 XY 主控制链、坐标系口径、`FRAME_BODY_NED` 发布方式、CSV logger、pipeline timing 和视觉链 launch 结构。

## 与 dynamic_tracking_v2 的唯一控制差异

v3 只在 Z 方向和 yaw_rate 输出链路增加平滑逻辑；XY 主控制逻辑不变。

平滑策略由三段组成：

- 测量低通：`aruco_z` 使用普通一阶低通，`yaw_rel_corrected` 使用角度一阶低通。
- 误差死区：`ez` 使用 `z_error_deadband`，`eyaw` 使用 `yaw_error_deadband`。
- 输出斜率限制：`vz` 使用 `vz_slew_rate_limit`，`yaw_rate` 使用 `yaw_rate_slew_rate_limit`。

Z 控制链路：

```text
aruco_z_raw -> z低通 -> ez -> z_error_deadband -> PID_Z -> velocity_deadband -> vz斜率限制 -> publish
```

yaw 控制链路：

```text
yaw_rel_corrected_raw -> yaw角度低通 -> eyaw -> yaw_error_deadband -> PID_Yaw -> 保持当前 yaw_rate 符号转换逻辑 -> yaw_rate斜率限制 -> publish
```

角度低通在内部使用 `wrap_to_pi` 计算角度差，避免 `+pi/-pi` 附近跳变。非 `OFFBOARD`、视觉超时或关闭 Z 保持时，会立即发布 0 并 reset PID、低通滤波器和斜率限制器，安全停机不会被平滑器延迟。

## 保持不变的 v2 口径

- XY 仍按 `ex_marker / ey_marker` 计算 marker 平面误差。
- `yaw_rel_raw = wrap_to_pi(-pose.yaw)` 和 `yaw_rel_corrected = wrap_to_pi(yaw_rel_raw + camera_yaw_compensation_rad)` 的原始口径保持不变。
- XY 旋转仍使用未平滑的 `yaw_rel_corrected_raw`，避免 Z/yaw 平滑影响 XY 主链。
- 控制输出继续发布到 `/mavros/setpoint_raw/local`，坐标系为 `FRAME_BODY_NED`。
- 发布口径继续保持 `msg.velocity.y = -vy_body`，`msg.velocity.z = vz_body_flu`。
- 视觉链仍为 `tvec.launch.py -> tvec_tf_node -> /debug/aruco_pose`。
- CSV logger 保留 v2 的全部记录功能，并新增平滑参数快照。

## 主要启动命令

```bash
ros2 launch dynamic_tracking_v3 dynamic_tracking_v3.launch.py \
  camera_profile:=icspring_1080 \
  target_z:=2.5 \
  kp_xy:=1.00 ki_xy:=0.0 kd_xy:=0.06 \
  kp_z:=0.5 ki_z:=0.0 kd_z:=0.03 \
  kp_yaw:=0.4 ki_yaw:=0.0 kd_yaw:=0.03 \
  v_limit:=0.8 vz_limit:=0.2 yaw_rate_limit:=0.4 \
  enable_z_yaw_smoothing:=true \
  z_lpf_tau_sec:=0.35 yaw_lpf_tau_sec:=0.35 \
  z_error_deadband:=0.04 yaw_error_deadband:=0.04 \
  vz_slew_rate_limit:=0.30 yaw_rate_slew_rate_limit:=0.35
```

以下参数保留默认值，主启动命令中不需要显式写出：

- `target_x=0.0`
- `target_y=0.0`
- `target_yaw=0.0`
- `camera_yaw_compensation_deg=0.0`
- `publish_annotated_image=false`
- `use_rqt=false`

## 新增平滑参数

- `enable_z_yaw_smoothing=true`
- `z_lpf_tau_sec=0.35`
- `yaw_lpf_tau_sec=0.35`
- `z_error_deadband=0.04`
- `yaw_error_deadband=0.04`
- `vz_slew_rate_limit=0.30`
- `yaw_rate_slew_rate_limit=0.35`

`enable_z_yaw_smoothing=false` 时，Z/yaw 低通、误差死区和斜率限制会旁路，控制表现回到 v2 口径。

## CSV 记录

CSV logger 节点名：

```text
dynamic_tracking_v3_csv_logger_node
```

默认日志路径：

- 单次 CSV 默认目录：`/home/zjh/project/rasip_pi_ws/log/tracking_csv`
- 单次 CSV 默认前缀：`dynamic_tracking_v3`
- summary CSV 默认路径：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/dynamic_tracking_v3_summary.csv`

CSV 参数快照和每行 header 新增：

- `enable_z_yaw_smoothing`
- `z_lpf_tau_sec`
- `yaw_lpf_tau_sec`
- `z_error_deadband`
- `yaw_error_deadband`
- `vz_slew_rate_limit`
- `yaw_rate_slew_rate_limit`

## 构建与验证

```bash
source /opt/ros/humble/setup.bash
cd /home/zjh/project/rasip_pi_ws
colcon build --symlink-install --packages-select dynamic_tracking_v3
source install/setup.bash
```

启动验证：

```bash
ros2 launch dynamic_tracking_v3 dynamic_tracking_v3.launch.py \
  camera_profile:=icspring_1080 \
  target_z:=2.5 \
  kp_xy:=1.00 ki_xy:=0.0 kd_xy:=0.06 \
  kp_z:=0.5 ki_z:=0.0 kd_z:=0.03 \
  kp_yaw:=0.4 ki_yaw:=0.0 kd_yaw:=0.03 \
  v_limit:=0.8 vz_limit:=0.2 yaw_rate_limit:=0.4 \
  enable_z_yaw_smoothing:=true \
  z_lpf_tau_sec:=0.35 yaw_lpf_tau_sec:=0.35 \
  z_error_deadband:=0.04 yaw_error_deadband:=0.04 \
  vz_slew_rate_limit:=0.30 yaw_rate_slew_rate_limit:=0.35
```
