# pid_tuning_v5

`pid_tuning_v5` 是直接从当前工作区中的 `src/pid_tuning_v4` 完整复制出来的独立 `ament_python` 功能包，`pid_tuning_v4` 本身没有被修改。`v5` 默认继续保持 `v4` 的控制语义，只在此基础上新增三项改动：

- 首次 `update()` 抑制 D kick：`PIDController.reset()` 后将 `prev_error` 置为 `None`，首次更新时强制 `d_term=0.0`，避免重新接管瞬间出现假导数冲击
- OFFBOARD 切入斜坡接管：检测到从非 `OFFBOARD` 切到 `OFFBOARD` 时，先 `reset_all_pids()`，再在 `offboard_entry_ramp_sec` 时间窗内对最终速度指令做线性斜坡缩放
- XY 合速度限幅改为保方向缩放：当 `sqrt(vx^2 + vy^2)` 超过 `vxy_limit` 时，对 `vx/vy` 同时按比例缩小，尽量保持速度方向始终指向当前 `e_body` 方向

除上述三点外，`v5` 继续保持 `v4` 的关键行为不变：

- 输出仍发布到 `/mavros/setpoint_raw/local`
- 继续使用 `mavros_msgs/msg/PositionTarget.FRAME_BODY_NED`
- 内部控制仍按 FLU 机体系理解速度
- 发布时继续保持 `msg.velocity.x = vx_body`、`msg.velocity.y = -vy_body`、`msg.velocity.z = vz_body_flu`
- 不做 yaw 闭环，`yaw_rate = 0.0`
- XY 误差定义继续保持 `ex_marker = target_x - pose.x`、`ey_marker = pose.y - target_y`
- yaw 修正继续保持 `yaw_rel_raw = wrap_to_pi(-pose.yaw)` 与 `yaw_rel_corrected = wrap_to_pi(yaw_rel_raw + camera_yaw_compensation_rad)`
- marker 到 body 的二维误差旋转继续保持 `e_body = R(-yaw_rel_corrected) * e_marker`
- Z 继续使用距离传感器 `/mavros/hrlv_ez4_pub`
- 测距仪无效时继续保持“XY 正常控制、`pid_z.reset()`、`vz=0.0`”

## 新增参数

- `offboard_entry_ramp_sec`
  - 默认值：`0.3`
  - 含义：OFFBOARD 上升沿触发后，最终 `vx/vy/vz` 指令从 0 线性爬升到 PID 正常输出所需的时间
  - 关闭方式：设为 `0` 或任意 `<= 0` 的值

当前采用的是“小速度斜坡接管”，不是纯零速保持。也就是说，窗口内仍会输出 PID 速度指令，只是按 `alpha` 做线性缩放：

```text
alpha = clamp((now - offboard_entry_time) / offboard_entry_ramp_sec, 0.0, 1.0)
vx_cmd = alpha * vx_pid
vy_cmd = alpha * vy_pid
vz_cmd = alpha * vz_pid
```

## XY 限幅语义

`v5` 中的 `vxy_limit` 已固定表示“水平合成速度上限”：

```text
vxy_raw = sqrt(vx_pid_raw^2 + vy_pid_raw^2)
```

当 `vxy_raw > vxy_limit` 时，控制节点会执行：

```text
scale = vxy_limit / vxy_raw
vx_cmd = scale * vx_pid_raw
vy_cmd = scale * vy_pid_raw
```

这样做的目的，是避免旧的“单轴先撞限、另一轴没撞限”把速度方向掰偏。

当前 `velocity_deadband` 对 XY 也会在“合速度限幅之后”统一按合速度做死区，因此在你主用的这组参数下：

```bash
kp_xy:=1.0 kd_xy:=0.06 \
kp_z:=0.60 vz_limit:=0.40 \
vxy_limit:=0.60 velocity_deadband:=0.02
```

XY 指令会优先保持方向，只有当整体 XY 速度已经很小的时候，才会一起压到 0。

说明：

- `vxy_limit`：在 `v5` 中是唯一生效的 XY 限幅
- `vx_limit / vy_limit`：在 `v5` 中仅为兼容保留参数，不再参与控制计算
- Z 轴 `vz_limit` 语义不变，仍然只约束 Z 速度

## 启动链路

`pid_tuning_v5.launch.py` 继续复用当前工作空间的视觉主链：

```text
usb_cam(可选) -> /debug/tvec -> tvec_tf_node -> /debug/aruco_pose
```

会启动以下节点：

- `tvec.launch.py`
- `tvec_tf_node`
- `pid_tuning_v5_node`
- `pid_tuning_v5_csv_logger_node`

## CSV 日志

`v5` 的 CSV logger 已独立命名，默认输出路径也切换为 `v5`：

- 单次 CSV 默认目录：`/home/zjh/project/rasip_pi_ws/log/tracking_csv`
- 单次 CSV 默认前缀：`pid_tuning_v5`
- summary CSV：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/pid_tuning_v5_summary.csv`

logger 会保留 `v4` 的原始观测字段，并额外记录 `offboard_entry_ramp_sec` 参数快照，便于离线分析同一批数据使用的接管窗口配置。

虽然 CSV 参数快照里仍会记录 `vx_limit / vy_limit`，但它们在 `v5` 中仅表示“启动时收到的兼容参数值”，不再决定 XY 实际限幅行为。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/zjh/project/rasip_pi_ws
colcon build --symlink-install --packages-select pid_tuning_v5
source install/setup.bash
```

## 最小运行命令

```bash
ros2 launch pid_tuning_v5 pid_tuning_v5.launch.py \
  use_rqt:=false
```

## 调参示例

```bash
ros2 launch pid_tuning_v5 pid_tuning_v5.launch.py \
  use_rqt:=false \
  target_z:=2.5 \
  kp_xy:=1.0 kd_xy:=0.06 \
  kp_z:=0.60 vz_limit:=0.40 \
  vxy_limit:=0.60 velocity_deadband:=0.02 \
  distance_sensor_timeout_sec:=0.5 \
  camera_yaw_compensation_deg:=0.0 \
  offboard_entry_ramp_sec:=0.3
```

关闭 OFFBOARD 斜坡接管：

```bash
ros2 launch pid_tuning_v5 pid_tuning_v5.launch.py \
  use_rqt:=false \
  offboard_entry_ramp_sec:=0
```

分轴参数示例：

```bash
ros2 launch pid_tuning_v5 pid_tuning_v5.launch.py \
  use_rqt:=false \
  kp_x:=0.70 ki_x:=0.00 kd_x:=0.06 \
  kp_y:=0.60 ki_y:=0.00 kd_y:=0.05 \
  vxy_limit:=0.60 \
  offboard_entry_ramp_sec:=0.3
```

如果旧命令里还带着 `vx_limit:=...` 或 `vy_limit:=...`，`v5` 仍然可以正常启动，但这两个参数在 `v5` 中不会再改变 XY 控制输出。
