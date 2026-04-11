# pid_tuning_v2

`pid_tuning_v2` 是基于 `pid_tuning` 复制出的独立功能包，用于并行对比测试。它保留了 `setpoint_raw/local + BODY_NED` 的速度控制方式，但核心控制逻辑调整为：

- 继续使用 `PositionTarget` 和 `FRAME_BODY_NED`
- 控制内部仍按 FLU 机体系理解速度，并保持 `msg.velocity.y = -vy_body`、`msg.velocity.z = vz_body_flu`
- 去掉 yaw 对准闭环，`yaw_rate` 固定为 `0.0`
- 保留 `X/Y` 对准与 `Z` 定高，默认 `target_z=2.5`
- 新增 `camera_yaw_compensation_deg`，用于补偿相机安装偏差带来的 XY 平面耦合

## 运行

```bash
ros2 launch pid_tuning_v2 pid_tuning_v2.launch.py
```

## 与 `pid_tuning` 的区别

- `pid_tuning` 会做 `X/Y/Yaw + Z` 闭环，`pid_tuning_v2` 只做 `X/Y + Z`
- `pid_tuning_v2` 不再提供 yaw PID 参数，也不再计算 yaw PID 输出
- `pid_tuning_v2` 新增 `camera_yaw_compensation_deg`，在 XY PID 之前先对视觉平面误差做二维旋转补偿

## 相机 yaw 补偿参数

参数名：

```bash
camera_yaw_compensation_deg
```

定义：

- 从俯视看，逆时针为正
- 它是控制层使用的补偿角，不要求与真实物理安装角完全相等
- 控制器会将原始 XY 误差向量按该角度做二维旋转，再送入 `pid_x` 与 `pid_y`

数学形式：

```text
[ex_comp]   [ cos(theta)  -sin(theta)] [ex_raw]
[ey_comp] = [ sin(theta)   cos(theta)] [ey_raw]
```

其中 `theta = camera_yaw_compensation_deg`

## 参数模式

### 1) 粗调模式（兼容）

不分轴，使用同一组 XY 参数：

```bash
ros2 launch pid_tuning_v2 pid_tuning_v2.launch.py \
  use_rqt:=false \
  kp_xy:=0.8 ki_xy:=0.0 kd_xy:=0.05
```

### 2) 分轴精调模式

X/Y 使用独立参数：

```bash
ros2 launch pid_tuning_v2 pid_tuning_v2.launch.py \
  use_rqt:=false \
  kp_x:=1.0 ki_x:=0.0 kd_x:=0.05 \
  kp_y:=0.95 ki_y:=0.0 kd_y:=0.06
```

### 3) 混合模式（分轴优先）

当同时传入 `kp_xy` 与 `kp_x/kp_y` 时，X/Y 分轴参数优先。

```bash
ros2 launch pid_tuning_v2 pid_tuning_v2.launch.py \
  use_rqt:=false \
  kp_xy:=0.6 ki_xy:=0.0 kd_xy:=0.10 \
  kp_x:=1.0 kd_x:=0.05 \
  kp_y:=0.9 kd_y:=0.06
```

### 4) Z 方向调参（定高）

Z 反馈源为 `aruco_pose.z`，可通过 `target_z` 与 z PID 调参：

```bash
ros2 launch pid_tuning_v2 pid_tuning_v2.launch.py \
  use_rqt:=false \
  target_z:=2.5 \
  kp_xy:=1.0 ki_xy:=0.0 kd_xy:=0.05 \
  kp_z:=0.80 ki_z:=0.0 kd_z:=0.06
```

### 5) X/Y 速度上限分开设置

默认使用统一上限 `vxy_limit`。若需要分别限制 X/Y，可显式传入 `vx_limit` 与 `vy_limit`：

```bash
ros2 launch pid_tuning_v2 pid_tuning_v2.launch.py \
  use_rqt:=false \
  kp_x:=0.6 ki_x:=0.00 kd_x:=0.02 \
  kp_y:=0.6 ki_y:=0.00 kd_y:=0.02 \
  vx_limit:=1.2 \
  vy_limit:=0.8 \
  camera_yaw_compensation_deg:=3.0
```

说明：

- 不传 `vx_limit`/`vy_limit` 时，会自动回退到 `vxy_limit`
- 传了 `vx_limit`/`vy_limit` 后，分别作用于 `pid_x` 和 `pid_y` 输出限幅

### 6) 最小补偿角示例

```bash
ros2 launch pid_tuning_v2 pid_tuning_v2.launch.py \
  use_rqt:=false \
  camera_yaw_compensation_deg:=0.0
```

```bash
ros2 launch pid_tuning_v2 pid_tuning_v2.launch.py \
  use_rqt:=false \
  camera_yaw_compensation_deg:=3.0
```

```bash
ros2 launch pid_tuning_v2 pid_tuning_v2.launch.py \
  use_rqt:=false \
  camera_yaw_compensation_deg:=-3.0
```

## 关键默认值

- `target_z=2.5`（Z 反馈源为 `aruco_pose.z`）
- `kp_xy=0.5, ki_xy=0.0, kd_xy=0.08`
- `kp_x/kp_y` 默认 `nan`（未设置时自动回退到 `kp_xy`）
- `camera_yaw_compensation_deg=0.0`
- `yaw_rate` 固定为 `0.0`，v2 不启用 yaw 闭环

## 输出与汇总

- 单次 CSV 目录默认：`/home/zjh/project/rasip_pi_ws/log/tracking_csv`
- 单次 CSV 前缀默认：`pid_tuning_v2`
- 汇总 CSV 默认：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/pid_tuning_v2_summary.csv`
- 参数快照中包含：`camera_yaw_compensation_deg`
- 终端汇总和 summary 重点输出：`fresh_ratio`、`max_stale_s`、`rmse_x/rmse_y/rmse_z/rmse_xy`、`p95_abs_x/p95_abs_y/p95_abs_z/p95_xy`、`cmd_jitter_x/cmd_jitter_y/cmd_jitter_xy`
- 会额外记录 `sp_yaw_rate`，用于确认 yaw 未参与控制且输出保持为 `0.0`

结束（`Ctrl-C`）时会：

- 在终端打印本次指标汇总
- 在 `pid_tuning_v2_summary.csv` 追加一行
- 自动将“本次运行 CSV + 汇总 CSV”复制到 `~/桌面/trackingcsv`（可配置）
