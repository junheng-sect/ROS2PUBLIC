# pid_tuning

`pid_tuning` 用于实机 ArUco 跟踪 PID 调参，提供：

- USB 相机 + ArUco 检测链路（复用 `tvec`）
- `pid_tuning_node`（X/Y/Yaw + Z 定高闭环）
- `pid_tuning_csv_logger_node`（CSV 记录 + 结束后指标汇总）

## 运行

```bash
ros2 launch pid_tuning pid_tuning.launch.py
```

## 参数模式

### 1) 粗调模式（兼容）

不分轴，使用同一组 XY 参数：

```bash
ros2 launch pid_tuning pid_tuning.launch.py \
  use_rqt:=false \
  kp_xy:=0.8 ki_xy:=0.0 kd_xy:=0.05
```

### 2) 分轴精调模式

X/Y 使用独立参数：

```bash
ros2 launch pid_tuning pid_tuning.launch.py \
  use_rqt:=false \
  kp_x:=1.0 ki_x:=0.0 kd_x:=0.05 \
  kp_y:=0.95 ki_y:=0.0 kd_y:=0.06
```

### 3) 混合模式（分轴优先）

当同时传入 `kp_xy` 与 `kp_x/kp_y` 时，X/Y 分轴参数优先。

```bash
ros2 launch pid_tuning pid_tuning.launch.py \
  use_rqt:=false \
  kp_xy:=0.6 ki_xy:=0.0 kd_xy:=0.10 \
  kp_x:=1.0 kd_x:=0.05 \
  kp_y:=0.9 kd_y:=0.06
```

### 4) Yaw 方向调参

单独调偏航环时，可先固定 XY/Z 参数，仅扫描 yaw PID：

```bash
ros2 launch pid_tuning pid_tuning.launch.py \
  use_rqt:=false \
  kp_xy:=1.0 ki_xy:=0.0 kd_xy:=0.05 \
  kp_z:=0.8 ki_z:=0.0 kd_z:=0.06 \
  kp_yaw:=1.10 ki_yaw:=0.0 kd_yaw:=0.05
```

### 5) Z 方向调参（定高）

Z 反馈源为 `aruco_pose.z`，可通过 `target_z` 与 z PID 调参：

```bash
ros2 launch pid_tuning pid_tuning.launch.py \
  use_rqt:=false \
  target_z:=2.5 \
  kp_xy:=1.0 ki_xy:=0.0 kd_xy:=0.05 \
  kp_yaw:=1.10 ki_yaw:=0.0 kd_yaw:=0.05 \
  kp_z:=0.80 ki_z:=0.0 kd_z:=0.06
```

### 6) X/Y 精调 + Yaw/Z 同时输入（推荐实测命令）

```bash
ros2 launch pid_tuning pid_tuning.launch.py \
  use_rqt:=false \
  csv_prefix:=pid_tuning_xy_fine \
  kp_x:=0.85 ki_x:=0.0 kd_x:=0.05 \
  kp_y:=0.85 ki_y:=0.0 kd_y:=0.05 \
  kp_yaw:=0.90 ki_yaw:=0.0 kd_yaw:=0.05 \
  target_z:=2.5 kp_z:=0.80 ki_z:=0.0 kd_z:=0.06 \
  vx_limit:=1.0 vy_limit:=1.0 \
  velocity_deadband:=0.03 yaw_rate_deadband:=0.03
```

### 7) X/Y 速度上限分开设置

默认使用统一上限 `vxy_limit`。若需要分别限制 X/Y，可显式传入 `vx_limit` 与 `vy_limit`：

```bash
ros2 launch pid_tuning pid_tuning.launch.py \
  use_rqt:=false \
  kp_x:=0.6 ki_x:=0.02 kd_x:=0.02 \
  kp_y:=0.6 ki_y:=0.02 kd_y:=0.02 \
  vx_limit:=1.2 \
  vy_limit:=0.8
```

说明：
- 不传 `vx_limit`/`vy_limit` 时，会自动回退到 `vxy_limit`。
- 传了 `vx_limit`/`vy_limit` 后，分别作用于 `pid_x` 和 `pid_y` 输出限幅。

## 关键默认值

- `target_z=2.5`（Z 反馈源为 `aruco_pose.z`）
- `kp_xy=0.5, ki_xy=0.0, kd_xy=0.08`
- `kp_x/kp_y` 默认 `nan`（未设置时自动回退到 `kp_xy`）

## 输出与汇总

- 单次 CSV 目录默认：`/home/zjh/project/rasip_pi_ws/log/tracking_csv`
- 单次 CSV 前缀默认：`pid_tuning`
- 汇总 CSV 默认：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/pid_tuning_summary.csv`
- 单次 CSV 实际文件示例：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/pid_tuning_xy_fine_YYYYMMDD_HHMMSS.csv`
- 树莓派部署验证文件示例：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/pi_pid_tuning_deploy_test_20260313_210208.csv`

结束（`Ctrl-C`）时会：

- 在终端打印：`RMSE/P95/cmd_jitter/max_stale/fresh_ratio`
- 在汇总 CSV 追加一行（含本次 PID 参数快照与指标结果）
- 自动将“本次运行 CSV + 汇总 CSV”复制到 `~/桌面/trackingcsv`（可配置）

## 结束自动同步到桌面

默认已开启：结束时自动把本次 CSV 与 `pid_tuning_summary.csv` 复制到 `sync_target_dir`。

```bash
ros2 launch pid_tuning pid_tuning.launch.py \
  sync_on_shutdown:=true \
  sync_target_dir:=/home/zjh/桌面/trackingcsv
```

若在树莓派上希望额外触发远程同步（例如推送到 laptop），可通过 `sync_cmd` 传入自定义命令：

```bash
ros2 launch pid_tuning pid_tuning.launch.py \
  sync_on_shutdown:=true \
  sync_cmd:=\"rsync -av {run_csv} {summary_csv} zjh@<laptop_ip>:/home/zjh/桌面/trackingcsv/\"
```
