# yaw_then_xy_tracking

`yaw_then_xy_tracking` 实现两阶段控制逻辑：

1. 第一阶段仅控制 `yaw`，`x/y/z` 全部保持不动；
2. `yaw` 对正后，第二阶段开始控制 `x/y/z` 到目标（同时 `yaw` 继续 PID 闭环）。

控制输出使用 `mavros_msgs/msg/PositionTarget`，坐标系为 `FRAME_BODY_NED`。

## 运行

```bash
ros2 launch yaw_then_xy_tracking yaw_then_xy_tracking.launch.py
```

## 默认 PID 参数（按你的要求）

- `kp_x:=0.6 ki_x:=0.0 kd_x:=0.02`
- `kp_y:=0.6 ki_y:=0.0 kd_y:=0.02`
- `kp_yaw:=0.60 ki_yaw:=0.0 kd_yaw:=0.02`
- `target_z:=2.5 kp_z:=0.60 ki_z:=0.0 kd_z:=0.06`
- `vx_limit:=1.0 vy_limit:=1.0`
- `velocity_deadband:=0.03 yaw_rate_deadband:=0.03`

## 阶段切换参数

- `yaw_align_threshold_deg`：yaw 对正阈值（默认 `5.0` 度）
- `yaw_align_hold_sec`：yaw 持续满足阈值时间（默认 `0.5` 秒）

## 典型命令（显式传参）

```bash
[ros2 launch yaw_then_xy_tracking yaw_then_xy_tracking.launch.py \
  kp_x:=0.6 ki_x:=0.0 kd_x:=0.02 \
  kp_y:=0.6 ki_y:=0.0 kd_y:=0.02 \
  kp_yaw:=0.60 ki_yaw:=0.0 kd_yaw:=0.02 \
  target_z:=2.5 kp_z:=0.60 ki_z:=0.0 kd_z:=0.06 \
  vx_limit:=1.0 vy_limit:=1.0 \
  velocity_deadband:=0.03 yaw_rate_deadband:=0.03]
```

## CSV 记录与 Summary

- 已内置 `yaw_then_xy_tracking_csv_logger_node`，默认随 launch 启动（`enable_csv_logger:=true`）。
- 运行明细 CSV 默认输出到：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/`
- Summary 默认输出到：`/home/zjh/project/rasip_pi_ws/log/tracking_csv/yaw_then_xy_tracking_summary.csv`
- 指标口径：仅使用 `OFFBOARD` **最后 5 秒**且 `aruco_fresh=1` 的样本计算 `RMSE/P95/CmdJitter`，避免初始状态差异影响稳定态精度比较。

关闭 CSV 记录：

```bash
ros2 launch yaw_then_xy_tracking yaw_then_xy_tracking.launch.py enable_csv_logger:=false
```
