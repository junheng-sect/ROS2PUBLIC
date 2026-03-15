# land_with_tracking

`land_with_tracking` 任务流程：

1. 进入 `OFFBOARD` 后先对齐 ArUco。
2. 对齐成功后悬停 `1s`。
3. 以 `0.5m/s` 下落，下降过程中持续执行 `xy+yaw` 闭环。
4. 进入落地末段下压并周期发送 `disarm`，直到 `armed=false`。

## 阶段

- `ALIGN`：分两步对齐。
- `ALIGN/YAW_ONLY`：先仅调整 `yaw`，`x/y/z` 保持不动。
- `ALIGN/XYZ_ALIGN`：再调整 `x/y/z` 到目标（默认 `target_z=2.5`），同时继续 `yaw` 闭环。
- `HOVER_BEFORE_LAND`：对齐后悬停 1 秒。
- `DESCEND_WITH_TRACK`：边下降边闭环控制 `xy+yaw`。
- `TOUCHDOWN_DISARM`：最低油门下压并每秒请求 `disarm`。
- `DONE`：任务完成。

## 默认对齐成功判据

- `xy <= 0.10m`
- `|z-target_z| <= 0.10m`（默认 `target_z=2.5`）
- `yaw <= 5deg`
- 连续保持 `1.0s`

## 默认 tracking 参数（ALIGN 与下降跟踪统一）

- `kp_x:=0.6 ki_x:=0.0 kd_x:=0.02`
- `kp_y:=0.6 ki_y:=0.0 kd_y:=0.02`
- `kp_yaw:=0.60 ki_yaw:=0.0 kd_yaw:=0.02`
- `target_z:=2.5 kp_z:=0.60 ki_z:=0.0 kd_z:=0.06`
- `vx_limit:=1.0 vy_limit:=1.0`
- `velocity_deadband:=0.03 yaw_rate_deadband:=0.03`
- 日志修正项：`track_vx_sign:=-1.0 track_vy_sign:=1.0`
- 阶段限幅：`align_max_vxy:=0.45 descend_max_vxy:=0.45`

## 主要话题

- 订阅：`/mavros/state`
- 订阅：`/mavros/extended_state`
- 订阅：`/mavros/global_position/rel_alt`
- 订阅：`/mavros/local_position/velocity_local`
- 订阅：`/mavros/local_position/pose`
- 订阅：`/debug/aruco_pose`
- 发布：`/mavros/setpoint_velocity/cmd_vel`
- 服务：`/mavros/cmd/arming`

## 运行

```bash
ros2 launch land_with_tracking land_with_tracking.launch.py
```
