# land_with_tracking_v2

`land_with_tracking_v2` 任务流程：

1. `ALIGN`：分两步对齐。
2. `ALIGN/YAW_ONLY`：先仅调整 `yaw`，`x/y/z` 保持不动。
3. `ALIGN/XYZ_ALIGN`：再调整 `x/y/z` 到目标（默认 `target_z=2.5`），同时继续 `yaw` 闭环。
4. `HOVER_BEFORE_LAND`：对齐后悬停 `1s`。
5. `DESCEND_WITH_TRACK`：边下降边闭环控制 `xy+yaw`。
6. `TOUCHDOWN_DISARM`：最低油门下压并每秒请求 `disarm`。
7. `DONE`：任务完成。

## 默认 PID 参数（按需求）

- `kp_x:=0.6 ki_x:=0.0 kd_x:=0.02`
- `kp_y:=0.6 ki_y:=0.0 kd_y:=0.02`
- `kp_yaw:=0.60 ki_yaw:=0.0 kd_yaw:=0.02`
- `kp_z:=0.60 ki_z:=0.0 kd_z:=0.06`
- `track_target_z:=2.5`
- `vx_limit:=1.0 vy_limit:=1.0`
- `velocity_deadband:=0.03 yaw_rate_deadband:=0.03`

## XY 控制说明

- `land_with_tracking_v2` 当前采用机体误差直控 `xy`（对齐 `yaw_then_xy_tracking` 思路）。
- 不使用动态 yaw 误差旋转链路。

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
ros2 launch land_with_tracking_v2 land_with_tracking_v2.launch.py
```
