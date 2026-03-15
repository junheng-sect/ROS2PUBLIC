# land_with_tracking

`land_with_tracking` 任务流程：

1. 进入 `OFFBOARD` 后先对齐 ArUco。
2. 对齐成功后悬停 `1s`。
3. 以 `0.3m/s` 下落，下降过程中持续执行 `xy+yaw` 闭环。
4. 进入落地末段下压并周期发送 `disarm`，直到 `armed=false`。

## 阶段

- `ALIGN`：视觉对齐阶段（严格阈值）。
- `HOVER_BEFORE_LAND`：对齐后悬停 1 秒。
- `DESCEND_WITH_TRACK`：边下降边闭环控制 `xy+yaw`。
- `TOUCHDOWN_DISARM`：最低油门下压并每秒请求 `disarm`。
- `DONE`：任务完成。

## 默认对齐成功判据

- `xy <= 0.10m`
- `yaw <= 5deg`
- 连续保持 `1.0s`

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
