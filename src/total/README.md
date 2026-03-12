# total

`total` 将返航、视觉对正与降落整合为一个总任务节点，流程如下：

1. 进入 `OFFBOARD` 后先升到 `3m`。
2. 通过 GPS 返回 `home` 点。
3. 到达后根据 ArUco 视觉进行对准和正对。
4. 对准成功后悬停 `1s`。
5. 进入降落阶段，持续下压直至 `disarm` 完成。

默认启用“OFFBOARD 进入时锁定当前高度”：即进入 OFFBOARD 上升沿时记录当前 `rel_alt` 作为全任务高度参考。

## 主要话题

- 订阅：`/mavros/state`
- 订阅：`/mavros/extended_state`
- 订阅：`/mavros/global_position/global`
- 订阅：`/mavros/home_position/home`
- 订阅：`/mavros/global_position/rel_alt`
- 订阅：`/mavros/local_position/velocity_local`
- 订阅：`/mavros/local_position/pose`
- 订阅：`/debug/aruco_pose`
- 发布：`/mavros/setpoint_velocity/cmd_vel`
- 服务：`/mavros/cmd/arming`

## 运行

```bash
ros2 launch total total.launch.py
```

## 阶段说明

- `ASCEND`：升到 3m。
- `RETURN`：GPS 返航至 home。
- `TRACK`：ArUco 对准与正对。
- `HOVER_BEFORE_LAND`：对准成功后悬停 1s。
- `LAND`：按 `0.5m/s` 降落。
- `TOUCHDOWN_DISARM`：最低油门下压并每秒申请 disarm。
- `DONE`：`armed=false` 后任务结束。
