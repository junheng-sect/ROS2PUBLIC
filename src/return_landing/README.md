# return_landing

`return_landing` 是一个实机任务功能包：
进入 `OFFBOARD` 后，按状态机自动执行：

1. `ASCEND`：爬升到目标高度（默认 `3.0m`）
2. `RETURN`：按 GPS 返回 home 点上方
3. `LAND`：固定下降速度降落
4. `TOUCHDOWN_DISARM`：接地后自动发送 `disarm`
5. `DONE`：任务完成后保持零速

## 话题与服务

- 订阅：`/mavros/state`（`mavros_msgs/msg/State`）
- 订阅：`/mavros/extended_state`（`mavros_msgs/msg/ExtendedState`）
- 订阅：`/mavros/global_position/global`（`sensor_msgs/msg/NavSatFix`）
- 订阅：`/mavros/home_position/home`（`mavros_msgs/msg/HomePosition`）
- 订阅：`/mavros/global_position/rel_alt`（`std_msgs/msg/Float64`）
- 订阅：`/mavros/local_position/velocity_local`（`geometry_msgs/msg/TwistStamped`）
- 发布：`/mavros/setpoint_velocity/cmd_vel`（`geometry_msgs/msg/TwistStamped`）
- 服务：`/mavros/cmd/arming`（`mavros_msgs/srv/CommandBool`）

## 运行

```bash
ros2 launch return_landing return_landing.launch.py
```

## 注意事项

- 若你的 MAVROS 前缀是 `/uas1/mavros`，请在 launch 参数中把所有 `/mavros/...` 改为 `/uas1/mavros/...`。
- 本功能包默认要求飞控进入 `OFFBOARD` 后才开始任务。
