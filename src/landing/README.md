# landing

`landing` 功能包用于执行自动降落流程：

1. 进入 `OFFBOARD` 后，以固定速度 `0.5m/s` 向下下降。
2. 参考 PX4 落地检测思路进行落地判定：
   - 优先读取 `/mavros/extended_state` 的 `landed_state==ON_GROUND`；
   - 同时提供启发式兜底（低高度 + 低垂速 + 低水平速度 + 持续时间）。
3. 判定落地后自动调用 `/mavros/cmd/arming` 发送 `disarm`。
4. `armed=false` 后认为降落完成，输出零速。

## 主要话题与服务

- 订阅：`/mavros/state`（`mavros_msgs/msg/State`）
- 订阅：`/mavros/extended_state`（`mavros_msgs/msg/ExtendedState`）
- 订阅：`/mavros/global_position/rel_alt`（`std_msgs/msg/Float64`）
- 订阅：`/mavros/local_position/velocity_local`（`geometry_msgs/msg/TwistStamped`）
- 发布：`/mavros/setpoint_velocity/cmd_vel`（`geometry_msgs/msg/TwistStamped`）
- 服务：`/mavros/cmd/arming`（`mavros_msgs/srv/CommandBool`）

## 运行

```bash
ros2 launch landing landing.launch.py
```

## 关键参数

- `descent_speed_mps`：下降速度，默认 `0.5`
- `land_rel_alt_threshold_m`：落地高度阈值，默认 `0.15`
- `land_vz_abs_max_mps`：垂向速度阈值，默认 `0.20`
- `land_vxy_abs_max_mps`：水平速度阈值，默认 `0.25`
- `land_detect_hold_sec`：启发式判定持续时间，默认 `1.0`
- `allow_heuristic_disarm_fallback`：若 landed_state 长时间不置位，是否允许启发式兜底触发 disarm，默认 `True`
- `heuristic_disarm_hold_sec`：启发式满足多长时间后允许兜底 disarm，默认 `3.0`
- `min_throttle_descent_speed_mps`：兜底 disarm 前的最低油门下压速度，默认 `0.35`
- `min_throttle_disarm_duration_sec`：最低油门阶段持续时间，默认 `5.0`；该阶段内按 `disarm_retry_interval_sec` 周期发送 disarm
- `disarm_retry_interval_sec`：disarm 失败重试间隔，默认 `1.0`
