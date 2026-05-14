# altitude_descent

基于 MAVROS 测距仪高度的定速下降功能包。

本包不会自动解锁、不会自动切换 `OFFBOARD`、不会自动上锁，只负责在安全条件满足时发布 Z 轴速度 setpoint。

## 控制逻辑

控制输出使用 `/mavros/setpoint_raw/local` 的 `mavros_msgs/msg/PositionTarget`：

- `vx=0`
- `vy=0`
- `yaw_rate=0`
- `vz` 由目标下降速度前馈和高度轨迹 PID 修正共同决定

`target_descent_speed` 使用正数表示向下下降速度，例如 `0.2` 表示目标向下 `0.2m/s`。节点内部发布的下降 `vz` 为负值。

首次进入有效控制时，节点以当前测距高度作为起点：

```text
reference_z = trajectory_start_z - target_descent_speed * elapsed_sec
height_error = reference_z - Range.range
vz = -target_descent_speed + PID(height_error)
```

高度来源默认使用 `/mavros/hrlv_ez4_pub` 的 `sensor_msgs/msg/Range.range`。

## 运行

```bash
ros2 launch altitude_descent altitude_descent.launch.py \
  target_descent_speed:=0.2 \
  kp_z:=0.8 ki_z:=0.0 kd_z:=0.06 \
  vz_limit:=0.3 \
  min_height_stop:=0.20 \
  enable_csv_logger:=true
```

## 主要参数

- `target_descent_speed`：目标下降速度，正数表示向下，默认 `0.2`
- `kp_z` / `ki_z` / `kd_z`：高度轨迹 PID 参数
- `pid_i_limit`：高度轨迹 PID 积分限幅
- `vz_limit`：Z 轴速度输出限幅
- `velocity_deadband`：小速度死区
- `min_height_stop`：低高度停止阈值，默认 `0.20m`
- `require_offboard`：为 `true` 时，仅在飞控模式为 `OFFBOARD` 时闭环控制
- `distance_sensor_topic`：测距仪话题，默认 `/mavros/hrlv_ez4_pub`
- `distance_sensor_timeout_sec`：测距仪超时阈值
- `motor_signal_source`：电机信号来源，默认 `auto`

## 电机 RPM / 油门信号

`motor_signal_source:=auto` 时按以下优先级选择当前记录来源：

1. `/mavros/esc_status/status`，读取 `ESCStatus.esc_status[].rpm`
2. `/mavros/esc_telemetry/telemetry`，读取 `ESCTelemetry.esc_telemetry[].rpm`
3. `/mavros/rc/out`，读取 `RCOut.channels` 的 PWM 并归一化为油门比例

ESC RPM 是否有数据取决于飞控、电调和 MAVLink 是否上报对应消息。若没有 RPM，本包会使用 RCOut PWM 作为能代表螺旋桨输出的替代观测量。

RCOut 默认从第 1 路开始取 4 个电机通道：

- `rc_motor_channel_offset:=0`
- `rc_motor_channel_count:=4`
- `rc_pwm_min:=1000.0`
- `rc_pwm_max:=2000.0`

## CSV 记录

默认随 launch 启动 `altitude_descent_csv_logger_node`：

- `enable_csv_logger:=true`
- `csv_output_dir:=/home/zjh/project/rasip_pi_ws/log/altitude_descent_csv`
- `csv_prefix:=altitude_descent`

输出文件名格式：

```text
altitude_descent_<YYYYMMDD_HHMMSS>.csv
```

主要字段包括：

- 飞控模式、解锁状态、连接状态
- 目标下降速度、起始高度、参考高度、测距高度、测距有效性
- 高度误差、PID 参数、PID 分项、`vz` 指令
- `setpoint_raw/local` frame、type_mask、速度字段
- ESC RPM、RCOut PWM、归一化油门比例
- 当前使用的电机信号来源和低高度停止状态

## 安全行为

- 非 `OFFBOARD` 且 `require_offboard:=true` 时输出零速并重置 PID。
- 测距仪无数据、超时、非有限数或超出有效量程时输出零速并重置 PID。
- `Range.range <= min_height_stop` 时输出零速并重置 PID。
- 本包不自动解锁、不自动切换飞控模式、不自动上锁。
