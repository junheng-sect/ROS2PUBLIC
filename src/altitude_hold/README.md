# altitude_hold

基于 MAVROS 测距仪高度的定高悬停功能包。

控制节点只输出 Z 轴速度闭环：

- `vx=0`
- `vy=0`
- `yaw_rate=0`
- `vz` 由测距仪高度 PID 输出

高度来源默认使用 `/mavros/hrlv_ez4_pub` 的 `sensor_msgs/msg/Range.range`。

## 运行

```bash
ros2 launch altitude_hold altitude_hold.launch.py \
  target_z:=1.5 \
  kp_z:=0.8 ki_z:=0.0 kd_z:=0.06 \
  vz_limit:=0.3 \
  enable_csv_logger:=true
```

`target_z` 单位为米，可以在启动命令中直接透传目标定高高度。

## 主要参数

- `target_z`：目标离地高度，默认 `1.5`
- `kp_z` / `ki_z` / `kd_z`：高度 PID 参数
- `pid_i_limit`：高度 PID 积分限幅
- `vz_limit`：Z 轴速度输出限幅
- `velocity_deadband`：小速度死区
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

默认随 launch 启动 `altitude_hold_csv_logger_node`：

- `enable_csv_logger:=true`
- `csv_output_dir:=/home/zjh/project/rasip_pi_ws/log/altitude_hold_csv`
- `csv_prefix:=altitude_hold`

输出文件名格式：

```text
altitude_hold_<YYYYMMDD_HHMMSS>.csv
```

主要字段包括：

- 飞控模式、解锁状态、连接状态
- 目标高度、测距高度、测距有效性、高度误差
- PID 参数、`vz` 指令、`setpoint_raw/local` 原始字段
- ESC RPM、RCOut PWM、归一化油门比例
- 当前使用的电机信号来源和控制节点状态文本

## 安全行为

- 非 `OFFBOARD` 且 `require_offboard:=true` 时输出零速并重置 PID。
- 测距仪无数据、超时、非有限数或超出有效量程时输出零速并重置 PID。
- 本包不自动解锁、不自动切换飞控模式。
