# mavros_profiles

本功能包用于提供当前工作空间可复用的 MAVROS 最小化启动配置，不修改系统安装目录下的 `mavros` 文件。

## 目标

- 覆盖当前工作空间下实际会使用 MAVROS 的功能包。
- 尽量减少无关插件，降低机载电脑 CPU、内存和话题负担。
- 保留原有 `/opt/ros/.../mavros` 完整启动方式，便于随时切回。

## 当前最小插件集合

本配置覆盖以下接口需求：

- `/mavros/state`
- `/mavros/extended_state`
- `/mavros/local_position/pose`
- `/mavros/local_position/velocity_local`
- `/mavros/imu/data`
- `/mavros/global_position/global`
- `/mavros/global_position/rel_alt`
- `/mavros/altitude`
- `/mavros/distance_sensor/*`
- `/mavros/home_position/home`
- `/mavros/setpoint_velocity/cmd_vel`
- `/mavros/setpoint_raw/local`
- `/mavros/cmd/arming`

对应保留插件：

- `sys_*`
- `command`
- `local_position`
- `imu`
- `global_position`
- `altitude`
- `distance_sensor`
- `home_position`
- `setpoint_velocity`
- `setpoint_raw`

## 启动命令

实机默认示例（默认会自动探测常见飞控串口）：

```bash
ros2 launch mavros_profiles workspace_minimal_px4.launch.py
```

若希望启动时关闭 `imu` 插件，可直接使用：

```bash
ros2 launch mavros_profiles workspace_minimal_px4.launch.py \
  enable_imu:=false
```

若自动探测没有选到正确串口，可手动覆盖：

```bash
ros2 launch mavros_profiles workspace_minimal_px4.launch.py \
  fcu_url:=/dev/serial0:921600
```

如果你使用的命名空间不是默认的 `mavros`，可覆盖：

```bash
ros2 launch mavros_profiles workspace_minimal_px4.launch.py \
  namespace:=uas1/mavros
```

若你希望完全手动接管插件清单，也可以直接覆盖：

```bash
ros2 launch mavros_profiles workspace_minimal_px4.launch.py \
  pluginlists_yaml:=/home/zjh/project/rasip_pi_ws/src/mavros_profiles/config/workspace_minimal_pluginlists_no_imu.yaml
```

说明：

- `fcu_url:=auto` 时，本包会自动按平台探测常见飞控串口：
  - 树莓派/ARM：优先尝试 `/dev/serial0`、`/dev/ttyAMA0`、`/dev/ttyS0`
  - 通用 USB 飞控：优先尝试 `/dev/serial/by-id/*`、`/dev/ttyACM*`、`/dev/ttyUSB*`
- 若当前主机根本没有探测到飞控串口，launch 会直接报错并提示手动传入 `fcu_url`，不再误用笔记本上无效的 `/dev/ttyS0`
- `enable_imu:=true/false` 只用于在本包自带的两份最小插件清单之间切换
- 若同时传入 `pluginlists_yaml:=...`，则以你手动指定的 YAML 为准

## 适用范围

这套最小配置已考虑当前工作空间内常见 MAVROS 使用者：

- `aruco_tracking`
- `aruco_tracking_minimal`
- `body_frame_tracking`
- `body_frame_tracking_minimal`
- `landing`
- `land_with_tracking`
- `return_home`
- `return_track_hover`
- `total`
- 以及订阅 `/mavros/local_position/pose` 的 TF/记录节点
- 以及需要从 `/mavros/imu/data` 读取本机姿态的记录节点

## 说明

- 本包只裁剪 MAVROS 插件，不会修改任何现有 tracking/landing/return 功能包。
- 如果后续新增功能包开始依赖新的 MAVROS 话题或服务，需要同步更新 `config/workspace_minimal_pluginlists.yaml`。
- 若需要完整插件集，仍可继续使用系统自带的：

```bash
ros2 launch mavros px4.launch
```
