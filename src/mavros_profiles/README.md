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
- `global_position`
- `altitude`
- `distance_sensor`
- `home_position`
- `setpoint_velocity`
- `setpoint_raw`

## 启动命令

实机默认示例（默认串口与 `mavros px4.launch` 保持一致）：

```bash
ros2 launch mavros_profiles workspace_minimal_px4.launch.py
```

若串口不是 `/dev/ttyS0:921600`，可手动覆盖：

```bash
ros2 launch mavros_profiles workspace_minimal_px4.launch.py \
  fcu_url:=/dev/serial0:921600
```

如果你使用的命名空间不是默认的 `mavros`，可覆盖：

```bash
ros2 launch mavros_profiles workspace_minimal_px4.launch.py \
  namespace:=uas1/mavros
```

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

## 说明

- 本包只裁剪 MAVROS 插件，不会修改任何现有 tracking/landing/return 功能包。
- 如果后续新增功能包开始依赖新的 MAVROS 话题或服务，需要同步更新 `config/workspace_minimal_pluginlists.yaml`。
- 若需要完整插件集，仍可继续使用系统自带的：

```bash
ros2 launch mavros px4.launch
```
