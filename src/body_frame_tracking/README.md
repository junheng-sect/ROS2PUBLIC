# body_frame_tracking

`body_frame_tracking` 是一个新的独立功能包，用于**直接在机体坐标系输出速度指令**，
不修改现有动态 yaw->ENU 变换链路。

## 功能说明

1. 读取 `/debug/aruco_pose` 与 `/mavros/state`
2. 在误差坐标下直接做 `x/y/yaw` PID 闭环（不做动态 yaw 旋转到 ENU）
3. 输出到 `/mavros/setpoint_raw/local`，坐标系使用 `FRAME_BODY_NED`
4. 默认仅控制 `x/y/yaw`，`z` 默认不控（`vz=0`）

## 坐标与符号

- 控制内部按 FLU 语义理解：`x前 / y左 / z上`
- 输出使用 `BODY_NED`：`x前 / y右 / z下`
- 发布前自动转换：
  - `vy_ned = -vy_flu`
  - `vz_ned = -vz_flu`

## 主要话题

- 订阅：`/debug/aruco_pose`（`debug_interface/msg/ArucoBasePose`）
- 订阅：`/mavros/state`（`mavros_msgs/msg/State`）
- 发布：`/mavros/setpoint_raw/local`（`mavros_msgs/msg/PositionTarget`）

## 运行

```bash
ros2 launch body_frame_tracking body_frame_tracking.launch.py
```
