# offboard_control

## 功能说明
`offboard_control` 是一个基于 ROS 2 Jazzy + MAVROS 的 Offboard 任务控制功能包。

当前提供节点：
- `offboard_mission_node`
- `offboard_random_mission_node`

当前任务流程：
1. 预发送 setpoint
2. 切换 `OFFBOARD`
3. 解锁（Arm）
4. 起飞至 `2.5m`（默认）
5. 悬停 `5s`（默认）
6. 切换 `AUTO.LAND`
7. 上锁（Disarm）

随机任务流程（`offboard_random_mission_node`）：
1. 预发送 setpoint
2. 切换 `OFFBOARD`
3. 解锁（Arm）
4. 起飞至 `2.5m`（默认）
5. 飞到随机目标点：`(x,y)` 位于以 `(0,0)` 为圆心、半径 `1` 的圆周上，`yaw` 随机
6. 悬停 `5s`（默认）
7. 返回原点上空 `(0,0,2.5)`（默认）
8. 切换 `AUTO.LAND`
9. 上锁（Disarm）

## 运行方式
```bash
source /opt/ros/jazzy/setup.bash
cd ~/project/project_ws
colcon build --symlink-install --packages-select offboard_control
source install/setup.bash
ros2 run offboard_control offboard_mission_node
ros2 run offboard_control offboard_random_mission_node
```

## 可调参数
- `takeoff_height`：起飞目标高度，默认 `2.5`
- `hover_time_sec`：悬停时长，默认 `5.0`
- `prestream_time_sec`：切换 Offboard 前 setpoint 预发送时间，默认 `1.5`
- `control_rate_hz`：控制循环频率，默认 `20.0`
- `height_tolerance`：到达目标高度判定阈值，默认 `0.15`
- `landed_height_threshold`：近地判定阈值，默认 `0.15`
- `radius`：随机点圆半径，默认 `1.0`（`offboard_random_mission_node`）

## 修改记录
- 2026-03-04：创建 `offboard_control` 功能包（`ament_python`，Apache-2.0）。
- 2026-03-04：新增 `offboard_mission_node`，实现“解锁→起飞 2.5m→悬停 5s→降落→上锁”流程。
- 2026-03-04：新增 `offboard_random_mission_node`，实现“解锁→起飞 2.5m→随机圆周点+随机 yaw→悬停 5s→回原点→降落→上锁”流程。
