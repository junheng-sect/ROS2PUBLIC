# marker_in_cam

`marker_in_cam` 用于把 `/debug/tvec` 中的 `tvec/rvec` 转换为：
marker 在 camera optical 坐标系下的 `x/y/z/yaw`，并发布到 `/debug/marker_in_cam`。
其中坐标定义为：`x向右、y向下、z向前`。

## 输入输出

- 输入：`/debug/tvec`（`debug_interface/msg/TVecRVec`）
- 输出：`/debug/marker_in_cam`（`debug_interface/msg/ArucoBasePose`）
- 日志：`1Hz`

## 启动

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select marker_in_cam
source install/setup.bash
ros2 launch marker_in_cam marker_in_cam_with_tvec.launch.py world_name:=rover
```
