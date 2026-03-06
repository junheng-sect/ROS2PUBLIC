# cam_in_marker

`cam_in_marker` 用于根据 `/debug/tvec` 的 `tvec/rvec`，
输出“cam 在 marker 坐标系（FLU）下”的 `x/y/z/yaw`，并发布到 `/debug/cam_in_marker`。

## 输入输出

- 输入：`/debug/tvec`（`debug_interface/msg/TVecRVec`）
- 输出：`/debug/cam_in_marker`（`debug_interface/msg/ArucoBasePose`）
- 日志：`1Hz`

## 联合启动（tvec + marker_in_cam + cam_in_marker）

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select cam_in_marker
source install/setup.bash
ros2 launch cam_in_marker cam_in_marker_with_tvec_marker_in_cam.launch.py world_name:=rover
```
