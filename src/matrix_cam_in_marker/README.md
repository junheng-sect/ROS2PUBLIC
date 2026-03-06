# matrix_cam_in_marker

`matrix_cam_in_marker` 用于将 `/debug/matrix/marker_in_cam` 的 4x4 变换矩阵做反方向变换，
得到 `cam->marker` 变换矩阵，并发布到 `/debug/matrix/cam_in_marker`。

## 输入输出

- 输入：`/debug/matrix/marker_in_cam`（`debug_interface/msg/TransformMatrix4x4`）
- 输出：`/debug/matrix/cam_in_marker`（`debug_interface/msg/TransformMatrix4x4`）
- 日志：`1Hz`

## 运行

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select matrix_cam_in_marker
source install/setup.bash
ros2 run matrix_cam_in_marker matrix_cam_in_marker_node
```

## 联合启动（tvec + marker_in_cam + matrix_marker_in_cam + matrix_cam_in_marker）

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select matrix_cam_in_marker
source install/setup.bash
ros2 launch matrix_cam_in_marker matrix_cam_in_marker_full.launch.py world_name:=rover
```
