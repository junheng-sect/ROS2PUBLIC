# matrix_marker_in_cam

`matrix_marker_in_cam` 用于将 `/debug/tvec` 的 `tvec/rvec` 转换为
`marker -> camera` 的 4x4 齐次变换矩阵，并发布到 `/debug/matrix/marker_in_cam`。

## 输入输出

- 输入：`/debug/tvec`（`debug_interface/msg/TVecRVec`）
- 输出：`/debug/matrix/marker_in_cam`（`debug_interface/msg/TransformMatrix4x4`）
- 日志：`1Hz`

## 启动（含 tvec）

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select matrix_marker_in_cam
source install/setup.bash
ros2 launch matrix_marker_in_cam matrix_marker_in_cam_with_tvec.launch.py world_name:=rover
```
