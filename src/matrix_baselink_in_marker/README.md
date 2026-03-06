# matrix_baselink_in_marker

`matrix_baselink_in_marker` 用于通过矩阵乘法将
`marker->cam` 与 `baselink->cam` 组合为 `marker->baselink`，
并发布到 `/debug/matrix/baselink_in_marker`。

## 输入输出

- 输入1：`/debug/matrix/marker_in_cam`（`debug_interface/msg/TransformMatrix4x4`）
- 输入2：`/debug/matrix/baselink_in_cam`（`debug_interface/msg/TransformMatrix4x4`）
- 输出：`/debug/matrix/baselink_in_marker`（`debug_interface/msg/TransformMatrix4x4`）
- 日志：`1Hz`

矩阵关系：
- `T_mb = inv(T_bc) @ T_mc`
- 其中：
  - `T_mc`：`marker->cam`
  - `T_bc`：`baselink->cam`
  - `T_mb`：`marker->baselink`

## 联合启动（包含相关节点）

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select matrix_baselink_in_marker
source install/setup.bash
ros2 launch matrix_baselink_in_marker matrix_baselink_in_marker_full.launch.py world_name:=rover
```
