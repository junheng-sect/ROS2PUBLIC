# matrix_baselink_in_cam

`matrix_baselink_in_cam` 用于发布固定外参矩阵 `baselink->cam`：
- 平移：`x=y=z=0`
- 旋转轴约定：
  - `baselink_x` 指向 `cam_-y`
  - `baselink_y` 指向 `cam_-x`
  - `baselink_z` 指向 `cam_-z`

输出话题：`/debug/matrix/baselink_in_cam`
消息类型：`debug_interface/msg/TransformMatrix4x4`

## 运行

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select matrix_baselink_in_cam
source install/setup.bash
ros2 run matrix_baselink_in_cam matrix_baselink_in_cam_node
```
