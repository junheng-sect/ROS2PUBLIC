# tf_broadcast

`tf_broadcast` 提供静态 TF 广播节点：
`map(ENU) -> aruco_marker(NWU)`。

默认参数：
- 平移：`(0.0, 0.0, 0.3)` m
- 旋转：绕 `Z` 轴 `+90°`（等价于 ENU 到 NWU 的坐标轴关系）

## 运行

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select tf_broadcast
source install/setup.bash
ros2 run tf_broadcast map_aruco_static_tf_node
```

## 动态 TF 节点（订阅 Pose）

新增节点 `map_base_link_tf_node`：
- 订阅 `PoseStamped`（默认 `/mavros/local_position/pose`）
- 广播动态 TF：`map -> base_link`

运行命令：

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select tf_broadcast
source install/setup.bash
ros2 run tf_broadcast map_base_link_tf_node
```

## 矩阵转 TF 节点

新增节点：
- `cam_tf_node`
  - 订阅 `/debug/matrix/cam_in_marker`
  - 广播 `aruco_marker -> cam`
- `vision_pose_tf_node`
  - 订阅 `/debug/matrix/baselink_in_marker`
  - 广播 `aruco_marker -> vision_pose`
