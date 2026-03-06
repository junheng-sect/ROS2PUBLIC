# rivz

`rivz` 是一个聚合启动包，用于一键启动：
- `tvec`（桥接 + 检测 + RViz）
- matrix 相关节点
  - `matrix_marker_in_cam_node`
  - `matrix_cam_in_marker_node`
  - `matrix_baselink_in_cam_node`
  - `matrix_baselink_in_marker_node`
- `tf_broadcast` 静态 TF 节点

## 启动

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select rivz
source install/setup.bash
ros2 launch rivz rivz_full.launch.py world_name:=rover
```
