# tvec_tf

`tvec_tf` 将 `/debug/tvec` 转换为 TF 广播：
- 父坐标系：`arucomarker`
- 子坐标系：`vision_pose`
- 平移映射：
  - `trans.x = tvec[1]`
  - `trans.y = tvec[0]`
  - `trans.z = tvec[2]`
- 旋转：由 `rvec` 提取 `yaw`，并写入 `arucomarker->vision_pose` 的 TF 旋转（仅绕 Z 轴）

并发布调试位姿话题：
- 话题：`/debug/aruco_pose`
- 类型：`debug_interface/msg/ArucoBasePose`
- 内容：计算后的 `x/y/z/yaw`，以及从 `tvec_rvec_node` 透传下来的 timing 字段

## 时序埋点

`/debug/aruco_pose` 现在会保留：

- `header.stamp`：原始 `/image_raw.header.stamp`
- 上游 `tvec_*` 时间戳
- 本级 `tf_cb_start_stamp`
- 本级 `tf_pub_stamp`

这样控制节点和 CSV logger 可以继续沿用原始图像时间作为基准，同时拆分出第 2 级和第 3 级各自的排队/计算耗时。

## 一键启动（含图像与 RViz）

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select tvec_tf
source install/setup.bash
ros2 launch tvec_tf tvec_tf.launch.py world_name:=rover
```
