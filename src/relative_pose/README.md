# relative_pose

`relative_pose` 用于将 `/debug/tvec` 中的 `tvec/rvec` 转换为：
“ArUco 坐标系（FLU）下机体坐标系（FLU）的 `x/y/z/yaw`”，并发布到 `/debug/relative_pose`。

## 输入输出

- 输入话题：`/debug/tvec`（`debug_interface/msg/TVecRVec`）
- 输出话题：`/debug/relative_pose`（`debug_interface/msg/ArucoBasePose`）
- 日志频率：`1Hz`

## 坐标变换说明

1. `tvec/rvec` 先按 OpenCV 语义解释为 `marker -> camera`（optical 坐标）
2. 通过固定矩阵将 optical 坐标转换到 FLU 坐标
3. 对 `marker -> camera` 取逆，得到 ArUco 坐标系下机体（近似相机）位姿
4. 从旋转矩阵提取 `yaw`

## 运行

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select relative_pose
source install/setup.bash
ros2 run relative_pose relative_pose_node
```

## 联合启动（tvec + relative_pose）

```bash
cd ~/project/zjh_ws
colcon build --symlink-install --packages-select relative_pose
source install/setup.bash
ros2 launch relative_pose relative_pose_with_tvec.launch.py world_name:=rover
```
