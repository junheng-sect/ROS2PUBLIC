# tvec

用于分析 ArUco 检测输出 `tvec/rvec` 与坐标系关系的测试功能包。

## 功能
- 订阅相机图像并检测 ArUco。
- 计算并输出每次检测的相对位姿 `tvec/rvec`。
- 以 **1Hz** 日志输出最新 `tvec/rvec`。
- 新增 `rvec_yaw_node`：从 `rvec` 计算相对 `yaw` 并以 **1Hz** 日志输出。
- 将 `tvec/rvec` 按时间戳写入 CSV。
- 发布标注图像到 `/tvec/image_annotated`，并在 RViz 显示。

## 启动
```bash
cd ~/project/zjh_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tvec tvec.launch.py world_name:=rover
```

可选参数：
- `world_name`：默认 `aruco`
- `model_name`：默认 `x500_mono_cam_down_0`
- `ros_image_topic`：默认 `/image_raw`

单独启动 yaw 计算节点：
```bash
cd ~/project/zjh_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tvec rvec_yaw_node
```

## 日志与CSV
- 1Hz 日志示例：
  - `tvec=(tx, ty, tz)`
  - `rvec=(rx, ry, rz)`
- CSV 默认目录：
  - `~/project/zjh_ws/src/tvec/log/`

## 目的
用于彻底理清 `tvec/rvec` 与相机坐标系、标记坐标系之间的对应关系，便于后续 TF 设计与控制器建模。
