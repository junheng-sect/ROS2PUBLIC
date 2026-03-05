# tcec_test

用于分析 ArUco 检测输出 `tvec/rvec` 与坐标系关系的测试功能包。

## 功能
- 订阅相机图像并检测 ArUco。
- 计算并输出每次检测的相对位姿 `tvec/rvec`。
- 以 **1Hz** 日志输出最新 `tvec/rvec`。
- 将 `tvec/rvec` 按时间戳写入 CSV。
- 发布标注图像到 `/tcec_test/image_annotated`，并在 RViz 显示。

## 启动
```bash
cd ~/project/project_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tcec_test tcec_test.launch.py world_name:=rover
```

可选参数：
- `world_name`：默认 `aruco`
- `model_name`：默认 `x500_mono_cam_down_0`
- `ros_image_topic`：默认 `/camera/image_raw`

## 日志与CSV
- 1Hz 日志示例：
  - `tvec=(tx, ty, tz)`
  - `rvec=(rx, ry, rz)`
- CSV 默认目录：
  - `~/project/project_ws/src/tcec_test/log/`

## 目的
用于彻底理清 `tvec/rvec` 与相机坐标系、标记坐标系之间的对应关系，便于后续 TF 设计与控制器建模。
