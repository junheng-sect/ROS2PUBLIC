# ArUco + PX4 无人机视觉导航系统

## 项目概述

这是一个基于 **ROS2 Jazzy** 的无人机视觉导航系统，主要功能包括：

- **ArUco 标记检测**：通过摄像头识别 ArUco 标记，计算其 6-DOF 位姿（位置 + 姿态）
- **Gazebo 仿真**：支持在 Gazebo Harmonic 环境中进行仿真测试
- **PX4 飞控集成**：与 PX4 飞控系统对接，实现基于视觉的位置控制
- **双模式支持**：支持 Gazebo 仿真模式和真实硬件模式

## 助理人设

**角色定位**：ROS 工程师和程序员

**技术栈**：
- 🐧 **系统**：Ubuntu 24.04
- 🤖 **框架**：ROS 2 Jazzy
- 🚁 **飞控**：PX4 v1.16.0（精通代码及 ROS 2↔PX4 通信）
- 🔗 **通信**：MAVROS（ROS 2 与 PX4 之间的通信桥接）
- 🌍 **仿真**：Gazebo Harmonic（精通仿真环境搭建与通信）
- 💻 **技能**：擅长编程，能根据需求编写带适当注释的代码

**开发规范**：
- 默认使用 `ament-python` 创建功能包
- 使用 `ament_cmake` 时需说明原因
- 协议默认使用 **Apache-2.0**

**工作空间**：`~/project/project_ws`

## 技术栈

| 组件 | 技术 |
|------|------|
| 框架 | ROS2 Jazzy |
| 语言 | Python 3.12 (ament_python) |
| 仿真 | Gazebo Harmonic + ros_gz_bridge |
| 飞控 | PX4 + MAVROS |
| 视觉 | OpenCV + cv_bridge |
| 摄像头驱动 | usb_cam / v4l2_camera |

## 项目结构

```
project_ws/
├── src/                          # 源代码目录
│   ├── aruco_interfaces/         # ArUco 自定义消息接口 (ArucoPose.msg)
│   ├── px4_interfaces/           # PX4 自定义服务接口 (SetTargetPosition.srv)
│   ├── aruco_detector/           # 真实摄像头 ArUco 检测节点
│   ├── aruco_detector_mid/       # 中带检测变体
│   ├── aruco_csv/                # 检测数据导出到 CSV
│   ├── aruco_threshold/          # 阈值处理变体
│   ├── aruco_gazebo/             # Gazebo 仿真 ArUco 检测节点
│   ├── aruco_gazebo_rviz/        # Gazebo + RViz 可视化
│   ├── aruco_tf_vision/          # ArUco+TF 整合包 (map→vision_pose+base_link)
│   ├── aruco_position_controller/# 基于 ArUco 视觉的位置控制器（PID 对齐）
│   ├── px4_position_controller/  # PX4 位置控制节点
│   ├── tf_display/               # TF 坐标显示工具
│   ├── camera_calibration_pkg/   # 相机标定工具
│   ├── camera_test/              # 相机测试工具
│   └── gazebo_image_test/        # Gazebo 图像测试
├── build/                        # 构建输出目录
├── install/                      # 安装目录
└── log/                          # 日志目录
```

## 构建与运行

### 环境准备

```bash
# 设置 ROS2 环境
source /opt/ros/jazzy/setup.bash
```

### 构建项目

```bash
# 在工作空间根目录执行
colcon build --symlink-install
source install/setup.bash
```

### 常用 Launch 命令

**PX4 Gazebo 仿真:**
```bash
make px4_sitl gz_x500_mono_cam_down_aruco
```

**MAVROS 启动:**
```bash
ros2 launch mavros px4.launch
```

**ArUco 检测（真实相机）:**
```bash
ros2 launch aruco_detector aruco_system.launch.py
```

**Gazebo 仿真 + ArUco 检测:**
```bash
ros2 launch aruco_gazebo aruco_gazebo.launch.py
```

**ArUco + TF 可视化（整合包）:**
```bash
ros2 launch aruco_tf_vision aruco_tf_vision.launch.py
```

**ArUco 位置控制（视觉对齐）:**
```bash
ros2 launch aruco_position_controller aruco_position_control.launch.py
```

**TF 显示（仅 MAVROS + TF）:**
```bash
ros2 launch tf_display tf_display.launch.py
```

**相机标定:**
```bash
ros2 launch camera_calibration_pkg camera_calibration.launch.py
```

**PX4 位置控制:**
```bash
ros2 run px4_position_controller position_controller_node
```

### 自定义服务调用

```bash
# 设置目标位置 (PX4 控制器)
ros2 service call /set_target_position px4_interfaces/srv/SetTargetPosition "{x: 1.0, y: 0.0, z: 2.0}"

# 启用 ArUco 位置对齐
ros2 service call /enable_alignment std_srvs/srv/SetBool "{data: true}"

# 重置 PID 积分
ros2 service call /reset_pid std_srvs/srv/SetBool "{data: true}"
```

### 自定义消息接口

**ArucoPose.msg** - ArUco 标记位姿消息：
```
int32 id           # 标记 ID
float32 x, y, z    # 位置 (米)
float32 roll, pitch, yaw  # 欧拉角 (弧度)
```

**SetTargetPosition.srv** - PX4 目标位置服务：
```
# Request
float64 x, y, z

# Response
bool success
string message
```

## TF Tree 结构

```
map ├── base_link (MAVROS 位置，50Hz)
    └── vision_pose (ArUco 视觉估计)
```

**坐标系约定：**
- **map**: 世界坐标系（ENU：东 - 北 - 天），起飞点
- **base_link**: 无人机机体坐标系
- **vision_pose**: 基于 ArUco 的视觉估计位姿
- **camera_link**: 相机坐标系（X 向前，Y 向右，Z 向下）

**位置坐标约定：**
- X 正：向东（右）
- Y 正：向北（前）
- Z 正：向上（天）
- 位置为相对起飞点的偏移

## 关键配置

### Gazebo 相机参数 (SDF: 640x480, hfov=1.74 rad)

```python
fx = fy = 268.5  # width / (2 * tan(hfov/2))
cx = 320.0
cy = 240.0
```

### ArUco 标记尺寸

| 标记 ID | 尺寸 | 说明 |
|--------|------|------|
| 33 | 0.5m × 0.5m | 远距离标记 |
| 42 | 0.063m × 0.063m | 近距离标记 |

### 坐标修正参数

```python
# 位置修正
vision_x = -tvec[0] + vision_offset_x  # X 取反
vision_y = tvec[1] + vision_offset_y   # Y 不取反
vision_z = tvec[2] + vision_offset_z   # Z 不取反

# 旋转修正：绕 X 轴 180° + 四元数求逆 + 绕 Z 轴 +90°
vision_offset_z = -0.167  # 消除初始高度偏移
```

### VSCode 配置

```json
{
    "ROS2.distro": "jazzy",
    "python.autoComplete.extraPaths": [
        "/opt/ros/jazzy/lib/python3.12/site-packages"
    ],
    "python.analysis.extraPaths": [
        "/opt/ros/jazzy/lib/python3.12/site-packages"
    ]
}
```

### 相机标定

- 标定板内角点数量 = 方格数 - 1（如 8×6 方格对应 7×5 内角点）
- 标定文件位于：`~/.ros/camera_info/*.yaml`

## 开发规范

### 代码风格

- 使用 Python 3.12
- 遵循 PEP8 规范
- 使用 `ament_flake8` 和 `ament_pep257` 进行代码检查

### 测试

```bash
# 运行代码检查
colcon test

# 运行 pytest
pytest
```

### 包命名约定

- `aruco_*`：ArUco 视觉相关功能
- `px4_*`：PX4 飞控相关功能
- `camera_*`：相机驱动和标定
- `tf_*`：坐标变换相关

## 常用命令

```bash
# 查看话题列表
ros2 topic list

# 查看节点列表
ros2 node list

# 查看 ArUco 位姿话题
ros2 topic echo /aruco_pose

# 查看图像话题频率
ros2 topic hz /camera/image_raw

# 启动 rqt 图像查看
rqt_image_view

# 查看 Gazebo 话题
gz topic -l | grep image
```

## 日志文件

- ArUco 检测日志：`src/aruco_gazebo/aruco_log/gazebo_detection_log.csv`
- 系统日志：`log/` 目录

## 关键依赖

```bash
# ROS2 Jazzy 核心
sudo apt install ros-jazzy-rclpy ros-jazzy-sensor-msgs ros-jazzy-cv-bridge

# Gazebo 桥接
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim

# MAVROS
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-msgs

# TF 变换
sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf-transformations

# 相机驱动
sudo apt install ros-jazzy-usb-cam ros-jazzy-v4l2-camera

# 可视化工具
sudo apt install ros-jazzy-rviz2 ros-jazzy-rqt-image-view
```
