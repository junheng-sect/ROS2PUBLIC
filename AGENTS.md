# 项目初始化说明

## 代理人设与技术栈

**技术栈**：
- 🐧 **系统**：Ubuntu 24.04
- 🤖 **框架**：ROS 2 Jazzy
- 🚁 **飞控**：PX4 v1.16.0（精通代码及 ROS 2↔PX4 通信）
- 🔗 **通信**：MAVROS（ROS 2 与 PX4 之间的通信桥接）
- 🌍 **仿真**：Gazebo Harmonic（精通仿真环境搭建与通信）
- 💻 **技能**：擅长编程，能根据需求编写带适当注释的代码

**工作空间**：`~/project/project_ws`

## 协作与记录要求

- 每次创建功能包后，生成对应 `readme.md`。
- 在本 `AGENTS.md` 中持续记录每次进行的修改。
- 每完成一轮对话，更新一次本文件的“修改记录”。

## 开发规范

- 默认使用 `ament-python` 创建功能包。
- 使用 `ament_cmake` 时需说明原因。
- 协议默认使用 **Apache-2.0**。

## 项目概述

本仓库是一个基于 ROS 2 Jazzy 的工作空间，用于 ArUco 视觉与 PX4 无人机控制。
支持仿真（Gazebo + RViz）和实机两类流程。

核心能力：
- ArUco 检测与位姿解算
- 基于视觉的跟踪、位置控制与降落
- 通过服务与消息接口和 PX4 交互
- 相机标定与图像链路验证

## 工作空间结构

```text
project_ws/
├── src/        # ROS 2 功能包源码（主目录）
├── build/      # colcon 构建产物
├── install/    # colcon 安装产物
└── log/        # 运行与构建日志
```

`src/` 下主要功能包：
- `aruco_detector`, `aruco_detector_mid`, `aruco_threshold`, `aruco_csv`
- `aruco_gazebo`, `aruco_gazebo_rviz`, `aruco_tf_vision`
- `aruco_position_controller`, `aruco_tracking`, `aruco_landing`
- `px4_position_controller`, `px4_disarm`
- `aruco_interfaces`, `px4_interfaces`
- `camera_test`, `camera_calibration_pkg`, `gazebo_image_test`, `tf_display`

## 环境要求

推荐基础环境：
- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- colcon

## 构建与运行

```bash
source /opt/ros/jazzy/setup.bash
cd /home/zjh/project/project_ws
colcon build --symlink-install
source install/setup.bash
```

常用启动示例：

```bash
ros2 launch aruco_detector aruco_system.launch.py
ros2 launch aruco_gazebo aruco_gazebo.launch.py
ros2 launch aruco_gazebo_rviz aruco_gazebo_rviz.launch.py
ros2 launch aruco_tracking aruco_tracking.launch.py
ros2 launch aruco_landing aruco_landing.launch.py
```

## 接口定义

自定义接口：
- `aruco_interfaces/msg/ArucoPose.msg`
- `px4_interfaces/srv/SetTargetPosition.srv`
- `px4_interfaces/srv/Disarm.srv`

## 调试命令

```bash
ros2 node list
ros2 topic list
ros2 topic echo /aruco_pose
ros2 topic hz /camera/image_raw
```

常用日志/数据目录：
- `src/aruco_gazebo/aruco_log/`
- `src/aruco_csv/aruco_log/`
- `log/`

## 开发建议

- 保持“检测逻辑”和“控制逻辑”解耦。
- 保持仿真与实机的话题命名、坐标系约定一致。
- 在控制节点中加入安全参数（超时、限幅、失效保护等）。

## 修改记录

- 2026-03-03：新增“代理人设与技术栈”“协作与记录要求”“开发规范”章节，写入默认工作空间与文档维护规则。
- 2026-03-03：配置 Git 全局撤销别名（`undo`、`unstage`、`discard`、`last`），用于快速撤销工作区/暂存区改动。
- 2026-03-03：完成仓库初始化后的基础整理：分支由 `master` 调整为 `main`，新增 `.gitignore`（忽略 `build/install/log`、IDE 与缓存文件）。
- 2026-03-03：完成首个提交（commit: `043cd45`），绑定远程仓库 `origin=https://github.com/junheng-sect/junhengWS.git`；推送阶段因 GitHub 凭据未配置而中断。
- 2026-03-03：按 HTTPS 方案配置 `git credential.helper=store`，等待提供 GitHub PAT 后继续执行 `git push -u origin main`。
