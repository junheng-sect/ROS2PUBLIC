# 项目初始化说明

## 代理人设与技术栈

**技术栈**：
- 🐧 **系统**：Ubuntu 24.04
- 🤖 **框架**：ROS 2 Jazzy
- 🚁 **飞控**：PX4 v1.16.0（精通代码及 ROS 2↔PX4 通信）
- 🔗 **通信**：MAVROS（ROS 2 与 PX4 之间的通信桥接）
- 🌍 **仿真**：Gazebo Harmonic（精通仿真环境搭建与通信）
- 💻 **技能**：擅长编程，能根据需求编写带适当注释的代码

**工作空间**：`~/project/zjh_ws`

## 协作与记录要求

- 在本 `AGENTS.md` 中持续记录每次进行的修改。
- 当前默认工作空间固定为 `~/project/zjh_ws`，后续操作均以该路径为准。
- 创建功能包在把src路径下执行ros2 pkg create！
- 问的问题也进行记录，开一栏新的标题“问题记录”里面写问的问题和解答。
- 每轮对话都必须记录：修改了代码写入“修改记录”，解答了问题写入“问题记录”。
- 每完成一轮对话，更新一次本文件的“修改记录”。
- 功能包测试完成后，仅在你明确回复“成功”后执行 Git 提交并上传远程仓库。
- 每次我修改代码后，自动执行 `colcon build` 并执行 `source install/setup.bash`。
- 每次涉及 launch 的修改后，自动运行对应 `ros2 launch` 进行验证；确认启动成功后再停止。
- 功能包文档统一使用 `README.md`（全大写）；禁止生成小写 `readme.md`。
- 编写代码时必须添加详细注释，说明关键变量、计算过程与控制逻辑。
- 代码注释统一使用中文。
- 配置git功能使代码能够撤回，配置本地.git仓库。

## 开发规范

- 默认使用 `ament-python` 创建功能包。
- 使用 `ament_cmake` 时需说明原因。创建接口 interface 功能包时要使用 cmake。
- 协议默认使用 **Apache-2.0**。

## Git 分支策略

- `main`：用于 `Ubuntu 24.04 + ROS 2 Jazzy` 的日常程序开发与功能迭代。
- `ubuntu22.04-ros-humble`：用于 `Ubuntu 22.04 + ROS 2 Humble` 的版本适配与实机实验验证。
- `simple`：用于 `Ubuntu 24.04 + ROS 2 Jazzy` 的重要功能，和 `main` 的区别是 `simple` 没有测试用功能包。

## 项目概述

本仓库是一个基于 ROS 2 Jazzy 的工作空间，用于 ArUco 视觉与 PX4 无人机控制。
支持仿真（Gazebo + RViz）和实机两类流程。

## 工作空间结构

```text
zjh_ws/
├── src/        # ROS 2 功能包源码（主目录）
├── build/      # colcon 构建产物
├── install/    # colcon 安装产物
└── log/        # 运行与构建日志
```

## 全部功能包简要介绍

- （待补充）

## 环境要求

推荐基础环境：
- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- colcon

## 构建与运行

```bash
source /opt/ros/jazzy/setup.bash
cd /home/zjh/project/zjh_ws
colcon build --symlink-install
source install/setup.bash
```

## 接口定义

- （待补充）

## 调试命令

```bash
ros2 node list
ros2 topic list
```

## 开发建议

- 保持“检测逻辑”和“控制逻辑”解耦。
- 保持仿真与实机的话题命名、坐标系约定一致。
- 在控制节点中加入安全参数（超时、限幅、失效保护等）。

## 常见问题与解决方案（跨分支日志沉淀）

- **QoS 不匹配导致“节点在跑但无数据”**
  - 现象：订阅无回调、无报错。
  - 方案：MAVROS 高频状态类话题优先用 `BEST_EFFORT + VOLATILE`；`home` 等 latched 数据考虑 `TRANSIENT_LOCAL`。

- **MAVROS 前缀不一致（`/mavros` vs `/uas1/mavros`）**
  - 现象：控制节点启动正常但完全无效。
  - 方案：所有话题前缀参数化；启动前执行 `ros2 topic list | rg mavros` 确认实际前缀。

- **仿真迁移实机时相机链路未切干净**
  - 现象：launch 正常但没有图像或 ArUco 检测不到。
  - 方案：从 `gz bridge` 切到 `usb_cam`；统一图像话题到 `/image_raw`；保证 `camera_info` 使用实机标定参数。

- **USB 摄像头格式/设备号问题**
  - 现象：`Invalid v4l2 format`、相机偶发打不开。
  - 方案：像素格式优先 `mjpeg2rgb`；设备路径参数化；优先使用 `/dev/v4l/by-id/...` 避免 `videoX` 漂移。

- **`cv_bridge` 与 `numpy` 兼容性冲突**
  - 现象：报 `_ARRAY_API not found` 等 Python 运行错误。
  - 方案：将 `numpy` 固定到 ROS 兼容版本（历史记录中 `1.26.4` 可用），并在目标环境重新验证相机链路。

- **MAVROS 串口权限导致无法连接飞控**
  - 现象：`DeviceError:serial:open: Permission denied`。
  - 方案：将串口权限修正为 `root:dialout 660`，并添加 udev 规则持久化。

- **坐标系/符号混用导致跟踪反向或绕圈**
  - 现象：`yaw` 正常但 `xy` 越调越偏或某轴反向。
  - 方案：先统一 FLU/ENU/NED/body 语义；单轴阶跃验证正负方向；必要时加入动态 yaw 旋转补偿并记录 `err_marker/err_cmd`。

- **OFFBOARD 门控与视觉/位姿超时引发零速保护**
  - 现象：飞行中控制指令突然归零。
  - 方案：显式状态机门控：非 OFFBOARD、视觉超时、本地位姿超时均归零；恢复后再进入闭环。

- **降落末段卡在 LAND，已接地但不 disarm**
  - 现象：长时间“接地准备中”。
  - 方案：在保留 `ON_GROUND` 优先的同时，增加“最低油门下压 + 周期 disarm”兜底分支。

- **长时间通电后高度参考漂移**
  - 现象：进入 OFFBOARD 后持续上升或高度控制异常。
  - 方案：在 OFFBOARD 上升沿锁定本次任务高度参考；低空尽量使用测距/视觉高度，不仅依赖 GPS 相对高度。

- **launch 参数覆盖方式误用**
  - 现象：`--ros-args -p` 传参看似成功但未生效。
  - 方案：对 launch 声明参数使用 `kp_xy:=...` 形式覆盖，避免与节点参数覆盖方式混用。

- **构建后未 `source` 导致“改了不生效”**
  - 现象：仍读取旧参数/旧设备默认值。
  - 方案：每次构建后执行 `source install/setup.bash`，必要时新开终端复验。


## 日志文件

- 详细“问题记录/修改记录”已迁移至 [LOG.md](/home/zjh/project/zjh_ws/LOG.md)。
