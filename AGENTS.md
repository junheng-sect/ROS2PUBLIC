# 项目初始化说明

## 代理人设与技术栈

**技术栈**：
- 🐧 **系统**：Ubuntu 22.04
- 🤖 **框架**：ROS 2 Humble
- 🚁 **飞控**：PX4 v1.16.0（精通代码及 ROS 2↔PX4 通信）
- 🔗 **通信**：MAVROS（ROS 2 与 PX4 之间的通信桥接）
- 🌍 **仿真**：Gazebo Harmonic（精通仿真环境搭建与通信）
- 💻 **技能**：擅长编程，能根据需求编写带适当注释的代码

**工作空间**：`~/project/rasip_pi_ws`

## 协作与记录要求

- 在本 `AGENTS.md` 中持续记录每次进行的修改。
- 树莓派ssh用户名zjh，密码123，IP地址172.24.134.110
- 在树莓派中的操作记录保存至PI_LOG.md中，实机和仿真分开。
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
- 配置git功能使代码在ide中能够撤回，配置本地.git仓库。

## 开发规范

- 默认使用 `ament-python` 创建功能包。
- 使用 `ament_cmake` 时需说明原因。创建接口 interface 功能包时要使用 cmake。
- 协议默认使用 **Apache-2.0**。

## Git 分支策略

- `main`：用于 `Ubuntu 24.04 + ROS 2 Jazzy` 的日常程序开发与功能迭代。
- `ubuntu22.04-ros-humble`：用于 `Ubuntu 22.04 + ROS 2 Humble` 的版本适配与实机实验验证。
- `simple`：用于 `Ubuntu 24.04 + ROS 2 Jazzy` 的重要功能，和 `main` 的区别是 `simple` 没有测试用功能包。


## 项目概述

本仓库是一个基于 ROS 2  的工作空间，用于 ArUco 视觉与 PX4 无人机控制。
支持仿真（Gazebo + RViz）和实机两类流程。

## 实机usb摄像头参数
image_width: 640
image_height: 480
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [810.78076,   0.     , 346.24076,
           0.     , 813.75141, 251.50143,
           0.     ,   0.     ,   1.     ]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.410508, 0.102062, 0.001503, -0.000384, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1., 0., 0.,
         0., 1., 0.,
         0., 0., 1.]
projection_matrix:
  rows: 3
  cols: 4
  data: [751.89913,   0.     , 350.54287,   0.     ,
           0.     , 782.43472, 252.86193,   0.     ,
           0.     ,   0.     ,   1.     ,   0.     ]

## 实机aruco码和仿真aruco码尺寸

### 仿真
self.declare_parameter('marker_size_33', 0.5)
self.declare_parameter('marker_size_42', 0.063)

### 实机
self.declare_parameter('marker_size_33', 0.5)
self.declare_parameter('marker_size_42', 0.063)

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
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- colcon

## 构建与运行

```bash
source /opt/ros/Humble/setup.bash
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


## 日志文件

- 详细“问题记录/修改记录”已迁移至 [LOG.md](/home/zjh/project/zjh_ws/LOG.md)。
