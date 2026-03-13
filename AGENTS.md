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
- 树莓派ssh用户名zjh，密码123，IP地址10.250.57.110
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
self.declare_parameter('marker_size_33', 0.193)
self.declare_parameter('marker_size_42', 0.025)

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

## 问题记录（本轮补充）
- 2026-03-13 | 问题：将 `simple` 分支中的 `body_frame_tracking_minimal`、`body_frame_tracking`、`aruco_tracking_minimal`、`land_with_tracking` 同步到笔记本和树莓派，并构建测试至无报错。
  解答：已完成。两端均已从 `origin/simple` 同步 4 个功能包并通过构建与启动自检。

## 修改记录（本轮补充）
- 2026-03-13：笔记本 `rasip_pi_ws` 已同步 4 个功能包并完成 `colcon build --packages-select body_frame_tracking_minimal body_frame_tracking aruco_tracking_minimal land_with_tracking`。
- 2026-03-13：树莓派 `rasip_pi_ws`（`10.250.57.110`）已同步 4 个功能包并完成同样构建。
- 2026-03-13：两端已分别执行 4 个 launch 的限时启动自检，日志无 `ERROR/Traceback/process has died`。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：参考本地 aruco_tracking 的 USB 相机链路，把 `body_frame_tracking_minimal`、`body_frame_tracking`、`aruco_tracking_minimal`、`land_with_tracking` 的图像来源从 GZ bridge 切到 USB 相机。
  解答：已完成。4 个包均改为复用 `tvec_tf -> tvec` 的 USB 相机链路，不再在这 4 个包内启动 `ros_gz_bridge/parameter_bridge`。

## 修改记录（本轮补充）
- 2026-03-13：`aruco_tracking_minimal.launch.py` 移除 `gz_bridge + 本地tvec`，改为 `IncludeLaunchDescription(tvec_tf.launch.py)`；新增 `use_rqt` 参数；默认 `ros_image_topic` 改为 `/image_raw`。
- 2026-03-13：`body_frame_tracking_minimal.launch.py` 做同样改造（移除 gz bridge，改为复用 `tvec_tf` USB 链路）。
- 2026-03-13：`body_frame_tracking.launch.py`、`land_with_tracking.launch.py` 增加 `use_rqt` 透传，并统一默认 `ros_image_topic=/image_raw`。
- 2026-03-13：完成构建与启动验证：`colcon build --packages-select aruco_tracking_minimal body_frame_tracking_minimal body_frame_tracking land_with_tracking tvec tvec_tf` 通过；4 个 launch 限时启动均无报错。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：两个 minimal 功能包中禁用 `map_aruco_static_tf_node`，并将 rqt 启动参数默认改为 false。
  解答：已完成。两个 minimal launch 不再包含会拉起静态 TF 的 `tvec_tf.launch.py`，改为 `tvec.launch.py + tvec_tf_node`；`use_rqt` 默认值已设为 `false`。

## 修改记录（本轮补充）
- 2026-03-13：`aruco_tracking_minimal.launch.py` 移除静态 TF 链路依赖，默认 `use_rqt=false`。
- 2026-03-13：`body_frame_tracking_minimal.launch.py` 移除静态 TF 链路依赖，默认 `use_rqt=false`。
- 2026-03-13：laptop 与树莓派均完成构建与启动验证，日志不再出现 `map_aruco_static_tf_node` 和 `rqt_image_view`。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：`tvec_tf_node` 提示“尚未收到 /debug/tvec 数据”，怀疑两个 minimal 功能包没有收到视觉数据。
  解答：排查后确认链路正常。两包在启动后均持续输出 `aruco_pose`，`/debug/tvec` 数据正常；该提示为启动初期未检测到 marker 的瞬时日志，不是持续故障。

## 修改记录（本轮补充）
- 2026-03-13：完成两包链路诊断（`usb_cam -> /image_raw -> tvec_rvec_node -> /debug/tvec -> tvec_tf_node`），确认数据流正常且可复现连续 `aruco_pose` 输出。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：`land_with_tracking` 功能包不需要拉起静态 TF 节点和 rqt。
  解答：已完成。`land_with_tracking.launch.py` 不再包含 `tvec_tf.launch.py`，改为 `tvec.launch.py + tvec_tf_node`，并将 `use_rqt` 默认设为 `false`。

## 修改记录（本轮补充）
- 2026-03-13：更新 `src/land_with_tracking/launch/land_with_tracking.launch.py`，禁用静态 TF 链路与默认 rqt 拉起。
- 2026-03-13：laptop 与树莓派均构建并启动验证通过，日志不再出现 `map_aruco_static_tf_node` 与 `rqt_image_view`。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：将两个 minimal 功能包 CSV 文件默认存储目录改到 `rasip_pi_ws` 工作空间的 `log` 下。
  解答：已完成。两个包的 launch 默认参数与 csv_logger 节点参数默认值均改为 `/home/zjh/project/rasip_pi_ws/log/tracking_csv`，并在 laptop 与树莓派验证生效。

## 修改记录（本轮补充）
- 2026-03-13：更新 `aruco_tracking_minimal`、`body_frame_tracking_minimal` 的 `csv_output_dir/output_dir` 默认路径为 `~/project/rasip_pi_ws/log/tracking_csv`。
- 2026-03-13：同步更新两个 README 的 CSV 路径说明。
- 2026-03-13：laptop 与树莓派均构建并启动验证，日志输出路径已指向 `rasip_pi_ws/log/tracking_csv`。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：如何在 `ros2 launch` 命令里修改 PID 参数。
  解答：可在 launch 命令后追加 `--ros-args -p 参数名:=值`。已给出 `body_frame_tracking_minimal` 与 `aruco_tracking_minimal` 的可直接执行示例命令。

## 修改记录（本轮补充）
- 2026-03-13：新增 PID 参数在线覆盖命令说明（不改代码即可试参）。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：`ros2 launch` 使用 `--ros-args -p` 方式设置 PID 报参数错误，如何在启动命令里改 PID。
  解答：已修复为 launch 参数方式。两个 minimal launch 新增 PID/死区参数声明并传入节点，可直接用 `kp_xy:=...` 形式覆盖。

## 修改记录（本轮补充）
- 2026-03-13：`body_frame_tracking_minimal.launch.py` 新增 PID 与死区相关 launch 参数并映射到 `body_frame_tracking_node`。
- 2026-03-13：`aruco_tracking_minimal.launch.py` 新增 PID、z保持与死区相关 launch 参数并映射到 `aruco_tracking_node`。
- 2026-03-13：更新两个 README，增加可直接执行的 PID 调参命令示例。
- 2026-03-13：laptop 与树莓派均构建并启动验证通过（按 `kp_xy:=...` 命令可正常拉起）。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：对 `body_frame_tracking` 多组参数 CSV（1.csv、2.csv、kpxy=0.6、kpxy=0.8、kpxy=0.8,kdxy=0.2、kpxy=0.8,kdxy=0.05、kpxy=1.0,kdxy=0.05、kpxy=1.0,kdxy=0.05,kdyaw=0.05）做对比，找出更优参数并给出下一步调试方向。
  解答：已完成量化分析。综合误差与抖动指标最优参数为 `kpxy=1.0,kdxy=0.05`；`kdxy=0.2` 抖动显著增大；`kdxy` 从 `0.2` 降到 `0.05` 后抖动明显改善。已给出下一步精细调参建议。

## 修改记录（本轮补充）
- 2026-03-13：基于 8 份 CSV 完成统一指标统计与排序（xy误差、yaw误差、测量抖动、控制抖动、丢码时长），形成下一步调参建议。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：将多组 body_frame_tracking CSV 的关键参数作图，清晰展示各 PID 参数下 tracking 优劣。
  解答：已完成，生成指标对比图、综合评分图、关键时序对比图与汇总表。

## 修改记录（本轮补充）
- 2026-03-13：基于 `~/桌面/3.13` 下 8 份 CSV 生成可视化结果（PNG + TSV），输出到 `~/桌面/3.13/plots`。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：阅读 `rasip_pi_ws` 和 `simple_ws` 两个工作空间下的 `LOG.md`，总结今天之前七天的进展，限制约 150 字。
  解答：已完成。通读两边日志后确认，今天之前七天内的有效进展主要集中在 2026-03-06 至 2026-03-08，核心包括视觉接口与位姿链路建设、TF/矩阵链路打通、`aruco_tracking` PID 跟踪控制迭代，以及 `rasip_pi_ws` 中相机标定链路和仓库同步。

## 修改记录（本轮补充）
- 2026-03-13：读取并汇总 `rasip_pi_ws/LOG.md` 与 `simple_ws/LOG.md` 的近七天记录，形成 150 字进展总结；同步补充本轮问答记录。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：可参考 `simple` 分支中的 `LOG.md` 以及 `questions` 目录下的 `LOG.md`，重新总结 2026-03-06 至 2026-03-13 的项目进展。
  解答：已完成。改为基于 `origin/simple:LOG.md` 与 `origin/simple:questions/LOG.md` 汇总，确认进展覆盖视觉位姿链路、`land_with_tracking`/`body_frame_tracking`/两个 minimal 包与 CSV 记录功能，以及地面效应补偿、评价指标、CFD 与实机实验方案等研究文档；`simple` 远程分支截至 2026-03-13 的最近提交日期为 2026-03-12。

## 修改记录（本轮补充）
- 2026-03-13：按用户指定改为读取 `simple` 分支主日志与 `questions/LOG.md`，修正近七天项目进展总结口径。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：提供一段“本周进展”示例文字，要求模仿该语言风格描述项目进展。
  解答：已完成。按照用户提供的周报语气，将本周项目进展改写为连续叙述风格，突出“本周完成内容、当前效果、存在问题、后续优化方向”等表达方式。

## 修改记录（本轮补充）
- 2026-03-13：根据用户提供的示例段落，调整本周进展总结的表述风格，并补充本轮问答记录。
