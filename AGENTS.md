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
- 问的问题也进行记录，开一栏新的标题“问题记录”里面写问的问题和解答。
- 每轮对话都必须记录：修改了代码写入“修改记录”，解答了问题写入“问题记录”。
- 每完成一轮对话，更新一次本文件的“修改记录”。
- 功能包测试完成后，仅在你明确回复“成功”后执行 Git 提交并上传远程仓库。
- 每次我修改代码后，自动执行 `colcon build` 并执行 `source install/setup.bash`。
- 每次涉及 launch 的修改后，自动运行对应 `ros2 launch` 进行验证；确认启动成功后再停止。

## 开发规范

- 默认使用 `ament-python` 创建功能包。
- 使用 `ament_cmake` 时需说明原因。
- 协议默认使用 **Apache-2.0**。

## Git 分支策略

- `main`：用于 `Ubuntu 24.04 + ROS 2 Jazzy` 的日常程序开发与功能迭代。
- `ubuntu22.04-ros-humble`：用于 `Ubuntu 22.04 + ROS 2 Humble` 的版本适配与实机实验验证。

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
- `px4_position_controller`, `px4_disarm`, `offboard_control`, `rover_teleop_ackermann`, `rover_auto_motion`
- `aruco_interfaces`, `px4_interfaces`
- `camera_test`, `camera_calibration_pkg`, `gazebo_image_test`, `tf_display`

## 全部功能包简要介绍

- `aruco_csv`：ArUco 检测并将检测结果记录为 CSV 日志。
- `aruco_detector`：基础 ArUco 检测节点，发布标记位姿信息。
- `aruco_detector_mid`：ArUco 检测中间实验版，含阈值/CSV/Z 轴标定等变体。
- `aruco_gazebo`：面向 Gazebo 图像源的 ArUco 检测节点。
- `aruco_gazebo_rviz`：Gazebo 场景下结合 RViz 的检测与可视化方案。
- `aruco_interfaces`：自定义消息接口包（如 `ArucoPose.msg`）。
- `aruco_landing`：基于视觉的自动降落控制节点。
- `aruco_position_controller`：基于 ArUco 位姿的无人机位置控制节点。
- `aruco_tf_vision`：将视觉检测结果转换并广播为 TF。
- `aruco_threshold`：采用阈值策略的 ArUco 检测实现。
- `aruco_tracking`：基于视觉目标的跟踪控制节点。
- `camera_calibration_pkg`：相机标定与标定板生成工具包。
- `camera_test`：相机图像链路测试与转发节点。
- `gazebo_image_test`：Gazebo 图像话题连通性测试包。
- `px4_disarm`：PX4 解锁/上锁（arm/disarm）服务与节点。
- `px4_interfaces`：PX4 相关自定义服务接口包（如目标位置、解锁服务）。
- `px4_position_controller`：PX4 Offboard 位置控制节点。
- `offboard_control`：Offboard 任务控制节点（解锁、起飞、悬停、降落、上锁）。
- `rover_teleop_ackermann`：方向键遥控与简化阿克曼运动学控制包（通过 Gazebo `SetEntityPose` 驱动 `rover`）。
- `rover_auto_motion`：Rover 自动运动节点（前进 1m + 右转 90°@1m 半径）循环执行。
- `tf_display`：TF 广播与可视化辅助包。

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

## 问题记录

- 2026-03-04 | 问题：`theta` 的物理含义是什么？
  解答：`theta` 是 `vision_pose` 相对 `map` 的偏航角（yaw，弧度），用于将检测得到的局部位移旋转到 `map` 坐标系后再进行平移计算。
- 2026-03-04 | 问题：TF 变换和控制如何解耦？是否应拆成两个节点？
  解答：推荐拆成两个节点；感知/TF 节点只做坐标变换与 TF 发布，控制节点只读 TF 并输出控制量，避免传感与控制逻辑互相影响。
- 2026-03-04 | 问题：当前“已接近解耦”还差什么？
  解答：仍存在 TF 语义约定、`yaw_aruco` 角度约定、速度符号映射、launch 编排与任务状态机等层面的耦合，后续可通过统一接口契约继续弱化。
- 2026-03-04 | 问题：一个功能包内可以写多个功能的控制文件吗？
  解答：可以。一个功能包可包含多个控制节点/脚本（多个 `console_scripts`），建议按功能拆分为独立节点并通过话题、服务或动作组合。
- 2026-03-04 | 问题：`offboard_mission_node` 的默认 yaw 是多少？
  解答：默认值已改为 `90°`（参数 `target_yaw_deg=90.0`）。
- 2026-03-04 | 问题：这个 yaw 是订阅哪个话题得到的？
  解答：不是订阅得到，而是由参数 `target_yaw_deg` 生成并写入 setpoint 姿态四元数。
- 2026-03-04 | 问题：该节点发布的话题是否是 local position？
  解答：是，发布到 `/mavros/setpoint_position/local`（`PoseStamped`）。
- 2026-03-04 | 问题：local position 里的 yaw 表示什么？`yaw=0` 机头朝哪？
  解答：在 ENU 约定下 yaw 是机体前向相对本地坐标 `+X` 的偏航角；`yaw=0` 通常机头朝 `+X`（East），`yaw=90°` 朝 `+Y`（North）。
- 2026-03-04 | 问题：Gazebo 仿真打开后视角默认朝向是否是东？
  解答：不一定。GUI 视角朝向由用户相机默认位姿决定，不等同于世界坐标固定朝向；ENU 约定下仅坐标轴定义为 `+X=East`、`+Y=North`。
- 2026-03-04 | 问题：如何查看 GUI 相机默认位姿？
  解答：可在 Gazebo 界面查看用户相机的 Position/Orientation，或保存 GUI 配置后在配置文件中读取 `camera pose`（`x y z roll pitch yaw`）。
- 2026-03-04 | 问题：`pitch`、`roll`、`yaw` 分别绕哪个轴旋转？
  解答：`roll` 绕 `X` 轴，`pitch` 绕 `Y` 轴，`yaw` 绕 `Z` 轴（右手系约定）。
- 2026-03-04 | 问题：如何用 Gazebo 打开位于桌面的 rover 车？
  解答：若桌面有 rover 的 `.sdf/.world` 文件，可直接用 `gz sim /home/zjh/Desktop/xxx.sdf` 或 `gz sim /home/zjh/Desktop/xxx.world`；若是模型目录，则先通过 `GZ_SIM_RESOURCE_PATH` 指向桌面模型路径，再在 world 中 `include` 模型。
- 2026-03-04 | 问题：你能控制桌面上的文件吗，或者进行打开操作？
  解答：可以在终端权限范围内读写、创建、删除和打开文件（例如执行 `ls/cat/cp/mv/rm`、`gz sim`、`xdg-open` 等命令）；但我不能像人一样直接点鼠标操作桌面 GUI。
- 2026-03-04 | 问题：如何查看并打开 `/home/zjh/桌面/rover`？
  解答：先检查目录内容；若缺少 `model.sdf/model.config/world` 需补齐后再用 `GZ_SIM_RESOURCE_PATH=/home/zjh/桌面` 启动 `gz sim /home/zjh/桌面/rover/rover.world`。
- 2026-03-04 | 问题：如何在 Gazebo 中测量小车尺寸？
  解答：可用 GUI 的测量工具（Measure）直接量两点距离；也可在模型 `model.sdf` 里查看 `collision/visual` 的 `size/scale`，或用话题/TF 获取关键点坐标后计算距离。
- 2026-03-04 | 问题：可以通过修改 `model.sdf` 来缩放车的尺寸吗？
  解答：可以。若是 mesh 模型可修改 `<mesh><scale>x y z</scale></mesh>`；若是几何体可直接改 `<box><size>...</size></box>` 等尺寸参数，需同步调整 `collision`、惯量和传感器安装位姿。
- 2026-03-04 | 问题：如何把车宽调到 0.6m 并整体等比例放大/缩放？
  解答：先以当前宽度计算缩放系数 `k=目标宽度/当前宽度`，再在 `model.sdf` 里同步修改 `mesh scale`、`collision size`，并按比例更新位姿高度与惯量参数。
- 2026-03-04 | 问题：如何把当前模型整体放大 5 倍？
  解答：将几何尺度参数统一乘以 5（如 `mesh scale`、`collision size`、参考位姿 `z`），若保持物理一致性还需同步更新 `mass`（`×5^3`）与惯量（`×5^5`）。
- 2026-03-04 | 问题：模型在当前基础上再扩大 2 倍怎么做？
  解答：在现有参数基础上将几何尺寸与位姿尺度乘 `2`，并同步物理参数：`mass × 2^3`、惯量 `× 2^5`，保持动力学一致。
- 2026-03-04 | 问题：模型在当前基础上再扩大 1.5 倍怎么做？
  解答：在现有参数基础上将几何与位姿尺度乘 `1.5`，并同步物理参数：`mass × 1.5^3`、惯量 `× 1.5^5`，保证物理一致性。
- 2026-03-04 | 问题：如何在 `~/PX4_Firmware/Tools/simulation/gz/worlds` 新建保留 aruco 内容并加入 `rover_ackermann` 的世界？
  解答：复制 `aruco.sdf` 为新文件（如 `aruco_rover.sdf`），保留原内容后新增 `<include><uri>model://rover_ackermann</uri><pose>...</pose></include>`，并将 `pose` 放到远离原点的位置避免与无人机重叠。
- 2026-03-04 | 问题：如何执行到能够正常打开带 rover 的 aruco 仿真？
  解答：使用 `PX4_GZ_WORLD=aruco_rover make px4_sitl gz_x500_mono_cam_down` 启动；若卡在 `Waiting for Gazebo world`，需检查世界文件的 `<world name>` 与 `PX4_GZ_WORLD` 完全一致。
- 2026-03-04 | 问题：无人机默认在原点，如何生成更高并放在 rover 上方？
  解答：启动 PX4 SITL 时设置 `PX4_GZ_MODEL_POSE="x,y,z,roll,pitch,yaw"`，例如 `PX4_GZ_MODEL_POSE="0,0,4.2,0,0,0"`；其中 `z` 调大即可抬高出生点。
- 2026-03-04 | 问题：如何在 Gazebo 中通过键盘方向键控制 rover（最好阿克曼转向）？需要创建功能包吗？
  解答：建议创建 ROS 2 功能包。当前 rover 是静态长方体，无法直接运动；要实现方向键+阿克曼，需要改成可动底盘模型（转向关节+驱动轮）并接入阿克曼控制（插件或控制节点），再由键盘节点发布控制命令。
- 2026-03-04 | 问题：确认采用“简化伪阿克曼+方向键”后如何落地？
  解答：已新增 `rover_teleop_ackermann` 包，使用“方向键节点发布 `/rover/ackermann_cmd` + 运动学控制节点调用 `/world/rover/set_pose`”实现；默认限幅 `v_max=0.6m/s`、`steer_max=25°`。
- 2026-03-04 | 问题：为什么 rover 只能前后、不能转向，且前进+左转会停住？
  解答：终端对组合方向键的重复上报不稳定，原逻辑按“轴独立超时”会把另一轴清零。已改为“任意方向键共享超时”，只要有按键活动就保持当前 `v/delta`，可同时前进并转向。
- 2026-03-04 | 问题：改成 WASD 后仍有卡顿，尤其 A/D 转向时灵时不灵怎么办？
  解答：已将键盘读取改为 `raw + os.read`（避免文本缓冲导致的按键丢失/延迟），并把 `key_timeout_sec` 默认调大到 `0.55s` 以覆盖系统按键重复延迟；同时新增 `SPACE` 一键急停，提升可控性。
- 2026-03-04 | 问题：`W+D` 后松开 `D` 会停住、再按 `A/D` 无效，且 `Ctrl+C` 无法关闭节点怎么办？
  解答：已将终端模式从 `raw` 改为 `cbreak`（恢复 `Ctrl+C` 信号），并把键盘发布频率提升到 `40Hz`、超时增大到 `0.90s`，降低组合键切换时因按键重复间隙触发清零的问题。

## 修改记录

### 工作空间与协作规范
- 2026-03-03：新增“代理人设与技术栈”“协作与记录要求”“开发规范”章节，写入默认工作空间与文档维护规则。
- 2026-03-03：新增“全部功能包简要介绍”章节，补全 `src/` 下全部功能包的一句话说明。
- 2026-03-03：新增提交流程约定：功能包测试完成后，需用户明确回复“成功”才执行 Git 提交。
- 2026-03-03：新增流程约定：每次代码修改后自动执行 `colcon build` 与 `source install/setup.bash`。
- 2026-03-04：新增“Git 分支策略”章节，明确 `main`（Ubuntu 24.04 + ROS 2 Jazzy 开发）与 `ubuntu22.04-ros-humble`（Ubuntu 22.04 + ROS 2 Humble 适配与实机）职责划分。
- 2026-03-04：新增“问题记录”章节，按“问题+解答”记录关键问答，便于后续追踪设计决策与调试结论。
- 2026-03-04：新增功能包 `offboard_control`（`ament_python`，Apache-2.0），并补充到工作空间结构与功能包简介。
- 2026-03-04：按要求同步更新“问题记录”，新增 `offboard_control` 的 yaw 与 local position 相关问答。
- 2026-03-04：补充记录准则：每轮对话必须同步记录（代码改动写“修改记录”，问题解答写“问题记录”）；并补录 Gazebo 视角与 GUI 相机位姿查看问题。
- 2026-03-04：补录姿态角基础问答：`roll/pitch/yaw` 对应旋转轴分别为 `X/Y/Z`。
- 2026-03-04：补录 Gazebo 使用问答：如何从桌面路径启动 rover 模型/世界文件。
- 2026-03-04：补录能力边界问答：可在终端权限范围内操作桌面文件与执行打开命令，但不进行鼠标式 GUI 点击。
- 2026-03-04：已检查 `/home/zjh/桌面/rover` 目录并补齐 Gazebo 所需文件（`model.config`、`model.sdf`、`rover.world`），可通过 `gz sim` 按路径加载。
- 2026-03-04：补录 Gazebo 尺寸测量问答：GUI 测量工具与模型文件参数两种方式。
- 2026-03-04：补录模型缩放问答：可通过 `model.sdf` 的 `scale/size` 调整车体尺寸，并建议同步修正碰撞与惯量参数。
- 2026-03-04：按“车宽=0.6m、整体等比例”修改 `/home/zjh/桌面/rover/model.sdf`：缩放系数 `0.75`，更新 `mesh scale`、`collision size`，并同步调整 `pose.z` 与惯量参数。
- 2026-03-04：将 `/home/zjh/桌面/rover/model.sdf` 在当前基础上整体放大 5 倍：更新 `mesh scale`、`collision size`、`pose.z`，并按比例同步 `mass` 与惯量参数。
- 2026-03-04：将 `/home/zjh/桌面/rover/model.sdf` 在当前基础上再次整体放大 2 倍：更新 `mesh scale`、`collision size`、`pose.z`，并同步 `mass`（×8）与惯量（×32）。
- 2026-03-04：将 `/home/zjh/桌面/rover/model.sdf` 在当前基础上再次整体放大 1.5 倍：更新 `mesh scale`、`collision size`、`pose.z`，并同步 `mass`（×1.5³）与惯量（×1.5⁵）。
- 2026-03-04：在 `~/PX4_Firmware/Tools/simulation/gz/worlds` 新建 `aruco_rover.sdf`：完整保留 `aruco.sdf` 内容，并新增 `rover_ackermann`（`pose=8 0 0.04 0 0 0`）以避免与原点无人机重叠。
- 2026-03-04：联调修复 `aruco_rover` 启动卡住问题：将世界文件名与世界内部 `<world name>` 对齐为 `aruco_rover`，并验证 `Gazebo world is ready`、`world: aruco_rover` 正常输出。
- 2026-03-04：按新要求移除 `rover_ackermann`：将桌面模型 `/home/zjh/桌面/rover` 复制到 `~/PX4_Firmware/Tools/simulation/gz/models/rover`，并把 `aruco_rover.sdf` 的 `include` 改为 `model://rover`（保持 `pose=8 0 0.04 0 0 0` 避免与无人机重叠）；启动验证日志显示 `Gazebo world is ready`。
- 2026-03-04：将 `~/PX4_Firmware/Tools/simulation/gz/models/rover/model.sdf` 按当前尺寸整体缩小到 `2/3`：同步更新 `mesh scale`（`11.25 -> 7.5`）、`collision size`（`13.5 9.0 5.625 -> 9.0 6.0 3.75`）、`pose.z`（`2.25 -> 1.5`）及惯量参数（`mass` 与 `ixx/iyy/izz`）以保持物理一致性。
- 2026-03-04：将世界名称统一改为 `rover`：`~/PX4_Firmware/Tools/simulation/gz/worlds/aruco_rover.sdf` 重命名为 `rover.sdf`，并将文件内 `<world name>` 同步改为 `rover`，避免 `PX4_GZ_WORLD` 与 world 名不一致导致等待世界超时。
- 2026-03-04：修复“无人机与 rover 悬浮”问题：`rover` 模型碰撞体较大（`9m x 6m`），当 `include pose` 改为 `x=1` 时与原点无人机出生区重叠导致相互顶起；已将 `~/PX4_Firmware/Tools/simulation/gz/worlds/rover.sdf` 中 rover 位姿改回 `x=8`（`pose=8 0 0.04 0 0 0`）并完成启动验证。
- 2026-03-04：为实现“先加载 rover，再将无人机刷在 rover 上方”，已将 `~/PX4_Firmware/Tools/simulation/gz/worlds/rover.sdf` 中 rover `include pose` 调整到原点（`0 0 0.04 0 0 0`）；启动时通过 `PX4_GZ_MODEL_POSE` 指定无人机出生高度（示例 `0,0,4.2,0,0,0`），日志确认 `Spawning model at position: 0 0 4.2`。
- 2026-03-04：按要求将 `rover` 碰撞体改为“自身体积”：`~/PX4_Firmware/Tools/simulation/gz/models/rover/model.sdf` 的 `collision` 从 `box` 改为与 `visual` 同源 `mesh`（`model://rover/meshes/ogv.dae`，`scale=7.5 7.5 7.5`）。
- 2026-03-04：调整 `rover` 世界中的 ArUco 布置：删除原先无位姿的世界 ArUco include，并将 `model://myaruco` 以显式位姿放置到 rover 顶面（`pose=0 0 3.43 0 0 0`）。
- 2026-03-04：将 `rover` 世界中的复杂 rover 模型替换为简化平台：在 `~/PX4_Firmware/Tools/simulation/gz/worlds/rover.sdf` 中新增静态长方体 `rover_box`（`1 x 0.7 x 0.3`，中心位于世界原点上方 `z=0.15`）；`myaruco` 调整到长方体上表面中心（`pose=0 0 0.301 0 0 0`）；并通过 `PX4_GZ_MODEL_POSE="0,0,0.65,0,0,0"` 验证无人机可刷在平台上方。
- 2026-03-04：按“直接改 rover 模型和世界”的要求重构：`~/PX4_Firmware/Tools/simulation/gz/models/rover/model.sdf` 直接改为静态长方体（`1 x 0.7 x 0.3`），`~/PX4_Firmware/Tools/simulation/gz/worlds/rover.sdf` 改回 `include model://rover`（位于世界中心），并保留 `myaruco` 在上表面中心（`pose=0 0 0.301 0 0 0`）。
- 2026-03-04：修正长方体“下沉 0.15m”问题：将 `models/rover/model.sdf` 的模型内 `pose` 归零，改为在 `worlds/rover.sdf` 的 `include model://rover` 中设置 `z=0.15`，确保 `0.3m` 高盒体底面贴地。
- 2026-03-04：实现 ArUco 与长方体刚性绑定：将 ArUco 从 `worlds/rover.sdf` 的独立 `include` 移除，改为在 `models/rover/model.sdf` 内新增 `aruco_link`（顶面中心平面贴图）并通过 `fixed joint` 连接到 `base_link`，保证长方体移动时 ArUco 同步跟随。
- 2026-03-04：补录问答：Gazebo 键盘控制 rover 的实现路径（需动态底盘 + 阿克曼控制链路，建议新建 ROS 2 功能包承载键盘到控制命令映射）。
- 2026-03-04：补录问答：如何通过 `PX4_GZ_MODEL_POSE` 将无人机出生点抬高并放置到 rover 上方。
- 2026-03-04：新增功能包 `rover_teleop_ackermann`（`ament_python`，Apache-2.0）：包含 `keyboard_arrow_teleop_node`（方向键发布 `/rover/ackermann_cmd`）与 `ackermann_kinematic_controller_node`（自行车模型积分并调用 `/world/rover/set_pose`）。
- 2026-03-04：新增启动文件 `rover_teleop_ackermann.launch.py`，拉起 `ros_gz_bridge parameter_bridge`（`/world/rover/set_pose@ros_gz_interfaces/srv/SetEntityPose`）与两个控制节点；并新增包内 `README.md/readme.md` 使用说明。
- 2026-03-04：完成构建与验证：`colcon build --packages-select rover_teleop_ackermann` 通过，`ros2 launch rover_teleop_ackermann rover_teleop_ackermann.launch.py` 启动成功（桥接与控制节点正常启动）。
- 2026-03-04：修复 `keyboard_arrow_teleop_node` 组合键控制异常：将 `v`/`delta` 的独立按键超时改为共享超时（`_last_any_key_time`），解决“前进+转向时停住”和“无法持续转向”问题；已重新 `colcon build --packages-select rover_teleop_ackermann` 并验证 launch 可正常拉起。
- 2026-03-04：将 `rover_teleop_ackermann` 键位由方向键改为 `W/A/S/D`：`keyboard_arrow_teleop_node` 改为识别 `w/s/a/d`（含大写），并同步更新 `README.md/readme.md` 键位说明；已完成 `colcon build --packages-select rover_teleop_ackermann` 与 `source install/setup.bash`。
- 2026-03-04：进一步修复 `WASD` 间歇卡顿：`keyboard_arrow_teleop_node` 改为 `tty.setraw + os.read` 读取 TTY 原始字节，降低按键丢失；将 `key_timeout_sec` 默认由 `0.12/0.55` 统一为 `0.55s` 以规避系统按键重复初始延迟，并新增 `SPACE` 急停；同步更新 `README.md/readme.md`，并完成 `colcon build --packages-select rover_teleop_ackermann` 与 `source install/setup.bash`。
- 2026-03-04：针对“`W+D` 松开后停住、`Ctrl+C` 无法退出”继续修复：`keyboard_arrow_teleop_node` 终端模式由 `tty.setraw` 改为 `tty.setcbreak`（恢复中断信号），并将 `publish_rate_hz` 默认提升到 `40.0`、`key_timeout_sec` 调整为 `0.90`，减小组合键切换导致的误停；同步更新 `README.md/readme.md`，并完成 `colcon build --packages-select rover_teleop_ackermann`、`source install/setup.bash` 与 `py_compile` 自检。
- 2026-03-04：新增功能包 `rover_auto_motion`（`ament_python`，Apache-2.0）：新增 `rover_auto_loop_node`，按“前进 1m -> 右转 90°（半径 1m）-> 循环”发布 `/rover/ackermann_cmd`；并新增 `rover_auto_motion.launch.py`（拉起 `ros_gz_bridge`、`ackermann_kinematic_controller_node` 与自动运动节点）及包内 `README.md/readme.md`。
- 2026-03-04：完成 `rover_auto_motion` 构建与验证：`colcon build --packages-select rover_auto_motion` 与 `py_compile` 通过；`ros2 launch rover_auto_motion rover_auto_motion.launch.py` 启动成功，日志可见阶段切换 `straight -> turn_right -> straight`。

### Git 仓库与远程
- 2026-03-03：配置 Git 全局撤销别名（`undo`、`unstage`、`discard`、`last`），用于快速撤销工作区/暂存区改动。
- 2026-03-03：完成仓库初始化后的基础整理：分支由 `master` 调整为 `main`，新增 `.gitignore`（忽略 `build/install/log`、IDE 与缓存文件）。
- 2026-03-03：完成首个提交（commit: `043cd45`），绑定远程仓库 `origin=https://github.com/junheng-sect/junhengWS.git`；推送阶段因 GitHub 凭据未配置而中断。
- 2026-03-03：按 HTTPS 方案配置 `git credential.helper=store`，等待提供 GitHub PAT 后继续执行 `git push -u origin main`。
- 2026-03-03：使用 PAT 执行 `git push -u origin main` 返回 403（`Write access to repository not granted`），需确认令牌权限或仓库写权限后重试。
- 2026-03-03：按要求切换远程到 `https://github.com/junheng-sect/ROS2PUBLIC.git` 并推送；GitHub 返回 403（当前凭据对该仓库无写权限）。
- 2026-03-03：使用具备写权限的 PAT 完成认证；处理远程非 fast-forward（`git pull --rebase origin main`）后，成功推送 `main` 到 `ROS2PUBLIC`。
- 2026-03-03：用户确认“成功”后，按约定执行本轮 Git 提交流程（仅纳入代码与文档变更，排除运行期 CSV 日志文件）。

### aruco_gazebo
- 2026-03-03：修复 `aruco_gazebo` 无图像问题：`launch` 中检测节点改为订阅 bridge 实际输出话题（`/world/.../image`）；同时修复 `Ctrl+C` 退出时重复 `rclpy.shutdown()` 异常。

### aruco_tf_vision
- 2026-03-03：更新 `aruco_tf_vision`：在 2Hz 日志流中新增 `tvec` 三轴分量输出（`tvec[0]`、`tvec[1]`、`tvec[2]`）。
- 2026-03-03：修复 `aruco_tf_vision` 启动时报 `PackageNotFoundError: aruco-tf-vision`：移除 `setup.py` 中多余的 `rosidl_typesupport_c` 安装项，并新增“修改后运行 launch 验证成功再停止”的规则。
- 2026-03-03：完成 `aruco_tf_vision` 修复后验证：`colcon build --symlink-install --packages-select aruco_tf_vision` 通过，且 `importlib.metadata.distribution('aruco-tf-vision')` 可正常读取元数据。
- 2026-03-03：更新 `tf_broadcaster_node.pose_callback`，新增 yaw 日志输出（rad/deg）；已通过 `ros2 topic pub /mavros/local_position/pose` 注入测试消息验证日志生效。
- 2026-03-03：将 `aruco_tf_vision` 功能包日志频率统一为 1Hz（`aruco_detector_node`、`tf_broadcaster_node`）；并修复 `tf_broadcaster_node` 在超时/中断测试时的退出栈追踪（`ExternalShutdownException` 与重复 shutdown）。
- 2026-03-03：在 `tf_broadcaster_node` 中新增 yaw 初值归零逻辑：首次 yaw 作为偏移基准，日志输出相对 yaw 并归一化到 `[-180°, 180°]`；注入测试验证首帧为 `0.0°`、后续正确增量输出。
- 2026-03-03：修复 `vision_pose` 旋转参考系错误：`aruco_detector_node` 中 Z 轴 +90° 修正由左乘改为右乘（局部坐标系旋转），避免表现为绕 `map` Z 轴旋转。
- 2026-03-03：在 `aruco_tf_vision.launch.py` 运行场景下新增 CSV 记录：`aruco_detector_node` 每秒写入 `timestamp,stage,base_x,base_y,base_z,base_yaw_deg,vision_x,vision_y,vision_z,vision_yaw_deg`，并将检测日志中的 `Yaw={np.degrees(yaw):.1f}°` 同步写入 `detector_yaw_deg` 列。
- 2026-03-03：扩展 `aruco_tf_vision` CSV 字段，新增 `tvec_x,tvec_y,tvec_z` 三列，对应写入检测时的 `tvec[0], tvec[1], tvec[2]`。
- 2026-03-03：按指定公式更新 `map->vision_pose` 平移：`x=-tvec0*cos(theta)-tvec1*sin(theta)+offset_x`，`y=-tvec0*sin(theta)+tvec1*cos(theta)+offset_y`，其中 `theta` 使用 `vision_pose` 的 yaw（度转弧度）。
- 2026-03-03：根据要求对调 `map->vision_pose` 平移公式中的 XY 赋值（交换 X/Y 两行公式）。
- 2026-03-03：继续调整 `map->vision_pose` 平移公式：在对调后的基础上将 `Y` 分量整体取反。
- 2026-03-04：在 `aruco_detector_node` 的 1Hz 日志回调中新增 `theta` 输出（`theta=... rad (... deg)`），并在 `_publish_vision_tf` 计算后缓存该值用于周期日志打印。
- 2026-03-04：在 `aruco_detector_node` 新增 `yaw_aruco` 角度参数（`yaw_aruco = theta - 90°`，并归一化到 `[-180°, 180°]`），在 1Hz 日志中与 `theta` 同行输出。

### aruco_position_controller
- 2026-03-04：修改 `aruco_position_controller` 的视觉位姿来源：由订阅 `/aruco_pose` 改为直接查询 TF `map->vision_pose`（tf2），使控制输入与 `aruco_tf_vision` 的最终坐标变换保持一致；同时在 `package.xml` 增加 `tf2_ros` 依赖。
- 2026-03-04：修复 `aruco_position_controller` 退出流程，兼容 `ExternalShutdownException` 并避免重复 `rclpy.shutdown()` 报错（短时运行测试通过）。
- 2026-03-04：修复 `aruco_position_controller` 左右方向控制发散：`/mavros/setpoint_velocity/cmd_vel` 的 `linear.y` 改为 `-vy`，与当前机体系/控制方向一致。
- 2026-03-04：按联调反馈继续修正 `aruco_position_controller` 速度映射：`/mavros/setpoint_velocity/cmd_vel` 的 `linear.x` 与 `linear.y` 均调整为直接使用 PID 输出（等效相对上一版两轴同时取反）。

### aruco_landing
- 2026-03-04：按 `aruco_position_controller` 已验证逻辑修改 `aruco_landing`：视觉输入由 `/aruco_pose` 改为 tf2 查询 `map->vision_pose`（位置+yaw），并将 `cmd_vel` 的 `linear.x/linear.y` 映射调整为与 `aruco_position_controller` 一致；同时增加 `tf2_ros` 依赖并修复节点退出时重复 shutdown 异常。
- 2026-03-04：修复 `aruco_landing` 无法手动进入 OFFBOARD：移除启动阶段对 `/disarm` 服务的阻塞等待，改为运行中按需非阻塞重试，确保节点及时进入 spin 并持续发布 setpoint。
- 2026-03-04：调整 `aruco_landing` 角度闭环变量：由 `theta`（yaw）切换为 `yaw_aruco`（`yaw - 90°` 归一化），Yaw 控制目标由“`theta -> 0`”改为“`yaw_aruco -> 0`”。

### offboard_control
- 2026-03-04：创建 `offboard_control` 功能包并新增 `offboard_mission_node`，实现基础任务流程：解锁、起飞至 2.5m、悬停 5s、降落、上锁。
- 2026-03-04：联调修复 `offboard_mission_node` 无动作问题：`/mavros/local_position/pose` 订阅 QoS 改为 `BEST_EFFORT`，解决与 MAVROS 的 RELIABILITY 不兼容；实测状态机完整执行 `OFFBOARD->ARM->TAKEOFF->HOVER->AUTO.LAND->DISARM`。
- 2026-03-04：将 `offboard_mission_node` 默认目标航向改为 `yaw=90°`（新增参数 `target_yaw_deg`，默认 `90.0`，发布 setpoint 时按该角度生成四元数）。
- 2026-03-04：新增 `offboard_random_mission_node`：解锁、起飞至 2.5m、飞至半径 1 圆周随机 `(x,y)` 与随机 `yaw`、悬停 5s、回原点、降落、上锁；已完成构建并联调通过到 `TAKEOFF` 阶段。
- 2026-03-04：调整 `offboard_random_mission_node` 收尾姿态：回原点后进入降落阶段前，将目标 `yaw` 固定为 `90°`。
