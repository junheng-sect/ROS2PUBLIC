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
- `px4_position_controller`, `px4_disarm`, `offboard_control`
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
