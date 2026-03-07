# 开发日志
PX4_GZ_WORLD=rover PX4_GZ_MODEL_POSE="0,0,2,0,0,0" make px4_sitl gz_x500_mono_cam_down

ros2 launch rover_teleop_ackermann rover_teleop_ackermann.launch.py

ros2 run rover_teleop_ackermann keyboard_arrow_teleop_node

ros2 launch aruco_tracking aruco_tracking.launch.py world_name:=rover

ros2 launch aruco_tracking aruco_tracking.launch.py world_name:=rover model_name:=x500_mono_cam_down_0

## 问题记录

- 2026-03-07 | 问题：创建远程新分支 `laptop`，并将当前工作空间代码上传到远程和本地 Git。
  解答：已在本地基于当前代码创建分支 `laptop`，完成提交并推送到 `origin/laptop`，同时保留本地分支与上游跟踪关系。
- 2026-03-07 | 问题：`rover` 世界下现在可以解锁，询问具体修改内容并要求记录。
  解答：已将 `rover` 世界从“极简版本”改为“默认世界骨架 + rover 模型 include”。核心修改为补齐 `physics / magnetic_field / atmosphere / scene / spherical_coordinates`，并保持 `<world name=\"rover\">` 与 `PX4_GZ_WORLD=rover` 一致；这样恢复了 EKF/GPS 所需环境，`POSCTL` 下可正常解锁。
- 2026-03-07 | 问题：PX4 提示 `Arming denied: Resolve system health failures first`，无法解锁。
  解答：这是预检未通过导致；需先执行 `commander check` 查看具体失败项。当前场景常见原因为 EKF 未就绪或链路未满足。已给出处理顺序：先等待/确认 `Gazebo world is ready` 与 EKF 收敛，再保证 Offboard 设定值连续发送；若仅做仿真调试，可临时放宽检查（如 `COM_ARM_WO_GPS=1`）或使用 `commander arm -f` 强制解锁。
- 2026-03-07 | 问题：`PX4_GZ_WORLD=rover ...` 启动时持续 `Waiting for Gazebo world...`，需要修复。
  解答：根因是世界文件中的 `<world name>` 为 `rover_world`，与 `PX4_GZ_WORLD=rover` 不一致；已将 `/home/zjh/PX4_Firmware/Tools/simulation/gz/worlds/rover.sdf`（以及源文件 `/home/zjh/桌面/rover/rover.world`）的世界名修正为 `rover`，复测后日志显示 `Gazebo world is ready`、`world: rover`。
- 2026-03-07 | 问题：将 `rover.world` 与 `rover` 模型加入 PX4 Gazebo 资源库，使 `PX4_GZ_WORLD=rover PX4_GZ_MODEL_POSE=\"0,0,2,0,0,0\" make px4_sitl gz_x500_mono_cam_down` 可正确打开世界与模型。
  解答：已将 `model.config/model.sdf` 复制到 `/home/zjh/PX4_Firmware/Tools/simulation/gz/models/rover/`；由于 `PX4_GZ_WORLD=rover` 实际读取 `worlds/rover.sdf`，已将 `/home/zjh/桌面/rover/rover.world` 同步覆盖到 `/home/zjh/PX4_Firmware/Tools/simulation/gz/worlds/rover.sdf`（旧文件备份为 `rover.sdf.bak_20260307`）；启动日志确认加载 `.../worlds/rover.sdf`。
- 2026-03-07 | 问题：把远程仓库中 `simple` 下载至 `zjh_ws`，替代 `zjh_ws` 中的原有内容。
  解答：已执行 `git fetch origin simple`、切换本地 `simple` 分支并 `git reset --hard origin/simple`、`git clean -fd`，本地工作区已对齐远程 `origin/simple`（提交 `df0d3ed`）。
- 2026-03-05 | 问题：将 `zjh_ws` 接入 GitHub 远程并创建 `simple` 分支上传源码。
  解答：已在本工作空间初始化 Git、配置撤回别名、接入远程仓库并推送 `simple` 分支；提交中排除了运行日志目录。
- 2026-03-05 | 问题：远程 `simple` 已存在，是否覆盖远程分支？
  解答：已按确认执行强制推送，使用本地 `zjh_ws` 的 `simple` 覆盖远程 `origin/simple`。
- 2026-03-05 | 问题：将 `tcec_test` 功能包改名为 `tvec` 并构建。
  解答：已完成目录、包名、模块名、launch/rviz/readme 的全量重命名（`tcec_test -> tvec`），并通过构建与启动验证。
- 2026-03-05 | 问题：把 `source install/setup.bash` 加入 `.bashrc`。
  解答：已将 `.bashrc` 中工作空间 source 行切换为 `zjh_ws`，并使用存在性判断避免报错：`if [ -f ~/project/zjh_ws/install/setup.bash ]; then source ...; fi`。
- 2026-03-05 | 问题：不要再生成小写 `readme.md`，并删除已有小写文档。
  解答：已将规则写入“协作与记录要求”（仅允许 `README.md`），并删除现有小写 `readme.md` 文件。
- 2026-03-05 | 问题：把当前工作空间内容上传到远程仓库。
  解答：已在 `zjh_ws` 的 `simple` 分支完成提交并推送到 `origin/simple`。
- 2026-03-06 | 问题：在 `debug_interface` 中创建新接口 `aruco_base_pose`，用于存放 ArUco 坐标系（FLU）下机体坐标系（FLU）的 `xyz+yaw`。
  解答：已新增 `msg/ArucoBasePose.msg`，并纳入 `CMakeLists.txt` 的 `rosidl_generate_interfaces`；构建后通过 `ros2 interface show debug_interface/msg/ArucoBasePose` 验证成功。
- 2026-03-06 | 问题：创建 `relative_pose` 功能包，从 `tvec/rvec` 计算 ArUco(FLU) 下机体(FLU)的 `xyz+yaw`，1Hz 日志输出并发布 `/debug/relative_pose`。
  解答：已创建 `relative_pose`（`ament_python`），新增 `relative_pose_node`：订阅 `/debug/tvec`（`TVecRVec`），计算并发布 `/debug/relative_pose`（`ArucoBasePose`），同时按 1Hz 输出日志；已完成构建与话题类型验证。
- 2026-03-06 | 问题：在 `relative_pose` 功能包中加入 launch，可同时启动 `tvec` 与 `relative_pose` 节点。
  解答：已新增 `relative_pose_with_tvec.launch.py`，通过 `IncludeLaunchDescription` 复用 `tvec.launch.py`，并同时拉起 `relative_pose_node`；已完成构建、source 和 launch 启动验证。
- 2026-03-06 | 问题：创建功能包 `marker_in_cam`，用 `tvec/rvec` 表示 marker 在 cam 坐标系中的 `x/y/z/yaw`，使用已有接口并创建 launch 启动 `tvec`。
  解答：已创建 `marker_in_cam`（`ament_python`），复用 `ArucoBasePose` 接口发布 `/debug/marker_in_cam`，并新增 `marker_in_cam_with_tvec.launch.py` 实现与 `tvec` 联合启动；已完成构建与 launch 验证。
- 2026-03-06 | 问题：相机坐标系定义是 `x右/y下/z前`，当前 `marker_in_cam` 的 `xyz` 输出不对。
  解答：已修正 `marker_in_cam` 为直接输出 camera optical 坐标（不再做 FLU 变换），`x/y/z` 直接对应 `tvec`，并同步调整 `frame_id` 为 `cam_optical` 与文档说明；已完成构建和 launch 回归验证。
- 2026-03-06 | 问题：新增功能包 `cam_in_marker`，输出 cam 在 marker(FLU) 下的 `xyz+yaw`，并提供与 `tvec`、`marker_in_cam` 的联合启动。
  解答：已创建 `cam_in_marker`（`ament_python`），新增 `cam_in_marker_node` 订阅 `/debug/tvec` 并发布 `/debug/cam_in_marker`（`ArucoBasePose`），同时新增联合 launch 一次拉起 `tvec + marker_in_cam + cam_in_marker`；已完成构建与启动验证。
- 2026-03-06 | 问题：创建功能包 `matrix_marker_in_cam`，将 `tvec/rvec` 转为变换矩阵并发布到 `/debug/matrix/marker_in_cam`，launch 需包含启动 `tvec`。
  解答：已创建 `matrix_marker_in_cam`（`ament_python`），新增 `matrix_marker_in_cam_node` 将 `tvec/rvec` 组装为 `marker->cam` 的 4x4 齐次矩阵并发布到 `/debug/matrix/marker_in_cam`，并新增联合 launch 启动 `tvec`；已完成构建和启动验证。
- 2026-03-06 | 问题：要求将输出旋转矩阵绕 Z 轴再旋转 90° 后发布到 ROS 话题。
  解答：已在 `matrix_marker_in_cam_node` 中对旋转矩阵增加 `Rz(+90°)` 修正（右乘方式），并重新构建与环境加载，发布话题仍为 `/debug/matrix/marker_in_cam`。
- 2026-03-06 | 问题：在 `debug_interface` 新增一个 4x4 变换矩阵接口（`R` 为 3x3、`t` 为 3x1、最后一行为 `[0 0 0 1]`）。
  解答：已新增 `TransformMatrix4x4.msg`（`float64[16] matrix`，行优先存储并写明矩阵约定），并纳入 `CMakeLists.txt` 的接口生成列表；构建后通过 `ros2 interface show debug_interface/msg/TransformMatrix4x4` 验证成功。
- 2026-03-06 | 问题：确认功能正确后，将当前工作空间代码上传到远程仓库。
  解答：已在 `simple` 分支完成提交并推送到 `origin/simple`。

## 修改记录

### PX4 仿真资源
- 2026-03-07：重构 `/home/zjh/PX4_Firmware/Tools/simulation/gz/worlds/rover.sdf`：由极简世界改为默认世界参数骨架（补齐 `physics/magnetic_field/atmosphere/scene/spherical_coordinates`），同时保留 `model://rover` include。
- 2026-03-07：同步更新源文件 `/home/zjh/桌面/rover/rover.world`，确保后续再复制时不回退到极简版本。
- 2026-03-07：问题定位结论：`POSCTL` 无法解锁的根因是世界配置过简导致水平状态估计异常（此前观测到 `estimator_status` 的 `velocity_horiz` 与 `pos_horiz` 为 false）。
- 2026-03-07：补充解锁失败处理说明：`Arming denied` 需先通过 `commander check` 定位预检项（常见为 EKF/链路），并按“先恢复健康、后解锁”的顺序处理；仅仿真调试可临时放宽检查或强制解锁。
- 2026-03-07：修复 PX4 启动卡住问题：将 `rover.sdf/rover.world` 中 `<world name="rover_world">` 改为 `<world name="rover">`，与 `PX4_GZ_WORLD=rover` 对齐。
- 2026-03-07：回归验证通过：启动日志出现 `Gazebo world is ready`、`Spawning model at position: 0 0 2`、`world: rover, model: x500_mono_cam_down_0`。
- 2026-03-07：新增 PX4 Gazebo 自定义资源：`/home/zjh/PX4_Firmware/Tools/simulation/gz/models/rover/model.config`、`/home/zjh/PX4_Firmware/Tools/simulation/gz/models/rover/model.sdf`、`/home/zjh/PX4_Firmware/Tools/simulation/gz/worlds/rover.world`。
- 2026-03-07：为匹配 `PX4_GZ_WORLD=rover` 的实际读取规则，已将 `rover.world` 覆盖写入 `/home/zjh/PX4_Firmware/Tools/simulation/gz/worlds/rover.sdf`，并备份原文件为 `rover.sdf.bak_20260307`。
- 2026-03-07：执行 `PX4_GZ_WORLD=rover PX4_GZ_MODEL_POSE=\"0,0,2,0,0,0\" make px4_sitl gz_x500_mono_cam_down` 启动验证，日志确认加载 `.../worlds/rover.sdf`。

### 工作空间与协作规范
- 2026-03-05：进入新工作空间 `~/project/zjh_ws` 后，保留规则并重置历史记录内容（保留标题）。
- 2026-03-05：完成本工作空间 Git 初始化与本地撤回别名配置（`undo/unstage/discard/last`）。
- 2026-03-05：更新 `~/.bashrc` 自动加载当前工作空间环境：由 `project_ws` 改为 `zjh_ws` 的 `install/setup.bash`（带文件存在判断）。
- 2026-03-05：新增文档规范：功能包文档仅使用大写 `README.md`，禁止创建小写 `readme.md`。
- 2026-03-05：确认当前开发工作空间为 `~/project/zjh_ws`，并将其固化为默认工作路径。
- 2026-03-05：新增代码规范：后续代码需编写详细注释，重点解释关键变量、计算流程与控制逻辑。
- 2026-03-05：新增注释语言规范：后续代码注释统一使用中文。
- 2026-03-05：日志结构调整：`### 功能包修改记录` 改为按功能包小标题分组记录，并在该节加入固定记录要求。

### Git 仓库与远程
- 2026-03-07：新建分支 `laptop` 并上传当前工作空间代码至远程 `origin/laptop`，本地分支已跟踪远程同名分支。
- 2026-03-07：按要求将本地工作区强制替换为远程 `origin/simple` 内容，当前对齐提交为 `df0d3ed`。
- 2026-03-05：接入远程仓库 `origin=https://github.com/junheng-sect/ROS2PUBLIC.git`。
- 2026-03-05：创建并推送 `simple` 分支，上传本工作空间源码。
- 2026-03-05：执行 `git push -f origin simple`，远程 `simple` 已由 `71fdeb0` 强制更新覆盖。
- 2026-03-05：执行常规推送，提交 `426eaaf` 已上传到 `origin/simple`。
- 2026-03-06：执行常规推送，提交 `03e2da2` 已上传到 `origin/simple`。

### 功能包修改记录
- 记录要求：本节必须按“功能包小标题”分组记录修改，后续新增记录不得混写在同一列表中。

#### tvec
- 2026-03-05：将功能包 `tcec_test` 全量重命名为 `tvec`：目录 `src/tvec`、Python 包 `tvec`、资源索引、`package.xml/setup.py/setup.cfg`、launch 文件 `tvec.launch.py`、RViz 配置 `tvec.rviz`、话题前缀 `/tvec/...`、README/readme 与默认 CSV 路径均已同步更新。
- 2026-03-05：完成 `tvec` 验证：`colcon build --packages-select tvec`、`source install/setup.bash`、`ros2 launch tvec tvec.launch.py world_name:=rover` 启动成功（桥接、节点、RViz 正常拉起）。
- 2026-03-05：按新文档规范删除小写文件 `src/tvec/readme.md`，仅保留 `src/tvec/README.md`。
- 2026-03-05：按“详细注释”要求补充现有代码注释：`src/tvec/tvec/tvec_rvec_node.py` 增加参数语义、检测与位姿估计流程、CSV记录与发布流程注释；`src/tvec/launch/tvec.launch.py` 增加世界切换与桥接逻辑注释；并完成 `colcon build` 与 `source install/setup.bash`。
- 2026-03-05：将现有英文注释统一改为中文：覆盖 `src/tvec/tvec/tvec_rvec_node.py`、`src/tvec/launch/tvec.launch.py`；完成 `colcon build`、`source install/setup.bash`，并执行 `ros2 launch tvec tvec.launch.py` 启动验证后停止。
- 2026-03-05：为 `tvec` 新增调试话题发布：在 `src/tvec/tvec/tvec_rvec_node.py` 增加 `debug_interface/msg/TVecRVec` 发布器，实时发布到 `/debug/tvec`；更新 `src/tvec/package.xml` 增加 `debug_interface` 依赖。已完成 `colcon build`、`source install/setup.bash`，并通过 `ros2 topic info /debug/tvec` 验证话题类型为 `debug_interface/msg/TVecRVec`。

#### debug_interface
- 2026-03-05：补齐 `debug_interface` 接口包实现：新增 `msg/TVecRVec.msg`（`header+tvec+rvec`），并更新 `CMakeLists.txt/package.xml` 的 `rosidl` 生成配置；重新构建后通过 `ros2 interface show debug_interface/msg/TVecRVec` 测试。
- 2026-03-05：将 `src/debug_interface/msg/TVecRVec.msg` 的字段注释统一改为中文（时间戳/参考系、tvec 语义、rvec 语义）。
- 2026-03-06：新增接口 `src/debug_interface/msg/ArucoBasePose.msg`，字段为 `header + x + y + z + yaw`（FLU 语义）；同步更新 `src/debug_interface/CMakeLists.txt` 的接口生成列表，并完成构建与接口显示验证。
- 2026-03-06：新增接口 `src/debug_interface/msg/TransformMatrix4x4.msg`，用于 4x4 齐次变换矩阵（`R(3x3)+t(3x1)+[0 0 0 1]`）；字段采用行优先 `float64[16] matrix`，并同步更新 `src/debug_interface/CMakeLists.txt` 生成配置，构建与 `interface show` 验证通过。

#### relative_pose
- 2026-03-06：新建功能包 `src/relative_pose`（`ament_python`，Apache-2.0），新增节点 `relative_pose_node`：订阅 `/debug/tvec`（`debug_interface/msg/TVecRVec`），将 `tvec/rvec` 转换为 ArUco(FLU) 下机体(FLU)位姿并发布到 `/debug/relative_pose`（`debug_interface/msg/ArucoBasePose`）。
- 2026-03-06：在 `relative_pose_node` 中实现 1Hz 日志输出（`x/y/z/yaw`），并加入中文详细注释说明光学坐标系到 FLU 的基变换、位姿求逆与 yaw 提取流程。
- 2026-03-06：更新 `src/relative_pose/package.xml` 与 `src/relative_pose/setup.py` 依赖和入口点，新增 `src/relative_pose/README.md`；完成 `colcon build --symlink-install`、`source install/setup.bash`，并通过 `ros2 topic info /debug/relative_pose` 验证话题类型为 `debug_interface/msg/ArucoBasePose`。
- 2026-03-06：新增联合启动文件 `src/relative_pose/launch/relative_pose_with_tvec.launch.py`，可一次启动 `tvec` 全链路（桥接+检测+RViz）与 `relative_pose_node`；同步更新 `src/relative_pose/setup.py` 安装 launch 文件、`src/relative_pose/package.xml` 增加 `tvec/launch/launch_ros` 依赖，`README.md` 补充联合启动命令。
- 2026-03-06：执行 `ros2 launch relative_pose relative_pose_with_tvec.launch.py world_name:=rover` 验证通过：`parameter_bridge`、`tvec_rvec_node`、`rviz2`、`relative_pose_node` 均正常拉起。

#### marker_in_cam
- 2026-03-06：新建功能包 `src/marker_in_cam`（`ament_python`，Apache-2.0），新增节点 `marker_in_cam_node`：订阅 `/debug/tvec`（`debug_interface/msg/TVecRVec`），计算并发布 marker 在 cam(FLU) 下的 `x/y/z/yaw` 到 `/debug/marker_in_cam`（`debug_interface/msg/ArucoBasePose`）。
- 2026-03-06：在 `marker_in_cam_node` 中加入 1Hz 日志输出与中文详细注释，说明 optical->FLU 转换、旋转矩阵求解与 yaw 提取逻辑。
- 2026-03-06：新增联合启动文件 `src/marker_in_cam/launch/marker_in_cam_with_tvec.launch.py`，可一次启动 `tvec` 全链路与 `marker_in_cam_node`；同步更新 `src/marker_in_cam/package.xml` 依赖与 `src/marker_in_cam/setup.py` 安装项，新增 `src/marker_in_cam/README.md`。
- 2026-03-06：执行 `ros2 launch marker_in_cam marker_in_cam_with_tvec.launch.py world_name:=rover` 验证通过：`parameter_bridge`、`tvec_rvec_node`、`rviz2`、`marker_in_cam_node` 均正常拉起。
- 2026-03-06：按 optical 坐标定义修正输出：`src/marker_in_cam/marker_in_cam/marker_in_cam_node.py` 改为直接发布 `tvec` 作为 `x/y/z`（`x右/y下/z前`），取消 FLU 变换；`yaw` 从 `R_cm_opt` 提取，默认 `frame_id` 改为 `cam_optical`，并同步更新 launch 与 README。

#### cam_in_marker
- 2026-03-06：新建功能包 `src/cam_in_marker`（`ament_python`，Apache-2.0），新增节点 `cam_in_marker_node`：订阅 `/debug/tvec`（`debug_interface/msg/TVecRVec`），计算并发布 cam 在 marker(FLU) 下的 `x/y/z/yaw` 到 `/debug/cam_in_marker`（`debug_interface/msg/ArucoBasePose`）。
- 2026-03-06：在 `cam_in_marker_node` 中加入中文详细注释与 1Hz 日志输出，包含 optical->FLU 变换、位姿求逆（`marker->cam` 转 `cam in marker`）与 yaw 提取流程。
- 2026-03-06：新增联合启动文件 `src/cam_in_marker/launch/cam_in_marker_with_tvec_marker_in_cam.launch.py`，同时拉起 `tvec`、`marker_in_cam_node`、`cam_in_marker_node`；同步更新 `src/cam_in_marker/package.xml`、`src/cam_in_marker/setup.py` 与 `src/cam_in_marker/README.md`。
- 2026-03-06：执行 `ros2 launch cam_in_marker cam_in_marker_with_tvec_marker_in_cam.launch.py world_name:=rover` 验证通过：`parameter_bridge`、`tvec_rvec_node`、`rviz2`、`marker_in_cam_node`、`cam_in_marker_node` 均正常拉起。

#### matrix_marker_in_cam
- 2026-03-06：新建功能包 `src/matrix_marker_in_cam`（`ament_python`，Apache-2.0），新增节点 `matrix_marker_in_cam_node`：订阅 `/debug/tvec`（`debug_interface/msg/TVecRVec`），将 `tvec/rvec` 转换为 `marker->cam` 4x4 齐次变换矩阵，并发布到 `/debug/matrix/marker_in_cam`（`debug_interface/msg/TransformMatrix4x4`）。
- 2026-03-06：新增联合启动文件 `src/matrix_marker_in_cam/launch/matrix_marker_in_cam_with_tvec.launch.py`，通过 include 启动 `tvec.launch.py` 并拉起 `matrix_marker_in_cam_node`；同步更新 `src/matrix_marker_in_cam/package.xml`、`src/matrix_marker_in_cam/setup.py` 和 `src/matrix_marker_in_cam/README.md`。
- 2026-03-06：执行 `ros2 launch matrix_marker_in_cam matrix_marker_in_cam_with_tvec.launch.py world_name:=rover` 验证通过：`parameter_bridge`、`tvec_rvec_node`、`rviz2`、`matrix_marker_in_cam_node` 正常拉起。
- 2026-03-06：按需求调整输出旋转：`src/matrix_marker_in_cam/matrix_marker_in_cam/matrix_marker_in_cam_node.py` 新增 `Rz(+90°)` 修正并应用到输出旋转矩阵（`R_out = R_in @ Rz90`），保持平移向量与话题名不变。
- 2026-03-06 | 问题：`matrix_baselink_in_marker_node.py` 文件被删，要求还原。
  解答：已重建 `src/matrix_baselink_in_marker/matrix_baselink_in_marker/matrix_baselink_in_marker_node.py`（恢复为基础链式变换版本，不含额外 Z 轴 180°补偿），并通过构建验证。
- 2026-03-06 | 问题：新增 `matrix_baselink_in_cam`，并将 `matrix_baselink_in_marker` 改为通过矩阵链乘计算，同时联动 `vision_pose_tf_node` 和 `rivz`。
  解答：已完成新包创建与节点实现，按 `T_mb = inv(T_bc) @ T_mc` 计算 `marker->baselink`，并将新节点接入 `rivz_full.launch.py`；同时更新 `vision_pose_tf_node` 的输入语义说明。已执行构建、source 和相关 launch 启动验证。

#### matrix_baselink_in_cam
- 2026-03-06：新建功能包 `src/matrix_baselink_in_cam`（`ament_python`，Apache-2.0），新增节点 `matrix_baselink_in_cam_node`，发布固定外参矩阵到 `/debug/matrix/baselink_in_cam`（平移 `xyz=0`，旋转轴映射：`baselink_x->cam_-y`、`baselink_y->cam_-x`、`baselink_z->cam_-z`）。
- 2026-03-06：新增启动文件 `src/matrix_baselink_in_cam/launch/matrix_baselink_in_cam.launch.py` 与 `README.md`，并在 `setup.py/package.xml` 中补齐入口点与依赖。
- 2026-03-06：执行 `ros2 launch matrix_baselink_in_cam matrix_baselink_in_cam.launch.py` 验证通过，节点正常发布矩阵并输出 1Hz 日志。

#### matrix_baselink_in_marker
- 2026-03-06：重构 `src/matrix_baselink_in_marker/matrix_baselink_in_marker/matrix_baselink_in_marker_node.py`，由“固定外参硬编码”改为“双输入矩阵链乘”模式：订阅 `/debug/matrix/marker_in_cam` 与 `/debug/matrix/baselink_in_cam`，按 `T_mb = inv(T_bc) @ T_mc` 计算并发布 `/debug/matrix/baselink_in_marker`。
- 2026-03-06：更新 `matrix_baselink_in_marker_full.launch.py`，新增 `matrix_baselink_in_cam_node` 并调整参数名为 `marker_in_cam_topic/baselink_in_cam_topic`；同步更新 `README.md` 与 `package.xml` 依赖。
- 2026-03-06：执行 `ros2 launch matrix_baselink_in_marker matrix_baselink_in_marker_full.launch.py world_name:=rover` 启动验证通过。

#### tf_broadcast
- 2026-03-06：更新 `src/tf_broadcast/tf_broadcast/vision_pose_tf_node.py` 的语义说明与日志文案，明确输入为 `marker->baselink` 矩阵来源，便于与新矩阵链路对齐。

#### rivz
- 2026-03-06：更新 `src/rivz/launch/rivz_full.launch.py`，新增 `matrix_baselink_in_cam_node` 并将 `matrix_baselink_in_marker_node` 切换为双输入矩阵参数；更新 `src/rivz/package.xml` 依赖和 `src/rivz/README.md` 节点说明。
- 2026-03-06：执行 `ros2 launch rivz rivz_full.launch.py world_name:=rover` 启动验证通过（节点可全部拉起，TF/矩阵链路运行正常）。
- 2026-03-06 | 问题：为什么 RViz 中 `vision_pose` 和 `cam` 看起来在 `aruco_marker` 两侧近似对称？
  解答：核心原因是 `cam_tf_node` 订阅的是 `/debug/matrix/cam_in_marker`（语义为 `cam->marker`），却按 `aruco_marker->cam` 直接发布 TF，方向被反用了；而 `vision_pose_tf_node` 使用的是 `marker->baselink`。一个正向一个反向，位置会表现为绕 marker 的镜像/对称分布。
- 2026-03-06 | 问题：目标是让 `vision_pose` 原点与 `cam` 重合，同时保持 `vision_pose` 当前朝向不变。
  解答：已修改 `vision_pose_tf_node` 为双输入组合：位置取 `marker->cam`，姿态取 `marker->baselink`，实现“同位置、保朝向”目标；并完成构建与环境加载。

#### tf_broadcast
- 2026-03-06：更新 `src/tf_broadcast/tf_broadcast/vision_pose_tf_node.py`：新增双话题订阅 `pose_matrix_topic=/debug/matrix/baselink_in_marker` 与 `cam_matrix_topic=/debug/matrix/marker_in_cam`，发布时采用“平移来自 `marker->cam`、旋转来自 `marker->baselink`”的组合策略，使 `vision_pose` 原点与 `cam` 重合且朝向保持原逻辑不变。
- 2026-03-06 | 问题：创建新功能包 `tvec_tf`，广播 `arucomarker->vision_pose`（`x=tvec[1], y=tvec[0], z=tvec[2]`，旋转先置零），并提供 launch 同时启动 `tvec`、`arucomarker` 静态广播和 RViz 显示。
  解答：已完成 `tvec_tf` 包、TF 广播节点和联合启动文件；`tvec.launch.py` 已包含图像桥接与 RViz，因此可直接在 RViz 同时看到图像与 TF。已构建并完成 launch 启动验证。

#### tvec_tf
- 2026-03-06：新建功能包 `src/tvec_tf`（`ament_python`，Apache-2.0），新增节点 `tvec_tf_node`：订阅 `/debug/tvec`（`debug_interface/msg/TVecRVec`），广播 `arucomarker->vision_pose`，平移映射为 `x=tvec[1], y=tvec[0], z=tvec[2]`，旋转采用单位四元数 `q=(0,0,0,1)`（欧拉角全零）。
- 2026-03-06：新增 `src/tvec_tf/launch/tvec_tf.launch.py`：联合启动 `tvec.launch.py`（图像桥接+RViz）、`tf_broadcast/map_aruco_static_tf_node`（`map->arucomarker`）和 `tvec_tf_node`。
- 2026-03-06：补充 `src/tvec_tf/README.md`，并更新 `src/tvec_tf/setup.py/package.xml` 的入口点、launch 安装与依赖说明。
- 2026-03-06：完成 `colcon build --symlink-install`、`source install/setup.bash`，并执行 `ros2 launch tvec_tf tvec_tf.launch.py world_name:=rover` 启动验证通过。
- 2026-03-06 | 问题：在 `tvec` 功能包中增加一个节点，通过 `rvec` 计算相对 `yaw` 并在日志输出。
  解答：已新增 `rvec_yaw_node`，订阅 `/debug/tvec`，将 `rvec` 通过 Rodrigues 转旋转矩阵并提取 `yaw=atan2(R10,R00)`，按 1Hz 输出（rad/deg）。已完成构建、source 和节点启动验证。

#### tvec
- 2026-03-06：新增节点 `src/tvec/tvec/rvec_yaw_node.py`，订阅 `debug_interface/msg/TVecRVec` 并由 `rvec` 计算相对 `yaw`（支持角度归一化到 `[-pi, pi]`），以 1Hz 日志输出。
- 2026-03-06：更新 `src/tvec/setup.py`，新增可执行入口 `rvec_yaw_node`；更新 `src/tvec/package.xml` 补充 `python3-numpy` 运行依赖；更新 `src/tvec/README.md` 增加新节点说明与运行命令。
- 2026-03-06：完成 `colcon build --symlink-install`、`source install/setup.bash`，并通过 `ros2 run tvec rvec_yaw_node` 启动验证节点可正常运行。
- 2026-03-06 | 问题：把计算 yaw 的节点加入 `tvec_tf` 功能包的 launch 中。
  解答：已在 `tvec_tf.launch.py` 中新增 `tvec/rvec_yaw_node` 启动项，并完成构建、source、以及 `ros2 launch tvec_tf tvec_tf.launch.py world_name:=rover` 启动验证，确认图像、TF 与 yaw 日志可同时运行。

#### tvec_tf
- 2026-03-06：更新 `src/tvec_tf/launch/tvec_tf.launch.py`，新增 `rvec_yaw_node`（`package=tvec`）到联合启动链路，实现与 `tvec_rvec_node`、静态 TF、`tvec_tf_node` 同步启动。
- 2026-03-06 | 问题：将 `yaw` 加入 `arucomarker->vision_pose` 的 TF 旋转中。
  解答：已在 `tvec_tf_node` 中基于 `rvec` 计算 `yaw`，并将其转换为 Z 轴四元数写入 TF 旋转；平移映射保持不变。已构建、source，并通过 `tvec_tf.launch.py` 启动验证日志中可见 `yaw=xx deg`。

#### tvec_tf
- 2026-03-06：更新 `src/tvec_tf/tvec_tf/tvec_tf_node.py`：新增 `rvec->旋转矩阵->yaw` 计算（`yaw=atan2(R10,R00)`，归一化到 `[-pi, pi]`），并将 yaw 以四元数形式写入 `arucomarker->vision_pose` 的 TF 旋转；日志新增 yaw 角度输出。
- 2026-03-06：完成 `colcon build --symlink-install`、`source install/setup.bash`，并执行 `ros2 launch tvec_tf tvec_tf.launch.py world_name:=rover` 回归验证通过。
- 2026-03-06 | 问题：将 `tvec_tf_node.py` 中平移改为指定公式：`tx=tvec[1]*cos(yaw)+tvec[0]*sin(yaw)`、`ty=-tvec[1]*sin(yaw)+tvec[0]*sin(yaw)`。
  解答：已按给定公式原样修改 `tx/ty` 计算，并保持 `tz=tvec[2]` 与 yaw 旋转写入逻辑不变；已完成构建与环境加载。

#### tvec_tf
- 2026-03-06：更新 `src/tvec_tf/tvec_tf/tvec_tf_node.py` 的平移计算公式，改为基于 `yaw` 的组合形式：`tx=tvec[1]*cos(yaw)+tvec[0]*sin(yaw)`，`ty=-tvec[1]*sin(yaw)+tvec[0]*sin(yaw)`，`tz=tvec[2]`。
- 2026-03-06：完成 `colcon build --symlink-install` 与 `source install/setup.bash`。
- 2026-03-06 | 问题：在 `tvec_tf.launch.py` 中加入 baselink 坐标系广播节点 `map_base_link_tf_node`。
  解答：已在 launch 中新增 `tf_broadcast/map_base_link_tf_node`（`pose_topic=/mavros/local_position/pose`，`map->base_link`），并完成构建、source 与启动验证。

#### tvec_tf
- 2026-03-06：更新 `src/tvec_tf/launch/tvec_tf.launch.py`，新增 `map_base_link_tf_node` 启动项，联合发布 `map->base_link` 动态 TF。
- 2026-03-06：完成 `colcon build --symlink-install`、`source install/setup.bash`，并执行 `ros2 launch tvec_tf tvec_tf.launch.py world_name:=rover` 验证通过（日志确认 `map_base_link_tf_node` 成功启动并发布 TF）。
- 2026-03-06 | 问题：将得到的 `xyz` 和 `yaw` 以 1Hz 日志输出，并通过 `ArucoBasePose.msg` 发布到 `/debug/aruco_pose`。
  解答：已在 `tvec_tf_node` 中新增 `ArucoBasePose` 发布器，发布话题 `/debug/aruco_pose`；日志按 1Hz 输出 `x/y/z/yaw`（rad+deg）。已完成构建与环境加载，并验证 `/debug/aruco_pose` 发布器存在。

#### tvec_tf
- 2026-03-06：更新 `src/tvec_tf/tvec_tf/tvec_tf_node.py`，新增 `debug_interface/msg/ArucoBasePose` 发布到 `/debug/aruco_pose`，字段包含计算后的 `x/y/z/yaw`；1Hz 日志改为输出同一组 `aruco_pose` 数据。
- 2026-03-06：更新 `src/tvec_tf/README.md`，补充 `/debug/aruco_pose` 话题与消息类型说明，并将旋转说明更新为“由 `rvec` 提取 yaw”。
- 2026-03-06：完成 `colcon build --symlink-install`、`source install/setup.bash`，并通过 `ros2 topic info /debug/aruco_pose` 验证发布器存在。
- 2026-03-06 | 问题：屏蔽三类日志输出：`tvec_rvec_node` 的 `ID/tvec/rvec` 行、`rvec_yaw_node` 的 `rvec/yaw` 行、`map_base_link_tf_node` 的 `发布TF map->base_link` 行。
  解答：已在对应节点的 1Hz `log_callback` 中将上述 `INFO` 输出注释掉，功能逻辑与话题发布保持不变；已完成构建与环境加载。

#### tvec
- 2026-03-06：在 `src/tvec/tvec/tvec_rvec_node.py` 注释掉 1Hz 周期日志（含 `No detection in last period` 和 `ID/tvec/rvec` 输出）。
- 2026-03-06：在 `src/tvec/tvec/rvec_yaw_node.py` 注释掉 1Hz 周期日志（含 `尚未收到 /debug/tvec 数据` 和 `rvec/yaw` 输出）。

#### tf_broadcast
- 2026-03-06：在 `src/tf_broadcast/tf_broadcast/map_base_link_tf_node.py` 注释掉 1Hz 周期日志（含 `发布TF map->base_link` 与“尚未收到 Pose 数据”输出）。
- 2026-03-06：完成 `colcon build --symlink-install` 与 `source install/setup.bash`。
- 2026-03-07 | 问题：在本工作空间创建 `aruco_tracking` 功能包，要求进入 OFFBOARD 后通过 PID 调速控制无人机飞到标志上方并正对，输入从 `/debug/aruco_pose` 读取。
  解答：已新建 `aruco_tracking`（ament_python），实现 `tracking_node`：订阅 `/debug/aruco_pose` 与 `/mavros/state`，在 `OFFBOARD` 且视觉有效时执行 PID 控制并发布 `/mavros/setpoint_velocity/cmd_vel`，目标默认为 `x=0,y=0,z=1.5,yaw=0`；视觉超时或非 OFFBOARD 自动零速悬停。

#### aruco_tracking
- 2026-03-07：新建功能包 `src/aruco_tracking`（`ament_python`，Apache-2.0），新增节点 `src/aruco_tracking/aruco_tracking/tracking_node.py`，实现基于 `/debug/aruco_pose` 的 PID 速度控制（x/y/z/yaw）与 OFFBOARD 门控逻辑。
- 2026-03-07：新增启动文件 `src/aruco_tracking/launch/aruco_tracking.launch.py`，联合启动 `tvec_tf.launch.py` 与 `tracking_node`。
- 2026-03-07：完善 `src/aruco_tracking/setup.py`、`src/aruco_tracking/package.xml`、`src/aruco_tracking/README.md`，补齐入口点、launch 安装与使用说明。
- 2026-03-07：完成 `colcon build --symlink-install`、`source install/setup.bash`，并执行 `ros2 launch aruco_tracking aruco_tracking.launch.py world_name:=rover` 启动验证通过。
- 2026-03-07 | 问题：跟踪轨迹呈圆弧，控制量看起来不对。
  解答：已在 `aruco_tracking` 中加入坐标系补偿：先在 marker 坐标系计算 XY 误差，再按 `marker_in_map_yaw_deg`（默认 +90°）旋转到 map/ENU 后输出速度控制，避免因误差坐标系与速度命令坐标系不一致导致绕圈。

#### aruco_tracking
- 2026-03-07：更新 `src/aruco_tracking/aruco_tracking/tracking_node.py`，新增参数 `rotate_error_to_map`（默认 `True`）和 `marker_in_map_yaw_deg`（默认 `90.0`），并实现 `err_marker -> err_cmd(map)` 的二维旋转补偿再做 PID。
- 2026-03-07：状态日志改为同时输出 `err_marker` 与 `err_cmd`，便于区分“输入误差”和“参与控制的误差”。
- 2026-03-07：更新 `src/aruco_tracking/README.md` 说明坐标旋转补偿用途，完成 `colcon build --symlink-install` 与 `source install/setup.bash`。
- 2026-03-07 | 问题：这次控制正确后，询问“刚才具体改了什么”。
  解答：核心改动是给 `aruco_tracking` 增加“误差坐标旋转补偿”：先在 marker 坐标系计算 `ex_m/ey_m`，再按 `marker_in_map_yaw_deg`（默认 +90°）旋转成 map/ENU 下 `ex_cmd/ey_cmd` 后进入 PID，从而避免因坐标系不一致导致圆弧轨迹；同时日志增加 `err_marker` 与 `err_cmd` 的分离输出便于验证。
- 2026-03-07 | 问题：去掉 z 方向 PID 调节，保持相同高度。
  解答：已在 `aruco_tracking` 控制器中移除 z 方向 PID 链路，控制输出改为 `vz=0` 常量；节点仅控制 x/y/yaw。同步更新 launch 参数与 README 说明，并完成构建、source、launch 启动验证。

#### aruco_tracking
- 2026-03-07：更新 `src/aruco_tracking/aruco_tracking/tracking_node.py`，删除 `target_z`、`kp_z/ki_z/kd_z`、`vz_limit`、`enable_z_control` 等 z-PID 参数及 `pid_z` 控制器，控制环中 `vz` 固定为 `0.0`。
- 2026-03-07：更新 `src/aruco_tracking/launch/aruco_tracking.launch.py`，移除 z 方向相关参数传递。
- 2026-03-07：更新 `src/aruco_tracking/README.md`，明确“z 方向不做 PID，固定 `vz=0` 保持高度”。
- 2026-03-07：完成 `colcon build --symlink-install`、`source install/setup.bash`，并执行 `ros2 launch aruco_tracking aruco_tracking.launch.py world_name:=rover` 启动验证通过。
- 2026-03-07 | 问题：为何日志中高度（`/debug/aruco_pose.z`）一直下降？
  解答：`/debug/aruco_pose.z` 是基于视觉解算得到的相对量（marker 与相机的相对几何距离），不是飞控绝对高度；在 Offboard 下当前控制器 `vz=0` 仅表示“不给垂向速度指令”，不等于“严格锁高”。若要稳定真实高度，应查看 `/mavros/local_position/pose.z` 并增加高度保持策略（例如位置锁高或非PID/小P补偿）。
- 2026-03-07 | 问题：希望使用“相对高度保持不变”方式定高，并用 PID 控制相对高度。
  解答：已在 `aruco_tracking` 中新增“相对高度保持 PID”：进入 OFFBOARD 后锁定当前 `aruco_pose.z` 为 `z_ref`，后续用 `ez=z_ref-z_now` 经 PID 计算 `vz`；同时保留 x/y/yaw 跟踪。期间修复了一处 f-string 语法错误并通过构建与 launch 验证。

#### aruco_tracking
- 2026-03-07：在 `src/aruco_tracking/aruco_tracking/tracking_node.py` 新增参数 `enable_relative_z_hold`、`reset_z_ref_on_offboard`、`kp_z_hold/ki_z_hold/kd_z_hold`、`vz_limit`，并新增 `pid_z_hold` 控制器与 `z_ref` 状态。
- 2026-03-07：控制循环改为“OFFBOARD后首次锁定 z_ref，再按 `ez=z_ref-z_now` 输出 `vz`”，实现相对高度 PID 保持。
- 2026-03-07：更新 `src/aruco_tracking/launch/aruco_tracking.launch.py`，加入相对高度保持参数默认值。
- 2026-03-07：更新 `src/aruco_tracking/README.md`，说明 z 方向由“相对高度保持 PID”实现。
- 2026-03-07：修复 `tracking_node.py` 的 f-string 语法错误（`float("nan")` -> `float("nan")` 的转义问题已更正为合法表达式），完成 `colcon build --symlink-install`、`source install/setup.bash`，并执行 `ros2 launch aruco_tracking aruco_tracking.launch.py world_name:=rover` 验证通过。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：把 `project_ws` 中的 `rover_auto_motion` 功能包迁移到当前工作空间 `zjh_ws`。
  解答：已完成迁移：`/home/zjh/project/project_ws/src/rover_auto_motion -> /home/zjh/project/zjh_ws/src/rover_auto_motion`，并按规范删除小写 `readme.md`，仅保留 `README.md`。

## 修改记录（本轮补充）
### 功能包修改记录
#### rover_auto_motion
- 2026-03-07：从 `project_ws` 迁移 `rover_auto_motion` 到 `zjh_ws/src`，保留包结构、launch、节点与配置文件。
- 2026-03-07：按文档规范删除 `src/rover_auto_motion/readme.md`，仅保留 `src/rover_auto_motion/README.md`。
- 2026-03-07：已执行 `colcon build --symlink-install --packages-select rover_auto_motion` 并执行 `source install/setup.bash`，包可被 `ros2 pkg list` 正常识别。
- 2026-03-07 | 问题：给定一段 `aruco_tracking` 日志，跟踪失败原因是什么？
  解答：从日志看是“控制方向与坐标系未完全一致导致目标越追越远 + 最终目标丢失触发视觉超时”。表现为：`err_marker.x` 从约 1m 持续增到 3m、`vy` 长时间打到限幅 `-0.800`，说明控制在持续推向错误方向或横向补偿不足；随后 `tvec_tf_node` 多秒输出完全相同数据，`tracking_node` 判定 `pose_timeout_sec` 超时进入零速悬停。根因可归纳为 `aruco_pose` 语义与 `/cmd_vel` 控制坐标系之间仍存在符号/旋转不一致，叠加速度限幅与视场边界后导致目标离开相机视野。
- 2026-03-07 | 问题：希望无论 ArUco 坐标系 yaw 如何变化，无人机都能正确跟踪 xyz 和 yaw；当前 yaw 正常但 xy 输出固定在 E/N。
  解答：已在 `aruco_tracking` 中将 XY 误差变换从“固定角旋转”改为“动态 yaw 旋转”，实时使用 `yaw_map_marker = yaw_map_base - yaw_marker_base`（`yaw_map_base` 来自 `/mavros/local_position/pose`，`yaw_marker_base` 来自 `/debug/aruco_pose.yaw`）；并新增本地位姿超时保护，超时时零速悬停。

#### aruco_tracking
- 2026-03-07：更新 `src/aruco_tracking/aruco_tracking/tracking_node.py`，新增参数 `base_pose_topic`、`use_dynamic_marker_yaw`、`fallback_marker_in_map_yaw_deg`、`base_pose_timeout_sec`，并新增 `/mavros/local_position/pose` 订阅用于提取 `yaw_map_base`。
- 2026-03-07：在控制循环中将 XY 误差变换改为动态旋转：`e_map = R(yaw_map_marker) * e_marker`，其中 `yaw_map_marker = wrap(yaw_map_base - yaw_marker_base)`；保留 yaw 跟踪目标 `yaw_marker_base -> 0` 与相对高度 PID 策略。
- 2026-03-07：新增本地位姿新鲜度判断 `is_base_pose_fresh()`，当启用动态 yaw 且本地位姿超时时，直接输出零速悬停，避免错误速度发散。
- 2026-03-07：增强 1Hz 状态日志，新增 `yaw_map_base`、`yaw_marker_base`、`yaw_map_marker`、`err_map` 字段，便于在线排查坐标变换。
- 2026-03-07：更新 `src/aruco_tracking/launch/aruco_tracking.launch.py`，替换为新参数组（动态 yaw + 回退角 + base_pose 超时）。
- 2026-03-07：更新 `src/aruco_tracking/README.md`，补充动态 yaw 旋转机制与超时保护说明。
- 2026-03-07：已执行 `colcon build --symlink-install --packages-select aruco_tracking`、`source install/setup.bash`，并执行 `ros2 launch aruco_tracking aruco_tracking.launch.py world_name:=rover` 启动验证通过后停止。
- 2026-03-07 | 问题：`aruco_tracking` 日志持续出现“本地位姿超时，输出零速悬停”。
  解答：该现象说明节点未持续收到 `/mavros/local_position/pose`（或频率过低低于 `base_pose_timeout_sec`），因此触发动态 yaw 模式下的保护逻辑。排查顺序：先确认 MAVROS 已启动并有本地位姿数据（`ros2 topic hz /mavros/local_position/pose`），再确认话题名是否一致；临时可通过增大 `base_pose_timeout_sec` 或关闭 `use_dynamic_marker_yaw` 进行对照测试。
- 2026-03-07 | 问题：要求“直接进行测试”，定位 `本地位姿超时，输出零速悬停`。
  解答：已完成在线测试并定位根因：`/mavros/local_position/pose` 发布端 QoS 为 `BEST_EFFORT`，`aruco_tracking` 原订阅使用默认 QoS（`RELIABLE`）导致不兼容，从而回调收不到本地位姿并持续触发超时悬停。修复后已复测通过。

#### aruco_tracking
- 2026-03-07：修复 `src/aruco_tracking/aruco_tracking/tracking_node.py` 中 `/mavros/local_position/pose` 订阅 QoS，改为显式 `BEST_EFFORT + VOLATILE + KEEP_LAST(depth=10)`，并补充中文注释说明兼容性原因。
- 2026-03-07：执行测试验证：
  1) `ros2 topic hz /mavros/local_position/pose` 约 30Hz；
  2) `ros2 topic info /mavros/local_position/pose -v` 确认发布端为 `BEST_EFFORT`；
  3) `ros2 launch aruco_tracking aruco_tracking.launch.py world_name:=rover` 复测中 `tracking_node` 已进入 `OFFBOARD跟踪中`，不再出现“本地位姿超时”连续报错。
- 2026-03-07：按规则执行 `colcon build --symlink-install --packages-select aruco_tracking` 与 `source install/setup.bash`，并完成 launch 启动验证后停止。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：`ros2 launch rover_auto_motion rover_auto_motion.launch.py` 启动失败，提示缺少 `rover_teleop_ackermann`。
  解答：已将 `rover_auto_motion.launch.py` 改为“可选加载控制器”：若检测不到 `rover_teleop_ackermann` 则仅打印提示并跳过该节点，launch 继续启动 `rover_auto_loop_node`，不再因缺包直接退出。

## 修改记录（本轮补充）
### 功能包修改记录
#### rover_auto_motion
- 2026-03-07：更新 `src/rover_auto_motion/launch/rover_auto_motion.launch.py`，新增 `OpaqueFunction + PackageNotFoundError` 检查，将 `rover_teleop_ackermann` 作为可选组件加载（缺失时跳过并提示）。
- 2026-03-07：更新 `src/rover_auto_motion/package.xml`，移除强制 `exec_depend`：`rover_teleop_ackermann`，新增 `ament_index_python` 依赖用于运行时包检测。
- 2026-03-07：更新 `src/rover_auto_motion/README.md`，补充“缺失 `rover_teleop_ackermann` 时的行为说明”。
- 2026-03-07：已执行 `colcon build`、`source install/setup.bash`，并执行 `ros2 launch rover_auto_motion rover_auto_motion.launch.py` 验证：launch 正常启动，`rover_auto_loop_node` 循环运行，缺失包仅提示不致命。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：`rover_auto_motion` 日志正常但 rover 不运动，要求继续调试直到按代码功能运动。
  解答：已定位原因为 `ros_gz_bridge parameter_bridge` 未成功桥接 `/world/rover/set_pose` 服务（ROS 侧服务不存在），导致控制命令未生效。已改为在节点中直接调用 `gz service` 设置模型位姿，绕过服务桥接差异；联调验证中 `pose_cmd` 连续变化，且 Gazebo `/world/rover/pose/info` 中 `rover` 的 `x/y/姿态` 实时变化，确认可按“直行+右转”循环运动。

## 修改记录（本轮补充）
### 功能包修改记录
#### rover_auto_motion
- 2026-03-07：重构 `src/rover_auto_motion/rover_auto_motion/rover_auto_loop_node.py` 控制链路：由发布 Ackermann 话题改为直接调用 `gz service /world/rover/set_pose`（`gz.msgs.Pose -> gz.msgs.Boolean`）驱动模型位姿。
- 2026-03-07：新增位姿积分日志 `pose_cmd`（1Hz），用于输出 `x/y/yaw` 并在线确认运动轨迹。
- 2026-03-07：更新 `src/rover_auto_motion/launch/rover_auto_motion.launch.py`，移除无效的 `parameter_bridge` 服务桥接节点，仅保留 `rover_auto_loop_node`。
- 2026-03-07：更新 `src/rover_auto_motion/package.xml` 与 `src/rover_auto_motion/README.md`，同步“直接调用 gz service”的实现说明与依赖关系。
- 2026-03-07：完成验证：`gz sim -r .../rover.sdf` + `ros2 launch rover_auto_motion rover_auto_motion.launch.py`，`rover_auto_loop_node` 输出 `pose_cmd` 连续变化，`gz topic -e -t /world/rover/pose/info` 观测到 `rover` 位姿持续更新。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：`rover` 运动卡顿，要求“丝滑一点”。
  解答：已优化 `rover_auto_motion` 的轨迹输出：将位姿更新频率提高到 30Hz，并在“直行/转弯”切换处增加速度与角速度一阶平滑（`cmd_smoothing_tau_sec`，默认 0.18s），减少轨迹跳变和顿挫。

## 修改记录（本轮补充）
### 功能包修改记录
#### rover_auto_motion
- 2026-03-07：更新 `src/rover_auto_motion/rover_auto_motion/rover_auto_loop_node.py`：`publish_rate_hz` 默认从 10Hz 提升到 30Hz；新增参数 `cmd_smoothing_tau_sec`（默认 0.18s）；控制环改为 `v/yaw_rate` 一阶滤波后再积分。
- 2026-03-07：优化 `set_pose` 调用开销：`subprocess.run` 改为 `stdout=DEVNULL`，缩短服务超时到 500ms，降低单次调用阻塞影响。
- 2026-03-07：更新 `src/rover_auto_motion/README.md` 参数说明，补充平滑参数与默认值。
- 2026-03-07：已执行 `colcon build --packages-select rover_auto_motion`、`source install/setup.bash`，并运行 `ros2 launch rover_auto_motion rover_auto_motion.launch.py` 验证：`pose_cmd` 连续变化，转弯段过渡更平滑。
