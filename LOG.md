# 开发日志
PX4_GZ_WORLD=rover PX4_GZ_MODEL_POSE="0,0,2,0,0,0" make px4_sitl gz_x500_mono_cam_down

ros2 launch rover_teleop_ackermann rover_teleop_ackermann.launch.py

ros2 run rover_teleop_ackermann keyboard_arrow_teleop_node

ros2 launch aruco_tracking aruco_tracking.launch.py world_name:=rover

ros2 launch aruco_tracking aruco_tracking.launch.py world_name:=rover model_name:=x500_mono_cam_down_0

## 问题记录

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
