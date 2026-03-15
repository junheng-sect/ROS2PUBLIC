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
- 2026-03-09 | 问题：使用 `imagegen` 技能生成一张图片，解释在 `RETURN` 阶段如何由 GPS 经纬度近似计算东北向误差与返航距离。
  解答：已读取 `imagegen` 技能说明并检查生成脚本，确认该任务应按 `infographic-diagram` 类型生成；当前终端环境缺少 `OPENAI_API_KEY`，暂时无法发起真实图片生成调用，待配置环境变量后可直接生成并输出到工作空间。

## 修改记录（本轮补充）
### 功能包修改记录
#### rover_auto_motion
- 2026-03-07：从 `project_ws` 迁移 `rover_auto_motion` 到 `zjh_ws/src`，保留包结构、launch、节点与配置文件。
- 2026-03-07：按文档规范删除 `src/rover_auto_motion/readme.md`，仅保留 `src/rover_auto_motion/README.md`。
- 2026-03-07：已执行 `colcon build --symlink-install --packages-select rover_auto_motion` 并执行 `source install/setup.bash`，包可被 `ros2 pkg list` 正常识别。
- 2026-03-07 | 问题：给定一段 `aruco_tracking` 日志，跟踪失败原因是什么？
  解答：从日志看是“控制方向与坐标系未完全一致导致目标越追越远 + 最终目标丢失触发视觉超时”。表现为：`err_marker.x` 从约 1m 持续增到 3m、`vy` 长时间打到限幅 `-0.800`，说明控制在持续推向错误方向或横向补偿不足；随后 `tvec_tf_node` 多秒输出完全相同数据，`tracking_node` 判定 `pose_timeout_sec` 超时进入零速悬停。根因可归纳为 `aruco_pose` 语义与 `/cmd_vel` 控制坐标系之间仍存在符号/旋转不一致，叠加速度限幅与视场边界后导致目标离开相机视野。
- 2026-03-07 | 问题：希望无论 ArUco 坐标系 yaw 如何变化，无人机都能正确跟踪 xyz 和 yaw；当前 yaw 正常但 xy 输出固定在 E/N。

#### 文档与图示
- 2026-03-09：按用户要求启用 `imagegen` 技能准备生成“GPS 经纬度到局部东北向距离近似换算”示意图；已完成技能说明、CLI 与环境检查，确认当前受阻于 `OPENAI_API_KEY` 未设置，尚未生成最终图片文件。
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

## 问题记录（本轮补充）
- 2026-03-07 22:25:05 CST | 问题：在这个对话中，我会问你各类问题，你需要把我问的问题和你的解答记录在 `zjh_ws` 工作空间的 `questions` 文件夹 `LOG.md` 文件中，标注上时间。
  解答：已创建 `questions/LOG.md`，从本轮对话开始，后续问题与解答都会按时间追加记录到该文件中。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-07 22:25:05 CST：新增问答专用日志文件 `questions/LOG.md`，用于记录本对话中的问题与解答，并按时间戳追加。
  3) `ros2 launch aruco_tracking aruco_tracking.launch.py world_name:=rover` 复测中 `tracking_node` 已进入 `OFFBOARD跟踪中`，不再出现“本地位姿超时”连续报错。
- 2026-03-07：按规则执行 `colcon build --symlink-install --packages-select aruco_tracking` 与 `source install/setup.bash`，并完成 launch 启动验证后停止。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：我之前的对话记录到哪去了？
  解答：历史记录没有丢失，已统一迁移并保存在工作区根目录 `LOG.md`（路径：`/home/zjh/project/zjh_ws/LOG.md`）；`AGENTS.md` 里仅保留规则与日志入口说明。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-07：按“每轮对话都记录”规则，新增一条问答记录，说明历史对话记录位置为 `LOG.md`，并确认当前日志入口有效。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：`.codex` 文件在哪里？
  解答：当前机器上的 `.codex` 路径为 `/home/zjh/.codex`（用户主目录下的隐藏目录）。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-07：按“每轮对话都记录”规则，补充记录 `.codex` 路径查询结果（`/home/zjh/.codex`）。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：阅读 `AGENTS.md` 和 `LOG.md`，了解项目进度。
  解答：已完成阅读并形成当前进度结论：视觉链路（`tvec`/`tvec_tf`/矩阵链）与 `aruco_tracking` 控制链均已打通，且已定位并修复 `/mavros/local_position/pose` 的 QoS 不匹配问题；当前可在 `ros2 launch aruco_tracking aruco_tracking.launch.py world_name:=rover` 下进入 OFFBOARD 跟踪流程。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-07：按“每轮对话都记录”规则，新增“阅读 AGENTS/LOG 并总结项目进度”的问答记录。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：记住我的各类要求。
  解答：已确认并将持续遵循当前工作区 `AGENTS.md` 约束（含每轮记录、构建/source、launch 验证、中文注释、README 命名与 Git 流程等）。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-07：按“每轮对话都记录”规则，新增“记住并持续遵循用户要求”的问答记录。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：根据输入的代码、实验步骤、现象、实验结果等内容，撰写论文片段初稿，参考指定 PDF 格式，并将 `.md` 输出到 `paper` 文件夹。
  解答：已基于当前工作区中的 `aruco_tracking` 控制代码、实验现象与日志，整理生成 Markdown 论文片段初稿 `paper/aruco_tracking_paper_draft.md`，内容包含摘要、系统架构、控制方法、公式、实验步骤、现象分析与结论，并补充 `paper/README.md` 说明目录用途。

## 修改记录（本轮补充）
### 文档与论文
- 2026-03-07：新增 `paper/aruco_tracking_paper_draft.md`，按本科论文常见结构整理基于 ArUco 视觉跟踪控制的正文初稿，包含可预览公式。
- 2026-03-07：新增 `paper/README.md`，说明 `paper/` 目录的用途与当前文件构成。
- 2026-03-07：按“每轮对话都记录”规则，在“问题记录”中补充本轮“论文片段初稿生成”问答。

## 问题记录（本轮补充）
- 2026-03-07 22:34:30 CST | 问题：`PX4` 中的 `Return` 模式实现原理是什么？
  解答：`Return` 本质上是 PX4 的自动返航状态机（RTL）。触发后，飞控会根据当前高度、返航点位置和返航参数，按“爬升到安全高度、返航到 Home/Rally、下降到指定高度、最终悬停或降落”的流程自动执行；多旋翼主要受 `RTL_RETURN_ALT`、`RTL_DESCEND_ALT` 及最终动作参数控制，不同机型还会叠加各自的进近/着陆逻辑。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-07 22:34:30 CST：按“每轮对话都记录”规则，新增“PX4 Return 模式实现原理”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-07 22:36:27 CST | 问题：返航目标点是怎么获得的？
  解答：PX4 返航目标点默认取自 `Home Position`，该位置通常在定位有效时由飞控自动记录；若系统配置了 `Rally Point` 等替代返航点，RTL 会按配置优先选择更合适的安全点，否则回到 `Home`。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-07 22:36:27 CST：按“每轮对话都记录”规则，新增“返航目标点来源”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-07 22:39:32 CST | 问题：`Home Position` 在 `PX4` 里具体是在哪个模块设置的、什么时候更新、`ROS 2/MAVROS` 怎么读到它？
  解答：`Home Position` 在 PX4 内由 `commander` 模块维护，满足定位与状态条件后自动设置/更新；PX4 通过 MAVLink `HOME_POSITION` 对外发布，MAVROS 会桥接成 `mavros_msgs/msg/HomePosition`，ROS 2 一般从 `/mavros/home_position/home` 订阅读取，主动设置则可用 `/mavros/cmd/set_home` 对应的 `CommandHome` 服务。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-07 22:39:32 CST：按“每轮对话都记录”规则，新增“Home Position 模块、更新时机与 ROS 2/MAVROS 读取方式”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-07 22:44:37 CST | 问题：继续讲 `Home Position` 和 `vehicle_global_position`、`local_position_ned`、`EKF` 原点这几个概念的区别。
  解答：`Home Position` 是返航/安全逻辑使用的参考点；`vehicle_global_position` 是飞行器当前的实时全球位置估计；`local_position_ned` 是飞行器相对局部原点的实时局部位置估计；`EKF` 原点则是该局部坐标系的零点定义。它们用途不同，`Home` 与 `EKF` 原点也不必相同。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-07 22:44:37 CST：按“每轮对话都记录”规则，新增“Home Position、global/local position 与 EKF 原点区别”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-07 22:48:08 CST | 问题：`home` 点刚需 `GPS` 吗？我如果要做一个返航 `home` 点的程序，是否刚需 `GPS`？
  解答：不刚需。对 PX4 原生 `RTL` 来说，核心要求是有效的全局三维位置估计，GPS 只是最常见来源；若用户自己写返航程序，则只要具备可靠位置估计并在同一坐标系下定义了 `home`，即使没有 GPS，也可以实现返航。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-07 22:48:08 CST：按“每轮对话都记录”规则，新增“home 点是否刚需 GPS、返航程序是否必须 GPS”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-07 23:57:02 CST | 问题：`PX4` 的落地检测机制是什么样的？
  解答：PX4 落地检测采用分阶段状态机而非单阈值判断，典型状态包括 `ground_contact`、`maybe_landed`、`landed`；多旋翼主要综合低推力、低竖直速度、低水平运动、低角速度等条件持续满足后，逐级判定为接地、可能落地、最终已落地。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-07 23:57:02 CST：按“每轮对话都记录”规则，新增“PX4 落地检测机制”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-08 00:47:40 CST | 问题：状态机是什么？
  解答：状态机是一种把程序流程划分为多个明确状态，并按条件触发状态切换的控制方法；它强调“当前状态、该状态下的动作、切换条件”三部分。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-08 00:47:40 CST：按“每轮对话都记录”规则，新增“状态机是什么”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-08 10:53:56 CST | 问题：AI 提示词 `prompt` 是什么？
  解答：`prompt` 是用户给 AI 的输入指令，用于说明任务目标、背景、约束和输出形式；提示越明确，生成结果通常越稳定、越贴近需求。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-08 10:53:56 CST：按“每轮对话都记录”规则，新增“AI 提示词 prompt 是什么”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-08 11:01:00 CST | 问题：介绍几个常用的 `skill` 并简单介绍功能。
  解答：当前会话环境中可确认的常用 `skill` 有两个：`skill-creator` 用于创建或更新 skill，便于沉淀可复用工作流；`skill-installer` 用于安装 skill，可从预置列表或仓库导入以扩展能力。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-08 11:01:00 CST：按“每轮对话都记录”规则，新增“常用 skill 及功能简介”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-08 11:03:42 CST | 问题：简单介绍图中 `skill` 的功能。
  解答：已根据截图中可见 skill 做分类说明，涵盖开发部署、设计前端、文档办公、自动化测试、代码协作与安全、平台集成及多模态生成等方向，并简要概括各自用途。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-08 11:03:42 CST：按“每轮对话都记录”规则，新增“截图中 skill 功能简介”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-08 11:06:15 CST | 问题：详细介绍一下 `Linear MCP` 和 `skill`。
  解答：已说明两者层级差异：`Linear MCP` 属于基于 MCP 协议的外部系统接入层，用于让 AI 操作 Linear；`skill` 属于任务方法与工作流封装层，用于指导 AI 如何完成某类任务。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-08 11:06:15 CST：按“每轮对话都记录”规则，新增“Linear MCP 与 skill 详细介绍”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-08 11:09:54 CST | 问题：`Linear` 是干什么的？
  解答：`Linear` 是面向软件研发团队的项目管理与 issue 跟踪平台，用于管理需求、缺陷、任务、迭代和项目进度，可理解为研发场景下的任务系统与协作平台。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-08 11:09:54 CST：按“每轮对话都记录”规则，新增“Linear 是干什么的”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-08 18:20:19 CST | 问题：`Windows` 系统能访问 `Ubuntu` 的文件吗？
  解答：可以，但取决于场景；`WSL` 下可直接通过 `\\\\wsl$\\` 访问，双系统的原生 Linux 分区通常不能被 Windows 原生稳定读写，网络场景则可通过共享、SSH 或 SFTP 访问。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-08 18:20:19 CST：按“每轮对话都记录”规则，新增“Windows 是否能访问 Ubuntu 文件”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-08 18:20:40 CST | 问题：我是双系统。
  解答：在双系统场景下，Windows 通常不能像访问 `NTFS` 一样稳定访问 Ubuntu 常用的 `ext4` 分区；更推荐使用共享数据分区、网络共享或 SFTP/SSH 方式交换文件，而不是让 Windows 直接写 Ubuntu 系统分区。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-08 18:20:40 CST：按“每轮对话都记录”规则，新增“双系统下 Windows 访问 Ubuntu 文件”的补充问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：创建一个依赖 GPS 的返航功能包 `return_home`，流程为“先升到 3m，再飞回 home 点水平位置并悬停”。
  解答：已新建 `return_home`（`ament_python`），实现 `return_home_node`：订阅 `/mavros/global_position/global`、`/mavros/global_position/rel_alt`、`/mavros/state`，采用三阶段状态机 `ASCEND -> RETURN -> HOVER` 控制 `/mavros/setpoint_velocity/cmd_vel`，满足“先升高、后返航、再悬停”的流程要求。

## 修改记录（本轮补充）
### 功能包修改记录
#### return_home
- 2026-03-07：在 `src` 下创建新功能包 `src/return_home`（`ament_python`，Apache-2.0），并补齐维护者与依赖（`rclpy/geometry_msgs/mavros_msgs/sensor_msgs/std_msgs/launch/launch_ros`）。
- 2026-03-07：新增节点 `src/return_home/return_home/return_home_node.py`：
  1) 首次有效 GPS 锁定 home 点；
  2) `ASCEND` 阶段仅控制 `vz` 升至 3m；
  3) `RETURN` 阶段按经纬度误差换算北东米误差并输出 `vx/vy` 返回 home；
  4) `HOVER` 阶段水平置零并维持 3m。
- 2026-03-07：新增启动文件 `src/return_home/launch/return_home.launch.py`，提供默认参数与单节点启动入口。
- 2026-03-07：新增文档 `src/return_home/README.md`（大写），说明返航流程、话题、参数和运行命令。
- 2026-03-07：更新 `src/return_home/setup.py` 与 `src/return_home/package.xml`，补齐入口点 `return_home_node`、launch/README 安装项及描述信息。
- 2026-03-07：按规则完成构建与验证：
  1) `colcon build --symlink-install --packages-select return_home` 通过；
  2) `source install/setup.bash` 已执行；
  3) `ros2 launch return_home return_home.launch.py` 启动验证通过（节点成功拉起，测试后主动停止）。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：进入 OFFBOARD 模式后才开始 `return_home`。
  解答：已完成：`return_home` 现在仅在 OFFBOARD 上升沿启动返航任务，并默认在该时刻锁定当前 GPS 为 home 点；未进入 OFFBOARD 时始终输出零速。

## 修改记录（本轮补充）
### 功能包修改记录
#### return_home
- 2026-03-07：更新 `src/return_home/return_home/return_home_node.py`，新增参数 `start_return_on_offboard_entry` 与 `lock_home_on_offboard_entry`（默认均为 `True`）。
- 2026-03-07：在 `state_callback` 中新增 OFFBOARD 上升沿触发逻辑：上升沿重置阶段为 `ASCEND`、重置 PID，并在条件满足时启动返航任务。
- 2026-03-07：将 home 锁定时机改为“OFFBOARD 上升沿（或 OFFBOARD 后 GPS 就绪时）”，保证返航基准与任务启动时刻一致。
- 2026-03-07：在控制循环中新增“未触发返航启动前保持零速”分支，确保进入 OFFBOARD 前后行为可控。
- 2026-03-07：更新 `src/return_home/launch/return_home.launch.py` 默认参数，开启 OFFBOARD 触发启动与 OFFBOARD 锁定 home。
- 2026-03-07：更新 `src/return_home/README.md`，补充 OFFBOARD 触发机制与相关参数说明。
- 2026-03-07：按规则完成验证：`colcon build --symlink-install --packages-select return_home`、`source install/setup.bash`、`ros2 launch return_home return_home.launch.py` 启动通过后停止。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：返航逻辑修改为“仍然先上升到 3m”，但 home 点改为订阅 `/mavros/home_position/home`。
  解答：已完成：节点在 OFFBOARD 后先执行 `ASCEND` 升至 3m，再读取 `HomePosition.geo` 的经纬度执行 `RETURN`，到达水平容差后进入 `HOVER`。

## 修改记录（本轮补充）
### 功能包修改记录
#### return_home
- 2026-03-07：重构 `src/return_home/return_home/return_home_node.py` 的 home 来源，新增订阅 `/mavros/home_position/home`（`mavros_msgs/msg/HomePosition`），以 `geo.latitude/longitude` 作为返航目标。
- 2026-03-07：保留“OFFBOARD 触发启动 + 先升高到 3m”策略：状态机仍为 `ASCEND -> RETURN -> HOVER`，其中 `ASCEND` 阶段通过 `rel_alt` 调整至 3m。
- 2026-03-07：新增 `try_start_after_data_ready()`，要求 `GPS + rel_alt + home_position` 数据齐全后才启动返航，避免数据缺失导致误控。
- 2026-03-07：更新 `src/return_home/launch/return_home.launch.py`，新增 `home_topic=/mavros/home_position/home` 默认参数，移除旧的 `lock_home_on_offboard_entry` 参数。
- 2026-03-07：更新 `src/return_home/README.md`，同步说明 home 点来源改为 `/mavros/home_position/home`，并保持“先升到 3m 再返航”的流程描述。
- 2026-03-07：按规则完成验证：`colcon build --symlink-install --packages-select return_home`、`source install/setup.bash`、`ros2 launch return_home return_home.launch.py` 启动通过后停止。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：日志显示“已进入 OFFBOARD，等待返航启动条件满足，输出零速”，为什么没有返航？
  解答：该日志表示 `return_home` 已进入 OFFBOARD，但尚未同时收到启动所需数据（`/mavros/global_position/global`、`/mavros/global_position/rel_alt`、`/mavros/home_position/home`）。本次现场检查中这 3 个话题均为 `Unknown topic`，因此节点会持续零速等待，不会进入返航阶段。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-07：按“每轮对话都记录”规则，补充“OFFBOARD 后等待返航启动条件”问题的定位结论与排查记录。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：日志显示“已进入 OFFBOARD，等待返航启动条件满足，输出零速”，为什么不返航？
  解答：根因是 `return_home` 与 MAVROS 话题的 QoS 不兼容，导致关键数据实际未被节点接收：`/mavros/global_position/global` 与 `/mavros/global_position/rel_alt` 发布端为 `BEST_EFFORT`，而节点原订阅为默认 `RELIABLE`；同时 `/mavros/home_position/home` 为 `TRANSIENT_LOCAL`，节点需匹配该 durability 才能稳定拿到 home 消息。已修复后，启动条件可正常满足。

## 修改记录（本轮补充）
### 功能包修改记录
#### return_home
- 2026-03-07：修复 `src/return_home/return_home/return_home_node.py` 订阅 QoS 兼容性：
  1) `/mavros/global_position/global` 订阅改为 `BEST_EFFORT + VOLATILE + KEEP_LAST(10)`；
  2) `/mavros/global_position/rel_alt` 订阅改为 `BEST_EFFORT + VOLATILE + KEEP_LAST(10)`；
  3) `/mavros/home_position/home` 订阅改为 `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)`。
- 2026-03-07：补充中文注释说明 MAVROS 话题 QoS 特征与兼容原因，避免再次出现“有话题但回调收不到数据”的问题。
- 2026-03-07：按规则执行 `colcon build --symlink-install --packages-select return_home`、`source install/setup.bash`，并启动 `return_home.launch.py` 验证。
- 2026-03-07：验证结果：`ros2 topic info -v` 显示 `return_home_node` 已正确挂到 3 个关键话题，且 QoS 与发布端匹配（`global/rel_alt` 为 `BEST_EFFORT`，`home_position` 为 `RELIABLE+TRANSIENT_LOCAL`）。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：功能验证“成功”，要求上传代码。
  解答：已按确认执行 Git 提交并推送流程，将 `return_home` 相关实现与日志更新上传到远程 `origin/simple`。

## 修改记录（本轮补充）
### Git 仓库与远程
- 2026-03-07：在收到“成功”确认后执行提交与推送，上传 `return_home` 功能包及本轮日志更新到 `origin/simple`。

## 问题记录（本轮补充）
- 2026-03-07 | 问题：创建新功能包 `return_track_hover`，将返航与 ArUco 跟踪结合：进入 OFFBOARD 后先升到 3m，再回 home 点，随后进行 ArUco 跟踪，正对 ArUco 后悬停。
  解答：已完成：新增 `return_track_hover` 功能包并实现单节点状态机 `ASCEND -> RETURN -> TRACK -> HOVER`；满足“先升高、后返航、再对正并悬停”的完整流程。

## 修改记录（本轮补充）
### 功能包修改记录
#### return_track_hover
- 2026-03-07：在 `src` 下新建功能包 `src/return_track_hover`（`ament_python`，Apache-2.0），并补齐依赖（`mavros_msgs/debug_interface/geometry_msgs/sensor_msgs/std_msgs/tvec_tf` 等）。
- 2026-03-07：新增节点 `src/return_track_hover/return_track_hover/return_track_hover_node.py`，实现四阶段控制：
  1) `ASCEND`：OFFBOARD 后爬升到 `target_alt_m=3.0m`；
  2) `RETURN`：按 `/mavros/home_position/home` 经纬度进行 GPS 返航；
  3) `TRACK`：基于 `/debug/aruco_pose` 执行 XY+Yaw PID 跟踪，持续满足阈值后判定对正完成；
  4) `HOVER`：输出零速度悬停。
- 2026-03-07：在节点中加入 MAVROS QoS 适配：`global/rel_alt/base_pose` 使用 `BEST_EFFORT`，`home_position` 使用 `RELIABLE + TRANSIENT_LOCAL`，避免订阅不兼容。
- 2026-03-07：新增联合启动文件 `src/return_track_hover/launch/return_track_hover.launch.py`，复用 `tvec_tf.launch.py` 并同时拉起 `return_track_hover_node`。
- 2026-03-07：新增文档 `src/return_track_hover/README.md`（大写），说明任务流程、话题与关键参数。
- 2026-03-07：更新 `src/return_track_hover/setup.py`、`src/return_track_hover/package.xml`，补齐入口点、launch/README 安装项与描述。
- 2026-03-07：按规则执行验证：
  1) `colcon build --symlink-install --packages-select return_track_hover` 通过；
  2) `source install/setup.bash` 已执行；
  3) `ros2 launch return_track_hover return_track_hover.launch.py world_name:=rover` 启动验证通过后停止。

## 问题记录（本轮补充）
- 2026-03-08 | 问题：新建 `landing` 功能包，要求进入 OFFBOARD 后以 0.5m/s 下降；参考 PX4 落地检测机制，检测落地后自动 disarm 并完成降落。
  解答：已完成：新增 `landing` 包并实现 `landing_node`，OFFBOARD 后持续下发 `vz=-0.5m/s`，采用“`extended_state` 落地状态优先 + 低高度/低速度/持续时间启发式”双通道落地判定，判定落地后自动调用 `/mavros/cmd/arming` 发送 `disarm`，`armed=false` 后进入完成态并保持零速。

## 修改记录（本轮补充）
### 功能包修改记录
#### landing
- 2026-03-08：新建功能包 `src/landing`（`ament_python`，Apache-2.0），补齐依赖（`rclpy/geometry_msgs/mavros_msgs/std_msgs/launch/launch_ros`）。
- 2026-03-08：新增节点 `src/landing/landing/landing_node.py`，实现自动降落状态机：
  1) OFFBOARD 后开始固定下降 `vz=-0.5m/s`；
  2) 按 `ExtendedState.landed_state==ON_GROUND` 直接判定落地；
  3) 同时提供启发式判定（`rel_alt`、`vxy`、`vz` 阈值 + 持续时间）作为兜底；
  4) 落地后调用 `CommandBool(false)` 自动 disarm，失败按间隔重试；
  5) 检测 `armed=false` 后判定“降落完成”。
- 2026-03-08：为 MAVROS 订阅补齐 QoS 适配（`BEST_EFFORT + VOLATILE`），降低话题兼容风险。
- 2026-03-08：新增启动文件 `src/landing/launch/landing.launch.py`，提供默认参数（下降速度、落地阈值、重试间隔等）。
- 2026-03-08：新增文档 `src/landing/README.md`（大写），说明流程、话题、服务与参数。
- 2026-03-08：更新 `src/landing/setup.py` 与 `src/landing/package.xml`，补齐入口点 `landing_node`、launch/README 安装项与描述。
- 2026-03-08：按规则完成验证：
  1) `colcon build --symlink-install --packages-select landing` 通过；
  2) `source install/setup.bash` 已执行；
  3) `ros2 launch landing landing.launch.py` 启动验证通过后停止。

## 问题记录（本轮补充）
- 2026-03-08 | 问题：`landing_node` 日志出现 `Disarming denied: not landed`，虽提示“已判定落地”但 disarm 被拒。
  解答：根因是原逻辑用启发式（低高低速）提前判定落地并立即 disarm，早于飞控 `landed_state` 真正确认。已修复为“仅在 `extended_state.landed_state==ON_GROUND` 时发送 disarm”；启发式仅用于“接地准备态”降速等待，避免提前 disarm 被拒。

## 修改记录（本轮补充）
### 功能包修改记录
#### landing
- 2026-03-08：更新 `src/landing/landing/landing_node.py` 的降落末段逻辑：
  1) disarm 触发条件改为飞控明确 `ON_GROUND`；
  2) 启发式判定改为 `touchdown_ready`，仅触发小速度下压，不直接 disarm；
  3) 新增参数 `touchdown_descent_speed_mps`（默认 `0.15`）用于接地准备阶段。
- 2026-03-08：更新 `src/landing/launch/landing.launch.py`，加入 `touchdown_descent_speed_mps` 默认参数。
- 2026-03-08：修复一次文件尾部入口函数缺失问题（补回 `log_callback/main`），并完成回归构建与启动验证。
- 2026-03-08：按规则执行 `colcon build --symlink-install --packages-select landing`、`source install/setup.bash`、`ros2 launch landing landing.launch.py` 启动验证通过后停止。

## 问题记录（本轮补充）
- 2026-03-08 | 问题：`landing_node` 仍卡在“接地准备中…等待 landed_state=ON_GROUND”。
  解答：已新增“启发式 disarm 兜底”路径：在 `landed_state` 长时间不置位时，若低高度/低速度启发式连续满足达到设定时长，则允许触发 disarm 请求，不再无限等待 `ON_GROUND`。同时仍保留 `ON_GROUND` 优先路径。

## 修改记录（本轮补充）
### 功能包修改记录
#### landing
- 2026-03-08：更新 `src/landing/landing/landing_node.py`，新增参数 `allow_heuristic_disarm_fallback`（默认 `True`）与 `heuristic_disarm_hold_sec`（默认 `3.0`）。
- 2026-03-08：新增 `is_heuristic_landed_confirmed()`，当启发式落地条件持续满足足够时长时，允许进入 disarm 兜底流程。
- 2026-03-08：控制循环增加兜底分支：若 `ON_GROUND` 未置位但启发式落地持续满足，则执行 `disarm` 重试，避免流程卡死在“接地准备中”。
- 2026-03-08：更新 `src/landing/launch/landing.launch.py`，增加 `allow_heuristic_disarm_fallback`、`heuristic_disarm_hold_sec` 默认参数。
- 2026-03-08：更新 `src/landing/README.md`，补充上述两个新参数说明。
- 2026-03-08：按规则执行 `colcon build --symlink-install --packages-select landing`、`source install/setup.bash`、`ros2 launch landing landing.launch.py` 启动验证通过后停止。

## 问题记录（本轮补充）
- 2026-03-08 | 问题：落地流程仍不稳定，要求在“启发式已持续满足落地条件”后先将油门拉至最低，再申请 disarm，以帮助飞控检测落地。
  解答：已实现该策略：启发式落地成立后，先进入“最低油门下压保持”阶段（默认 `vz=-0.35m/s` 持续 `1.0s`），随后再发起 disarm 请求；请求期间保持最低油门下压，不再直接零速 disarm。

## 修改记录（本轮补充）
### 功能包修改记录
#### landing
- 2026-03-08：更新 `src/landing/landing/landing_node.py`：在启发式兜底分支新增“最低油门阶段”，流程变为：
  1) 启发式落地持续满足；
  2) 以 `min_throttle_descent_speed_mps` 下压并保持 `min_throttle_hold_before_disarm_sec`；
  3) 再触发 disarm 兜底请求。
- 2026-03-08：新增参数 `min_throttle_descent_speed_mps`（默认 `0.35`）与 `min_throttle_hold_before_disarm_sec`（默认 `1.0`）。
- 2026-03-08：更新 `src/landing/launch/landing.launch.py`，加入上述参数默认值。
- 2026-03-08：更新 `src/landing/README.md`，补充最低油门阶段参数说明。
- 2026-03-08：按规则执行 `colcon build --symlink-install --packages-select landing`、`source install/setup.bash`、`ros2 launch landing landing.launch.py` 启动验证通过后停止。

## 问题记录（本轮补充）
- 2026-03-08 | 问题：要求将兜底流程改为“先进入最低油门下压阶段（默认 `vz=-0.35m/s`），下压持续 `5.0s`，期间每隔 1 秒发送 disarm；已 disarm 后不再发送”。
  解答：已按要求实现：启发式兜底触发后进入最低油门阶段，持续 `min_throttle_disarm_duration_sec=5.0s` 并按 `disarm_retry_interval_sec=1.0s` 节流发送 disarm；一旦检测 `armed=false` 即进入完成态并停止发送。

## 修改记录（本轮补充）
### 功能包修改记录
#### landing
- 2026-03-08：更新 `src/landing/landing/landing_node.py` 的兜底分支：在最低油门阶段内周期发送 disarm，默认阶段时长 `5.0s`。
- 2026-03-08：参数调整：`min_throttle_hold_before_disarm_sec` 替换为 `min_throttle_disarm_duration_sec`（默认 `5.0`）。
- 2026-03-08：保持 `disarm_retry_interval_sec=1.0` 用于“每隔一秒发送 disarm”的节流控制；`armed=false` 后由完成态分支自动停止发送。
- 2026-03-08：更新 `src/landing/launch/landing.launch.py` 默认参数与 `src/landing/README.md` 说明。
- 2026-03-08：按规则执行 `colcon build --symlink-install --packages-select landing`、`source install/setup.bash`、`ros2 launch landing landing.launch.py` 启动验证通过后停止。

## 问题记录（本轮补充）
- 2026-03-08 | 问题：创建新功能包 `total`，融合 `return_track_hover` 和 `landing`：先返回 home，再跟踪并正对 ArUco，正对成功后悬停 1s，最后降落直至 disarm。
  解答：已完成：新增 `total` 包并实现单节点状态机，按 `ASCEND -> RETURN -> TRACK -> HOVER_BEFORE_LAND -> LAND -> TOUCHDOWN_DISARM -> DONE` 执行全任务，满足“返航+对正+悬停+降落解锁”闭环流程。

## 修改记录（本轮补充）
### 功能包修改记录
#### total
- 2026-03-08：新建功能包 `src/total`（`ament_python`，Apache-2.0），并补齐依赖（`mavros_msgs/debug_interface/geometry_msgs/sensor_msgs/std_msgs/tvec_tf` 等）。
- 2026-03-08：新增节点 `src/total/total/total_node.py`，实现总任务状态机：
  1) `ASCEND`：OFFBOARD 后升至 3m；
  2) `RETURN`：GPS 返航到 home；
  3) `TRACK`：ArUco 跟踪并正对；
  4) `HOVER_BEFORE_LAND`：对正成功后悬停 1s；
  5) `LAND`：按 `0.5m/s` 下降；
  6) `TOUCHDOWN_DISARM`：最低油门下压并每秒申请 disarm；
  7) `DONE`：检测 `armed=false` 后结束。
- 2026-03-08：在 `total_node` 中集成 MAVROS QoS 兼容（`BEST_EFFORT` 与 `home TRANSIENT_LOCAL`），并集成 disarm 服务节流重试逻辑。
- 2026-03-08：新增联合启动文件 `src/total/launch/total.launch.py`，复用 `tvec_tf.launch.py` 并启动 `total_node`。
- 2026-03-08：新增文档 `src/total/README.md`（大写），说明完整流程、关键话题与阶段定义。
- 2026-03-08：更新 `src/total/setup.py`、`src/total/package.xml`，补齐入口点、launch/README 安装项与描述。
- 2026-03-08：按规则完成验证：
  1) `colcon build --symlink-install --packages-select total` 通过；
  2) `source install/setup.bash` 已执行；
  3) `ros2 launch total total.launch.py world_name:=rover` 启动验证通过后停止。

## 问题记录（本轮补充）
- 2026-03-08 | 问题：功能验证成功，要求上传代码。
  解答：已按确认执行 Git 提交并推送，将 `landing`、`return_track_hover`、`total` 及日志更新上传到远程 `origin/simple`。

## 修改记录（本轮补充）
### Git 仓库与远程
- 2026-03-08：在收到“成功”确认后执行提交与推送，上传本轮新增功能包与日志更新到 `origin/simple`。

## 问题记录（本轮补充）
- 2026-03-08 | 问题：根据 `total_node.py` 中完整的自动降落流程，写一份论文片段初稿。
  解答：已读取 `src/total/total/total_node.py` 与 `src/total/launch/total.launch.py`，基于其中的多阶段自动降落状态机、GPS 返航、ArUco 视觉对准、悬停确认、降落与 disarm 逻辑，生成论文片段初稿 `paper/total_auto_landing_paper_draft.md`。

## 修改记录（本轮补充）
### 文档与论文
- 2026-03-08：新增 `paper/total_auto_landing_paper_draft.md`，围绕 `total_node.py` 中的自动降落完整流程撰写论文片段初稿，包含状态机、控制律、落地判据与安全机制分析。
- 2026-03-08：按“每轮对话都记录”规则，在“问题记录”中补充本轮“total 自动降落论文片段初稿生成”问答。

## 问题记录（本轮补充）
- 2026-03-09 11:16:34 CST | 问题：实机测试时 `rqt` 中的 USB 相机画面延迟约 `0.5s`，如果改成在地面站笔记本计算，并用图传模块回传图像信息，会不会有更好的性能？给一个合理方案。
  解答：不建议把主视觉闭环迁到地面站。`rqt` 画面延迟通常主要来自显示链路，不等于算法链路同样慢；若改成图传到地面站再算再回传控制，端到端时延和抖动往往更差。更合理的方案是机载做视觉与控制闭环，地面站仅做监控、录包和非实时分析。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-09 11:16:34 CST：按“每轮对话都记录”规则，新增“实机图像延迟与地面站计算方案评估”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-09 13:13:45 CST | 问题：`/mavros/setpoint_attitude/cmd_vel` 和 `/mavros/setpoint_velocity/cmd_vel` 这两个话题有什么不同？
  解答：两者控制层级不同；`setpoint_velocity/cmd_vel` 用于给飞控发送速度目标，属于较高层控制入口，飞控内部仍会完成姿态与执行器分配；`setpoint_attitude/cmd_vel` 更接近姿态/角速度层输入，控制更低层，通常用于更直接地干预姿态内环。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-09 13:13:45 CST：按“每轮对话都记录”规则，新增“MAVROS 姿态速度话题与线速度话题区别”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-09 13:19:42 CST | 问题：`setpoint_velocity/cmd_vel` 的输入量有哪些？是在哪个坐标系下的？
  解答：该话题常用 `TwistStamped` 输入，核心量是 `linear.x/y/z` 与 `angular.z`；默认坐标系由 `mav_frame` 参数决定，在 MAVROS PX4 默认配置中通常为 `LOCAL_NED`，也可切到机体系如 `BODY_NED`。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-09 13:19:42 CST：按“每轮对话都记录”规则，新增“setpoint_velocity/cmd_vel 输入量与坐标系”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-09 16:28:52 CST | 问题：GPS 的数据时间长了会漂吗？尤其是高度方向。
  解答：会，且高度方向通常比水平更容易漂。普通 GPS 在静止状态下也会因卫星几何、噪声、多路径和大气误差产生位置抖动与慢漂，因此单靠 GPS 高度不适合做高精度定高或精确降落，通常需要气压计、测距、视觉或 RTK 等辅助。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-09 16:28:52 CST：按“每轮对话都记录”规则，新增“GPS 长时间漂移尤其是高度方向”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-09 16:28:52 CST | 问题：测试时让 GPS 和飞控一直通电，只在树莓派上反复启动/关闭程序；通电时间长后，原定高度 `3m`，进入 `OFFBOARD` 后无人机一直往上飞，重启 GPS 后恢复正常。这是不是 GPS 漂了？长时间运行有没有解决方案？
  解答：更可能是 PX4 高度参考漂移，而不是 GPS 单独漂移。当前 `total_node.py` 用固定 `3m` 去跟踪 `/mavros/global_position/rel_alt`，该量属于融合后的相对高度；长时间通电后只要估计器高度参考发生偏移，进入 `OFFBOARD` 就会误判“高度不足”而继续爬升。更稳妥的办法是在每次任务启动时重置高度参考，或改用测距/视觉/RTK 等更稳定的低空高度源。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-09 16:28:52 CST：按“每轮对话都记录”规则，新增“长时间通电后进入 OFFBOARD 持续上升的原因与解决方案”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-09 | 问题：将 `total_node.py` 改成“进入 OFFBOARD 时锁定当前高度”。
  解答：已完成：在 `OFFBOARD` 上升沿将当前 `rel_alt` 记录为高度参考 `alt_ref_m`，后续高度控制统一使用该锁定高度（若上升沿未拿到高度，则在 OFFBOARD 下首次收到 `rel_alt` 时补锁）。

## 修改记录（本轮补充）
### 功能包修改记录
#### total
- 2026-03-09：更新 `src/total/total/total_node.py`，新增参数 `lock_alt_on_offboard_entry`（默认 `True`）。
- 2026-03-09：新增状态变量 `alt_ref_m`，在 `OFFBOARD` 上升沿锁定当前 `rel_alt` 作为全流程高度参考。
- 2026-03-09：在 `rel_alt_callback` 增加“补锁”逻辑：若上升沿时无高度数据，待 OFFBOARD 下首次收到 `rel_alt` 后再锁定。
- 2026-03-09：高度控制由固定 `target_alt_m` 改为“优先使用 `alt_ref_m`，否则回退 `target_alt_m`”。
- 2026-03-09：更新 `src/total/launch/total.launch.py`，新增默认参数 `lock_alt_on_offboard_entry=True`。
- 2026-03-09：更新 `src/total/README.md`，补充 OFFBOARD 锁高机制说明。
- 2026-03-09：按规则完成验证：`colcon build --symlink-install --packages-select total`、`source install/setup.bash`、`ros2 launch total total.launch.py world_name:=rover` 启动验证通过后停止。

## 问题记录（本轮补充）
- 2026-03-09 23:44:51 CST | 问题：提高二维码降落的精度，要让无人机精确落在二维码上，有哪些改进的方法？缩小控制死区？在下落过程中保持 `xy yaw` 跟踪？
  解答：单独缩小死区不够，更有效的是分阶段精降策略：下降过程中持续保持 `xy+yaw` 闭环，随着高度下降逐步收紧误差阈值、降低速度上限和减小死区；同时改进低空高度感知、位姿滤波与触地前稳定判定，才能显著提高落点精度。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-09 23:44:51 CST：按“每轮对话都记录”规则，新增“提高二维码降落精度的方法”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-10 00:02:01 CST | 问题：帮我总结 `total` 中实现完整降落流程时，各个阶段使用的功能包有哪些？
  解答：已按阶段梳理完整降落链路的功能包依赖：`ASCEND/RETURN` 主要用 `total + mavros`；视觉对准与降落阶段在此基础上叠加 `tvec_tf`、`tvec`、`tf_broadcast` 和 `debug_interface`。整体 launch 链路由 `total.launch.py -> tvec_tf.launch.py -> tvec/tf_broadcast/tvec_tf` 构成。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-10 00:02:01 CST：按“每轮对话都记录”规则，新增“total 完整降落流程各阶段功能包总结”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-10 00:33:07 CST | 问题：后续想把无人机在降落过程中的地面效应考虑进来，用于提升落地平稳程度。计划构造一个 `z` 方向加速度与高度的函数输入给飞控，以抵消地面效应并让落地瞬间速度为 `0`，请给一些建议。
  解答：已建议采用“高度相关的下降轨迹 + 地面效应前馈补偿 + 低空闭环修正”的结构，而不是直接把单一 `a_z(h)` 函数裸输入飞控；对当前 MAVROS/PX4 上层控制链路，更实用的做法是分高度调度 `vz`、限制加速度/jerk，并结合测距或视觉高度在低空做更稳的垂向控制。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-10 00:33:07 CST：按“每轮对话都记录”规则，新增“地面效应补偿降落策略设计建议”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-10 00:45:53 CST | 问题：如果后续通过多次实验获得多条 `a_z-h` 曲线，并通过机器学习获得最佳曲线，这样的路线有没有可行性？
  解答：有可行性，但更适合做成“数据驱动补偿模型”而不是完全黑盒控制器。建议用实验数据辨识一个轻量前馈模型，并保留原有闭环、限幅和安全回退；若只用单一 `a_z-h` 曲线，泛化能力可能不足，最好同时考虑 `v_z`、推力/油门、姿态等特征。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-10 00:45:53 CST：按“每轮对话都记录”规则，新增“基于实验和机器学习获得地面效应最佳曲线的可行性”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-10 00:58:40 CST | 问题：如果还会加入 `CFD` 仿真，通过简化模型（四个螺旋桨模仿机身上的相对位置，在不同高度下的升力系数），应该如何融入？
  解答：建议把 `CFD` 作为先验建模工具融入，而不是直接替代实机数据。更合理的流程是：用简化四桨 CFD 提取不同高度下的升力增益或推力修正趋势，再用实机实验进行校正，最终将 `CFD + 实验` 融合为前馈补偿或低空下降调度模型。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-10 00:58:40 CST：按“每轮对话都记录”规则，新增“CFD 如何融入地面效应补偿路线”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-10 12:57:17 CST | 问题：把整个路线写成一份详细的计划书，以 `md` 文档的形式放在 `questions` 目录下。
  解答：已新建计划书 `questions/ground_effect_landing_plan.md`，系统整理了地面效应补偿降落的研究背景、目标、技术路线、CFD 先验建模、实机实验辨识、数据驱动补偿模型、控制器融合、对比实验、风险控制与论文写作建议。

## 修改记录（本轮补充）
### 文档与论文
- 2026-03-10 12:57:17 CST：新增 `questions/ground_effect_landing_plan.md`，形成“地面效应补偿降落研究计划书”Markdown 文档。
- 2026-03-10 12:57:17 CST：按“每轮对话都记录”规则，补充本轮“计划书输出到 questions 目录”的问答记录。

## 问题记录（本轮补充）
- 2026-03-11 19:05:53 CST | 问题：现在 `CFD/实验` 计划分三步：1) 做单桨不同转速推力测试，已得到幂函数拟合 `n=2.03`；2) 做单桨离地不同高度推力测试；3) 做四旋翼离地不同高度推力测试。这样的计划有没有问题，还有什么改进空间？
  解答：总体顺序合理，但若仅做静态推力-高度测试，模型会缺少真实降落中的下降速度与非定常信息。建议在现有三步基础上补充“不同 `v_z` 条件下的动态下降测试/仿真”和“实机校正验证”，并尽量使用 `h/R`、`C_T`、推力增益比等无量纲指标统一分析单桨与四旋翼结果。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-11 19:05:53 CST：按“每轮对话都记录”规则，新增“CFD 三步实验计划评审与改进建议”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-11 19:11:38 CST | 问题：如何在仿真中实现螺旋桨下降？我使用的是 `STAR-CCM+`。
  解答：已说明两种实现方式：一是以不同离地高度建立多个离散工况的准静态方法，适合当前地面效应曲线研究；二是采用非定常仿真加动态网格/运动区域实现连续下降，适合后续研究下降速度影响，但计算成本更高。对当前阶段建议优先使用多高度、多速度工况的离散方案。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-11 19:11:38 CST：按“每轮对话都记录”规则，新增“STAR-CCM+ 中实现螺旋桨下降仿真”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-11 19:20:27 CST | 问题：对于对准 ArUco 码降落的过程，可以有哪些评判标准，什么样的降落过程比较优秀？例如降落精度、降落时间、落地是否反弹等。给出一个参数化的评判标准。
  解答：已给出面向 ArUco 精降的参数化评价体系，覆盖精度、过程质量、平稳性和可靠性四类指标，并建议用加权总分方法统一比较不同控制策略或参数组合。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-11 19:20:27 CST：按“每轮对话都记录”规则，新增“ArUco 精降过程参数化评价标准”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-11 19:37:45 CST | 问题：`h_rebound` 这个参数的测量是一个难点，有什么方法可以测量？
  解答：已建议将 `h_rebound` 定义为“首次触地后再次离地的最大高度”，并推荐使用高速相机、动作捕捉系统或高频激光测距作为主要测量手段；在条件受限时，可结合机载测距/视觉高度和触地事件检测做近似估算。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-11 19:37:45 CST：按“每轮对话都记录”规则，新增“h_rebound 参数测量方法”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-11 19:44:00 CST | 问题：用 `e_land`、`e_yaw_td`、`v_z_td`、`h_rebound`、`T_total`、`P_success` 这六个参数，在 `questions` 目录写一个文档，写明各个指标的详细参数标准和最后的加权总分计算公式。
  解答：已新建 `questions/aruco_landing_metrics.md`，系统整理了 6 个核心指标的定义、分档标准、归一化评分方式和综合加权总分公式，可直接用于精降实验评估与论文撰写。

## 修改记录（本轮补充）
### 文档与论文
- 2026-03-11 19:44:00 CST：新增 `questions/aruco_landing_metrics.md`，用于统一评估 ArUco 精确降落的核心指标与综合评分。
- 2026-03-11 19:44:00 CST：按“每轮对话都记录”规则，补充本轮“六项指标评价文档输出”的问答记录。

## 问题记录（本轮补充）
- 2026-03-11 19:46:46 CST | 问题：反弹高度不太好测量，换成触地后姿态扰动 `theta_post`。这个应该可以通过飞行日志获得。
  解答：已将 `aruco_landing_metrics.md` 中的 `h_rebound` 替换为 `theta_post`，并改为基于飞行日志姿态数据计算首次触地后短时间窗口内的最大姿态扰动。

## 修改记录（本轮补充）
### 文档与论文
- 2026-03-11 19:46:46 CST：更新 `questions/aruco_landing_metrics.md`，将评价指标 `h_rebound` 替换为 `theta_post`，并同步修改定义、分档标准、归一化评分与加权总分公式。

## 问题记录（本轮补充）
- 2026-03-12 00:20:20 CST | 问题：修改 ArUco 精确降落评价标准，增加“基线组 vs 补偿组”的对比规则。
  解答：已为 `aruco_landing_metrics.md` 增补“基线组与补偿组对比规则”章节，补充了对比原则、对比方式、核心优势指标、优势判定标准和推荐结论模板。

## 修改记录（本轮补充）
### 文档与论文
- 2026-03-12 00:20:20 CST：更新 `questions/aruco_landing_metrics.md`，新增“基线组与补偿组对比规则”章节。

## 问题记录（本轮补充）
- 2026-03-12 00:23:28 CST | 问题：设计不考虑地面效应补偿时的 z 方向降落策略。
  解答：已给出基线组 A 的 z 向降落策略：不引入地面效应补偿，仅采用常规分段 `v_z_ref` 调度与原有闭环控制，作为后续补偿策略的公平对照基线。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-12 00:23:28 CST：按“每轮对话都记录”规则，新增“不考虑地面效应补偿时的 z 方向降落策略设计”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-12 00:26:16 CST | 问题：速度从 `0.5` 到 `0.25` 再到 `0.1`，最后怎么变成 `0`？
  解答：已说明基线策略不能只停留在三段固定下降速度，最后还需要极低空收敛段；建议在最底部高度区间采用 `v_z_ref = -k_h \cdot h` 一类渐近收敛形式，使 `h -> 0` 时 `v_z_ref -> 0`。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-12 00:26:16 CST：按“每轮对话都记录”规则，新增“基线组 z 向速度如何最终收敛到 0”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-12 00:31:23 CST | 问题：把基线组 A 的降落策略写成 `questions` 目录下的一个文档。
  解答：已新增 `questions/baseline_z_landing_strategy.md`，完整说明了不考虑地面效应补偿时的基线组 A z 向降落策略。

## 修改记录（本轮补充）
### 文档与论文
- 2026-03-12 00:31:23 CST：新增 `questions/baseline_z_landing_strategy.md`，用于描述基线组 A 的 z 向降落控制策略。

## 问题记录（本轮补充）
- 2026-03-12 13:39:28 CST | 问题：分析文件 `单桨不同高度.xlsx` 中的 CFD 仿真数据。
  解答：已完成数据读取与拟合分析。结果显示该文件当前是“转速-推力”数据表，拟合关系为 `T = 3.2241e-07 * rpm^2.0366`，`R^2 = 0.999998`，可用于建立单桨基础推力模型，但暂不能用于不同高度地面效应分析。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-12 13:39:28 CST：按“每轮对话都记录”规则，新增“单桨 CFD Excel 数据分析”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-12 13:43:33 CST | 问题：更正后重新分析文件 `单桨不同高度.xlsx`，当前表为固定 `4000 rpm` 下的单桨不同离地高度数据，分析地面效应。
  解答：已完成地面效应分析。结果表明：相对无穷远基准推力 `6.9836 N`，推力增益随高度下降逐渐增大，在 `h=0.3` 时约 `+3.22%`，`h=0.25` 时约 `+4.24%`，`h=0.2` 时约 `+5.85%`，说明单桨地面效应在 `h<=0.3` 区间明显增强。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-12 13:43:33 CST：按“每轮对话都记录”规则，新增“单桨不同高度 CFD 数据的地面效应分析”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-12 14:04:47 CST | 问题：现在已经有实机可以实验，是否可以简化 `CFD` 仿真，重点进行实机实验，用来构建 `T=f(n,h,v_z)` 的拟合关系？
  解答：可以将重点转向实机，但不建议完全放弃 `CFD`。更合理的是保留轻量 `CFD` 或台架试验作为物理先验，再用实机数据做校正；同时建议实机拟合的目标优先设为“等效补偿模型”而不是直接从飞行日志恢复绝对推力真值。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-12 14:04:47 CST：按“每轮对话都记录”规则，新增“是否可简化 CFD 并转向实机拟合 `T=f(n,h,v_z)`”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-12 14:08:23 CST | 问题：上述做的两个表格的 `CFD` 计算够了吗？
  解答：若目标是提供基础物理依据并支撑后续实机研究，这两个表格已基本够用；若目标是直接构建完整的 `T=f(n,h,v_z)` 模型或直接支撑控制器设计，则还不够，仍需补充不同转速、不同下降速度及多旋翼干扰工况。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-12 14:08:23 CST：按“每轮对话都记录”规则，新增“当前两组 CFD 表格是否足够”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-12 14:17:57 CST | 问题：设计实机实验。初步想法是：1) 做近地不同高度的悬停实验，获得各个电机的平均转速获得 `T=f(h)`；2) 做不同速度匀速降落的实验，取同一时刻的高度和电机转速，获得 `T=f(h,v_z)`；3) 给无人机增加配重，重复 1、2 实验，获得 `T=f(n,h,v_z)`。这样的实验设计有没有问题？给出改进建议和方案。
  解答：总体路线可行，但需避免将自由飞行中的平均电机转速直接视为绝对推力真值。更合理的做法是将实验分成静态近地悬停、准稳态匀速下降和不同载荷迁移验证三层，并把最终拟合目标定义为“等效地面效应增益/补偿模型”，同时补充姿态、电压和环境条件等记录字段。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-12 14:17:57 CST：按“每轮对话都记录”规则，新增“地面效应实机实验设计评审与改进建议”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-12 14:27:23 CST | 问题：把修改后的实机实验设计写一个文档。
  解答：已新增 `questions/real_flight_ground_effect_experiment_plan.md`，用于系统说明地面效应补偿的实机实验设计方案。

## 修改记录（本轮补充）
### 文档与论文
- 2026-03-12 14:27:23 CST：新增 `questions/real_flight_ground_effect_experiment_plan.md`，整理修正后的实机实验设计方案。

## 问题记录（本轮补充）
- 2026-03-12 14:41:34 CST | 问题：控制量 `u` 应该怎么测得？
  解答：已说明 `u` 应具体定义为可获取的等效控制量，而非抽象量。优先推荐使用飞控日志中的总油门或归一化推力指令；若无法直接获取，则可用四电机转速构造与总升力单调对应的等效输入，如平方和均值。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-12 14:41:34 CST：按“每轮对话都记录”规则，新增“控制量 u 如何测得”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-12 14:47:17 CST | 问题：从当前 `mavros` 话题列表看，有总推力指令吗？
  解答：当前话题列表中没有一个明确可直接当作“PX4 内部总推力指令输出”的标准话题。`/mavros/setpoint_attitude/thrust` 属于外部输入设定，`/mavros/actuator_control` 更偏底层执行器控制组，若要定义实验中的等效控制量，仍更建议基于 ESC 转速构造总输入代理量。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-12 14:47:17 CST：按“每轮对话都记录”规则，新增“MAVROS 话题中是否存在总推力指令”的问答记录，并同步写入 `questions/LOG.md`。
- 2026-03-12 16:34:32 CST：按“每轮对话都记录”规则，新增“ArUco 对准时，动态 yaw 旋转到 ENU 与直接机体系速度输出的效果比较”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-11 20:03:21 CST | 问题：帮我设计不考虑地面效应和考虑地面效应的两种落地策略，通过加入地面效应补偿控制项，能够有指标能够体现加入地面效应补偿项的优势。
  解答：已设计基线策略 A（无地面效应补偿）和补偿策略 B（低空加入地面效应前馈/调度补偿）两套降落方案，并指出 `v_z_td`、`theta_post`、`P_success` 是最能体现补偿优势的核心指标，`e_land` 与 `T_total` 可作为补充对比项。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-11 20:03:21 CST：按“每轮对话都记录”规则，新增“无补偿与地面效应补偿两种降落策略设计”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-11 23:51:55 CST | 问题：如果策略 A 和策略 B 的目标都是以 `v_z=0` 着陆，那如何展现引入地面效应补偿项的优势？
  解答：已说明优势不在于“终点目标不同”，而在于“实现该目标的过程质量不同”。即使两者都追求 `v_z=0`，仍可通过实际触地速度均值与方差、低空速度波动、姿态扰动、控制动作幅度和成功率来体现地面效应补偿的价值。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-11 23:51:55 CST：按“每轮对话都记录”规则，新增“同为 `v_z=0` 目标时如何体现地面效应补偿优势”的问答记录，并同步写入 `questions/LOG.md`。

## 问题记录（本轮补充）
- 2026-03-10 | 问题：创建新功能包 `land_with_tracking`，流程为“先对齐 ArUco，再以 0.5m/s 下落且持续 xy+yaw 闭环，最后执行 landing 的下压与 disarm 逻辑”。
  解答：已完成：新增 `land_with_tracking` 包并实现单节点状态机 `ALIGN -> HOVER_BEFORE_LAND -> DESCEND_WITH_TRACK -> TOUCHDOWN_DISARM -> DONE`，满足“对齐-悬停-闭环下降-下压解锁”全流程。

## 修改记录（本轮补充）
### 功能包修改记录
#### land_with_tracking
- 2026-03-10：新建功能包 `src/land_with_tracking`（`ament_python`，Apache-2.0），补齐依赖（`mavros_msgs/debug_interface/geometry_msgs/std_msgs/tvec_tf` 等）。
- 2026-03-10：新增主节点 `src/land_with_tracking/land_with_tracking/land_with_tracking_node.py`，实现任务状态机：
  1) `ALIGN`：先对齐 ArUco（严格阈值 `xy<=0.10m`、`yaw<=5deg`、持续 `1.0s`）；
  2) `HOVER_BEFORE_LAND`：对齐成功后悬停 `1.0s`；
  3) `DESCEND_WITH_TRACK`：以 `vz=-0.5m/s` 下降并持续执行 `xy+yaw` 闭环；
  4) `TOUCHDOWN_DISARM`：最低油门下压并按周期发送 disarm；
  5) `DONE`：`armed=false` 后任务结束。
- 2026-03-10：在节点中复用动态 yaw 误差旋转与 MAVROS QoS 适配（`BEST_EFFORT + VOLATILE`），并集成 disarm 节流重试逻辑。
- 2026-03-10：新增启动文件 `src/land_with_tracking/launch/land_with_tracking.launch.py`，默认包含 `tvec_tf.launch.py`，可一键拉起视觉链路与控制节点。
- 2026-03-10：新增 `src/land_with_tracking/README.md`（大写），说明阶段流程、默认判据、话题与运行命令。
- 2026-03-10：更新 `src/land_with_tracking/setup.py`、`src/land_with_tracking/package.xml`，补齐入口点、launch/README 安装项与描述。
- 2026-03-10：按规则执行验证：
  1) `colcon build --symlink-install --packages-select land_with_tracking` 通过；
  2) `source install/setup.bash` 已执行；
  3) `ros2 launch land_with_tracking land_with_tracking.launch.py world_name:=rover` 启动验证通过后停止。

## 问题记录（本轮补充）
- 2026-03-12 | 问题：现有链路通过动态 yaw 变换后输出 ENU 速度指令，要求新建一个独立功能包，直接在机体坐标系输出速度指令，且不改动原有功能包。
  解答：已新增独立功能包 `body_frame_tracking`，直接发布 `FRAME_BODY_NED` 速度指令到 `/mavros/setpoint_raw/local`，不修改原有任何功能包，原链路可继续独立使用。

## 修改记录（本轮补充）
### 功能包修改记录
#### body_frame_tracking
- 2026-03-12：新建功能包 `src/body_frame_tracking`（`ament_python`，Apache-2.0），补齐依赖（`rclpy/mavros_msgs/debug_interface/launch/launch_ros/tvec_tf`）。
- 2026-03-12：新增节点 `src/body_frame_tracking/body_frame_tracking/body_frame_tracking_node.py`：
  1) 订阅 `/debug/aruco_pose` 与 `/mavros/state`；
  2) 直接在机体误差坐标下做 `x/y/yaw` PID；
  3) 发布 `mavros_msgs/msg/PositionTarget` 到 `/mavros/setpoint_raw/local`，`coordinate_frame=FRAME_BODY_NED`；
  4) 对 `vy/vz` 做 FLU->BODY_NED 符号转换；
  5) 保留 OFFBOARD 门控与视觉超时零速保护。
- 2026-03-12：新增启动文件 `src/body_frame_tracking/launch/body_frame_tracking.launch.py`，默认包含 `tvec_tf.launch.py` 并启动新节点。
- 2026-03-12：新增文档 `src/body_frame_tracking/README.md`（大写），说明机体系输出、符号转换、话题与运行方式。
- 2026-03-12：更新 `src/body_frame_tracking/setup.py` 与 `src/body_frame_tracking/package.xml`，补齐入口点、launch/README 安装项与描述。
- 2026-03-12：按规则执行验证：
  1) `colcon build --symlink-install --packages-select body_frame_tracking` 通过；
  2) `source install/setup.bash` 已执行；
  3) `ros2 launch body_frame_tracking body_frame_tracking.launch.py world_name:=rover` 启动验证通过后停止。

## 问题记录（本轮补充）
- 2026-03-12 | 问题：机体坐标系按 FLU（xyz）测试时，`body_frame_tracking` 的 y 方向控制量反了。
  解答：已修正 `body_frame_tracking_node.py` 中 y 轴误差符号，按实测将 y 方向控制反向后输出。

## 修改记录（本轮补充）
### 功能包修改记录
#### body_frame_tracking
- 2026-03-12：更新 `src/body_frame_tracking/body_frame_tracking/body_frame_tracking_node.py`，将 y 轴误差由 `target_y - pose.y` 改为 `pose.y - target_y`，以修正 FLU 语义下 y 控制方向反向问题。
- 2026-03-12：按规则执行 `colcon build --symlink-install --packages-select body_frame_tracking` 与 `source install/setup.bash`。

## 问题记录（本轮补充）
- 2026-03-12 | 问题：执行 `aruco_tracking` 和 `body_frame_tracking` 时，分别会拉起哪些节点？
  解答：已按 launch 文件核对：两者都会包含 `tvec_tf.launch.py` 的整套视觉链路节点；区别仅在各自新增的控制节点不同：`aruco_tracking_node` vs `body_frame_tracking_node`。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-12：按“每轮对话都记录”规则，补充一次“节点拉起清单查询”的问答记录。

## 问题记录（本轮补充）
- 2026-03-12 | 问题：在正常实现功能的前提下，想最小化性能消耗，应该只保留哪些节点？
  解答：若目标是“视觉闭环控制可用且性能最省”，应去掉 RViz、额外 TF 广播与日志节点，仅保留图像桥接、Aruco检测、`aruco_pose` 计算和单个控制节点。仿真下最小组合通常为：`gz_bridge_image`、`tvec_rvec_node`、`tvec_tf_node` +（`aruco_tracking_node` 或 `body_frame_tracking_node` 二选一）。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-12：按“每轮对话都记录”规则，补充一次“最小节点保留策略”的问答记录。

## 问题记录（本轮补充）
- 2026-03-12 | 问题：在不破坏当前功能包功能的情况下，新增两个功能包，使用最小可用组合，分别使用 `aruco_tracking_node` 和 `body_frame_tracking_node`。
  解答：已完成：新增 `aruco_tracking_minimal` 与 `body_frame_tracking_minimal` 两个独立包，均采用最小链路（`gz_bridge_image + tvec_rvec_node + tvec_tf_node + 控制节点`），不修改原有功能包。

## 修改记录（本轮补充）
### 功能包修改记录
#### aruco_tracking_minimal
- 2026-03-12：新建功能包 `src/aruco_tracking_minimal`（`ament_python`，Apache-2.0）。
- 2026-03-12：新增启动文件 `src/aruco_tracking_minimal/launch/aruco_tracking_minimal.launch.py`，最小链路只拉起：
  1) `gz_bridge_image`；
  2) `tvec_rvec_node`；
  3) `tvec_tf_node`；
  4) `aruco_tracking_node`。
- 2026-03-12：新增 `src/aruco_tracking_minimal/README.md`，说明最小组合与运行方式。
- 2026-03-12：更新 `src/aruco_tracking_minimal/setup.py`、`package.xml`，补齐 launch/README 安装与描述。

#### body_frame_tracking_minimal
- 2026-03-12：新建功能包 `src/body_frame_tracking_minimal`（`ament_python`，Apache-2.0）。
- 2026-03-12：新增启动文件 `src/body_frame_tracking_minimal/launch/body_frame_tracking_minimal.launch.py`，最小链路只拉起：
  1) `gz_bridge_image`；
  2) `tvec_rvec_node`；
  3) `tvec_tf_node`；
  4) `body_frame_tracking_node`。
- 2026-03-12：新增 `src/body_frame_tracking_minimal/README.md`，说明最小组合与运行方式。
- 2026-03-12：更新 `src/body_frame_tracking_minimal/setup.py`、`package.xml`，补齐 launch/README 安装与描述。

#### 构建与验证
- 2026-03-12：按规则执行 `colcon build --symlink-install --packages-select aruco_tracking_minimal body_frame_tracking_minimal` 与 `source install/setup.bash`。
- 2026-03-12：按规则执行启动验证：
  1) `ros2 launch aruco_tracking_minimal aruco_tracking_minimal.launch.py world_name:=rover` 启动通过后停止；
  2) `ros2 launch body_frame_tracking_minimal body_frame_tracking_minimal.launch.py world_name:=rover` 启动通过后停止。

## 问题记录（本轮补充）
- 2026-03-12 | 问题：如果要调节 tracking 中 XY 和 yaw 的 PID 参数，应该记录哪些数据？
  解答：建议至少同步记录“误差、控制输出、系统状态、约束触发”四类数据：`/debug/aruco_pose`（x/y/yaw 原始误差源）、控制命令话题（`/mavros/setpoint_velocity/cmd_vel` 或 `/mavros/setpoint_raw/local`）、飞控状态（`/mavros/state`）以及本机位姿（`/mavros/local_position/pose`）；并保留时间戳用于对齐和评估时延。若有节点内日志，需同时记录 `err_marker/err_cmd`、限幅/死区触发与数据超时信息。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-12：按“每轮对话都记录”规则，新增一次“tracking PID 调参数据记录项”问答记录。

## 问题记录（本轮补充）
- 2026-03-14 12:40:16 CST | 问题：阅读远程仓库 `simple` 与 `rasip` 分支中的 `LOG`、子目录日志及 `PILOG`，整理常见问题并在 `questions` 目录输出文档。
  解答：已完成。基于远程 `origin/simple` 与 `origin/rasip_pi` 的日志内容，归纳形成常见问题总结文档 `questions/common_issues_from_simple_rasip_logs.md`，覆盖 QoS 不匹配、MAVROS 前缀、仿真迁移实机相机输入、USB 设备与像素格式、依赖兼容、串口权限、坐标系/符号、OFFBOARD 门控、落地 disarm、高度参考漂移等高频问题，并附排障清单。

## 修改记录（本轮补充）
### 文档与日志
- 2026-03-14 12:40:16 CST：新增问题总结文档 `questions/common_issues_from_simple_rasip_logs.md`，来源为 `origin/simple:LOG.md`、`origin/simple:questions/LOG.md`、`origin/rasip_pi:LOG.md`、`origin/rasip_pi:PI_LOG.md`。
- 2026-03-14 12:40:16 CST：按“每轮对话都记录”规则，同步更新 `questions/LOG.md` 与本文件。

## 问题记录（本轮补充）
- 2026-03-14 12:48:05 CST | 问题：把常见问题和解决方案写入 `AGENTS`，新开一栏标题。
  解答：已完成。在 `AGENTS.md` 新增章节 **“常见问题与解决方案（跨分支日志沉淀）”**，集中固化 QoS、MAVROS 前缀、仿真迁移实机相机、USB 设备/像素格式、`cv_bridge/numpy` 兼容、串口权限、坐标系符号、OFFBOARD 超时、LAND disarm、高度参考漂移、launch 参数覆盖、构建后 source 等高频问题及对应修复动作。

## 修改记录（本轮补充）
### 文档与日志
- 2026-03-14 12:48:05 CST：更新 `AGENTS.md`，新增“常见问题与解决方案（跨分支日志沉淀）”栏目。
- 2026-03-14 12:48:05 CST：按“每轮对话都记录”规则，同步更新 `questions/LOG.md` 与本文件。

## 问题记录（本轮补充）
- 2026-03-14 21:06:46 CST | 问题：`Position` 模式下，摇杆的输入映射的是目标速度吗？
  解答：基本是。PX4 多旋翼 `Position` 模式中，横滚/俯仰摇杆通常先映射为水平速度目标，偏航摇杆对应偏航变化，油门杆对应垂向运动请求；飞控再通过位置环和速度环把这些目标转成姿态与推力。中位回杆后速度目标回零，当前位置被锁成新的定点目标。

## 问题记录（本轮补充）
- 2026-03-14 21:23:25 CST | 问题：使用二维码 tracking 功能包时是否需要 MAVROS 全部功能，能否只启动部分插件减轻机载负载；启动 MAVROS 后机器变卡且 VNC 色彩异常。
  解答：可以裁剪。Jazzy 版 MAVROS 的 `px4.launch` 支持 `pluginlists_yaml`，可仅保留 tracking 需要的最小插件集。对 `aruco_tracking` 类速度控制链路，通常保留 `sys_status`、`local_position`、`setpoint_velocity`，必要时加 `command` 即可；对 `body_frame_tracking` 则将 `setpoint_velocity` 换成 `setpoint_raw`。只有返航/total 等流程才需要额外的 `global_position` 与 `home_position`。mission、gimbal、ftp、adsb、distance_sensor、image_pub、px4flow 等多数插件可关闭。启动后整机变卡与 VNC 颜色异常更像是系统资源占用过高导致，减插件、减少无关话题与图像显示负担通常会改善。

## 问题记录（本轮补充）
- 2026-03-14 21:34:05 CST | 问题：帮我做一个包含该工作空间下所有功能包要使用插件的最小集合的 MAVROS 启动。
  解答：已完成。新建独立配置包 `mavros_profiles`，在工作空间内提供最小 MAVROS 启动而不修改系统安装目录。新增 `launch/workspace_minimal_px4.launch.py` 与 `config/workspace_minimal_pluginlists.yaml`，保留插件为 `sys_*`、`command`、`local_position`、`global_position`、`home_position`、`setpoint_velocity`、`setpoint_raw`，覆盖当前工作空间所有依赖 MAVROS 的功能包所需接口。已完成单包构建、环境加载与限时启动验证，日志确认仅目标插件被加载，其余插件被忽略。

## 修改记录（本轮补充）
### 文档与日志
- 2026-03-14 21:06:46 CST：按“每轮对话都记录”规则，同步更新 `questions/LOG.md` 与本文件。
- 2026-03-14 21:23:25 CST：按“每轮对话都记录”规则，同步更新 `questions/LOG.md` 与本文件。

### 功能包修改记录
#### mavros_profiles
- 2026-03-14：使用 `ros2 pkg create` 新建功能包 `src/mavros_profiles`（`ament_python`，Apache-2.0）。
- 2026-03-14：新增 `src/mavros_profiles/config/workspace_minimal_pluginlists.yaml`，采用“全禁用 + 仅允许最小插件集”策略。
- 2026-03-14：新增 `src/mavros_profiles/launch/workspace_minimal_px4.launch.py`，复用系统 `mavros/launch/node.launch` 并注入工作空间内的最小插件表。
- 2026-03-14：新增 `src/mavros_profiles/README.md`，说明目标插件集、覆盖范围和启动方法。
- 2026-03-14：更新 `src/mavros_profiles/package.xml`、`src/mavros_profiles/setup.py`，补齐 `launch/launch_ros/mavros` 依赖和 launch/config/README 安装项。

#### 构建与验证
- 2026-03-14：执行 `colcon build --symlink-install --packages-select mavros_profiles`，构建通过。
- 2026-03-14：执行 `source install/setup.bash`。
- 2026-03-14：执行 `timeout 8s ros2 launch mavros_profiles workspace_minimal_px4.launch.py` 启动验证通过；日志确认仅 `command`、`global_position`、`home_position`、`local_position`、`setpoint_raw`、`setpoint_velocity`、`sys_status`、`sys_time` 被加载，其余插件均被忽略。

## 问题记录（本轮补充）
- 2026-03-15 00:06:35 CST | 问题：已经删除 `gitpat.md`，为什么 `git push` 仍然被 GitHub Push Protection 拦截？
  解答：已确认原因是密钥仍存在于待推送的提交历史里，而不只是当前工作区文件是否存在。当前 `simple` 相对 `origin/simple` 还多出两个本地提交：`0f5bfe3` 和 `f1d97b8`，其中 `0f5bfe3` 仍包含 `gitpat.md`，`f1d97b8` 只是将其删除。GitHub 会扫描整条待推送提交链，因此仍会在 `0f5bfe3` 中发现 PAT 并拒绝推送。解决方式是重写本地提交历史，把该文件从待推送提交中彻底移除；同时建议立即撤销该 PAT。

## 问题记录（本轮补充）
- 2026-03-15 00:11:14 CST | 问题：帮我把这条分支整理干净。
  解答：已完成。将本地 `simple` 分支软重置到 `origin/simple`，保留真实文件改动并重新提交，从而移除原先含有 `gitpat.md` 的两次待推送提交历史。清理后 `origin/simple..simple` 之间仅剩一个新提交 `9997748`，工作区恢复干净，并对最新提交执行了 PAT 关键字检索，未再发现 `github_pat` / `ghp_` 痕迹。仍建议尽快撤销原 PAT。

## 修改记录（本轮补充）
### 文档与日志
- 2026-03-15 00:06:35 CST：按“每轮对话都记录”规则，同步更新 `questions/LOG.md` 与本文件。
- 2026-03-15 00:11:14 CST：按“每轮对话都记录”规则，同步更新 `questions/LOG.md` 与本文件。

## 问题记录（本轮补充）
- 2026-03-12 | 问题：把“记录调节 tracking 的关键数据并导出 CSV”的功能加入两个 minimal 功能包。
  解答：已完成：在 `aruco_tracking_minimal` 和 `body_frame_tracking_minimal` 中分别新增 CSV 记录节点，并默认由各自 launch 拉起；可按参数配置目录、前缀和采样率，运行后自动生成 CSV 文件。

## 修改记录（本轮补充）
### 功能包修改记录
#### aruco_tracking_minimal
- 2026-03-12：新增 `src/aruco_tracking_minimal/aruco_tracking_minimal/csv_logger_node.py`，订阅 `/debug/aruco_pose`、`/mavros/state`、`/mavros/local_position/pose`、`/mavros/setpoint_velocity/cmd_vel`，按固定频率写入 CSV（含新鲜度、消息时延、位姿与控制量）。
- 2026-03-12：更新 `src/aruco_tracking_minimal/setup.py`，新增入口点 `csv_logger_node`。
- 2026-03-12：更新 `src/aruco_tracking_minimal/package.xml`，补齐 `geometry_msgs`、`mavros_msgs`、`debug_interface` 依赖。
- 2026-03-12：更新 `src/aruco_tracking_minimal/launch/aruco_tracking_minimal.launch.py`，默认拉起 CSV logger，并新增参数：`enable_csv_logger`、`csv_output_dir`、`csv_prefix`、`csv_sample_rate_hz`。
- 2026-03-12：更新 `src/aruco_tracking_minimal/README.md`，新增 CSV 记录说明与示例命令。

#### body_frame_tracking_minimal
- 2026-03-12：新增 `src/body_frame_tracking_minimal/body_frame_tracking_minimal/csv_logger_node.py`，订阅 `/debug/aruco_pose`、`/mavros/state`、`/mavros/local_position/pose`、`/mavros/setpoint_raw/local`，按固定频率写入 CSV（含 `PositionTarget` 关键字段）。
- 2026-03-12：更新 `src/body_frame_tracking_minimal/setup.py`，新增入口点 `csv_logger_node`。
- 2026-03-12：更新 `src/body_frame_tracking_minimal/package.xml`，补齐 `geometry_msgs`、`mavros_msgs`、`debug_interface` 依赖。
- 2026-03-12：更新 `src/body_frame_tracking_minimal/launch/body_frame_tracking_minimal.launch.py`，默认拉起 CSV logger，并新增参数：`enable_csv_logger`、`csv_output_dir`、`csv_prefix`、`csv_sample_rate_hz`。
- 2026-03-12：更新 `src/body_frame_tracking_minimal/README.md`，新增 CSV 记录说明与示例命令。

#### 构建与验证
- 2026-03-12：按规则执行 `colcon build --symlink-install --packages-select aruco_tracking_minimal body_frame_tracking_minimal`，构建通过。
- 2026-03-12：按规则执行 `source install/setup.bash`。
- 2026-03-12：按规则执行启动验证并停止：
  1) `ros2 launch aruco_tracking_minimal aruco_tracking_minimal.launch.py world_name:=rover model_name:=x500_mono_cam_down_0`（logger 启动成功并生成 CSV）；
  2) `ros2 launch body_frame_tracking_minimal body_frame_tracking_minimal.launch.py world_name:=rover model_name:=x500_mono_cam_down_0`（logger 启动成功并生成 CSV）。

## 问题记录（本轮补充）
- 2026-03-14 | 问题：直接帮忙安装 Ubuntu 蓝牙驱动。
  解答：已远程执行硬件与服务诊断，确认 `bluetooth.service` 运行、`btusb/btmtk` 模块已加载，但控制器 `hci0` 处于 `DOWN` 且 `BD Address` 为全 0，表现为固件/初始化未完成；由于当前会话无法输入本机 sudo 密码，系统级安装与重载步骤需用户本机执行一次提权命令后继续。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-14：按“每轮对话都记录”规则，补充一次“蓝牙驱动安装与诊断”问答记录（未修改功能包代码）。

## 问题记录（本轮补充）
- 2026-03-14 | 问题：提供 sudo 密码后，要求直接完成 Ubuntu 蓝牙驱动安装。
  解答：已代执行安装与驱动重载：`bluez/bluez-tools/rfkill/linux-firmware` 安装完成，`btusb/btmtk` 模块已加载；但硬件仍报 `Bluetooth: hci0: Opcode 0x0c03 failed: -110`，表现为 `hci0 DOWN` 且 `BD Address=00:00:00:00:00:00`，属于设备初始化超时，当前需进一步做冷重启/内核回退或 BIOS 电源管理项排查。

## 修改记录（本轮补充）
### 系统运维记录
- 2026-03-14：在主机执行蓝牙环境安装与排障：
  1) 安装/确认 `bluez`、`bluez-tools`、`rfkill`、`linux-firmware`；
  2) 重载 `btusb/btmtk/bluetooth` 模块并重启 `bluetooth.service`；
  3) 尝试 USB 设备 `13d3:3596` 解绑/重绑与 `btusb` 参数调优（`reset=0`、`enable_autosuspend=0`）；
  4) 结果：控制器仍未完成初始化，日志持续出现 `Opcode 0x0c03 failed: -110`。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：将远程仓库 `note` 克隆到主目录 `obsidian` 中。
  解答：已完成克隆，目标路径为 `/home/zjh/obsidian/note`，远程为 `https://github.com/junheng-sect/note.git`，当前分支 `main`。

## 修改记录（本轮补充）
### 系统运维记录
- 2026-03-15：执行 `git clone https://github.com/junheng-sect/note.git` 到 `/home/zjh/obsidian/`，并验证远程地址与目录内容。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：将 `rasip_pi` 分支中的 `pid_tuning` 功能包部署到 `simple` 分支，并修改相机话题参数后完成构建测试。
  解答：已完成：从 `origin/rasip_pi` 迁移 `src/pid_tuning` 到当前 `simple` 工作区；将默认相机话题改为 `/camera/image_raw`，并将默认 CSV 路径改为 `zjh_ws`；构建通过且 launch 启动验证通过。

## 修改记录（本轮补充）
### 功能包修改记录
#### pid_tuning
- 2026-03-15：从 `origin/rasip_pi` 迁移功能包 `src/pid_tuning` 到 `simple` 工作区（包含节点、launch、README、测试与安装文件）。
- 2026-03-15：更新 `src/pid_tuning/launch/pid_tuning.launch.py`：
  1) 默认相机话题参数 `ros_image_topic` 由 `/image_raw` 改为 `/camera/image_raw`；
  2) 默认 CSV 目录由 `~/project/rasip_pi_ws/log/tracking_csv` 改为 `~/project/zjh_ws/log/tracking_csv`；
  3) 默认汇总 CSV 路径同步改为 `~/project/zjh_ws/log/tracking_csv/pid_tuning_summary.csv`。
- 2026-03-15：更新 `src/pid_tuning/README.md`，同步说明默认相机话题与默认 CSV 路径。

#### 构建与验证
- 2026-03-15：按规则执行 `colcon build --symlink-install --packages-select pid_tuning`，构建通过。
- 2026-03-15：按规则执行 `source install/setup.bash`。
- 2026-03-15：按规则执行 launch 验证并停止：
  `ros2 launch pid_tuning pid_tuning.launch.py world_name:=rover model_name:=x500_mono_cam_down_0 use_usb_cam:=false use_rqt:=false`。
  启动结果：`pid_tuning_node`、`pid_tuning_csv_logger_node`、`tvec_rvec_node`、`tvec_tf_node` 正常拉起，`tvec_rvec_node` 订阅话题为 `/camera/image_raw`。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：新建功能包，实现“先保持 XY 不动，先把 yaw 调正，再使无人机与二维码相对 XY 变为 0”，并使用指定 PID 参数默认值。
  解答：已完成：新增独立功能包 `yaw_then_xy_tracking`，主节点实现两阶段状态机 `YAW_ALIGN -> XY_ALIGN`；默认 PID 参数与给定值一致，且已完成构建与 launch 验证。

## 修改记录（本轮补充）
### 功能包修改记录
#### yaw_then_xy_tracking
- 2026-03-15：在 `src/` 下使用 `ros2 pkg create` 新建 `ament_python` 功能包 `yaw_then_xy_tracking`（Apache-2.0）。
- 2026-03-15：新增主节点 `src/yaw_then_xy_tracking/yaw_then_xy_tracking/yaw_then_xy_tracking_node.py`：
  1) 订阅 `/debug/aruco_pose` 与 `/mavros/state`；
  2) 发布 `/mavros/setpoint_raw/local`（`FRAME_BODY_NED`）；
  3) 两阶段逻辑：
     - `YAW_ALIGN`：锁定 `vx/vy=0`，仅控制 `yaw`（和可选 `z`）；
     - `XY_ALIGN`：在继续 yaw+z 闭环的同时控制 `xy` 到目标；
  4) 默认 PID 参数按需求设置：`kp_x/kp_y=0.6, kd_x/kd_y=0.02, kp_yaw=0.60, kd_yaw=0.02, target_z=2.5, kp_z=0.60, kd_z=0.06`，并设置 `vx_limit/vy_limit=1.0`、`velocity_deadband=0.03`、`yaw_rate_deadband=0.03`。
- 2026-03-15：新增启动文件 `src/yaw_then_xy_tracking/launch/yaw_then_xy_tracking.launch.py`，复用 `tvec.launch.py + tvec_tf_node`，默认图像话题 `/camera/image_raw`。
- 2026-03-15：更新 `src/yaw_then_xy_tracking/setup.py` 与 `package.xml`，补齐入口点、launch/README 安装项与描述。
- 2026-03-15：新增 `src/yaw_then_xy_tracking/README.md`，说明两阶段流程、默认参数与启动命令。

#### 构建与验证
- 2026-03-15：按规则执行 `colcon build --symlink-install --packages-select yaw_then_xy_tracking`，构建通过。
- 2026-03-15：按规则执行 `source install/setup.bash`。
- 2026-03-15：按规则执行启动验证并停止：
  `ros2 launch yaw_then_xy_tracking yaw_then_xy_tracking.launch.py world_name:=rover model_name:=x500_mono_cam_down_0 use_usb_cam:=false use_rqt:=false`。
  启动结果：`yaw_then_xy_tracking_node`、`tvec_rvec_node`、`tvec_tf_node` 正常拉起，节点在非 OFFBOARD 下安全输出零速。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：调整两阶段控制逻辑为“第一步只调正 yaw；第二步再调正 xyz；且第二步中 yaw 仍持续 PID 对正”。
  解答：已完成：`yaw_then_xy_tracking_node` 第一阶段改为仅 yaw 控制（x/y/z 全锁死）；第二阶段改为 xyz + yaw 同时闭环。

## 修改记录（本轮补充）
### 功能包修改记录
#### yaw_then_xy_tracking
- 2026-03-15：更新 `src/yaw_then_xy_tracking/yaw_then_xy_tracking/yaw_then_xy_tracking_node.py`：
  1) 第一阶段 `YAW_ALIGN` 改为仅 yaw 控制，强制 `vx=vy=vz=0`，并重置 `pid_z`；
  2) 第二阶段保持 yaw 闭环，同时控制 `x/y/z` 到目标；
  3) 更新状态日志文案，明确阶段含义为 `YAW_ALIGN(仅Yaw)` 与 `XY_ALIGN(XYZ+Yaw)`。
- 2026-03-15：更新 `src/yaw_then_xy_tracking/README.md`，同步说明新流程：先仅 yaw，再 xyz+yaw。

#### 构建与验证
- 2026-03-15：按规则执行 `colcon build --symlink-install --packages-select yaw_then_xy_tracking`，构建通过。
- 2026-03-15：按规则执行 `source install/setup.bash`。
- 2026-03-15：执行 `ros2 launch yaw_then_xy_tracking yaw_then_xy_tracking.launch.py world_name:=rover model_name:=x500_mono_cam_down_0 use_usb_cam:=false use_rqt:=false` 冒烟验证通过并停止。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：修改 `land_with_tracking` 的 `ALIGN` 逻辑为“先 yaw，再 xyz（含 z 到 target_z，默认 2.5m）”，后续降落流程保持不变；并将下降过程 tracking 参数改为与指定 `pid_tuning` 参数一致。
  解答：已完成：`ALIGN` 改为子阶段 `YAW_ONLY -> XYZ_ALIGN`；`YAW_ONLY` 仅控制 yaw，`XYZ_ALIGN` 控制 xyz 并持续 yaw 闭环；`DESCEND_WITH_TRACK` 保持原流程（固定下降速度 + xy/yaw 闭环）不变，且默认 tracking 参数已切换到指定值。

## 修改记录（本轮补充）
### 功能包修改记录
#### land_with_tracking
- 2026-03-15：更新 `src/land_with_tracking/land_with_tracking/land_with_tracking_node.py`：
  1) 新增 `track_target_z`（默认 `2.5`）与 `z_align_tolerance_m`（默认 `0.10`）；
  2) 将 `ALIGN` 拆分为 `ALIGN/YAW_ONLY` 与 `ALIGN/XYZ_ALIGN` 两个子阶段；
  3) `YAW_ONLY` 子阶段仅输出 `wz`，强制 `vx=vy=vz=0`；
  4) `XYZ_ALIGN` 子阶段输出 `vx/vy/vz/wz`，要求 `xy/z/yaw` 同时满足阈值并保持 `align_hold_sec`；
  5) 保持 `DESCEND_WITH_TRACK` 逻辑不变（`vz=-descent_speed_mps` + `xy/yaw` 闭环）；
  6) tracking 默认 PID 参数切换为：
     - `kp_x=0.6 ki_x=0.0 kd_x=0.02`
     - `kp_y=0.6 ki_y=0.0 kd_y=0.02`
     - `kp_yaw=0.60 ki_yaw=0.0 kd_yaw=0.02`
     - `track_target_z=2.5 kp_z=0.60 ki_z=0.0 kd_z=0.06`
     - `vx_limit=1.0 vy_limit=1.0`
     - `velocity_deadband=0.03 yaw_rate_deadband=0.03`
- 2026-03-15：更新 `src/land_with_tracking/launch/land_with_tracking.launch.py`，同步默认参数到 launch。
- 2026-03-15：更新 `src/land_with_tracking/README.md`，说明新 `ALIGN` 两阶段流程和默认参数。

#### 构建与验证
- 2026-03-15：按规则执行 `colcon build --symlink-install --packages-select land_with_tracking`，构建通过。
- 2026-03-15：按规则执行 `source install/setup.bash`。
- 2026-03-15：按规则执行启动验证并停止：
  `ros2 launch land_with_tracking land_with_tracking.launch.py world_name:=rover model_name:=x500_mono_cam_down_0`。
  启动结果：`land_with_tracking_node` 及视觉链路正常拉起，节点在非 OFFBOARD 下安全输出零速。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：`xy tracking` 有问题，要求根据日志修正控制量。
  解答：已根据日志现象修正：在 `ALIGN/XYZ_ALIGN` 中观察到 `ex` 增大时 `vx` 同号输出导致发散，故新增 `x/y` 控制方向参数并默认将 `x` 通道反向；同时新增 `ALIGN` 与 `DESCEND` 阶段的 `xy` 限幅，避免大速度导致丢码和发散。

## 修改记录（本轮补充）
### 功能包修改记录
#### land_with_tracking
- 2026-03-15：更新 `src/land_with_tracking/land_with_tracking/land_with_tracking_node.py`：
  1) 新增控制方向参数：`track_vx_sign`（默认 `-1.0`）、`track_vy_sign`（默认 `1.0`）；
  2) 新增阶段限幅参数：`align_max_vxy`（默认 `0.45`）、`descend_max_vxy`（默认 `0.45`）；
  3) 在 `compute_tracking_cmd()` 对 `vx/vy` 应用方向修正；
  4) 在 `ALIGN/XYZ_ALIGN` 和 `DESCEND_WITH_TRACK` 发布前分别应用限幅夹紧。
- 2026-03-15：更新 `src/land_with_tracking/launch/land_with_tracking.launch.py`，补充以上参数默认值。
- 2026-03-15：更新 `src/land_with_tracking/README.md`，补充日志修正项与阶段限幅参数说明。

#### 构建与验证
- 2026-03-15：按规则执行 `colcon build --symlink-install --packages-select land_with_tracking`，构建通过。
- 2026-03-15：按规则执行 `source install/setup.bash`。
- 2026-03-15：按规则执行启动验证并停止：
  `ros2 launch land_with_tracking land_with_tracking.launch.py world_name:=rover model_name:=x500_mono_cam_down_0`。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：新建 `land_with_tracking_v2` 功能包，流程为 `ALIGN(YAW_ONLY->XYZ_ALIGN) -> HOVER_BEFORE_LAND -> DESCEND_WITH_TRACK -> TOUCHDOWN_DISARM -> DONE`，并采用指定 PID 参数。
  解答：已完成：新增独立包 `land_with_tracking_v2`，实现所需状态机与参数默认值；前两步对齐逻辑参考 `yaw_then_xy_tracking`，落地下压+周期 disarm 参考 landing/land_with_tracking 机制。

## 修改记录（本轮补充）
### 功能包修改记录
#### land_with_tracking_v2
- 2026-03-15：在 `src/` 下使用 `ros2 pkg create` 新建 `ament_python` 包 `land_with_tracking_v2`（Apache-2.0）。
- 2026-03-15：新增主节点 `src/land_with_tracking_v2/land_with_tracking_v2/land_with_tracking_v2_node.py`：
  1) `ALIGN/YAW_ONLY`：仅调整 yaw，`x/y/z` 不动；
  2) `ALIGN/XYZ_ALIGN`：调整 `x/y/z` 到目标（默认 `target_z=2.5`）并持续 yaw 闭环；
  3) `HOVER_BEFORE_LAND`：悬停 1 秒；
  4) `DESCEND_WITH_TRACK`：固定下降速度 + `xy+yaw` 闭环；
  5) `TOUCHDOWN_DISARM`：最低油门下压并每秒请求 disarm；
  6) `DONE`：完成后零速保持。
- 2026-03-15：默认 PID 参数按要求写入：
  `kp_x=0.6 ki_x=0.0 kd_x=0.02`、
  `kp_y=0.6 ki_y=0.0 kd_y=0.02`、
  `kp_yaw=0.60 ki_yaw=0.0 kd_yaw=0.02`、
  `kp_z=0.60 ki_z=0.0 kd_z=0.06`、
  `track_target_z=2.5`、
  `vx_limit=1.0 vy_limit=1.0`、
  `velocity_deadband=0.03 yaw_rate_deadband=0.03`。
- 2026-03-15：新增启动文件 `src/land_with_tracking_v2/launch/land_with_tracking_v2.launch.py`，默认包含 `tvec_tf.launch.py`。
- 2026-03-15：新增文档 `src/land_with_tracking_v2/README.md`，说明流程、参数与运行方式。
- 2026-03-15：更新 `src/land_with_tracking_v2/setup.py`、`package.xml`，补齐入口点、launch/README 安装项与描述。

#### 构建与验证
- 2026-03-15：按规则执行 `colcon build --symlink-install --packages-select land_with_tracking_v2`，构建通过。
- 2026-03-15：按规则执行 `source install/setup.bash`。
- 2026-03-15：按规则执行启动验证并停止：
  `ros2 launch land_with_tracking_v2 land_with_tracking_v2.launch.py world_name:=rover model_name:=x500_mono_cam_down_0`。
  启动结果：`land_with_tracking_v2_node` 与视觉链路节点正常拉起。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：`land_with_tracking_v2` 在 `ALIGN/XYZ_ALIGN` 阶段跟踪失败，分析原因。
  解答：根据日志，主因是 XY 控制通道出现“误差同号加速”导致正反馈发散（`|ex|`/`|ey|` 持续增大且 `vx`/`vy` 与误差方向长期同向）；次因是 z 误差偶发突变触发较大 `vz`（如 `ez=-0.32 -> vz=-0.60`），叠加造成视野内目标快速漂移并最终超时丢码。yaw 通道误差始终较小（约 0~2 度），不是主要问题。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-15：按“每轮对话都记录”规则，新增一次 `land_with_tracking_v2` 跟踪失败日志分析记录（本轮未修改代码）。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：先固定 `xy` 变换为机体系直控（对齐 `yaw_then_xy_tracking` 思路，去掉动态旋转环节）。
  解答：已完成：`land_with_tracking_v2` 的 `compute_track_cmd()` 改为机体误差直控 `ex=target_x-pose.x`、`ey=pose.y-target_y`，不再执行动态 yaw 旋转与 base_pose 参与的坐标变换。

## 修改记录（本轮补充）
### 功能包修改记录
#### land_with_tracking_v2
- 2026-03-15：更新 `src/land_with_tracking_v2/land_with_tracking_v2/land_with_tracking_v2_node.py`：
  1) `xy` 误差计算改为机体误差直控；
  2) 删除 `control_loop` 中“base_pose 超时等待”分支（不再依赖动态旋转结果）；
  3) 保持其余状态机与降落流程不变。
- 2026-03-15：更新 `src/land_with_tracking_v2/README.md`，补充“XY 机体系直控、不使用动态 yaw 旋转链路”说明。

#### 构建与验证
- 2026-03-15：按规则执行 `colcon build --symlink-install --packages-select land_with_tracking_v2`，构建通过。
- 2026-03-15：按规则执行 `source install/setup.bash`。
- 2026-03-15：按规则执行启动验证并停止：
  `ros2 launch land_with_tracking_v2 land_with_tracking_v2.launch.py world_name:=rover model_name:=x500_mono_cam_down_0`。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：`land_with_tracking_v2` 机体系左右与前后控制量对调，且仍出现发散。
  解答：已继续修正控制量映射：在 `xy` 输出“通道交换”基础上，增加“双通道反向”处理（先交换，再同时取反），用于消除日志中 `ex/ey` 与 `vx/vy` 同号正反馈问题。

## 修改记录（本轮补充）
### 功能包修改记录
#### land_with_tracking_v2
- 2026-03-15：更新 `src/land_with_tracking_v2/land_with_tracking_v2/land_with_tracking_v2_node.py` 中 `compute_track_cmd()`：
  1) 保持 `x/y` 通道交换；
  2) 对交换后的 `vx/vy` 同时取反，修正同号发散问题。

#### 构建与验证
- 2026-03-15：按规则执行 `colcon build --symlink-install --packages-select land_with_tracking_v2`，构建通过。
- 2026-03-15：按规则执行 `source install/setup.bash`。
- 2026-03-15：执行 `ros2 launch land_with_tracking_v2 land_with_tracking_v2.launch.py world_name:=rover model_name:=x500_mono_cam_down_0` 冒烟验证通过并停止。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：`land_with_tracking_v2` 现在 xy 对应关系正确，但控制量方向反了，x/y 都发散。
  解答：已修正为“仅做 x-y 通道交换，不做符号反向”；撤销上一轮双通道取反逻辑。

## 修改记录（本轮补充）
### 功能包修改记录
#### land_with_tracking_v2
- 2026-03-15：更新 `src/land_with_tracking_v2/land_with_tracking_v2/land_with_tracking_v2_node.py`：
  1) `compute_track_cmd()` 中保留 `x/y` 通道交换；
  2) 删除对交换后 `vx/vy` 的负号反向。

#### 构建与验证
- 2026-03-15：按规则执行 `colcon build --symlink-install --packages-select land_with_tracking_v2`，构建通过。
- 2026-03-15：按规则执行 `source install/setup.bash`。
- 2026-03-15：执行 `ros2 launch land_with_tracking_v2 land_with_tracking_v2.launch.py world_name:=rover model_name:=x500_mono_cam_down_0` 冒烟验证通过并停止。
