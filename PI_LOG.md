# 树莓派开发日志

## 实机记录

### 问题记录
- 2026-03-08 | 问题：是否可通过 SSH 在树莓派终端进行真机调试。
  解答：已验证可行，成功 SSH 登录树莓派并确认系统环境（`zjh@zjh`，Ubuntu 22.04.5，`/home/zjh`）。
- 2026-03-08 | 问题：在树莓派 `~/project` 下创建 `rasip_pi_ws`，下载远程 `rasip_pi` 分支并配置本地 Git 撤回。
  解答：已完成 `git clone --branch rasip_pi --single-branch` 到 `~/project/rasip_pi_ws`，并配置本地别名 `undo/unstage/discard/last`。
- 2026-03-08 | 问题：先不安装依赖，先列出依赖清单。
  解答：已在树莓派提取 `package.xml` 依赖；`rosdep` 未初始化（提示需 `sudo rosdep init && rosdep update`）。
- 2026-03-08 | 问题：在树莓派工作空间执行 `colcon build`。
  解答：已执行 `colcon build --symlink-install`，18 个包构建通过（约 1 分 37 秒），并完成 `source install/setup.bash`。
- 2026-03-08 | 问题：`mavros` 启动报错 `DeviceError:serial:open: Permission denied`，无法连接飞控。
  解答：已定位并修复串口权限问题，`/dev/ttyS0` 由异常权限修正为 `root:dialout 660`，回归验证 `ros2 launch mavros px4.launch fcu_url:=/dev/serial0:921600` 成功，日志出现 `HEARTBEAT, connected. FCU: PX4 Autopilot`。
- 2026-03-08 | 问题：把 `source` 加入树莓派 `.bashrc`，使终端自动加载工作空间环境。
  解答：已在树莓派 `~/.bashrc` 新增 `~/project/rasip_pi_ws/install/setup.bash` 自动加载逻辑（带存在性判断），并验证新 shell 环境生效。
- 2026-03-08 | 问题：准备实机测试 `landing` 与 `offboard_test`，是否无需修改；后续视觉链路是否只需把图像源从仿真改为 USB。
  解答：`landing/offboard_test` 本身不依赖摄像头，可直接测；需先确认 MAVROS 前缀是否一致（`/mavros/...` 或 `/uas1/mavros/...`）。后续接入相机时需同时切换图像源、`camera_info` 与相机坐标系/内参配置。
- 2026-03-08 | 问题：新增 `return_landing` 功能包，要求先执行 return_home 再执行 landing，并在树莓派上验证运行无报错。
  解答：已在树莓派工作空间完成新包同步、构建与启动验证；节点按状态机启动正常，无报错后停止。
- 2026-03-08 | 问题：`return_landing` 在降落后无法 disarm，要求改成与 `landing` 一致的下压油门 disarm 方案。
  解答：已在树莓派同步修复后的节点代码，构建并启动验证通过；新版 `LAND` 逻辑已支持“最低油门下压 + 周期 disarm”兜底。
- 2026-03-08 | 问题：将 `PI_LOG` 的修改记录改成与 `LOG` 一致，要求按“功能包小标题”分组记录。
  解答：已重排 `PI_LOG.md` 结构，新增并统一使用“### 功能包修改记录”章节，且按功能包小标题分组记录。
- 2026-03-08 | 问题：确认并记住当前 `ost.yaml` 为实机 USB 摄像头参数。
  解答：已确认并记录：`/home/zjh/project/rasip_pi_ws/src/camera_calibration_pkg/calibrationdata_extracted/ost.yaml` 作为当前实机摄像头参数基准文件。
- 2026-03-08 | 问题：同步树莓派与 laptop 的 `rasip_pi_ws`，并完成本地与远程 Git 上传。
  解答：已完成：laptop 提交并推送 `a6892b6` 到 `origin/rasip_pi`；树莓派执行 `git fetch/pull --ff-only` 后已对齐到同一提交。

### 修改记录

#### 工作空间与协作规范
- 2026-03-08：确认树莓派远程联调链路可用，后续实机调试通过 SSH 进行。
- 2026-03-08：在树莓派 `~/.bashrc` 追加自动环境加载：
  1) `if [ -f ~/project/rasip_pi_ws/install/setup.bash ]; then`
  2) `  source ~/project/rasip_pi_ws/install/setup.bash`
  3) `fi`
- 2026-03-08：验证 `.bashrc` 生效：`grep` 命中新配置，`bash -lc 'source ~/.bashrc'` 后 `AMENT_PREFIX_PATH` 非空。
- 2026-03-08：`PI_LOG.md` 重构为统一结构（实机/仿真分区 + 功能包分组记录）。

#### Git 仓库与远程
- 2026-03-08：在树莓派创建 `~/project/rasip_pi_ws` 并克隆 `ROS2PUBLIC` 的 `rasip_pi` 分支。
- 2026-03-08：在树莓派仓库配置本地 Git 撤回别名：
  1) `undo=reset --soft HEAD~1`
  2) `unstage=reset HEAD --`
  3) `discard=checkout --`
  4) `last=log -1 --stat`

#### 依赖与环境检查
- 2026-03-08：执行依赖解析时发现 `rosdep` 未初始化（当前仅记录依赖，不执行安装）。
- 2026-03-08：整理外部依赖清单（含 `rclpy/geometry_msgs/sensor_msgs/std_msgs/mavros_msgs/tf2_ros/cv_bridge/rviz2/ros_gz_bridge` 等）。

#### 构建与验证
- 2026-03-08：在树莓派执行 `source /opt/ros/humble/setup.bash && colcon build --symlink-install`，构建汇总 `18 packages finished`。
- 2026-03-08：构建后执行 `source install/setup.bash`，环境加载成功（`sourced_ok`）。

#### MAVROS 串口修复
- 2026-03-08：排查确认故障点：`/dev/ttyS0` 权限为 `crw--w---- root:tty`，导致 MAVROS 无法以读写方式打开串口。
- 2026-03-08：新增持久化 udev 规则 `/etc/udev/rules.d/99-px4-serial.rules`：
  1) `KERNEL=="ttyS0", GROUP="dialout", MODE="0660"`
  2) `KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"`
- 2026-03-08：执行 `udevadm control --reload-rules` 与 `udevadm trigger`，并立即修正当前设备权限为 `crw-rw---- root:dialout /dev/ttyS0`。
- 2026-03-08：回归验证 `ros2 launch mavros px4.launch fcu_url:=/dev/serial0:921600`，`link[1000] opened successfully`，与 PX4 心跳连接恢复正常。

### 功能包修改记录
- 记录要求：本节必须按“功能包小标题”分组记录修改，后续新增记录不得混写在同一列表中。

#### return_landing
- 2026-03-08：在树莓派 `~/project/rasip_pi_ws/src` 同步新包 `return_landing`，并执行 `colcon build --symlink-install --packages-select return_landing` 构建通过。
- 2026-03-08：执行 `ros2 launch return_landing return_landing.launch.py` 启动验证无报错后停止。
- 2026-03-08：同步更新 `src/return_landing/return_landing/return_landing_node.py`（落地末段改为 `landing` 同款“最低油门下压 + 周期 disarm”兜底逻辑），并完成树莓派侧构建与启动验证。

#### camera_calibration_pkg
- 2026-03-08：登记实机 USB 摄像头标定参数文件：`src/camera_calibration_pkg/calibrationdata_extracted/ost.yaml`（作为后续实机视觉参数基准）。

#### 仓库同步
- 2026-03-08：树莓派 `~/project/rasip_pi_ws` 执行 `git fetch origin rasip_pi && git pull --ff-only origin rasip_pi`，同步到提交 `a6892b6`。

## 仿真记录

### 问题记录
- 2026-03-08：本轮无新增仿真操作记录。

### 修改记录
- 2026-03-08：本轮无新增仿真侧修改。

### 功能包修改记录
- 记录要求：本节必须按“功能包小标题”分组记录修改，后续新增记录不得混写在同一列表中。
- 2026-03-08：本轮无新增仿真功能包修改。

## 问题记录（本轮补充）
- 2026-03-08 | 问题：把树莓派工作空间相机参数切到实机 USB 参数，并把图像接口切到 USB。
  解答：已完成。默认图像话题改为 /image_raw；tvec 默认内参与畸变改为实机标定值；tvec.launch 默认启用 USB 相机并关闭 GZ bridge。

## 修改记录（本轮补充）
### 功能包修改记录
#### tvec
- 2026-03-08：更新 tvec_rvec_node.py 相机内参与畸变默认值为实机参数。
- 2026-03-08：更新 tvec.launch.py，新增 USB 相机参数并默认 use_usb_cam=true、use_gz_bridge=false。
- 2026-03-08：更新 tvec/package.xml，新增 usb_cam 运行依赖。

#### 多功能包 launch
- 2026-03-08：相关 launch 默认图像话题统一改为 /image_raw。

#### 构建验证
- 2026-03-08：树莓派 colcon build 通过。
- 2026-03-08：launch 验证显示当前环境缺少 usb_cam 包，需安装后可直接拉起 USB 相机。

## 问题记录（本轮补充）
- 2026-03-08 | 问题：已安装 usb_cam，要求进行实机链路测试。
  解答：已测试通过。USB 相机成功发布 /image_raw，aruco_tracking 整链 launch 可正常启动；此前的 numpy2 与 cv_bridge 兼容问题已消除。

## 修改记录（本轮补充）
### 功能包修改记录
#### 实机环境修复与验证
- 2026-03-08：修复 Python 环境：将用户态 numpy 调整到 1.26.4，解决 cv_bridge 报错 _ARRAY_API not found。
- 2026-03-08：将实机标定文件复制到 ~/.ros/camera_info/default_cam.yaml，usb_cam 启动时可读取相机参数文件。
- 2026-03-08：执行 ros2 launch tvec tvec.launch.py use_rviz:=false use_gz_bridge:=false use_usb_cam:=true 验证通过。
- 2026-03-08：执行 ros2 launch aruco_tracking aruco_tracking.launch.py ros_image_topic:=/image_raw 验证通过（流程节点均成功启动）。

## 问题记录（本轮补充）
- 2026-03-08 | 问题：将树莓派 rasip 工作空间内所有使用 GZ 相机的地方改为 USB 相机。
  解答：已完成。视觉主链路已移除 GZ 图像桥接节点，默认走 USB 相机并发布 /image_raw；回归启动不再出现 gz_bridge_image。

## 修改记录（本轮补充）
### 功能包修改记录
#### tvec
- 2026-03-08：重构 tvec.launch.py 为纯 USB 相机链路，删除 GZ 图像桥接节点。
- 2026-03-08：默认图像话题为 /image_raw，默认 use_usb_cam=true，保留 world/model 参数仅做上层接口兼容。
- 2026-03-08：tvec/package.xml 移除 ros_gz_bridge 依赖，保留 usb_cam 依赖。

#### tvec_tf 与 aruco_tracking
- 2026-03-08：上层 launch 默认 ros_image_topic 统一为 /image_raw。

#### 构建与验证
- 2026-03-08：树莓派执行 colcon build --packages-select tvec tvec_tf aruco_tracking 通过。
- 2026-03-08：执行 ros2 launch aruco_tracking aruco_tracking.launch.py use_rviz:=false 验证通过，日志仅出现 usb_cam_node_exe，不再出现 gz_bridge_image。

## 问题记录（本轮补充）
- 2026-03-08 | 问题：将树莓派工作空间所有 launch 中启动 RViz 的节点改为 rqt 显示 ArUco 标注图像。
  解答：已完成。当前 launch 文件中已无 rviz2 节点；统一改为 `rqt_image_view` 显示 `/tvec/image_annotated`。

## 修改记录（本轮补充）
### 功能包修改记录
#### tvec
- 2026-03-08：`tvec.launch.py` 中 RViz 节点替换为 `rqt_image_view` 节点（订阅参数 `annotated_image_topic`，默认 `/tvec/image_annotated`）。
- 2026-03-08：`use_rviz` 参数保留用于兼容，现用于控制 `rqt_image_view` 开关。
- 2026-03-08：`tvec/package.xml` 依赖从 `rviz2` 切换为 `rqt_image_view`。

#### 构建与验证
- 2026-03-08：树莓派执行 `colcon build --packages-select tvec tvec_tf aruco_tracking` 通过。
- 2026-03-08：安装 `ros-humble-rqt-image-view`，`ros2 pkg list` 已可查询到 `rqt_image_view`。
- 2026-03-08：全局检查 `src/*/launch/*.py` 已无 `rviz2` 节点声明。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：更新树莓派 SSH 地址。
  解答：已将工作区 `AGENTS.md` 中树莓派地址从 `172.24.134.110` 更新为 `10.250.57.110`。

## 修改记录（本轮补充）
### 工作空间与协作规范
- 2026-03-13：更新 SSH 连接信息：`zjh@10.250.57.110`（密码保持 `123`）。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：同步 `simple` 分支中的 4 个功能包到笔记本与树莓派，并构建测试到无报错。
  解答：已完成。`body_frame_tracking_minimal`、`body_frame_tracking`、`aruco_tracking_minimal`、`land_with_tracking` 已同步到两端并验证通过。

## 修改记录（本轮补充）
### 功能包修改记录
#### 从 simple 分支同步
- 2026-03-13：在 laptop 和树莓派工作空间执行 `git fetch origin simple`，并检出以下目录到 `rasip_pi_ws/src`：
  `body_frame_tracking_minimal`、`body_frame_tracking`、`aruco_tracking_minimal`、`land_with_tracking`。

#### 构建与验证
- 2026-03-13：laptop 执行 `colcon build --symlink-install --packages-select body_frame_tracking_minimal body_frame_tracking aruco_tracking_minimal land_with_tracking` 通过。
- 2026-03-13：树莓派执行同样构建命令通过。
- 2026-03-13：两端分别对 4 个 launch 执行限时启动验证，日志无 `ERROR`、`Traceback`、`process has died`。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：将新同步的 4 个功能包图像链路从 GZ bridge 切换为 USB 相机链路。
  解答：已完成。`aruco_tracking_minimal` 与 `body_frame_tracking_minimal` 已移除 `parameter_bridge`，统一复用 `tvec_tf/tvec` 的 USB 相机链路。

## 修改记录（本轮补充）
### 功能包修改记录
#### aruco_tracking_minimal
- 2026-03-13：`launch/aruco_tracking_minimal.launch.py` 改为包含 `tvec_tf.launch.py`；删除 `gz bridge + 本地 tvec + 本地 tvec_tf` 直接启动逻辑。
- 2026-03-13：新增 `use_rqt` 参数透传，默认图像话题改为 `/image_raw`。

#### body_frame_tracking_minimal
- 2026-03-13：`launch/body_frame_tracking_minimal.launch.py` 改为包含 `tvec_tf.launch.py`；删除 `gz bridge + 本地 tvec + 本地 tvec_tf` 直接启动逻辑。
- 2026-03-13：新增 `use_rqt` 参数透传，默认图像话题改为 `/image_raw`。

#### body_frame_tracking / land_with_tracking
- 2026-03-13：两个 launch 新增 `use_rqt` 参数透传至 `tvec_tf`，默认 `ros_image_topic` 统一为 `/image_raw`。

#### 构建与验证
- 2026-03-13：本地构建通过：`aruco_tracking_minimal`、`body_frame_tracking_minimal`、`body_frame_tracking`、`land_with_tracking`、`tvec`、`tvec_tf`。
- 2026-03-13：4 个 launch 限时启动验证通过，日志显示均由 `usb_cam_node_exe` 提供图像，未出现 `parameter_bridge`。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：禁用 `aruco_tracking_minimal` 与 `body_frame_tracking_minimal` 中无用的 `map_aruco_static_tf_node`，并将 rqt 默认关闭。
  解答：已完成。两个包已改为仅启动 `tvec` 与 `tvec_tf_node`，不再通过 `tvec_tf.launch.py` 拉起静态 TF；`use_rqt` 默认值为 `false`。

## 修改记录（本轮补充）
### 功能包修改记录
#### aruco_tracking_minimal / body_frame_tracking_minimal
- 2026-03-13：launch 结构由“包含 `tvec_tf.launch.py`”改为“包含 `tvec.launch.py` + 单独启动 `tvec_tf_node`”。
- 2026-03-13：默认参数 `use_rqt` 从 `true` 改为 `false`。

#### 构建与验证
- 2026-03-13：laptop 与树莓派分别构建通过：`aruco_tracking_minimal`、`body_frame_tracking_minimal`、`tvec`、`tvec_tf`。
- 2026-03-13：两端启动验证日志均确认：不再出现 `map_aruco_static_tf_node`、不再出现 `rqt_image_view`。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：两个 minimal 功能包启动时提示未收到 `/debug/tvec` 数据。
  解答：已定位为启动初期的正常等待提示。实测两包均可持续收到并发布 `aruco_pose`，视觉链路正常。

## 修改记录（本轮补充）
### 诊断记录
- 2026-03-13：在树莓派对 `aruco_tracking_minimal` 与 `body_frame_tracking_minimal` 分别抓取启动日志，确认 `tvec_tf_node` 连续输出 `aruco_pose`。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：将新建的 `pid_tuning` 功能包同时部署到树莓派并完成验证。
  解答：已完成。通过密码 SSH 通道同步 `src/pid_tuning` 到树莓派 `~/project/rasip_pi_ws/src`，并在树莓派执行构建与启动自检通过。

## 修改记录（本轮补充）
### 功能包修改记录
#### pid_tuning
- 2026-03-13：树莓派同步新增功能包目录 `src/pid_tuning`（含 `pid_tuning_node`、`pid_tuning_csv_logger_node`、`pid_tuning.launch.py`、`README.md`）。
- 2026-03-13：树莓派执行 `colcon build --symlink-install --packages-select pid_tuning` 构建通过。
- 2026-03-13：树莓派执行 `ros2 launch pid_tuning pid_tuning.launch.py use_rqt:=false kp_xy:=0.6 csv_prefix:=pi_pid_tuning_deploy_test` 限时自检通过，日志确认节点正常启动并输出参数来源。
- 2026-03-13：树莓派生成记录文件 `~/project/rasip_pi_ws/log/tracking_csv/pi_pid_tuning_deploy_test_20260313_210208.csv`，并已追加 `pid_tuning_summary.csv`。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：`pid_tuning` 启动时 `tvec_tf_node` 持续提示“尚未收到 /debug/tvec 数据”，且 `use_rqt:=true` 时无图像，要求在树莓派排查并修复。
  解答：已完成。树莓派实测确认修复后 `tvec_tf_node` 持续发布 `aruco_pose`；视觉链路恢复。`rqt` 需在 VNC 图形会话内启动才能显示窗口。

## 修改记录（本轮补充）
### 功能包修改记录
#### tvec
- 2026-03-13：`tvec_rvec_node.py` 新增 `image_qos_reliability` 参数，默认 `reliable`，用于适配 USB 相机图像订阅。
- 2026-03-13：`tvec.launch.py` 新增 `image_qos_reliability` launch 参数并传给检测节点。

#### pid_tuning
- 2026-03-13：`pid_tuning.launch.py` 新增并透传 USB 相机参数（`use_usb_cam/video_device/image_width/image_height/pixel_format/framerate`）与 `annotated_image_topic`、`image_qos_reliability`。

#### 构建与验证
- 2026-03-13：树莓派执行 `colcon build --symlink-install --packages-select tvec pid_tuning` 构建通过。
- 2026-03-13：树莓派执行 `ros2 launch pid_tuning pid_tuning.launch.py use_rqt:=false image_qos_reliability:=reliable` 验证通过，日志连续出现 `aruco_pose | ...`，不再停留在“尚未收到 /debug/tvec 数据”。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：`land_with_tracking` 不要启动静态 TF 发布节点与 rqt。
  解答：已完成。该包 launch 已切换为仅启动 `tvec` 与 `tvec_tf_node`，默认 `use_rqt=false`。

## 修改记录（本轮补充）
### 功能包修改记录
#### land_with_tracking
- 2026-03-13：`launch/land_with_tracking.launch.py` 从包含 `tvec_tf.launch.py` 改为包含 `tvec.launch.py` 并单独启动 `tvec_tf_node`。
- 2026-03-13：`use_rqt` 默认值改为 `false`。

#### 构建与验证
- 2026-03-13：laptop 与树莓派均完成 `colcon build --packages-select land_with_tracking tvec tvec_tf`。
- 2026-03-13：两端启动验证日志仅包含 `usb_cam_node_exe`、`tvec_rvec_node`、`tvec_tf_node`、`land_with_tracking_node`。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：两个 minimal 功能包的 CSV 文件存储到 `rasip_pi_ws/log`。
  解答：已完成，默认目录统一为 `/home/zjh/project/rasip_pi_ws/log/tracking_csv`。

## 修改记录（本轮补充）
### 功能包修改记录
#### aruco_tracking_minimal / body_frame_tracking_minimal
- 2026-03-13：`launch/*.launch.py` 中 `csv_output_dir` 默认值改为 `~/project/rasip_pi_ws/log/tracking_csv`。
- 2026-03-13：`csv_logger_node.py` 中 `output_dir` 参数默认值改为 `/home/zjh/project/rasip_pi_ws/log/tracking_csv`。
- 2026-03-13：README 中 CSV 默认路径与示例命令同步更新。

#### 构建与验证
- 2026-03-13：laptop 与树莓派均完成构建并启动验证，日志显示 CSV 输出路径已切换到 `rasip_pi_ws/log/tracking_csv`。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：在 launch 命令中如何调 PID 参数。
  解答：可使用 `--ros-args -p` 直接覆盖节点参数，已提供实测建议参数命令模板。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：`ros2 launch` 下用 `--ros-args -p` 修改 PID 失败。
  解答：已改为标准 launch 参数覆盖方案；现在可直接在命令中使用 `kp_xy:=...` 等参数。

## 修改记录（本轮补充）
### 功能包修改记录
#### body_frame_tracking_minimal / aruco_tracking_minimal
- 2026-03-13：两个 launch 新增 PID 与死区参数声明并传入控制节点，支持命令行直接覆盖。
- 2026-03-13：README 新增 PID 调参命令示例。

#### 构建与验证
- 2026-03-13：laptop 与树莓派均构建通过并验证 `kp_xy:=...` 参数命令可正常启动。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：分析多组 body_frame_tracking 实测 CSV，确定当前最优参数并给出下一步调试方向。
  解答：已完成。当前最佳组合为 `kpxy=1.0,kdxy=0.05`；高 `kdxy`（0.2）会显著放大抖动，降低到 0.05 后性能明显提升。

## 修改记录（本轮补充）
### 诊断记录
- 2026-03-13：完成 8 份 CSV 的统一量化评估与排序，输出下一步调参路径（优先围绕 `kpxy` 与 `kdxy` 做小步细化，后续再单独调 yaw 与死区）。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：对多组 PID 实验 CSV 进行作图对比，展示 tracking 优劣。
  解答：已完成，输出多维指标对比图与综合评分图。

## 修改记录（本轮补充）
### 诊断记录
- 2026-03-13：生成 `tracking_metrics_compare.png`、`tracking_composite_score.png`、`tracking_timeseries_key_runs.png`、`summary_metrics.tsv`。

## 问题记录（本轮补充）
- 2026-03-13 | 问题：将 30FPS 默认帧率改动同步到树莓派并完成测试验证。
  解答：已完成。树莓派已同步最新 `tvec/pid_tuning` launch 与检测节点代码，构建通过；`pid_tuning`、`aruco_tracking_minimal`、`body_frame_tracking_minimal` 三包启动日志均显示 `framerate: 30.000000`。

## 修改记录（本轮补充）
### 功能包修改记录
#### tvec / pid_tuning / minimal 链路同步
- 2026-03-13：树莓派同步文件：`src/tvec/launch/tvec.launch.py`、`src/tvec/tvec/tvec_rvec_node.py`、`src/pid_tuning/launch/pid_tuning.launch.py`。
- 2026-03-13：树莓派执行 `colcon build --symlink-install --packages-select tvec pid_tuning aruco_tracking_minimal body_frame_tracking_minimal` 构建通过。
- 2026-03-13：树莓派分别执行 3 个 launch 限时自检：
  1) `pid_tuning`
  2) `aruco_tracking_minimal`
  3) `body_frame_tracking_minimal`
  日志均确认 `framerate: 30.000000`（默认 30FPS 生效）。

## 问题记录（本轮补充）
- 2026-03-14 | 问题：将 `land_with_tracking` 的跟踪逻辑替换为 `body_frame_tracking_minimal` 同款，并先对齐 PID 参数；同时同步到树莓派并测试。
  解答：已完成。树莓派侧已同步新节点代码并通过构建与启动验证。

## 修改记录（本轮补充）
### 功能包修改记录
#### land_with_tracking
- 2026-03-14：`land_with_tracking_node.py` 的 `compute_tracking_cmd()` 改为 body-frame 跟踪逻辑：
  1) `ex=target_x-pose.x`
  2) `ey=pose.y-target_y`
  3) 不再进行 map 旋转和 `base_pose` 新鲜度门控。
- 2026-03-14：默认 PID 对齐为：`kp_track_xy=1.0`、`ki_track_xy=0.0`、`kd_track_xy=0.05`、`kp_yaw=1.10`、`ki_yaw=0.0`、`kd_yaw=0.05`。
- 2026-03-14：树莓派执行 `colcon build --symlink-install --packages-select land_with_tracking` 构建通过。
- 2026-03-14：树莓派执行 `ros2 launch land_with_tracking land_with_tracking.launch.py use_rqt:=false` 限时验证通过（节点正常启动，无报错）。

## 问题记录（本轮补充）
- 2026-03-14 | 问题：`pid_tuning_xy_fine_20260314_160112` 实验中进入 OFFBOARD 后高度下掉，要求定位原因并修复。
  解答：已完成。分析 CSV 后确认 OFFBOARD 段 `sp_vz` 长时间为 `-0.5` 且实机响应方向不符，定位为 z 方向速度发布符号问题；已修复并验证。

## 修改记录（本轮补充）
### 功能包修改记录
#### pid_tuning
- 2026-03-14：`pid_tuning_node.py` 修正 z 轴发布符号：`msg.velocity.z` 改为与内部 `vz_body_flu` 同号发布。
- 2026-03-14：树莓派执行 `colcon build --symlink-install --packages-select pid_tuning` 构建通过。
- 2026-03-14：树莓派执行 `ros2 launch pid_tuning pid_tuning.launch.py use_rqt:=false` 限时启动验证通过。

## 问题记录（本轮补充）
- 2026-03-14 | 问题：将树莓派 `log/tracking_csv` 中的 `3.14` 与 `summary` 移动到桌面 `trackingcsv` 文件夹。
  解答：已完成。`3.14` 目录和 `pid_tuning_summary.csv` 已移动到 `/home/zjh/桌面/trackingcsv`。

## 修改记录（本轮补充）
### 日志整理
- 2026-03-14：树莓派执行日志文件整理，将
  1) `/home/zjh/project/rasip_pi_ws/log/tracking_csv/3.14`
  2) `/home/zjh/project/rasip_pi_ws/log/tracking_csv/pid_tuning_summary.csv`
  移动到 `/home/zjh/桌面/trackingcsv/`。
- 2026-03-14：移动后源目录 `tracking_csv` 中不再残留 `3.14/summary` 相关项。

## 问题记录（本轮补充）
- 2026-03-14 | 问题：将树莓派桌面 `trackingcsv` 中的 `3.14` 和 `pid_tuning_summary.csv` 再移动到 laptop 桌面 `trackingcsv`。
  解答：已完成。通过 SSH 文件传输将两项拷贝到 laptop 后删除树莓派原件，移动成功。

## 修改记录（本轮补充）
### 日志整理
- 2026-03-14：执行跨设备日志迁移：
  1) `3.14` 目录：`/home/zjh/桌面/trackingcsv/3.14`（树莓派）-> `/home/zjh/桌面/trackingcsv/3.14`（laptop）
  2) `pid_tuning_summary.csv`：`/home/zjh/桌面/trackingcsv/pid_tuning_summary.csv`（树莓派）-> `/home/zjh/桌面/trackingcsv/pid_tuning_summary.csv`（laptop）
- 2026-03-14：校验结果：laptop 两项均存在，树莓派两项已删除。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：将 simple 分支中的最小 MAVROS 启动功能包同步到树莓派并构建。
  解答：已完成。通过 SSH 同步 `src/mavros_profiles` 到树莓派 `~/project/rasip_pi_ws/src`，并构建验证通过。

## 修改记录（本轮补充）
### 功能包修改记录
#### mavros_profiles
- 2026-03-15：树莓派接收同步目录：`~/project/rasip_pi_ws/src/mavros_profiles`。
- 2026-03-15：树莓派执行 `colcon build --symlink-install --packages-select mavros_profiles` 通过。
- 2026-03-15：树莓派执行 `source install/setup.bash` 后，`ros2 pkg list` 可见 `mavros_profiles`。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：将 `mavros_profiles` 串口默认值改为与树莓派 `mavros px4.launch` 一致。
  解答：已完成。树莓派 `px4.launch` 当前默认为 `/dev/ttyS0:921600`，`mavros_profiles` 已同步改为同值并构建通过。

## 修改记录（本轮补充）
### 功能包修改记录
#### mavros_profiles
- 2026-03-15：树莓派确认 `/opt/ros/humble/share/mavros/launch/px4.launch` 默认 `fcu_url=/dev/ttyS0:921600`。
- 2026-03-15：同步更新后的 `mavros_profiles` 到树莓派 `~/project/rasip_pi_ws/src/mavros_profiles`。
- 2026-03-15：树莓派执行 `colcon build --symlink-install --packages-select mavros_profiles` 通过。
- 2026-03-15：树莓派执行 `ros2 launch mavros_profiles workspace_minimal_px4.launch.py fcu_url:=udp://:14540@127.0.0.1:14557` 启动自检，MAVROS 正常拉起。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：树莓派 `tracking_csv` 中 log CSV 与 summary CSV 格式异常，要求仅修复 CSV 文件。
  解答：已完成。`~/project/rasip_pi_ws/log/tracking_csv` 下 CSV 已统一为 LF 行结束符，`pid_tuning_summary.csv` 已检查并保持单表头格式。

## 修改记录（本轮补充）
### 构建与验证
- 2026-03-15：在树莓派执行 CSV 一次性修复（统一行结束符、检查 summary 表头重复）。
- 2026-03-15：修复结果：`normalized=37`、`summary_dedup=0`、`files_with_CR=0`。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：树莓派 CSV 文件在 WPS/表格中全部显示在一列，要求按列分开。
  解答：已完成。原始 CSV 数据无损，新增“可直接分列打开”版本到 `~/project/rasip_pi_ws/log/tracking_csv/split_columns`。

## 修改记录（本轮补充）
### 构建与验证
- 2026-03-15：确认原始 `pid_tuning_summary.csv` 首行列数为 46，运行 CSV 首行列数为 48。
- 2026-03-15：生成 `split_columns` 输出：
  1) `*_excel.csv`（分号分隔、UTF-8 BOM，适配 WPS/Excel 自动分列）
  2) `*.tsv`（TAB 分隔）
- 2026-03-15：共转换 `17` 个 CSV，抽查输出分隔符格式正确。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：复制树莓派 `3.15` 和 `pid_tuning_summary.csv` 到 laptop 桌面。
  解答：已完成复制，源文件在树莓派保留不变。

## 修改记录（本轮补充）
### 诊断记录
- 2026-03-15：通过 `scp` 将 `~/project/rasip_pi_ws/log/tracking_csv/3.15` 与 `pid_tuning_summary.csv` 复制到 laptop `~/桌面`。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：将新建的 `dynamic_tracking` 同步到树莓派并完成构建。
  解答：已完成。`dynamic_tracking` 已同步到树莓派 `~/project/rasip_pi_ws/src`，构建与包识别验证通过。

## 修改记录（本轮补充）
### 功能包修改记录
#### dynamic_tracking
- 2026-03-15：树莓派接收同步目录 `~/project/rasip_pi_ws/src/dynamic_tracking`。
- 2026-03-15：树莓派执行 `colcon build --symlink-install --packages-select dynamic_tracking` 通过。
- 2026-03-15：树莓派执行 `source install/setup.bash` 后，`ros2 pkg list` 可见 `dynamic_tracking`。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：分析树莓派 `tracking_csv` 最新两个文件，并把反向控制量调回。
  解答：已完成。分析对象为 `pid_tuning_xy_fine_20260315_141429.csv` 与 `pid_tuning_xy_fine_20260315_141025.csv`，在 `OFFBOARD+aruco_fresh=1` 条件下，`aruco_y` 与 `sp_vy` 相关系数分别约 `-0.970/-0.972`（强负相关）。据此确认 `setpoint_raw/BODY_NED` 链路与 `cmd_vel` 链路语义不同；已将 `land_with_tracking` 的 `cmd_vel.y` 改回同号发布（取消取反），并同步到树莓派构建通过。

## 修改记录（本轮补充）
### 功能包修改记录
#### land_with_tracking
- 2026-03-15：将 `~/project/rasip_pi_ws/src/land_with_tracking/land_with_tracking/land_with_tracking_node.py` 同步到树莓派。
- 2026-03-15：树莓派侧恢复 `publish_cmd()` 的 y 通道为同号发布：`msg.twist.linear.y = vy`。
- 2026-03-15：树莓派执行 `colcon build --packages-select land_with_tracking --symlink-install` 通过，并完成 `source install/setup.bash`。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：将新建 `track_with_land_v2` 同步到树莓派并构建。
  解答：已完成。`track_with_land_v2` 已同步到树莓派 `~/project/rasip_pi_ws/src`，构建与环境加载均通过。

## 修改记录（本轮补充）
### 功能包修改记录
#### track_with_land_v2
- 2026-03-15：树莓派接收同步目录 `~/project/rasip_pi_ws/src/track_with_land_v2`。
- 2026-03-15：树莓派执行 `colcon build --packages-select track_with_land_v2 --symlink-install` 通过。
- 2026-03-15：树莓派执行 `source install/setup.bash`，构建结果可用。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：根据树莓派最新 CSV 观察到机体系左右误差发散，要求修正代码。
  解答：已完成。树莓派最新 CSV 分析后，已将 `track_with_land_v2` 的 y 通道发布符号调整为取反，并在树莓派构建通过。

## 修改记录（本轮补充）
### 功能包修改记录
#### track_with_land_v2
- 2026-03-15：树莓派接收更新文件 `~/project/rasip_pi_ws/src/track_with_land_v2/track_with_land_v2/track_with_land_v2_node.py`。
- 2026-03-15：`publish_cmd()` 中 y 通道调整为 `msg.twist.linear.y = -vy`。
- 2026-03-15：树莓派执行 `colcon build --packages-select track_with_land_v2 --symlink-install` 通过，并完成 `source install/setup.bash`。

## 问题记录（本轮补充）
- 2026-03-15 | 问题：分析 `pid_tuning_xy_fine_20260315_155129`（target_z=1m）与 2.5m 对比，判断精度与丢码原因。
  解答：已完成。1m 组在有码阶段误差更小，但 `fresh_ratio` 下降且存在长时连续丢码，说明主要问题是视觉可见性与重捕获，而不是纯 PID 精度不足。

## 修改记录（本轮补充）
### 诊断记录
- 2026-03-15：树莓派侧完成 CSV 指标计算与丢码段统计：`offboard_rows=4109`、`eval_rows=2894`、`fresh_ratio≈0.704`、`max_stale≈10.97s`。

## 问题记录（本轮补充）
- 2026-03-16 | 问题：将 `land_with_tracking_v2` 和 `yaw_then_xy_tracking` 部署到树莓派并完成构建。
  解答：已完成。两包已同步到树莓派工作空间并构建通过。

## 修改记录（本轮补充）
### 功能包修改记录
#### land_with_tracking_v2 / yaw_then_xy_tracking
- 2026-03-16：树莓派接收同步目录：
  1) `~/project/rasip_pi_ws/src/land_with_tracking_v2`
  2) `~/project/rasip_pi_ws/src/yaw_then_xy_tracking`
- 2026-03-16：树莓派执行 `colcon build --packages-select land_with_tracking_v2 yaw_then_xy_tracking --symlink-install` 通过。
- 2026-03-16：树莓派执行 `source install/setup.bash` 完成环境更新。
