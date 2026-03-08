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
