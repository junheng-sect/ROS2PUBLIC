# tvec

用于分析 ArUco 检测输出 `tvec/rvec` 与坐标系关系的测试功能包。

## 功能
- 订阅相机图像并检测 ArUco。
- 计算并输出每次检测的相对位姿 `tvec/rvec`。
- 以 **1Hz** 日志输出最新 `tvec/rvec`。
- 新增 `rvec_yaw_node`：从 `rvec` 计算相对 `yaw` 并以 **1Hz** 日志输出。
- 将 `tvec/rvec` 按时间戳写入 CSV。
- 可选发布标注图像到 `/tvec/image_annotated`，并在 RViz 显示。

## 启动
```bash
cd ~/project/rasip_pi_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tvec tvec.launch.py camera_profile:=old_cam
ros2 launch tvec tvec.launch.py camera_profile:=icspring_1080
ros2 launch tvec tvec.launch.py camera_profile:=icspring_640x480
ros2 launch tvec tvec.launch.py camera_profile:=icspring_800x600
```

### 最新启动命令

常用启动命令：

新相机实机启动：

```bash
ros2 launch tvec tvec.launch.py \
  camera_profile:=icspring_1080 \
  use_rqt:=false
```

旧相机实机启动：

```bash
ros2 launch tvec tvec.launch.py \
  camera_profile:=old_cam \
  use_rqt:=false
```

关闭 USB 相机、复用外部图像流：

```bash
ros2 launch tvec tvec.launch.py \
  camera_profile:=icspring_1080 \
  use_usb_cam:=false \
  use_rqt:=false \
  ros_image_topic:=/image_raw
```

仅保留位姿解算，不生成标注图：

```bash
ros2 launch tvec tvec.launch.py \
  camera_profile:=icspring_1080 \
  publish_annotated_image:=false \
  use_rqt:=false
```

可选参数：
- `world_name`：默认 `aruco`
- `model_name`：默认 `x500_mono_cam_down_0`
- `ros_image_topic`：默认 `/image_raw`
- `camera_profile`：默认 `old_cam`
- `publish_annotated_image`：默认 `true`，设为 `false` 后关闭画框、叠字和 `/tvec/image_annotated` 发布

## 相机 Profile

`camera_profile` 用于统一切换 `usb_cam` 与 `tvec_rvec_node` 使用的相机参数，配置文件位于：

- `src/tvec/config/camera_profiles.yaml`

当前内置 profile：

- `old_cam`：保留旧相机的 `640x480` 参数与历史内参。
- `icspring_1080`：使用新相机 `1920x1080` 标定结果与 `30 FPS` 建议配置。
- `icspring_640x480`：使用新相机 `640x480` 标定结果与 `30 FPS` 建议配置。
- `icspring_800x600`：使用新相机 `800x600` 标定结果与 `30 FPS` 建议配置。

如果只想覆盖 profile 中的个别字段，仍可在 launch 命令里显式传参，例如：

```bash
ros2 launch tvec tvec.launch.py \
  camera_profile:=icspring_1080 \
  video_device:=/dev/video2
```

如果主要只关心 `/debug/tvec` 和后续位姿链，可直接传：

```bash
ros2 launch tvec tvec.launch.py \
  camera_profile:=icspring_1080 \
  publish_annotated_image:=false
```

后续若要新增第三台相机，请直接在 `src/tvec/config/camera_profiles.yaml` 中追加一个新 profile；主代码无需再分叉。

单独启动 yaw 计算节点：
```bash
cd ~/project/rasip_pi_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run tvec rvec_yaw_node
```

## 日志与CSV
- 1Hz 日志示例：
  - `tvec=(tx, ty, tz)`
  - `rvec=(rx, ry, rz)`
- CSV 默认目录：
  - `~/project/zjh_ws/src/tvec/log/`

## 时序埋点

`/debug/tvec` 现在除原有 `header/tvec/rvec` 外，还会透传第 2 级内部时间戳：

- `tvec_cb_start_stamp`
- `tvec_cv_bridge_done_stamp`
- `tvec_detect_done_stamp`
- `tvec_pose_done_stamp`
- `tvec_pub_stamp`

语义约定：

- `header.stamp` 仍然保持原始 `/image_raw.header.stamp`
- 单帧内若检测到多个 marker，帧级时间戳可以相同
- `tvec_pose_done_stamp` 和 `tvec_pub_stamp` 对应当前这条实际发布的数据

新增原始图像 timing 探针话题：

- `/debug/image_raw_timing`

该话题由 `image_raw_timing_probe_node` 发布，作用是继续拆解
`tvec_queue_sec = tvec_cb_start_stamp - image_header_stamp`。

当前会额外给出：

- `image_source_stamp`
  - 来自 DDS 元信息 `source_timestamp`
  - 可近似视为 `usb_cam` 调用 `publish()` 的时刻
- `image_received_stamp`
  - 来自 DDS 元信息 `received_timestamp`
  - 表示探针订阅端真正取到这条图像消息的时刻
- `image_probe_pub_stamp`
  - 探针节点发布 `/debug/image_raw_timing` 的时刻

这样后续可把原先的 `tvec_queue_sec` 进一步拆成：

- `image_header_to_source_sec = image_source_stamp - image_header_stamp`
- `image_source_to_tvec_cb_sec = tvec_cb_start_stamp - image_source_stamp`

## 最新帧优先处理

`tvec_rvec_node` 当前已改成“回调只缓存最新图像，检测逻辑只处理当前最新帧”的结构：

- `/image_raw` 订阅回调只记录 `tvec_cb_start_stamp` 并覆盖最新缓存
- ArUco 检测与位姿估计在独立定时处理逻辑中执行
- 若检测尚未完成时又来了新帧，旧缓存会被新帧覆盖，不再按历史顺序补处理全部旧帧

这样做的目的不是提高每帧检测算力，而是优先减少“吃旧帧”造成的链路总延迟。

## 目的
用于彻底理清 `tvec/rvec` 与相机坐标系、标记坐标系之间的对应关系，便于后续 TF 设计与控制器建模。
