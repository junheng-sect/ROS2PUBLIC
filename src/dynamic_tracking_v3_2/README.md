# dynamic_tracking_v3_2

`dynamic_tracking_v3` 改进版，基于代码审查建议进行了以下优化：

## 与 v3 的差异

| 改进项 | 说明 |
|--------|------|
| ROS 单调时钟注入 | `PIDController`、`ScalarLowPassFilter`、`AngleLowPassFilter`、`SlewRateLimiter` 全部改用 ROS 单调时钟（`clock.now()`）替代 `time.time()`，避免 NTP 校时跳变导致 dt 异常 |
| PID D-term 保护 | 首次 `update()` 或 `reset()` 后，当 `dt <= 1e-6` 或 `dt > 1.0` 时跳过微分与积分累积，只输出比例项，防止尖峰 |
| 公共函数抽取 | `wrap_to_pi`、`parameter_as_bool`、`rotate_marker_error_to_body` 集中到 `_common.py`，控制节点与 CSV logger 共用，不再重复定义 |
| yaw_rate 符号参数化 | 新增 `invert_yaw_rate_output` 参数（默认 true），替代硬编码的 `-` 号 |
| 平滑器按需重置 | 新增 `_was_smoothing_enabled` 标志位，仅在平滑开关切换时触发 reset，消除每周期无效调用 |
| `publish_body_velocity` 拆分 | 拆出 `publish_pipeline_timing()` 独立发布管线延迟消息 |
| OrderedDict FIFO 淘汰 | `image_raw_timing_by_key` 改用 `collections.OrderedDict` + `popitem(last=False)`，替代手动 list+dict 组合 |
| `_write_row` 拆分 | 拆为 `_extract_aruco_data()`、`_extract_local_data()`、`_extract_setpoint_data()`、`_extract_timing_data()`、`_compute_timing_derived()` 五个方法 |

## 启动

### 前置准备

```bash
# 本机
source /home/zjh/project/rasip_pi_ws/install/setup.bash
# 树莓派
source /home/zjh/rasip_pi_ws/install/setup.bash
```

### 快速启动（使用默认值）

```bash
ros2 launch dynamic_tracking_v3_2 dynamic_tracking_v3_2.launch.py
```

### 调参入口（完整参数透传）

```bash
source /home/zjh/project/rasip_pi_ws/install/setup.bash

ros2 launch dynamic_tracking_v3_2 dynamic_tracking_v3_2.launch.py \
  camera_profile:=icspring_640x480 \
  use_rqt:=false \
  use_usb_cam:=true \
  ros_image_topic:=/image_raw \
  enable_csv_logger:=true \
  target_x:=0.0 \
  target_y:=0.0 \
  target_z:=1.0 \
  target_yaw:=0.0 \
  kp_xy:=0.5 ki_xy:=0.0 kd_xy:=0.08 \
  kp_z:=0.5 ki_z:=0.0 kd_z:=0.03 \
  kp_yaw:=0.4 ki_yaw:=0.0 kd_yaw:=0.03 \
  v_limit:=0.8 \
  vz_limit:=0.3 \
  yaw_rate_limit:=0.4 \
  velocity_deadband:=0.03 \
  control_rate_hz:=30.0 \
  pose_timeout_sec:=0.5 \
  require_offboard:=true \
  enable_z_hold:=true \
  enable_z_yaw_smoothing:=true \
  z_lpf_tau_sec:=0.35 \
  yaw_lpf_tau_sec:=0.35 \
  z_error_deadband:=0.04 \
  yaw_error_deadband:=0.04 \
  vz_slew_rate_limit:=0.30 \
  yaw_rate_slew_rate_limit:=0.35 \
  camera_yaw_compensation_deg:=0.0 \
  invert_yaw_rate_output:=true
```

### 分轴精调 XY（可选）

当 `kp_x`/`ki_x`/`kd_x` 中任一设为数值（非 nan）时，X 轴自动切换到分轴参数，忽略 `kp_xy`/`ki_xy`/`kd_xy`。Y 轴同理。

```bash
ros2 launch dynamic_tracking_v3_2 dynamic_tracking_v3_2.launch.py \
  camera_profile:=icspring_640x480 use_rqt:=false \
  target_x:=0.0 target_y:=0.0 target_z:=2.5 target_yaw:=0.0 \
  kp_x:=0.6 ki_x:=0.0 kd_x:=0.08 \
  kp_y:=0.5 ki_y:=0.0 kd_y:=0.08 \
  v_limit:=0.8 yaw_rate_limit:=0.4
```

### 关闭 USB 相机、复用外部图像流

```bash
ros2 launch dynamic_tracking_v3_2 dynamic_tracking_v3_2.launch.py \
  camera_profile:=icspring_1080 \
  use_usb_cam:=false \
  use_rqt:=false \
  ros_image_topic:=/camera/image_raw
```

## 全部可调参数一览

### 相机与图像
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `camera_profile` | `old_cam` | 相机 profile：`old_cam` / `icspring_1080` / `icspring_640x480` / `icspring_800x600` |
| `use_usb_cam` | `true` | 是否启动 usb_cam 节点 |
| `ros_image_topic` | `/image_raw` | 图像输入话题 |
| `use_rqt` | `false` | 是否启动 rqt_image_view |
| `publish_annotated_image` | `false` | 是否发布 ArUco 标注图像 |

### 控制目标
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_x` | 0.0 | 目标 X（机体前向，m） |
| `target_y` | 0.0 | 目标 Y（机体左向，m） |
| `target_z` | 2.5 | 目标 Z（高度，m） |
| `target_yaw` | 0.0 | 目标偏航角（rad） |

### XY PID
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kp_xy` / `ki_xy` / `kd_xy` | 0.5 / 0.0 / 0.08 | XY 统一粗调增益 |
| `kp_x` / `ki_x` / `kd_x` | nan / nan / nan | X 轴分轴增益（覆盖 kp_xy） |
| `kp_y` / `ki_y` / `kd_y` | nan / nan / nan | Y 轴分轴增益（覆盖 kp_xy） |

### Z PID
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kp_z` / `ki_z` / `kd_z` | 0.5 / 0.0 / 0.03 | Z 轴增益 |
| `enable_z_hold` | `true` | 是否启用 Z 轴高度闭环 |

### Yaw PID
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kp_yaw` / `ki_yaw` / `kd_yaw` | 0.4 / 0.0 / 0.03 | 偏航轴增益 |
| `camera_yaw_compensation_deg` | 0.0 | 相机安装偏角补偿（度） |
| `invert_yaw_rate_output` | `true` | 是否反转 yaw_rate 输出符号 |

### 安全限幅
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `v_limit` | 0.8 | XY 水平速度幅值上限（m/s） |
| `vz_limit` | 0.3 | Z 轴速度上限（m/s） |
| `yaw_rate_limit` | 0.4 | 偏航角速度上限（rad/s） |
| `velocity_deadband` | 0.03 | 速度死区（m/s 或 rad/s） |
| `control_rate_hz` | 30.0 | 控制循环频率（Hz） |
| `pose_timeout_sec` | 0.5 | 视觉位姿超时阈值（s） |
| `require_offboard` | `true` | 是否仅在 OFFBOARD 模式下输出 |

### Z / Yaw 平滑
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_z_yaw_smoothing` | `true` | 是否启用 Z/Yaw 低通滤波与斜率限制 |
| `z_lpf_tau_sec` | 0.35 | Z 低通时间常数（s） |
| `yaw_lpf_tau_sec` | 0.35 | Yaw 低通时间常数（s） |
| `z_error_deadband` | 0.04 | Z 误差死区（m） |
| `yaw_error_deadband` | 0.04 | Yaw 误差死区（rad） |
| `vz_slew_rate_limit` | 0.30 | Vz 斜率限制（m/s²） |
| `yaw_rate_slew_rate_limit` | 0.35 | Yaw rate 斜率限制（rad/s²） |

### CSV 日志
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_csv_logger` | `true` | 是否启用 CSV 调参记录 |

## 参数兼容性

除新增 `invert_yaw_rate_output` 外，所有参数名和默认值与 v3 完全一致，可直接替换 launch 调用。
