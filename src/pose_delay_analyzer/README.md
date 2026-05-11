# pose_delay_analyzer

`pose_delay_analyzer` 用于在实验期间单命令启动最小视觉链并在线记录：

- `local pose`：`/mavros/local_position/pose`
- `distance sensor`：`/mavros/hrlv_ez4_pub`
- `visual pose`：`/debug/aruco_pose`

节点退出时会自动输出原始 CSV、reference/visual 对比图、大动作边沿检测结果和
Markdown 实验报告。

## 功能说明

- 默认自带最小视觉链：
  - `tvec.launch.py`
  - `tvec_tf_node`
  - `pose_delay_analyzer_node`
- 默认不启动 MAVROS；要求外部已有：
  - `/mavros/local_position/pose`
  - `/mavros/hrlv_ez4_pub`
- 默认分析轴仍为 `z`，但 `reference(z)` 已切换为 `distance sensor`
- `reference(x/y)` 仍来自 `local pose`

## 输入、时间戳与 reference 口径

### 原始输入话题

- `/mavros/local_position/pose`：`geometry_msgs/msg/PoseStamped`
- `/mavros/hrlv_ez4_pub`：`sensor_msgs/msg/Range`
- `/debug/aruco_pose`：`debug_interface/msg/ArucoBasePose`

### 时间戳口径

- `local pose`：使用 `PoseStamped.header.stamp`
- `distance sensor`：使用 `Range.header.stamp`
- `reference(z)`：使用 `Range.header.stamp`，高度值为 `Range.range`
- `visual`：使用 `/debug/aruco_pose.tf_pub_stamp` 作为 `stamp_sec`
- `visual` 的原始图像时间戳仍保留为 `header_stamp_sec`，仅用于诊断
- 若 `tf_pub_stamp <= 0`，`visual.stamp_sec` 回退为 analyzer 回调接收时刻
  `rx_stamp_sec`，并在 `visual_stamp_source` 中标记 `fallback_rx_stamp`

### reference 构造规则

- `analysis_axis:=z`
  - `reference` 以 `distance sensor` 为主
  - `reference.stamp = Range.header.stamp`
  - `reference.z = Range.range`
  - `reference.x/reference.y` 取最近且时间不晚于该测距时刻的
    `local pose`
  - `reference.csv` 同时保留 `local_pose_z` 与 `distance_sensor_z`
- `analysis_axis:=x/y`
  - `reference` 以 `local pose` 为主
  - `distance sensor` 只作为附加列记录，不参与默认结论

### distance sensor 有效性

- `Range.range` 必须为有限数
- 默认忽略 `min_range`，只校验 `Range.range <= max_range`
- 仅有效样本参与 `analysis_axis:=z` 时的 `reference` 构造与事件检测
- 原始 `distance_sensor.csv` 仍保留全量样本，并输出：
  - `valid`
  - `status`
  - `gap_sec_from_prev`
  - `gap_exceeds_timeout`

## 输出内容

每次启动都会在 `output_dir` 下创建一个带时间戳的会话目录，默认路径为：

```text
/home/zjh/project/rasip_pi_ws/log/pose_delay/pose_delay_YYYYMMDD_HHMMSS/
```

目录中默认包含：

- `pose_delay_local_pose.csv`：原始 local pose 样本
- `pose_delay_distance_sensor.csv`：原始 distance sensor 样本
- `pose_delay_reference.csv`：用于延迟分析的 reference 流
- `pose_delay_visual.csv`：原始 visual pose 样本，`stamp_sec` 为分析使用时间
  （优先 `tf_pub_stamp`），并附带 `header_stamp_sec`、`tf_pub_stamp_sec`、
  `rx_stamp_sec`、`visual_stamp_source` 诊断列
- `pose_delay_reference_events.csv`：reference 侧大动作边沿
- `pose_delay_visual_events.csv`：visual 侧大动作边沿
- `pose_delay_matches.csv`：visual/reference 大动作边沿匹配与延迟结果
- `pose_delay_plot.png`：reference 与 visual 对比图
- `pose_delay_report.md`：Markdown 实验报告

## 启动

### 1. 构建

```bash
cd /home/zjh/project/rasip_pi_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pose_delay_analyzer
source install/setup.bash
```

### 2. 先确保 MAVROS 已运行

`pose_delay_analyzer` 不负责启动 MAVROS，实验前请先保证下面两个话题存在：

- `/mavros/local_position/pose`
- `/mavros/hrlv_ez4_pub`

### 3. 启动记录与最小视觉链

```bash
ros2 launch pose_delay_analyzer pose_delay_analyzer.launch.py
```

树莓派实机推荐：

```bash
ros2 launch pose_delay_analyzer pose_delay_analyzer.launch.py \
  camera_profile:=icspring_1080
```

如果你需要切换到其他已标定分辨率，可直接使用对应 profile：

800x600：

```bash
ros2 launch pose_delay_analyzer pose_delay_analyzer.launch.py \
  camera_profile:=icspring_800x600
```

640x480：

```bash
ros2 launch pose_delay_analyzer pose_delay_analyzer.launch.py \
  camera_profile:=icspring_640x480
```

720P：

当前仓库里还没有独立的 `1280x720` 标定 profile，因此暂时不要直接用
`icspring_1080` 去强行跑 `1280x720`。如果后续补充了 `icspring_1280x720`
标定，可使用下面的命令：

```bash
ros2 launch pose_delay_analyzer pose_delay_analyzer.launch.py \
  camera_profile:=icspring_1280x720
```

实验结束后按 `Ctrl+C` 停止节点，节点会在退出阶段自动执行分析与报告生成。

## 常用参数

- `enable_visual_chain`：默认 `true`
- `local_pose_topic`：默认 `/mavros/local_position/pose`
- `distance_sensor_topic`：默认 `/mavros/hrlv_ez4_pub`
- `visual_pose_topic`：默认 `/debug/aruco_pose`
- `distance_sensor_timeout_sec`：默认 `0.5`
- `output_dir`：默认 `/home/zjh/project/rasip_pi_ws/log/pose_delay`
- `file_prefix`：默认 `pose_delay`
- `analysis_axis`：默认 `z`，可选 `x/y/z`
- `min_samples`：默认 `30`
- `smooth_window_sec`：默认 `0.20`
- `slope_threshold`：默认 `0.05`
- `merge_gap_sec`：默认 `0.30`
- `major_edge_merge_gap_sec`：默认 `1.0`
- `major_edge_min_amplitude`：默认 `0.08`
- `major_edge_max_match_offset_sec`：默认 `2.0`
- `plot_relative_time`：默认 `true`
- `auto_report`：默认 `true`
- `camera_profile`：默认 `old_cam`
- `use_usb_cam`：默认 `true`
- `publish_annotated_image`：默认 `false`
- `use_rqt`：默认 `false`

示例：

```bash
ros2 launch pose_delay_analyzer pose_delay_analyzer.launch.py \
  camera_profile:=icspring_1080 \
  slope_threshold:=0.08 \
  file_prefix:=hover_test
```

如果你已经在外部启动了视觉链，可关闭包内视觉链，只保留 analyzer：

```bash
ros2 launch pose_delay_analyzer pose_delay_analyzer.launch.py \
  enable_visual_chain:=false
```

## 延迟判定口径

1. 时间戳口径固定为：
   - `reference(z)`：`Range.header.stamp`
   - `visual`：`/debug/aruco_pose.tf_pub_stamp`
   - `local pose`：`PoseStamped.header.stamp`
   - 若 `visual.tf_pub_stamp <= 0`，仅对该视觉样本回退到 `rx_stamp_sec`
2. 默认以最早有效样本时刻作为 `t=0`，统一映射到同一时间轴。
3. 对 `reference` 和 `visual` 的分析轴做时间窗平滑。
4. 对平滑序列做一阶差分，得到速度近似值。
5. 当 `abs(速度) >= slope_threshold` 时记为进入低层碎事件区间。
6. 若相邻超阈值片段间隔不超过 `merge_gap_sec`，则先合并为同一个低层碎事件。
7. 对低层碎事件做二次聚合：
   - 仅合并同方向碎事件
   - 仅合并时间相邻碎事件
   - 相邻碎事件间隔不超过 `major_edge_merge_gap_sec`
   - 聚合后幅值需满足 `abs(end_smooth - start_smooth) >= major_edge_min_amplitude`
8. 每个大动作边沿使用平滑曲线的 50% 中点穿越时刻作为单次延迟时间戳。
9. `reference` 和 `visual` 按 `rise/fall` 分方向、按时间顺序、一一匹配：
   - 只允许同方向匹配
   - 不允许跨周期回跳
   - 仅保留 `abs(delay_sec) <= major_edge_max_match_offset_sec` 的匹配
10. 单次延迟计算为：

```text
delay_sec = visual_mid_stamp_sec - reference_mid_stamp_sec
```

其中：

- `delay_sec > 0`：视觉大动作边沿的中点时刻晚于 reference，说明视觉链更滞后
- `delay_sec < 0`：视觉大动作边沿的中点时刻早于 reference
- `mean_delay`：对所有成功匹配的大动作边沿延迟直接求平均

## 注意事项

- `analysis_axis:=z` 时，主结论基于 `distance sensor`，不是
  `local_position.pose.z`
- `pose_delay_visual.csv` 的 `stamp_sec` 不是原始图像时间戳，而是视觉位姿实际
  发布时刻；原始图像时间戳请看 `header_stamp_sec`
- 近距离样本若小于消息里的 `min_range`，会标记为
  `valid_ignore_min_range`，但仍会进入 `reference(z)` 分析
- `reference.csv` 用来明确记录分析真实使用的数据来源
- `*_events.csv` 输出的是“大动作边沿”，不是低层碎事件
- 若 `reference` 或 `visual` 样本数小于 `min_samples`，报告会标记“样本不足”，
  但仍保留原始 CSV 和图像
- 为减轻树莓派负载，默认关闭标注图像发布和 `rqt`
