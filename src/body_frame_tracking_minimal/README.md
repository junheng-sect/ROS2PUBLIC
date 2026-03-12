# body_frame_tracking_minimal

`body_frame_tracking_minimal` 提供最小可用组合，不改动原有功能包：

- `gz_bridge_image`（仿真图像桥接）
- `tvec_rvec_node`（Aruco 检测）
- `tvec_tf_node`（生成 `/debug/aruco_pose`）
- `body_frame_tracking_node`（机体坐标系控制）

不包含 RViz、额外 TF 广播和辅助日志节点，以减少性能开销。

## 运行

```bash
ros2 launch body_frame_tracking_minimal body_frame_tracking_minimal.launch.py
```

## CSV 记录

默认会同时启动 `body_frame_tracking_csv_logger_node`，自动记录调参数据并输出 CSV：

- 输出目录默认：`/home/zjh/project/zjh_ws/log/tracking_csv`
- 文件名前缀默认：`body_frame_tracking_minimal`
- 记录频率默认：`30Hz`

可按需修改：

```bash
ros2 launch body_frame_tracking_minimal body_frame_tracking_minimal.launch.py \
  enable_csv_logger:=true \
  csv_output_dir:=/home/zjh/project/zjh_ws/log/tracking_csv \
  csv_prefix:=body_pid_test \
  csv_sample_rate_hz:=30.0
```
