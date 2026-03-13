# aruco_tracking_minimal

`aruco_tracking_minimal` 提供最小可用组合，不改动原有功能包：

- `gz_bridge_image`（仿真图像桥接）
- `tvec_rvec_node`（Aruco 检测）
- `tvec_tf_node`（生成 `/debug/aruco_pose`）
- `aruco_tracking_node`（控制）

不包含 RViz、额外 TF 广播和辅助日志节点，以减少性能开销。

## 运行

```bash
ros2 launch aruco_tracking_minimal aruco_tracking_minimal.launch.py
```

PID 参数可在 launch 命令直接覆盖，例如：

```bash
ros2 launch aruco_tracking_minimal aruco_tracking_minimal.launch.py \
  use_rqt:=false \
  kp_xy:=0.45 ki_xy:=0.0 kd_xy:=0.10 \
  kp_yaw:=1.10 ki_yaw:=0.0 kd_yaw:=0.10 \
  kp_z_hold:=0.80 ki_z_hold:=0.0 kd_z_hold:=0.06 \
  velocity_deadband:=0.04 yaw_rate_deadband:=0.03
```

## CSV 记录

默认会同时启动 `aruco_tracking_csv_logger_node`，自动记录调参数据并输出 CSV：

- 输出目录默认：`~/project/rasip_pi_ws/log/tracking_csv`
- 文件名前缀默认：`aruco_tracking_minimal`
- 记录频率默认：`30Hz`

可按需修改：

```bash
ros2 launch aruco_tracking_minimal aruco_tracking_minimal.launch.py \
  enable_csv_logger:=true \
  csv_output_dir:=/home/zjh/project/rasip_pi_ws/log/tracking_csv \
  csv_prefix:=aruco_pid_test \
  csv_sample_rate_hz:=30.0
```
