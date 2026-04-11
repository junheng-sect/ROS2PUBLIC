# altitude_measure

用于记录飞行过程中的三路高度数据到独立 CSV 文件，便于后续画图对比。

- `distance sensor`：`sensor_msgs/msg/Range.range`
- `GPS 高度`：`sensor_msgs/msg/NavSatFix.altitude`
- `气压计高度`：`mavros_msgs/msg/Altitude`（字段可配置）

## 运行

```bash
ros2 run altitude_measure altitude_measure_node
```

## 主要参数

- `distance_sensor_topic`（默认：`/mavros/hrlv_ez4_pub`）
- `gps_topic`（默认：`/mavros/global_position/global`）
- `barometer_topic`（默认：`/mavros/altitude`）
- `barometer_field`（默认：`amsl`）
  可选值：`monotonic` / `amsl` / `local` / `relative` / `terrain` / `bottom_clearance`
- `output_dir`（默认：`/home/zjh/project/rasip_pi_ws/log/altitude_measure`）
- `file_prefix`（默认：`altitude_measure`）
- `flush_interval_sec`（默认：`1.0`）

## 输出文件

节点每次启动会在 `output_dir` 下创建一个本次运行目录：

- `<file_prefix>_<时间>/`

并在该目录中生成 3 个文件：

- `distance_sensor.csv`
- `gps_altitude.csv`
- `barometer_<barometer_field>.csv`

每个 CSV 仅包含两列：

- `timestamp_sec`
- 对应高度列（单位米）
