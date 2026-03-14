# simple / rasip_pi 日志常见问题总结

## 说明

- 本文基于以下日志归纳：
- `origin/simple:LOG.md`
- `origin/simple:questions/LOG.md`
- `origin/rasip_pi:LOG.md`
- `origin/rasip_pi:PI_LOG.md`
- 备注：远程未发现 `rasip` 分支，实际对应为 `rasip_pi`。

## 高频问题总览

| 问题类别 | 典型现象 | 常见根因 | 优先修复动作 |
|---|---|---|---|
| QoS 不匹配 | 订阅不到 MAVROS/状态话题，节点无数据但无报错 | 发布端与订阅端 QoS（`reliability/durability`）不一致 | MAVROS 相关订阅优先用 `BEST_EFFORT + VOLATILE`；`home` 等 latched 数据考虑 `TRANSIENT_LOCAL` |
| MAVROS 前缀不一致 | 控制节点“看起来在跑”但完全不响应 | 运行环境实际前缀是 `/uas1/mavros/...`，代码写死 `/mavros/...` | 全部话题前缀参数化，启动前先 `ros2 topic list | rg mavros` |
| 仿真到实机相机迁移失败 | launch 正常但无图像/无检测 | 仍在用 `gz bridge`，未切换 USB 图像链路；`camera_info` 未同步 | 去掉 `gz_bridge_image`，统一改为 `/image_raw` + 有效 `camera_info` |
| USB 相机设备与格式问题 | `Invalid v4l2 format`、随机打不开相机 | `/dev/videoX` 漂移，像素格式不兼容（如 `yuyv2rgb`） | 默认改 `mjpeg2rgb`，设备号参数化，优先使用 `/dev/v4l/by-id/...` |
| Python 依赖冲突 | `cv_bridge` 报 `_ARRAY_API not found` 等错误 | `numpy` 版本与 ROS 二进制包 ABI 不兼容 | 固定 ROS 兼容的 `numpy` 版本（日志中用 `1.26.4` 解决） |
| 串口权限导致 MAVROS 失联 | `DeviceError:serial:open: Permission denied` | `/dev/ttyS0` 权限/组错误 | 修正为 `root:dialout 660`，并写 udev 规则持久化 |
| 坐标系/符号错误 | 飞机绕圈、越调越偏、y 方向反向 | `marker/body/map` 语义混用，FLU/ENU/NED 符号不统一 | 先统一坐标定义，再做旋转补偿；必要时实测修正 y 轴符号 |
| OFFBOARD 门控与超时 | 控制指令经常被清零 | 非 OFFBOARD、视觉超时、本地位姿超时 | 明确门控状态机：非 OFFBOARD/超时立即零速，恢复后再闭环 |
| 降落后无法 disarm | 触地了但长期卡在 LAND | 仅依赖 `landed_state`，地面检测置位慢/丢失 | 增加“最低油门下压 + 周期 disarm”兜底逻辑 |
| 长时间通电后高度异常 | 进入 OFFBOARD 后持续上升 | 相对高度参考漂移（不一定仅 GPS） | 在 OFFBOARD 上升沿锁定高度参考；低空优先测距/视觉高度 |
| launch 参数覆盖失败 | `--ros-args -p` 看似传参但不生效 | 把 launch 参数与节点参数覆盖方式混用 | 对 launch 声明参数用 `kp_xy:=...` 直接覆盖 |
| 环境未刷新导致“改了不生效” | 仍使用旧设备号/旧默认值 | 未 `source install/setup.bash`，命中旧环境 | 每次构建后强制 `source`，必要时新开 shell 验证 |

## 重点专题

### 1) QoS 不匹配（最隐蔽）

- 日志中多次出现“节点运行正常但无数据”的场景，最终定位到 QoS 不兼容。
- 在 MAVROS 相关话题上，实践中更稳的是：
- 高频状态/位姿：`BEST_EFFORT + VOLATILE`
- 类似 home 这种需要 latched 语义的话题：可用 `TRANSIENT_LOCAL`
- 建议做法：在节点启动时打印订阅 QoS 配置，便于现场快速核对。

### 2) 仿真链路迁移到实机 USB 相机

- 迁移时最容易漏掉的是“图像源切换只是第一步”。
- 实际要一起迁移的有四项：
- 图像来源：`gz bridge -> usb_cam`
- 图像话题：统一到 `/image_raw`
- 标定参数：确保 `camera_info` 读到实机 `ost.yaml`
- 可视化：按实机算力需要选择 `rqt_image_view` 或关闭可视化
- 迁移后常见二次问题：
- 设备号漂移（`/dev/video0/2` 变化）
- 像素格式不兼容（`yuyv2rgb` 崩溃）
- 解决原则：设备号与格式参数化，默认优先 `mjpeg2rgb`。

### 3) 坐标系与控制方向

- 典型故障：`yaw` 看起来正常，但 `xy` 跟踪错误、绕圈或反向。
- 根因通常不是 PID，而是坐标系语义不一致：
- `marker` 误差所在坐标
- 飞控速度命令坐标（ENU/NED/BODY）
- 中间是否做了动态 yaw 旋转补偿
- 实战建议：
- 先做“单轴阶跃测试”验证 x/y 正负方向
- 再上完整 PID
- 日志中保留 `err_marker` 与 `err_cmd`，便于区分“感知误差”和“控制误差”

### 4) 降落末段与 disarm 稳定性

- 实机中常见“已接地但不 disarm”。
- 只等 `ON_GROUND` 风险较大，建议保留兜底：
- 启发式满足后进入最低油门下压
- 按周期发送 disarm
- `armed=false` 后立即停止发送
- 该策略在日志中多次验证能显著降低“卡 LAND”概率。

## 可复用排障清单（建议每次实机前执行）

1. `ros2 topic list | rg -E "mavros|uas1/mavros"`：确认前缀。
2. `ros2 topic info <关键话题>`：确认消息类型与 QoS 兼容。
3. `ls -l /dev/ttyS0 /dev/serial0`：确认串口权限与组。
4. `v4l2-ctl --list-devices`：确认 USB 相机设备号。
5. 启动相机后检查 `/image_raw` 与 `camera_info` 是否同时存在。
6. `source install/setup.bash` 后再启动 launch，避免旧环境。
7. 先做 10 秒单轴低速测试，确认坐标方向再上全流程。

## 对后续开发的建议

- 把“前缀、QoS、图像话题、设备号、像素格式”全部参数化，避免写死。
- 保留最小可用 launch（无 RViz/无多余 TF），用于实机快速回归。
- 将“仿真配置”和“实机配置”分开文件管理，减少切换时的人为遗漏。
- 关键节点保留统一 CSV 记录，长期沉淀参数对比数据。
