# camera_calibration_pkg

当前默认标定板参数（已按你的标定板设置）：
- 内角点：`7x5`
- 单格尺寸：`0.041` m（41 mm）

## 启动标定

```bash
ros2 launch camera_calibration_pkg camera_calibration.launch.py
```

当前 launch 按单目相机使用，`COMMIT` 时会把标定结果通过 `usb_cam` 的 `/set_camera_info` 服务写回到 `camera_info_url` 指定的 yaml 文件。

可选参数：
- `camera_topic`：默认 `/image_raw`
- `video_device`：默认 `/dev/video0`
- `image_width`：默认 `640`
- `image_height`：默认 `480`
- `framerate`：默认 `15.0`（降低标定时卡死概率）
- `pixel_format`：默认 `mjpeg2rgb`（外接 USB 摄像头推荐）
- `camera_name`：默认 `default_cam`（建议给不同相机/分辨率单独命名）
- `camera_info_url`：默认 `file:///home/zjh/.ros/camera_info/default_cam.yaml`
- `size`：默认 `7x5`
- `square`：默认 `0.041`
- `calib_queue_size`：默认 `1`（建议保持 1）
- `max_chessboard_speed`：默认 `0.5`（抑制高速移动样本）

例如：

```bash
ros2 launch camera_calibration_pkg camera_calibration.launch.py video_device:=/dev/video0 pixel_format:=mjpeg2rgb image_width:=640 image_height:=480 framerate:=15.0 camera_topic:=/image_raw size:=7x5 square:=0.041 calib_queue_size:=1 max_chessboard_speed:=0.5
```

若要为新摄像头的 `1920x1080` 标定单独保存文件，避免覆盖旧标定，建议先解析真实设备路径再启动：

```bash
VIDEO_DEV=$(readlink -f /dev/v4l/by-id/usb-icSpring_icspring_camera_20240307110322-video-index0)
ros2 launch camera_calibration_pkg camera_calibration.launch.py \
  video_device:=${VIDEO_DEV} \
  pixel_format:=mjpeg2rgb \
  image_width:=1920 \
  image_height:=1080 \
  framerate:=15.0 \
  camera_topic:=/image_raw \
  camera_name:=icspring_cam_1920x1080 \
  camera_info_url:=file:///home/zjh/.ros/camera_info/icspring_cam_1920x1080.yaml \
  size:=7x5 \
  square:=0.041 \
  calib_queue_size:=1 \
  max_chessboard_speed:=0.5
```

若设备号变化，先执行：

```bash
ls -l /dev/v4l/by-id
```

再将外接摄像头索引 0 解析成真实设备并启动：

```bash
VIDEO_DEV=$(readlink -f /dev/v4l/by-id/*video-index0)
ros2 launch camera_calibration_pkg camera_calibration.launch.py video_device:=${VIDEO_DEV} pixel_format:=mjpeg2rgb image_width:=640 image_height:=480 framerate:=15.0 size:=7x5 square:=0.041 calib_queue_size:=1 max_chessboard_speed:=0.5
```

## 保存标定结果

```bash
ros2 run camera_calibration_pkg save_calibration
```

结果默认保存到：`~/.ros/camera_info/default_cam.yaml`

建议：完成采样后在标定窗口点击 `CALIBRATE`，再点击 `SAVE` 保存内参。

注意：
- 首次标定时如果日志出现 `Camera calibration file ... not found`，这是正常现象，只是说明目标 yaml 还不存在。
- `cameracalibrator` 启动时看到 `/left`、`/right` 的 warning 也不代表失败；官方工具单目模式下仍会创建双目订阅。
- 真正需要关注的是 `COMMIT` 后是否成功写入 `camera_info_url` 指定的文件。

## 本次修复说明

此前该 launch 把 `camera:=usb_cam` 传给了 `cameracalibrator`，会导致工具内部访问错误的 `set_camera_info` 服务名，点击 `COMMIT` 后同步调用可能一直卡住。

现在已改为显式将：

- `camera/set_camera_info -> /set_camera_info`

重映射到 `usb_cam` 实际提供的服务，因此单目标定的 `COMMIT` 会走正确的服务路径。

避免 `CALIBRATE` 卡死建议：
1. 先让 X/Y/Size/Skew 四个进度条基本变绿，再点击 `CALIBRATE`。
2. 采样数量控制在 30~60 组即可，不要长时间持续采集。
3. 若仍卡死，可进一步降载：`framerate:=10.0 image_width:=320 image_height:=240`。
