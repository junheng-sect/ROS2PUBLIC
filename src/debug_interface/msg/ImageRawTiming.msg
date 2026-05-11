# 原始图像 header.stamp，继续表示图像消息自身携带的时间基准
std_msgs/Header header
# DDS 元信息中的 source_timestamp，可近似视为 usb_cam 调用 publish() 的时刻
builtin_interfaces/Time image_source_stamp
# DDS 元信息中的 received_timestamp，表示探针订阅端真正取到该消息的时刻
builtin_interfaces/Time image_received_stamp
# 探针节点发布 /debug/image_raw_timing 的时刻
builtin_interfaces/Time image_probe_pub_stamp
