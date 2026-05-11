# 原始图像时间戳继续放在 header.stamp 中，作为整条链的输入基准
std_msgs/Header header
# 第 2 级：tvec_rvec_node 回调一进入时刻
builtin_interfaces/Time tvec_cb_start_stamp
# 第 2 级：完成 CvBridge 转换及必要图像准备时刻
builtin_interfaces/Time tvec_cv_bridge_done_stamp
# 第 2 级：完成 ArUco 检测时刻
builtin_interfaces/Time tvec_detect_done_stamp
# 第 2 级：完成当前这条检测结果位姿估计时刻
builtin_interfaces/Time tvec_pose_done_stamp
# 第 2 级：发布 /debug/tvec 时刻
builtin_interfaces/Time tvec_pub_stamp
# 第 3 级：tvec_tf_node 回调一进入时刻
builtin_interfaces/Time tf_cb_start_stamp
# 第 3 级：发布 /debug/aruco_pose 时刻
builtin_interfaces/Time tf_pub_stamp
# 第 4 级：dynamic_tracking_v2_node 收到 pose 回调时刻
builtin_interfaces/Time ctrl_cb_start_stamp
# 第 4 级：dynamic_tracking_v2_node 发布 PositionTarget 时刻
builtin_interfaces/Time setpoint_pub_stamp
