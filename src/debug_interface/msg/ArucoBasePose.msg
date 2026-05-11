# 时间戳与参考坐标系（这里用于描述 ArUco 坐标系 FLU）
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

# 机体坐标系（FLU）原点在 ArUco 坐标系（FLU）下的位置（单位：米）
float64 x
float64 y
float64 z

# 机体相对 ArUco 坐标系的偏航角（单位：弧度）
float64 yaw
