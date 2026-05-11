# 时间戳与坐标系标识，用于与图像流或 TF 数据对齐
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
# 位姿解算得到的平移向量（单位：米，参考系：相机坐标系）
float64[3] tvec
# 罗德里格斯形式旋转向量（单位：弧度，方向：marker -> camera）
float64[3] rvec
