# 时间戳与参考坐标系（这里用于描述 ArUco 坐标系 FLU）
std_msgs/Header header

# 机体坐标系（FLU）原点在 ArUco 坐标系（FLU）下的位置（单位：米）
float64 x
float64 y
float64 z

# 机体相对 ArUco 坐标系的偏航角（单位：弧度）
float64 yaw
