# 时间戳与坐标系标识，用于与图像流或 TF 数据对齐
std_msgs/Header header
# 位姿解算得到的平移向量（单位：米，参考系：相机坐标系）
float64[3] tvec
# 罗德里格斯形式旋转向量（单位：弧度，方向：marker -> camera）
float64[3] rvec
