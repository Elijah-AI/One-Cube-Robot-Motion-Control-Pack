# 头部时间戳
std_msgs/Header header    
sensor_msgs/Image camera_rgbd_color     # RGB彩图
sensor_msgs/Image camera_rgbd_depth     # RGB深度图
sensor_msgs/CameraInfo camera_info      # 相机内参、畸变系数、投影矩阵
string target_label                     # 可选 目标类别名称（如：box、bottle、target）
float32 seg_confidence                  # 预留   分割/检测置信度
bool data_valid                         # 预留   数据是否有效
