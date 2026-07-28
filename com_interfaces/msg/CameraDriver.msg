# RGBD Message
std_msgs/Header header
sensor_msgs/CameraInfo rgb_camera_info
sensor_msgs/CameraInfo depth_camera_info
sensor_msgs/Image rgb
sensor_msgs/Image depth
string target_label                           # 可选 目标类别名称（如：box、bottle、target）
float32 seg_confidence                        # 预留   分割/检测置信度
bool data_valid                               # 预留   数据是否有效