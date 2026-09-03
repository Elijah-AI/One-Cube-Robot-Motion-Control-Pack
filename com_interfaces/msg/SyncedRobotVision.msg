# 头部时间戳
std_msgs/Header header

# 相机原始数据
sensor_msgs/Image camera_rgbd_color     # RGB彩图
sensor_msgs/PointCloud2 camera_rgbd_pointcloud
sensor_msgs/Image camera_rgbd_depth     # RGB深度图
sensor_msgs/CameraInfo camera_info      # 相机内参、畸变系数、投影矩阵
string target_label                     # 可选 目标类别名称（如：box、bottle、target）
float32 seg_confidence                  # 预留   分割/检测置信度
bool data_valid                         # 预留   数据是否有效

# 机械臂运控数据
string[] joint_names                    # 关节名称列表
float64[] joint_positions               # 关节位置 (rad/m)
float64[] joint_velocities              # 关节速度 (rad/s or m/s)
float64[] joint_efforts                 # 关节力矩/力 (Nm/N)
geometry_msgs/Pose[] end_effector_pose  # 末端在基坐标系下的位姿
