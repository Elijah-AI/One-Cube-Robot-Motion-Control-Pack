std_msgs/Header header          # 统一的时间戳和坐标系参考

# 关节状态部分 (对应 sensor_msgs/JointState)
string[] joint_names            # 关节名称列表
float64[] joint_positions       # 关节位置 (rad/m)
float64[] joint_velocities      # 关节速度 (rad/s or m/s)
float64[] joint_efforts         # 关节力矩/力 (Nm/N)
geometry_msgs/Pose[] end_effector_pose  # 末端在基坐标系下的位姿
