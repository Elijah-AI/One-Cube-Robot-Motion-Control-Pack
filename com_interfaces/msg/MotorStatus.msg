std_msgs/Header header         
string[] joint_names
float64[] joint_positions     # 转动关节位置，单位：rad
float64[] joint_velocities    # 转动关节速度，单位：rad/s
float64[] joint_efforts
geometry_msgs/Pose[] end_effector_pose  # 左、右工具 TCP 在 header.frame_id 下的位姿，平移单位：米
