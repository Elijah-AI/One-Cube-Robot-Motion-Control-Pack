# TargetDetection.msg
string obj_name                            # 目标名称

# 6D位姿
geometry_msgs/PoseStamped pose             # 6D位姿（3D位置+3D姿态，带时间戳）
float32 confidence                         # 置信度

# 可选：目标物尺寸/包围盒
geometry_msgs/Vector3 size                 # 长宽高 (m)
geometry_msgs/PoseStamped bbox_center      # 包围盒中心（可能与pose不同）

# ========== 末端工具相关 ==========
string tool_type                           # 末端工具类型（如：gripper, suction）
