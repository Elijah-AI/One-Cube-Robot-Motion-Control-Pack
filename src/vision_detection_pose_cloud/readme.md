# 启动节点
cd /home/wyq/test_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vision_detection_pose_cloud detection_pose_cloud_server
# 终端 2 — 验证接口存在
source /opt/ros/humble/setup.bash
source /home/wyq/test_ws/install/setup.bash

# 查看 action 是否注册
ros2 action list
# 期望输出: /vision/detection_pose_cloud

# 查看 action 信息
ros2 action info /vision/detection_pose_cloud

# 发送 Goal 测试（带 feedback 实时反馈）
#ros2 action send_goal /vision/detection_pose_cloud \
  com_interfaces/action/DetectionPoseCloud \
  "{task_id: 't-001', target_obj_name: 'crak', expected_frame_id: 'camera_link', priority: 10, type: 0}" \
  --feedback
# 验证 TargetDetection 话题发布
  ros2 topic echo /vision/target_detection