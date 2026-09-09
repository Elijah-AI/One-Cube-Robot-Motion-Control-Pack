# 用一个临时目录避免权限问题
ros2 launch vision_calibration vision_calibration.launch.py \
  <!-- topic_name:=/fusion/synced_robot_vision \
  output_dir:=/tmp/calib_e2e -->

# 调用 CHECK_STATUS（检查数据并保存内参）
  ros2 action send_goal /vision/calibration_control \
  com_interfaces/action/CalibrationControl \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, camera_id: 0, arm_id: 0, command: 'CHECK_STATUS', auto_execute: false, mode: ''}"

# 调用 TRIGGER_CAPTURE（触发捕获）
  ros2 action send_goal /vision/calibration_control \
  com_interfaces/action/CalibrationControl \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, camera_id: 0, arm_id: 0, command: 'TRIGGER_CAPTURE', auto_execute: false, mode: 'TO_HAND'}"


# 重启节点指向历史数据目录
ros2 launch vision_calibration vision_calibration.launch.py \
  output_dir:=/home/wyq/test_ws/src/vision_calibration/data_cali/20260902_085806

# 直接执行标定
ros2 action send_goal /vision/calibration_control \
  com_interfaces/action/CalibrationControl \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, camera_id: 0, arm_id: 0, command: 'EXECUTE_CALIBRATION', auto_execute: false, mode: 'TO_HAND'}"