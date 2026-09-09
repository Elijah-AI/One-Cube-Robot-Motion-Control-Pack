#ifndef VISION_CALIBRATION__CALIBRATION_NODE_HPP_
#define VISION_CALIBRATION__CALIBRATION_NODE_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "com_interfaces/msg/synced_robot_vision.hpp"
#include "com_interfaces/msg/motor_status.hpp"
#include "com_interfaces/action/calibration_control.hpp"

#include "vision_calibration/handeye_calibrator.hpp"

namespace vision_calibration
{

// Action 类型别名
using CalibrationControl = com_interfaces::action::CalibrationControl;
using GoalHandleCalibration = rclcpp_action::ServerGoalHandle<CalibrationControl>;
// 消息类型别名
using SyncedRobotVision = com_interfaces::msg::SyncedRobotVision;
using MotorStatus = com_interfaces::msg::MotorStatus;

/**
 * 手眼标定数据采集节点
 *
 * 订阅 SyncedRobotVision 话题获取相机图像 + 内参 + 机器人位姿，
 * 通过 CalibrationControl action 控制整体流程：
 *   - CHECK_STATUS       : 检查相机数据是否就绪并保存相机内参
 *   - TRIGGER_CAPTURE    : 触发一次采集（保存图像 + 位姿）
 *   - EXECUTE_CALIBRATION: 执行标定（预留扩展位）
 */
class CalibrationNode : public rclcpp::Node
{
public:
  CalibrationNode();
  ~CalibrationNode() override = default;

private:
  // ---------- 话题回调 ----------
  void synced_vision_callback(const SyncedRobotVision::SharedPtr msg);
  // /Control/HeadPose 回调：缓存头部在基坐标系下的位姿
  void head_pose_callback(const MotorStatus::SharedPtr msg);

  // ---------- Action server 回调 ----------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CalibrationControl::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCalibration> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleCalibration> goal_handle);

  // ---------- 任务执行主体（独立线程） ----------
  void execute(const std::shared_ptr<GoalHandleCalibration> goal_handle);

  // ---------- 各命令处理 ----------
  // CHECK_STATUS: 检查数据是否就绪，并保存相机内参；同时锁定本次标定使用的 arm_id
  bool handle_check_status(
    uint8_t camera_id, uint8_t arm_id, std::string & err_msg);

  // TRIGGER_CAPTURE: 采集一帧（保存图像 + 机器人位姿）
  // arm_id 用于选择 end_effector_pose 数组的下标（0=左臂, 1=右臂）
  bool handle_trigger_capture(
    uint8_t camera_id, uint8_t arm_id, const std::string & mode,
    std::string & err_msg);

  // EXECUTE_CALIBRATION: 执行标定（预留）
  bool handle_execute_calibration(
    const std::string & mode, std::string & result_json,
    std::string & err_msg);

  // ---------- 辅助函数 ----------
  void publish_feedback(
    const std::shared_ptr<GoalHandleCalibration> & gh,
    const std::string & status, float progress,
    const std::string & info);

  // 取指定相机的最新一帧（线程安全）
  SyncedRobotVision::SharedPtr get_latest(uint8_t camera_id);

  // 取最新头部位姿（/Control/HeadPose，未收到返回 false）
  bool get_latest_head_pose(geometry_msgs::msg::Pose & pose);

  // 校验单帧数据是否完整可用
  bool is_data_valid(const SyncedRobotVision::SharedPtr & msg, std::string & err) const;

  // 目录与文件操作
  bool ensure_dir(const std::string & path) const;
  int count_saved_captures() const;
  int next_capture_index() const { return count_saved_captures() + 1; }
  // 生成默认时间戳目录 data_cali/YYYYMMDD_HHMMSS
  std::string make_timestamped_dir() const;

  // 保存相机内参 + 头部位姿（OpenCV FileStorage，与 offline_calib 兼容）
  bool save_intrinsics(
    const sensor_msgs::msg::CameraInfo & info,
    const geometry_msgs::msg::Pose & head_pose,
    const std::string & path) const;
  // 保存图像为 PNG
  bool save_image(const sensor_msgs::msg::Image & img, const std::string & path) const;
  // 保存机器人位姿为 pose_N.yaml（与 offline_calib 兼容格式）
  bool save_pose(
    const geometry_msgs::msg::Pose & pose,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & path) const;

  // ---------- 成员 ----------
  rclcpp::Subscription<SyncedRobotVision>::SharedPtr synced_sub_;
  // 头部位姿订阅（/Control/HeadPose，MotorStatus）
  rclcpp::Subscription<MotorStatus>::SharedPtr head_pose_sub_;
  rclcpp_action::Server<CalibrationControl>::SharedPtr action_server_;

  // 最新帧缓存（按 camera_id 索引）
  std::mutex data_mutex_;
  std::map<uint8_t, SyncedRobotVision::SharedPtr> latest_by_cam_;

  // 头部位姿缓存（/Control/HeadPose 的 end_effector_pose[0]）
  std::mutex head_pose_mutex_;
  geometry_msgs::msg::Pose head_pose_;
  bool head_pose_valid_{false};

  // 参数
  std::string topic_name_;      // 订阅的话题
  std::string action_name_;     // action 名称
  std::string output_dir_;      // 输出目录（空=每次 CHECK_STATUS 时新建时间戳目录）
  bool output_dir_explicit_{false};  // output_dir 是否被显式指定
  int board_cols_{11};
  int board_rows_{8};
  double square_size_{0.015};

  // 手眼标定器（EXECUTE_CALIBRATION 时调用）
  std::unique_ptr<HandEyeCalibrator> calibrator_;

  // 并发控制：同一时刻只允许一个 goal 执行
  std::atomic<bool> busy_{false};

  // CHECK_STATUS 锁定的 arm_id（-1 表示尚未初始化）
  // TRIGGER_CAPTURE / EXECUTE_CALIBRATION 必须使用相同的 arm_id
  std::atomic<int> armed_arm_id_{-1};
};

}  // namespace vision_calibration

#endif  // VISION_CALIBRATION__CALIBRATION_NODE_HPP_
