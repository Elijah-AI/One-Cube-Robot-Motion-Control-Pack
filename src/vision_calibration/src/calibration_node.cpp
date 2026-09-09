#include "vision_calibration/calibration_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"

namespace fs = std::filesystem;

namespace vision_calibration
{

// ============================================================================
// 构造函数
// ============================================================================
CalibrationNode::CalibrationNode()
: rclcpp::Node("calibration_node")
{
  // 声明参数
  this->declare_parameter<std::string>("topic_name", "/fusion/synced_robot_vision");
  this->declare_parameter<std::string>("action_name", "/vision/calibration_control");
  this->declare_parameter<std::string>("output_dir", "");
  this->declare_parameter<int>("board_cols", 8);
  this->declare_parameter<int>("board_rows", 11);
  this->declare_parameter<double>("square_size", 0.015);
  // 标定参数（透传给 HandEyeCalibrator）
  this->declare_parameter<double>("ransac_threshold", 0.002);
  this->declare_parameter<int>("ransac_sample_size", 8);
  this->declare_parameter<int>("ransac_n_iter", 5000);
  this->declare_parameter<int>("ransac_seed", 42);
  this->declare_parameter<double>("rotation_weight", 0.2);
  this->declare_parameter<int>("refine_iterations", 30);

  topic_name_ = this->get_parameter("topic_name").as_string();
  action_name_ = this->get_parameter("action_name").as_string();
  board_cols_ = this->get_parameter("board_cols").as_int();
  board_rows_ = this->get_parameter("board_rows").as_int();
  square_size_ = this->get_parameter("square_size").as_double();

  // 构造手眼标定器
  HandEyeParams calib_params;
  calib_params.ransac_threshold = this->get_parameter("ransac_threshold").as_double();
  calib_params.ransac_sample_size = this->get_parameter("ransac_sample_size").as_int();
  calib_params.ransac_n_iter = this->get_parameter("ransac_n_iter").as_int();
  calib_params.ransac_seed = static_cast<unsigned int>(this->get_parameter("ransac_seed").as_int());
  calib_params.rotation_weight = this->get_parameter("rotation_weight").as_double();
  calib_params.refine_iterations = this->get_parameter("refine_iterations").as_int();
  calibrator_ = std::make_unique<HandEyeCalibrator>(calib_params, this->get_logger());

  // 输出目录：
  //   - 显式指定 output_dir 参数: 固定使用该目录
  //   - 未指定: 每次 CHECK_STATUS 时新建 data_cali/<日期时间> 目录（批次隔离）
  //     路径: /home/neardi/ros2_ws/src/vision_calibration/data_cali/YYYYMMDD_HHMMSS
  output_dir_ = this->get_parameter("output_dir").as_string();
  output_dir_explicit_ = !output_dir_.empty();

  // 创建订阅者
  synced_sub_ = this->create_subscription<SyncedRobotVision>(
    topic_name_, rclcpp::SensorDataQoS(),
    std::bind(&CalibrationNode::synced_vision_callback, this, std::placeholders::_1));

  // 订阅头部位姿（/Control/HeadPose，取 end_effector_pose[0] 作为头部在基坐标系下的位姿）
  head_pose_sub_ = this->create_subscription<MotorStatus>(
    "/Control/HeadPose", rclcpp::QoS(10),
    std::bind(&CalibrationNode::head_pose_callback, this, std::placeholders::_1));

  // 创建 Action Server
  action_server_ = rclcpp_action::create_server<CalibrationControl>(
    this, action_name_,
    std::bind(&CalibrationNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&CalibrationNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&CalibrationNode::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "✅ CalibrationNode ready: subscribe '%s', action '%s', output '%s'",
    topic_name_.c_str(), action_name_.c_str(),
    output_dir_explicit_ ? output_dir_.c_str() : "(CHECK_STATUS 时新建时间戳目录)");
}

// ============================================================================
// 话题回调：缓存各相机的最新一帧
// ============================================================================
void
CalibrationNode::synced_vision_callback(const SyncedRobotVision::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_by_cam_[msg->camera_id] = msg;
}

SyncedRobotVision::SharedPtr
CalibrationNode::get_latest(uint8_t camera_id)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  auto it = latest_by_cam_.find(camera_id);
  if (it == latest_by_cam_.end() || !it->second) {
    return nullptr;
  }
  return it->second;
}

// /Control/HeadPose 回调：缓存 end_effector_pose[0]
void
CalibrationNode::head_pose_callback(const MotorStatus::SharedPtr msg)
{
  if (!msg || msg->end_effector_pose.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(head_pose_mutex_);
  head_pose_ = msg->end_effector_pose[0];
  head_pose_valid_ = true;
}

bool
CalibrationNode::get_latest_head_pose(geometry_msgs::msg::Pose & pose)
{
  std::lock_guard<std::mutex> lock(head_pose_mutex_);
  if (!head_pose_valid_) {
    return false;
  }
  pose = head_pose_;
  return true;
}

bool
CalibrationNode::is_data_valid(const SyncedRobotVision::SharedPtr & msg, std::string & err) const
{
  if (!msg) {
    err = "无数据";
    return false;
  }
  // if (!msg->data_valid) {
  //   err = "data_valid=false";
  //   return false; 
  // }
  if (msg->camera_rgbd_color.width == 0 || msg->camera_rgbd_color.height == 0) {
    err = "图像尺寸为 0";
    return false;
  }
  if (msg->camera_rgbd_color.data.empty()) {
    err = "图像数据为空";
    return false;
  }
  if (msg->camera_info.k.size() != 9) {
    err = "CameraInfo K 矩阵维度异常 (=" + std::to_string(msg->camera_info.k.size()) + ")";
    return false;
  }
  bool k_nonzero = false;
  for (double v : msg->camera_info.k) {
    if (std::abs(v) > 1e-9) { k_nonzero = true; break; }
  }
  if (!k_nonzero) {
    err = "K 矩阵全零";
    return false;
  }
  if (msg->end_effector_pose.empty()) {
    err = "end_effector_pose 为空" + std::to_string(msg->end_effector_pose.size());
    return false;
  }
  return true;
}

// ============================================================================
// Action server 回调
// ============================================================================
rclcpp_action::GoalResponse
CalibrationNode::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const CalibrationControl::Goal> goal)
{
  // 合法命令白名单
  static const std::set<std::string> kValidCmd = {
    "CHECK_STATUS", "TRIGGER_CAPTURE", "EXECUTE_CALIBRATION"
  };
  if (kValidCmd.find(goal->command) == kValidCmd.end()) {
    RCLCPP_WARN(this->get_logger(), "拒绝: 未知 command '%s'", goal->command.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  // 非法 mode（EXECUTE_CALIBRATION 需要校验，其余不校验）
  if (goal->command == "EXECUTE_CALIBRATION") {
    if (goal->mode != "TO_HAND" && goal->mode != "IN_HAND") {
      RCLCPP_WARN(this->get_logger(), "拒绝: mode '%s' 非法", goal->mode.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  // 同一时刻仅允许一个任务执行
  if (busy_.load()) {
    RCLCPP_WARN(this->get_logger(), "拒绝: 已有任务执行中");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
CalibrationNode::handle_cancel(const std::shared_ptr<GoalHandleCalibration> /*goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "收到取消请求, 允许取消");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
CalibrationNode::handle_accepted(const std::shared_ptr<GoalHandleCalibration> goal_handle)
{
  std::thread(
    std::bind(&CalibrationNode::execute, this, std::placeholders::_1),
    goal_handle).detach();
}

// ============================================================================
// 任务执行主体
// ============================================================================
void
CalibrationNode::execute(const std::shared_ptr<GoalHandleCalibration> goal_handle)
{
  auto logger = this->get_logger();
  const auto goal = goal_handle->get_goal();

  busy_.store(true);
  auto reset_busy_on_exit = [this]() { busy_.store(false); };

  auto result = std::make_shared<CalibrationControl::Result>();
  result->success = false;
  result->total_captures = count_saved_captures();

  auto finish_ok = [&](const std::string & msg) {
    result->error_message = msg;
    reset_busy_on_exit();
    goal_handle->succeed(result);
  };
  auto finish_fail = [&](const std::string & msg) {
    result->error_message = msg;
    reset_busy_on_exit();
    goal_handle->abort(result);
  };

  RCLCPP_INFO(logger, "▶ command=%s camera_id=%d arm_id=%d mode=%s",
    goal->command.c_str(), goal->camera_id, goal->arm_id, goal->mode.c_str());

  // ---------- CHECK_STATUS ----------
  if (goal->command == "CHECK_STATUS") {
    publish_feedback(goal_handle, "DETECTING", 20.0f, "检查相机数据是否就绪");
    std::string err;
    if (!handle_check_status(goal->camera_id, goal->arm_id, err)) {
      finish_fail("CHECK_STATUS 失败: " + err);
      return;
    }
    publish_feedback(goal_handle, "FINISHED", 100.0f, "数据就绪, 内参已保存");
    result->success = true;
    result->total_captures = count_saved_captures();
    finish_ok("数据就绪, 内参已保存至 " + (fs::path(output_dir_) / "camera_intrinsics.yaml").string());
    return;
  }

  // ---------- TRIGGER_CAPTURE ----------
  if (goal->command == "TRIGGER_CAPTURE") {
    publish_feedback(goal_handle, "ESTIMATING_POSE", 30.0f, "采集一帧图像与位姿");
    // 校验：必须先 CHECK_STATUS 且 arm_id 一致
    {
      const int armed = armed_arm_id_.load();
      if (armed < 0) {
        finish_fail("尚未执行 CHECK_STATUS 初始化 arm_id, 请先调用 CHECK_STATUS");
        return;
      }
      if (static_cast<uint8_t>(armed) != goal->arm_id) {
        finish_fail("arm_id 不匹配: CHECK_STATUS 锁定为 " + std::to_string(armed) +
                    ", 当前 TRIGGER_CAPTURE 请求 " + std::to_string(goal->arm_id));
        return;
      }
    }
    std::string err;
    // 采集前确保目录与内参已就绪
    if (!ensure_dir(output_dir_)) {
      finish_fail("创建输出目录失败: " + output_dir_);
      return;
    }
    // 若内参尚未保存，则先保存一次
    const std::string intrinsics_path = (fs::path(output_dir_) / "camera_intrinsics.yaml").string();
    if (!fs::exists(intrinsics_path)) {
      std::string e2;
      if (!handle_check_status(goal->camera_id, goal->arm_id, e2)) {
        finish_fail("采集前置检查失败: " + e2);
        return;
      }
    }
    if (!handle_trigger_capture(goal->camera_id, goal->arm_id, goal->mode, err)) {
      finish_fail("TRIGGER_CAPTURE 失败: " + err);
      return;
    }
    publish_feedback(goal_handle, "FINISHED", 100.0f, "采集完成");
    result->success = true;
    result->total_captures = count_saved_captures();
    // 若 auto_execute 且已采集足够帧，则触发标定（此处仅提示，预留）
    if (goal->auto_execute) {
      RCLCPP_INFO(logger, "auto_execute=true, 后续可执行 EXECUTE_CALIBRATION 进行标定");
    }
    finish_ok("采集成功, 当前总数=" + std::to_string(result->total_captures));
    return;
  }

  // ---------- EXECUTE_CALIBRATION ----------
  if (goal->command == "EXECUTE_CALIBRATION") {
    publish_feedback(goal_handle, "GENERATING_CLOUD", 50.0f, "执行标定");
    // 校验 arm_id：若尚未 CHECK_STATUS 但目标目录已有内参文件，自动初始化 arm_id
    {
      const int armed = armed_arm_id_.load();
      if (armed < 0) {
        const std::string intrinsics_path =
          (fs::path(output_dir_) / "camera_intrinsics.yaml").string();
        if (!fs::exists(intrinsics_path)) {
          finish_fail("尚未执行 CHECK_STATUS 且目标目录无内参文件: " + intrinsics_path);
          return;
        }
        // 目录已有内参，自动初始化 arm_id
        armed_arm_id_.store(static_cast<int>(goal->arm_id));
        RCLCPP_INFO(logger, "目标目录已有内参, 自动初始化 arm_id=%d", goal->arm_id);
      } else if (static_cast<uint8_t>(armed) != goal->arm_id) {
        finish_fail("arm_id 不匹配: CHECK_STATUS 锁定为 " + std::to_string(armed) +
                    ", 当前 EXECUTE_CALIBRATION 请求 " + std::to_string(goal->arm_id));
        return;
      }
    }
    std::string result_json;
    std::string err;
    if (!handle_execute_calibration(goal->mode, result_json, err)) {
      finish_fail("EXECUTE_CALIBRATION 失败: " + err);
      return;
    }
    publish_feedback(goal_handle, "FINISHED", 100.0f, "标定完成");
    result->success = true;
    result->calibration_result = result_json;
    result->total_captures = count_saved_captures();
    finish_ok("标定完成");
    return;
  }

  finish_fail("未处理的 command: " + goal->command);
}

// ============================================================================
// CHECK_STATUS: 检查数据是否就绪并保存内参；同时锁定本次标定的 arm_id
//   未显式指定 output_dir 时，每次执行都新建时间戳目录（批次隔离）
// ============================================================================
bool
CalibrationNode::handle_check_status(
  uint8_t camera_id, uint8_t arm_id, std::string & err_msg)
{
  // 未显式指定目录: 新建 data_cali/<时间戳>，本次标定批次从这里开始
  if (!output_dir_explicit_) {
    output_dir_ = make_timestamped_dir();
    RCLCPP_INFO(this->get_logger(), "新建标定数据目录: %s", output_dir_.c_str());
  }
  if (!ensure_dir(output_dir_)) {
    err_msg = "创建输出目录失败: " + output_dir_;
    return false;
  }
  auto msg = get_latest(camera_id);
  if (!msg) {
    err_msg = "尚未收到 camera_id=" + std::to_string(camera_id) + " 的数据";
    return false;
  }
  std::string err;
  if (!is_data_valid(msg, err)) {
    err_msg = "数据无效: " + err;
    return false;
  }
  // 校验 arm_id 在 end_effector_pose 范围内
  if (arm_id >= msg->end_effector_pose.size()) {
    err_msg = "arm_id=" + std::to_string(arm_id) + " 越界 (end_effector_pose.size()=" +
              std::to_string(msg->end_effector_pose.size()) + ")";
    return false;
  }
  // 读取头部位姿（/Control/HeadPose）
  geometry_msgs::msg::Pose head_pose;
  if (!get_latest_head_pose(head_pose)) {
    err_msg = "尚未收到 /Control/HeadPose 头部位姿数据";
    return false;
  }
  // 保存内参 + 头部位姿
  const std::string path = (fs::path(output_dir_) / "camera_intrinsics.yaml").string();
  if (!save_intrinsics(msg->camera_info, head_pose, path)) {
    err_msg = "保存内参失败: " + path;
    return false;
  }
  // 锁定本次标定使用的 arm_id，后续 TRIGGER_CAPTURE / EXECUTE_CALIBRATION 必须一致
  armed_arm_id_.store(static_cast<int>(arm_id));
  RCLCPP_INFO(this->get_logger(), "内参已保存: %s, 锁定 arm_id=%d", path.c_str(), arm_id);
  err_msg.clear();
  return true;
}

// ============================================================================
// TRIGGER_CAPTURE: 保存一帧图像与机器人位姿
// arm_id 选择 end_effector_pose 数组的下标（0=左臂, 1=右臂）
// ============================================================================
bool
CalibrationNode::handle_trigger_capture(
  uint8_t camera_id, uint8_t arm_id, const std::string & mode,
  std::string & err_msg)
{
  auto msg = get_latest(camera_id);
  if (!msg) {
    err_msg = "尚未收到 camera_id=" + std::to_string(camera_id) + " 的数据";
    return false;
  }
  std::string err;
  if (!is_data_valid(msg, err)) {
    err_msg = "数据无效1: " + err;
    return false;
  }
  // 边界防御：arm_id 必须在 end_effector_pose 数组范围内
  if (arm_id >= msg->end_effector_pose.size()) {
    err_msg = "arm_id=" + std::to_string(arm_id) +
              " 越界 (end_effector_pose.size()=" +
              std::to_string(msg->end_effector_pose.size()) + ")";
    return false;
  }

  const int idx = next_capture_index();
  // 统一编号格式 image_N.png / pose_N.yaml
  std::ostringstream oss;
  oss << std::setw(2) << std::setfill('0') << idx;
  const std::string num = oss.str();

  const std::string img_path = (fs::path(output_dir_) / ("image_" + num + ".png")).string();
  const std::string pose_path = (fs::path(output_dir_) / ("pose_" + num + ".yaml")).string();

  // 保存图像
  if (!save_image(msg->camera_rgbd_color, img_path)) {
    err_msg = "保存图像失败: " + img_path;
    return false;
  }
  // 保存位姿（按 arm_id 选择左臂/右臂）
  if (!save_pose(msg->end_effector_pose[arm_id], msg->header.stamp, pose_path)) {
    err_msg = "保存位姿失败: " + pose_path;
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "采集 #%d (arm_id=%d mode=%s): %s + %s",
    idx, arm_id, mode.c_str(), img_path.c_str(), pose_path.c_str());
  err_msg.clear();
  return true;
}

// ============================================================================
// EXECUTE_CALIBRATION: 委托给 HandEyeCalibrator 执行手眼标定
//   （内参/图像/位姿从 output_dir_ 读取，结果写回同一目录）
// ============================================================================
bool
CalibrationNode::handle_execute_calibration(
  const std::string & mode, std::string & result_json,
  std::string & err_msg)
{
  return calibrator_->calibrate(output_dir_, mode, result_json, err_msg);
}

// ============================================================================
// 辅助函数
// ============================================================================
void
CalibrationNode::publish_feedback(
  const std::shared_ptr<GoalHandleCalibration> & gh,
  const std::string & status, float progress,
  const std::string & info)
{
  auto fb = std::make_shared<CalibrationControl::Feedback>();
  fb->status = status;
  fb->progress = progress;
  fb->header.stamp = this->now();
  fb->header.frame_id = status;
  (void)info;
  gh->publish_feedback(fb);
}

bool
CalibrationNode::ensure_dir(const std::string & path) const
{
  std::error_code ec;
  fs::create_directories(path, ec);
  if (ec) {
    RCLCPP_ERROR(this->get_logger(), "创建目录 '%s' 失败: %s", path.c_str(), ec.message().c_str());
    return false;
  }
  return true;
}

// 生成默认时间戳目录 data_cali/YYYYMMDD_HHMMSS
std::string
CalibrationNode::make_timestamped_dir() const
{
  auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::ostringstream ts;
  ts << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
  return (fs::path("/home/neardi/ros2_ws/src/vision_calibration/data_cali") /
          ts.str()).string();
}

int
CalibrationNode::count_saved_captures() const
{
  if (!fs::exists(output_dir_)) return 0;
  int cnt = 0;
  for (auto & p : fs::directory_iterator(output_dir_)) {
    std::string name = p.path().filename().string();
    // 计 image_*.png 文件数
    if (name.rfind("image_", 0) == 0 && p.path().extension() == ".png") {
      ++cnt;
    }
  }
  return cnt;
}

// 保存相机内参（OpenCV FileStorage 格式，与 offline_calib 兼容）
bool
CalibrationNode::save_intrinsics(
  const sensor_msgs::msg::CameraInfo & info,
  const geometry_msgs::msg::Pose & head_pose,
  const std::string & path) const
{
  // 内参矩阵 K (3x3)
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  if (info.k.size() == 9) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        K.at<double>(i, j) = info.k[i * 3 + j];
      }
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "K 维度 != 9");
    return false;
  }
  // 畸变系数 D (1xN)
  const int n_d = static_cast<int>(info.d.size());
  cv::Mat D(1, n_d, CV_64F);
  for (int i = 0; i < n_d; ++i) {
    D.at<double>(0, i) = info.d[i];
  }

  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开写入: %s", path.c_str());
    return false;
  }
  fs << "camera_matrix" << K;
  fs << "dist_coeffs" << D;
  fs << "board_cols" << board_cols_;
  fs << "board_rows" << board_rows_;
  fs << "square_size" << square_size_;
  // 头部在基坐标系下的位姿（来自 /Control/HeadPose）
  fs << "head_position" <<
    cv::Vec3d(head_pose.position.x, head_pose.position.y, head_pose.position.z);
  fs << "head_orientation_xyzw" <<
    cv::Vec4d(head_pose.orientation.x, head_pose.orientation.y,
              head_pose.orientation.z, head_pose.orientation.w);
  fs.release();
  return true;
}

// 保存图像为 PNG
bool
CalibrationNode::save_image(const sensor_msgs::msg::Image & img, const std::string & path) const
{
  try {
    auto cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
    if (!cv_ptr) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败");
      return false;
    }
    if (!cv::imwrite(path, cv_ptr->image)) {
      RCLCPP_ERROR(this->get_logger(), "imwrite 失败: %s", path.c_str());
      return false;
    }
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
    return false;
  } catch (const cv::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV 异常: %s", e.what());
    return false;
  }
  return true;
}

// 保存机器人位姿为 pose_N.yaml（与 offline_calib 兼容的纯 YAML 格式）
bool
CalibrationNode::save_pose(
  const geometry_msgs::msg::Pose & pose,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & path) const
{
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开写入: %s", path.c_str());
    return false;
  }
  // 高精度输出
  ofs << std::setprecision(10);
  ofs << "%YAML:1.0\n";
  // tcp_translation / tcp_quaternion_xyzw 来自末端位姿（基坐标->末端）
  ofs << "tcp_translation: [ " << pose.position.x << ", "
      << pose.position.y << ", " << pose.position.z << " ]\n";
  ofs << "tcp_quaternion_xyzw: [ " << pose.orientation.x << ", "
      << pose.orientation.y << ", " << pose.orientation.z << ", "
      << pose.orientation.w << " ]\n";
  // target2cam 由后续 solvePnP 填充，先置零占位
  ofs << "target2cam_translation: [ 0., 0., 0. ]\n";
  ofs << "target2cam_quaternion_xyzw: [ 0., 0., 0., 1. ]\n";
  ofs << "tcp_pose_msg:\n";
  ofs << "   header_stamp_sec: " << stamp.sec << ".\n";
  ofs << "   header_stamp_nanosec: " << stamp.nanosec << ".\n";
  ofs << "   position: [ " << pose.position.x << ", "
      << pose.position.y << ", " << pose.position.z << " ]\n";
  ofs << "   orientation_xyzw: [ " << pose.orientation.x << ", "
      << pose.orientation.y << ", " << pose.orientation.z << ", "
      << pose.orientation.w << " ]\n";
  return true;
}

}  // namespace vision_calibration
