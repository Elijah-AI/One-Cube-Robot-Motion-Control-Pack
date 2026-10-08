/*******************************************************************************
 * camera_driver_node.cpp
 *
 * berxel_camera_driver : ROS2 driver node for Berxel Hawk (iHawk) depth cameras.
 *
 * 本文件以 camera_driver（Orbbec SDK 版）为功能基准逐项移植，仅替换底层获取
 * 图片原始数据的来源（Berxel Hawk SDK 替代 Orbbec SDK），其余功能完全一致：
 *   * 发布话题：/fusion/synced_robot_vision、/vision/rgb_raw(仅主相机、有订阅者时)、
 *     /SafeNode/Heartbeat、/SafeNode/status
 *   * 订阅话题：/Control/MotorInfo、/Control/HeadPose
 *   * Action 服务：/vision/Check_self(HardwareCheck)、/vision/Reset(HardwareReset)
 *   * 参数、状态机(UNKNOWN/INIT/CHECK/RESET/IDLE/RESET_DONE/RUNNING/EXCEPTION)、
 *     内部时间同步融合、多相机支持、主机时间同步、头部位姿点云坐标变换等
 *
 * Berxel SDK 无 pipeline/frameset/PointCloudFilter：彩色与深度帧通过各自的流
 * 回调（startStreams 的 onNewFrameCallback）独立到达。因此每个相机使用
 * “有界队列 + 本相机处理线程”的帧桥（见 CameraStreamState），SDK 回调线程把帧
 * 压入队列（帧所有权在 releaseFrame 前有效），处理线程取出配对的彩色/深度帧后
 * 统一处理；深度到彩色的对齐由设备侧 setRegistrationEnable() 完成。
 *******************************************************************************/
#include "berxel_camera_driver/camera_driver_node.hpp"

#include <algorithm>
#include <cmath>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <limits>

using namespace std::chrono_literals;

namespace berxel_camera_driver
{

// 时间融合固定参数（原 robot_time_sync_node，与 camera_driver 一致）
// 缓存需覆盖“相机帧打标时刻(cam_ts) → 融合执行时刻”的处理延迟：
// 点云构建等耗时可达 100~300ms，100 帧 @100Hz = 1s 窗口，保证能匹配到与
// cam_ts 真正接近的运控帧。
constexpr size_t MAX_MOTOR_CACHE = 100;
constexpr auto CAM_DELAY_AFTER = 2ms;          // 相机帧到达后的延迟融合时间
constexpr int64_t MAX_OFFSET_NS = 10'000'000;  // 10ms 最大同步误差

namespace
{
// Berxel 帧桥每个相机有界队列长度（SDK 回调线程 -> 本相机处理线程）
constexpr size_t kFrameQueueSize = 3;

// 由原始（出厂）内参所在的参考宽度推算当前活动宽度下的缩放系数：
// 深度帧由 1280/720/360 参考分辨率缩放得到，彩色帧由 1280 参考宽度缩放得到。
int pixelScaleForWidth(uint32_t width, bool isDepth)
{
  if (isDepth) {
    if (width == 640 || width == 568) return 2;
    if (width == 320 || width == 284) return 4;
    if (width == 160 || width == 100) return 8;
    return 1;
  }
  if (width == 640) return 2;
  if (width == 320) return 4;
  if (width == 160) return 8;
  return 1;
}
}  // namespace

// ============================================================================
// 初始化
// ============================================================================
void CameraDriverNode::init() {
  logger_ = get_logger();
  declareParameters();
  getParameters();

  // --- 融合结果发布器（原 robot_time_sync_node 的话题名称） ---
  // 融合已在本节点内部完成；多相机时所有相机都通过该话题发布，
  // 用 camera_id 字段区分来源相机（0=头部相机，1=腰部相机，2+预留）。
  fusion_pub_ = create_publisher<SyncedRobotVision>("/fusion/synced_robot_vision", rclcpp::QoS(10));

  // --- 原始 RGB 发布器（仅主相机）---
  // 话题 /vision/rgb_raw 只发送 RGB 原始数据（sensor_msgs/Image，RGB8）。
  // 通过 get_subscription_count() 检测订阅者：无订阅者时不发布。
  rgb_raw_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/rgb_raw", rclcpp::SensorDataQoS());

  // --- 订阅运控数据（原 robot_time_sync_node） ---
  sub_motor_ = create_subscription<MotorStatus>(
    "/Control/MotorInfo", rclcpp::QoS(20),
    std::bind(&CameraDriverNode::motor_callback, this, std::placeholders::_1));

  // --- 订阅头部位姿（/Control/HeadPose）：点云坐标变换（眼在手标定） ---
  // 取 end_effector_pose[0]（头部末端位姿）与固定标定矩阵相乘得到变换矩阵，
  // 在点云生成后将相机系点云变换到机器人基坐标系。
  sub_head_pose_ = create_subscription<MotorStatus>(
    "/Control/HeadPose", rclcpp::QoS(20),
    std::bind(&CameraDriverNode::head_pose_callback, this, std::placeholders::_1));

  // --- 延迟融合定时器：只创建一次，由 handle_camera_data 触发 ---
  fusion_timer_ = create_wall_timer(CAM_DELAY_AFTER, std::bind(&CameraDriverNode::fusion_cb, this));
  fusion_timer_->cancel();  // 初始为停止状态
  RCLCPP_INFO(logger_,
    "[TimeSync] fusion delay timer: %ldms, max sync offset: %ldms",
    CAM_DELAY_AFTER.count(), MAX_OFFSET_NS / 1000000);

  // --- 主机时间同步周期定时器：每 time_sync_period_ 秒重同步相机时钟到系统时间 ---
  if (enable_host_time_sync_) {
    const int64_t period_ms = static_cast<int64_t>(time_sync_period_ * 1000.0);
    if (period_ms > 0) {
      time_sync_timer_ = create_wall_timer(
          std::chrono::milliseconds(period_ms),
          std::bind(&CameraDriverNode::periodicTimeSyncCb, this));
      RCLCPP_INFO(logger_, "[TimeSync] host time sync timer created, period: %.1f s",
                  time_sync_period_);
    }
  }

  // --- 创建 HardwareCheck Action Server (/vision/Check_self) ---
  hw_check_server_ = rclcpp_action::create_server<HardwareCheck>(
    this,
    "/vision/Check_self",
    std::bind(&CameraDriverNode::handle_hw_check_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&CameraDriverNode::handle_hw_check_cancel, this, std::placeholders::_1),
    std::bind(&CameraDriverNode::handle_hw_check_accepted, this, std::placeholders::_1));
  RCLCPP_INFO(logger_, "HardwareCheck Action Server /vision/Check_self created.");

  // --- 创建 HardwareReset Action Server (/vision/Reset) ---
  hw_reset_server_ = rclcpp_action::create_server<HardwareReset>(
    this,
    "/vision/Reset",
    std::bind(&CameraDriverNode::handle_hw_reset_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&CameraDriverNode::handle_hw_reset_cancel, this, std::placeholders::_1),
    std::bind(&CameraDriverNode::handle_hw_reset_accepted, this, std::placeholders::_1));
  RCLCPP_INFO(logger_, "HardwareReset Action Server /vision/Reset created.");

  // --- Heartbeat & Status publishers ---
  hb_pub_ = create_publisher<com_interfaces::msg::HeartBeat>(
      "/SafeNode/Heartbeat", rclcpp::QoS(1).best_effort().durability_volatile());
  status_pub_ = create_publisher<com_interfaces::msg::NodeStatus>(
      "/SafeNode/status", rclcpp::QoS(10));

  // Heartbeat timer: 1Hz
  hb_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CameraDriverNode::heartbeat_cb, this));

  publish_status(CameraNodeSt::INIT, 0);

  // Initialize Berxel SDK context（进程内单例；重复调用返回同一实例）
  try {
    context_ = berxel::BerxelHawkContext::getBerxelContext();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Failed to create Berxel SDK context: %s", e.what());
    publish_status(CameraNodeSt::EXCEPTION, 0);
    return;
  }
  if (context_ == nullptr) {
    RCLCPP_ERROR(logger_, "Failed to create Berxel SDK context.");
    publish_status(CameraNodeSt::EXCEPTION, 0);
    return;
  }

  // Try to acquire cross-process device lock
  if (!acquireDeviceLock()) {
    RCLCPP_ERROR(logger_,
                 "Device is occupied by another process (berxel_camera_driver may be running).\n"
                 "Please stop the other instance first, or set parameter bypass_lock:=true to force access.");
    publish_status(CameraNodeSt::EXCEPTION, 0);
    return;
  }

  // Query and open device(s)（多相机时逐个打开所有相机）
  try {
    if (!openDevice()) {
      RCLCPP_ERROR(logger_, "Failed to open Berxel Hawk device");
      cleanupAfterInitFailure();
      publish_status(CameraNodeSt::EXCEPTION, 0);
      return;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Unexpected error opening device: %s", e.what());
    cleanupAfterInitFailure();
    publish_status(CameraNodeSt::EXCEPTION, 0);
    return;
  }

  // Configure and start streams（所有相机统一启动完整数据流）
  // Berxel SDK 无 pipeline：startStreams() 内部完成 registration 对齐配置、
  // 帧模式选择、内参与 CameraInfo 构建、主机时间同步以及帧桥线程的启动。
  try {
    if (!startStreams()) {
      RCLCPP_ERROR(logger_, "Failed to start streams");
      cleanupAfterInitFailure();
      publish_status(CameraNodeSt::EXCEPTION, 0);
      return;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Unexpected error starting streams: %s", e.what());
    cleanupAfterInitFailure();
    publish_status(CameraNodeSt::EXCEPTION, 0);
    return;
  }

  // Start frame rate monitoring timer (every 5 seconds)
  fps_last_check_ = std::chrono::steady_clock::now();
  fps_frame_count_ = 0;
  fps_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&CameraDriverNode::checkFrameRate, this));

  RCLCPP_INFO(logger_, "CameraDriverNode initialized successfully (%zu camera(s))", cameras_.size());
  change_status(CameraNodeSt::IDLE, 0);
}

/// Cleanup resources after init() failure (device lock already acquired)
void CameraDriverNode::cleanupAfterInitFailure() {
  stop();
  releaseDeviceLock();
  cleanupDeviceLock();
}

// ============================================================================
// 参数
// ============================================================================
void CameraDriverNode::declareParameters() {
  declare_parameter<int>("color_width", 640);
  declare_parameter<int>("color_height", 480);
  declare_parameter<int>("color_fps", 30);
  declare_parameter<int>("depth_width", 640);
  declare_parameter<int>("depth_height", 480);
  declare_parameter<int>("depth_fps", 30);
  declare_parameter<std::string>("serial_number", "");
  // 多相机 SN -> camera_id 映射（下标=camera_id，值=SN），字符串数组参数。
  // 例：camera_id_sn_map=["头部SN","腰部SN"] 表示 camera_id 0=头部，1=腰部。
  declare_parameter<std::vector<std::string>>("camera_id_sn_map", std::vector<std::string>{});
  declare_parameter<std::string>("frame_id", "camera_link");
  declare_parameter<std::string>("color_frame_id", "camera_color_optical_frame");
  declare_parameter<std::string>("depth_frame_id", "camera_depth_optical_frame");
  declare_parameter<bool>("bypass_lock", false);
  // enable_align is kept for backward compatibility (HW D2C).
  // Newer align_mode parameter takes precedence over enable_align.
  declare_parameter<bool>("enable_align", true);
  // align_mode: "OFF" | "HW" | "SW"。
  // Berxel SDK 只提供深度到彩色的设备内对齐（setRegistrationEnable），
  // 因此 HW 与 SW 均映射为 setRegistrationEnable(true)，OFF 映射为 false。
  declare_parameter<std::string>("align_mode", "HW");
  // align_target_stream: "COLOR"（默认；Berxel 仅支持 D2C，请求 "DEPTH" 会告警并回退 COLOR）
  declare_parameter<std::string>("align_target_stream", "COLOR");
  // 点云输出格式："rgb_point"（默认，RGB 点云，每点 xyz(float32)+rgb(uint32)）| "point"（仅坐标）
  declare_parameter<std::string>("pointcloud_format", "rgb_point");
  // 是否保留点云所有像素：true 时无效深度点用 NaN 表示（width=图像分辨率），false 时过滤无效点
  declare_parameter<bool>("pc_keep_invalid_points", true);
  // 点云坐标变换后的坐标系 frame_id（机器人基坐标系）
  declare_parameter<std::string>("pointcloud_base_frame_id", "base_link");
  // 主机时间同步：将相机内部时钟同步到主机系统时间（setSystemClock）
  declare_parameter<bool>("enable_host_time_sync", true);
  declare_parameter<double>("time_sync_period", 60.0);  // 秒，默认 1 分钟
}

void CameraDriverNode::getParameters() {
  color_width_ = get_parameter("color_width").as_int();
  color_height_ = get_parameter("color_height").as_int();
  color_fps_ = get_parameter("color_fps").as_int();
  depth_width_ = get_parameter("depth_width").as_int();
  depth_height_ = get_parameter("depth_height").as_int();
  depth_fps_ = get_parameter("depth_fps").as_int();
  serial_number_ = get_parameter("serial_number").as_string();
  camera_id_sn_map_ = get_parameter("camera_id_sn_map").as_string_array();
  frame_id_ = get_parameter("frame_id").as_string();
  color_frame_id_ = get_parameter("color_frame_id").as_string();
  depth_frame_id_ = get_parameter("depth_frame_id").as_string();
  bypass_lock_ = get_parameter("bypass_lock").as_bool();
  enable_align_ = get_parameter("enable_align").as_bool();
  align_mode_ = get_parameter("align_mode").as_string();
  align_target_stream_str_ = get_parameter("align_target_stream").as_string();
  pointcloud_format_ = get_parameter("pointcloud_format").as_string();
  pc_keep_invalid_points_ = get_parameter("pc_keep_invalid_points").as_bool();
  pointcloud_base_frame_id_ = get_parameter("pointcloud_base_frame_id").as_string();
  enable_host_time_sync_ = get_parameter("enable_host_time_sync").as_bool();
  time_sync_period_ = get_parameter("time_sync_period").as_double();
}

// ============================================================================
// HardwareCheck Action Server (自检)
// ============================================================================
rclcpp_action::GoalResponse CameraDriverNode::handle_hw_check_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const HardwareCheck::Goal> goal)
{
  RCLCPP_INFO(logger_, "[HardwareCheck] Received goal request with task_id: %s", goal->task_id.c_str());

  if (goal->priority < 1 || goal->priority > 25) {
    RCLCPP_WARN(logger_, "[HardwareCheck] Invalid priority: %d. Rejecting goal.", goal->priority);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CameraDriverNode::handle_hw_check_cancel(
  const std::shared_ptr<GoalHandleHardwareCheck> /*goal_handle*/)
{
  RCLCPP_INFO(logger_, "[HardwareCheck] Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CameraDriverNode::handle_hw_check_accepted(
  const std::shared_ptr<GoalHandleHardwareCheck> goal_handle)
{
  // Join any previous action thread before starting a new one
  {
    std::lock_guard<std::mutex> lock(action_mutex_);
    action_cancelled_ = false;
    if (action_thread_.joinable()) {
      action_thread_.join();
    }
    action_promise_ = std::make_unique<std::promise<void>>();
    action_thread_ = std::thread([this, goal_handle]() {
      execute_hw_check(goal_handle);
      {
        std::lock_guard<std::mutex> lock(action_mutex_);
        if (action_promise_) {
          action_promise_->set_value();
        }
      }
    });
  }
}

void CameraDriverNode::execute_hw_check(const std::shared_ptr<GoalHandleHardwareCheck> goal_handle)
{
  RCLCPP_INFO(logger_, "[HardwareCheck] Executing hardware self-check...");

  auto result = std::make_shared<HardwareCheck::Result>();
  auto feedback = std::make_shared<HardwareCheck::Feedback>();

  const auto goal = goal_handle->get_goal();

  auto send_feedback = [&](float progress, const std::string & step) -> bool {
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->status_code = 2;
      result->error_message = "Goal cancelled by client";
      goal_handle->canceled(result);
      RCLCPP_INFO(logger_, "[HardwareCheck] Goal canceled");
      return false;
    }
    feedback->progress_percent = progress;
    feedback->current_step = step;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(logger_, "[HardwareCheck] Feedback: %.1f%% - %s", progress, step.c_str());
    return true;
  };

  auto start_time = std::chrono::steady_clock::now();
  bool all_ok = true;
  std::string error_msg;

  // --- Step 1: Check device connection ---
  if (!send_feedback(10.0f, "Checking camera sensor connection...")) return;
  if (cameras_.empty()) {
    all_ok = false;
    error_msg = "No Berxel device connected";
    RCLCPP_ERROR(logger_, "[HardwareCheck] %s", error_msg.c_str());
  } else {
    RCLCPP_INFO(logger_, "[HardwareCheck] %zu camera(s) connected", cameras_.size());
  }

  // --- Step 2: Verify firmware version (主相机，index 0) ---
  if (all_ok && !send_feedback(30.0f, "Verifying firmware version...")) return;
  if (all_ok && !cameras_.empty()) {
    try {
      berxel::BerxelHawkDeviceInfo dev_info;
      std::memset(&dev_info, 0, sizeof(dev_info));
      berxel::BerxelHawkVersions ver;
      std::memset(&ver, 0, sizeof(ver));
      if (cameras_[0].device->getCurrentDeviceInfo(&dev_info) == 0) {
        std::string fw_str = "unknown";
        if (cameras_[0].device->getVersion(&ver) == 0) {
          char fw_buf[64];
          std::snprintf(fw_buf, sizeof(fw_buf), "%u.%u.%u",
                        static_cast<unsigned>(ver.fwVersion.major),
                        static_cast<unsigned>(ver.fwVersion.minor),
                        static_cast<unsigned>(ver.fwVersion.revision));
          fw_str = fw_buf;
        }
        RCLCPP_INFO(logger_, "[HardwareCheck] Device: %s, SN: %s, FW: %s",
                    dev_info.deviceAddress, dev_info.serialNumber, fw_str.c_str());
      }
    } catch (const std::exception &e) {
      all_ok = false;
      error_msg = "Failed to read device info: " + std::string(e.what());
      RCLCPP_ERROR(logger_, "[HardwareCheck] %s", error_msg.c_str());
    }
  }

  // --- Step 3: Test image sensor data validity ---
  if (all_ok && !send_feedback(50.0f, "Testing image sensor...")) return;
  if (all_ok && !cameras_.empty() && cameras_[0].device) {
    try {
      // Check if the stream is actively producing frames by verifying frame count
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
          now - fps_last_check_).count();
      if (elapsed >= 5 && fps_frame_count_ == 0) {
        all_ok = false;
        error_msg = "No frames received from camera stream";
        RCLCPP_ERROR(logger_, "[HardwareCheck] %s", error_msg.c_str());
      } else {
        RCLCPP_INFO(logger_, "[HardwareCheck] Stream active, frames received: %u",
                    fps_frame_count_);
      }
    } catch (const std::exception &e) {
      all_ok = false;
      error_msg = "Stream check failed: " + std::string(e.what());
    }
  }

  // --- Step 4: Validate calibration data (主相机，index 0) ---
  if (all_ok && !send_feedback(70.0f, "Validating calibration data...")) return;
  if (all_ok && !cameras_.empty()) {
    const auto &cam0 = cameras_[0];
    if (cam0.color_intrinsic_.fxParam <= 0 || cam0.color_intrinsic_.fyParam <= 0 ||
        cam0.depth_intrinsic_.fxParam <= 0 || cam0.depth_intrinsic_.fyParam <= 0) {
      all_ok = false;
      error_msg = "Invalid camera intrinsics (fx/fy are zero or negative)";
      RCLCPP_ERROR(logger_, "[HardwareCheck] %s", error_msg.c_str());
    } else {
      RCLCPP_INFO(logger_, "[HardwareCheck] Calibration valid: Color fx=%.2f, Depth fx=%.2f",
                  cam0.color_intrinsic_.fxParam, cam0.depth_intrinsic_.fxParam);
    }
  }

  // --- Step 5: Check frame rate ---
  if (all_ok && !send_feedback(90.0f, "Checking frame rate stability...")) return;
  if (all_ok) {
    double expected_fps = static_cast<double>(color_fps_);
    if (actual_color_fps_ > 0 && actual_color_fps_ < expected_fps * 0.5) {
      all_ok = false;
      error_msg = "Frame rate too low: actual=" +
                  std::to_string(actual_color_fps_) + "fps, expected=" +
                  std::to_string(expected_fps) + "fps";
      RCLCPP_WARN(logger_, "[HardwareCheck] %s", error_msg.c_str());
    } else {
      RCLCPP_INFO(logger_, "[HardwareCheck] Frame rate OK: %.1f fps", actual_color_fps_);
    }
  }

  // --- Finalize ---
  auto end_time = std::chrono::steady_clock::now();
  double execution_sec = std::chrono::duration<double>(end_time - start_time).count();

  if (all_ok) {
    if (!send_feedback(100.0f, "Hardware self-check completed successfully")) return;
    result->success = true;
    result->status_code = 0;
    result->error_message = "";
    result->execution_time = execution_sec;
    goal_handle->succeed(result);
    RCLCPP_INFO(logger_, "[HardwareCheck] Goal succeeded (%.2fs)", execution_sec);
  } else {
    result->success = false;
    result->status_code = 1;
    result->error_message = error_msg;
    result->execution_time = execution_sec;
    goal_handle->abort(result);
    RCLCPP_ERROR(logger_, "[HardwareCheck] Goal failed: %s", error_msg.c_str());
  }
}

// ============================================================================
// HardwareReset Action Server (复位)
// ============================================================================
rclcpp_action::GoalResponse CameraDriverNode::handle_hw_reset_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const HardwareReset::Goal> goal)
{
  RCLCPP_INFO(logger_, "[HardwareReset] Received goal request with task_id: %s", goal->task_id.c_str());

  if (goal->priority < 1 || goal->priority > 25) {
    RCLCPP_WARN(logger_, "[HardwareReset] Invalid priority: %d. Rejecting goal.", goal->priority);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CameraDriverNode::handle_hw_reset_cancel(
  const std::shared_ptr<GoalHandleHardwareReset> /*goal_handle*/)
{
  RCLCPP_INFO(logger_, "[HardwareReset] Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CameraDriverNode::handle_hw_reset_accepted(
  const std::shared_ptr<GoalHandleHardwareReset> goal_handle)
{
  // Join any previous action thread before starting a new one
  {
    std::lock_guard<std::mutex> lock(action_mutex_);
    action_cancelled_ = false;
    if (action_thread_.joinable()) {
      action_thread_.join();
    }
    action_promise_ = std::make_unique<std::promise<void>>();
    action_thread_ = std::thread([this, goal_handle]() {
      execute_hw_reset(goal_handle);
      {
        std::lock_guard<std::mutex> lock(action_mutex_);
        if (action_promise_) {
          action_promise_->set_value();
        }
      }
    });
  }
}

void CameraDriverNode::execute_hw_reset(const std::shared_ptr<GoalHandleHardwareReset> goal_handle)
{
  RCLCPP_INFO(logger_, "[HardwareReset] Executing hardware reset...");

  auto result = std::make_shared<HardwareReset::Result>();
  auto feedback = std::make_shared<HardwareReset::Feedback>();

  const auto goal = goal_handle->get_goal();

  auto send_feedback = [&](float progress, const std::string & step) -> bool {
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->status_code = 2;
      result->error_message = "Goal cancelled by client";
      goal_handle->canceled(result);
      RCLCPP_INFO(logger_, "[HardwareReset] Goal canceled");
      return false;
    }
    feedback->progress_percent = progress;
    feedback->current_step = step;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(logger_, "[HardwareReset] Feedback: %.1f%% - %s", progress, step.c_str());
    return true;
  };

  auto start_time = std::chrono::steady_clock::now();

  // --- Step 1 & 2: Stop camera streams & release device(s) ---
  if (!send_feedback(10.0f, "Stopping camera streams...")) return;
  try {
    stop();
  } catch (const std::exception &e) {
    RCLCPP_WARN(logger_, "[HardwareReset] Error stopping camera streams: %s", e.what());
  }

  // --- Step 3: Re-open device(s) and restart streams ---
  if (!send_feedback(60.0f, "Re-initializing sensor...")) return;
  bool reopen_ok = false;
  try {
    // 重新打开所有相机（多相机时逐个打开）；Berxel 无 pipeline
    if (openDevice()) {
      reopen_ok = startStreams();
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "[HardwareReset] Re-init failed: %s", e.what());
  }

  if (!reopen_ok) {
    result->success = false;
    result->status_code = 1;
    result->error_message = "Failed to re-initialize camera after reset";
    result->duration_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();
    goal_handle->abort(result);
    RCLCPP_ERROR(logger_, "[HardwareReset] Reset failed");
    return;
  }

  // --- Step 4: Verify device status (主相机，index 0) ---
  if (!send_feedback(80.0f, "Verifying device status...")) return;
  try {
    if (!cameras_.empty()) {
      berxel::BerxelHawkDeviceInfo dev_info;
      std::memset(&dev_info, 0, sizeof(dev_info));
      if (cameras_[0].device->getCurrentDeviceInfo(&dev_info) == 0) {
        RCLCPP_INFO(logger_, "[HardwareReset] Device reconnected: %s (SN: %s)",
                    dev_info.deviceAddress, dev_info.serialNumber);
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN(logger_, "[HardwareReset] Device info read failed: %s", e.what());
  }

  // --- Complete ---
  auto end_time = std::chrono::steady_clock::now();
  double duration_sec = std::chrono::duration<double>(end_time - start_time).count();

  if (!send_feedback(100.0f, "Hardware reset completed successfully")) return;

  result->success = true;
  result->status_code = 0;
  result->error_message = "";
  result->duration_sec = duration_sec;

  goal_handle->succeed(result);
  RCLCPP_INFO(logger_, "[HardwareReset] Goal succeeded (%.2fs)", duration_sec);
}

// ============================================================================
// 设备锁（跨进程互斥；Berxel 与 Orbbec 是物理上独立的不同设备，使用专属锁名）
// ============================================================================
bool CameraDriverNode::acquireDeviceLock() {
  if (bypass_lock_) {
    RCLCPP_WARN(logger_, "Bypassing device lock check via bypass_lock parameter");
    return true;
  }

  shm_fd_ = shm_open(BERXEL_DEVICE_LOCK_NAME, O_RDWR, 0666);
  if (shm_fd_ < 0) {
    RCLCPP_INFO(logger_,
                 "No device lock detected (berxel_camera_driver not running), accessing device directly");
    return true;
  }

  shm_addr_ = static_cast<uint8_t *>(
      mmap(nullptr, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
  if (shm_addr_ == MAP_FAILED) {
    RCLCPP_WARN(logger_, "Failed to map device lock shared memory (errno=%d), skipping lock check", errno);
    close(shm_fd_);
    shm_fd_ = -1;
    return true;
  }

  device_lock_ = reinterpret_cast<pthread_mutex_t *>(shm_addr_);

  int ret = pthread_mutex_trylock(device_lock_);
  if (ret == EBUSY) {
    RCLCPP_ERROR(logger_,
                 "Device is occupied by berxel_camera_driver.\n"
                 "Please stop it, or set parameter --ros-args -p bypass_lock:=true");
    return false;
  }
  if (ret != 0) {
    RCLCPP_WARN(logger_, "pthread_mutex_trylock returned %d, continuing", ret);
  }

  lock_acquired_ = true;
  RCLCPP_INFO(logger_, "Device lock acquired");
  return true;
}

void CameraDriverNode::releaseDeviceLock() {
  if (lock_acquired_ && device_lock_) {
    pthread_mutex_unlock(device_lock_);
    lock_acquired_ = false;
  }
}

void CameraDriverNode::cleanupDeviceLock() {
  releaseDeviceLock();
  if (shm_addr_ != nullptr && shm_addr_ != MAP_FAILED) {
    munmap(shm_addr_, sizeof(pthread_mutex_t));
    shm_addr_ = nullptr;
  }
  if (shm_fd_ >= 0) {
    close(shm_fd_);
    shm_fd_ = -1;
  }
  device_lock_ = nullptr;
}

// ============================================================================
// 设备操作（Berxel：getDeviceList -> openDevice 逐个打开）
// ============================================================================
bool CameraDriverNode::openDevice() {
  if (context_ == nullptr) {
    try {
      context_ = berxel::BerxelHawkContext::getBerxelContext();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger_, "Failed to create Berxel SDK context: %s", e.what());
      return false;
    }
    if (context_ == nullptr) {
      RCLCPP_ERROR(logger_, "Failed to create Berxel SDK context.");
      return false;
    }
  }

  berxel::BerxelHawkDeviceInfo * device_list = nullptr;
  uint32_t device_count = 0;
  try {
    const int32_t ret = context_->getDeviceList(&device_list, &device_count);
    if (ret != 0) {
      RCLCPP_ERROR(logger_, "Failed to query device list (ret=%d)", ret);
      return false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Failed to query device list: %s", e.what());
    return false;
  }

  if (device_list == nullptr || device_count == 0) {
    RCLCPP_ERROR(logger_, "No Berxel Hawk device found");
    return false;
  }

  RCLCPP_INFO(logger_, "Found %u device(s)", device_count);

  // 打印所有检测到的设备信息（含 SN 码与设备地址），便于多相机场景识别
  for (uint32_t i = 0; i < device_count; i++) {
    RCLCPP_INFO(logger_, "  Device[%u]: addr=%s (SN: %s, type: 0x%x)",
                i, device_list[i].deviceAddress, device_list[i].serialNumber,
                device_list[i].deviceType);
  }

  // 清空旧设备状态（stop 由调用方负责；此处仅清空本节点持有的相机状态）
  cameras_.clear();
  device_ = nullptr;

  // 建立 设备枚举索引 -> SN/名称 的查询表（名称取设备地址）
  std::vector<std::string> sn_by_index(device_count, "");
  std::vector<std::string> name_by_index(device_count, "");
  for (uint32_t i = 0; i < device_count; i++) {
    sn_by_index[i] = device_list[i].serialNumber;
    name_by_index[i] = device_list[i].deviceAddress;
  }

  // 打开设备：SDK openDevice 按值传入；返回 nullptr 表示失败（仍包 try/catch）
  auto try_open = [this](const berxel::BerxelHawkDeviceInfo & dev_info)
      -> berxel::BerxelHawkDevice * {
    try {
      return context_->openDevice(dev_info);
    } catch (const std::exception &e) {
      RCLCPP_WARN(logger_, "openDevice() threw: %s", e.what());
      return nullptr;
    }
  };

  // 打开一个设备并追加到 cameras_（CameraStreamState 不可移动，仅 emplace_back）
  auto add_camera = [this, &try_open](
      const berxel::BerxelHawkDeviceInfo & dev_info, uint8_t camera_id,
      const std::string & sn, const std::string & name) -> bool {
    berxel::BerxelHawkDevice * dev = try_open(dev_info);
    if (dev == nullptr) {
      RCLCPP_ERROR(logger_, "openDevice() failed (SN=%s, addr=%s).",
                   sn.c_str(), name.c_str());
      return false;
    }
    cameras_.emplace_back();
    CameraStreamState & cam = cameras_.back();
    cam.device = dev;
    cam.camera_id = camera_id;
    cam.serial_number = sn;
    cam.device_name = name;
    cam.callback_ctx.node = this;
    cam.callback_ctx.cam_index = cameras_.size() - 1;
    return true;
  };

  if (!camera_id_sn_map_.empty()) {
    // ── 情况1：配置了 camera_id_sn_map（多相机 SN -> camera_id 映射） ──
    RCLCPP_INFO(logger_, "Using camera_id_sn_map (%zu entry/ies) to map SN -> camera_id:", camera_id_sn_map_.size());
    for (size_t cid = 0; cid < camera_id_sn_map_.size(); ++cid) {
      const std::string & target_sn = camera_id_sn_map_[cid];
      RCLCPP_INFO(logger_, "  camera_id=%zu -> SN=%s", cid, target_sn.c_str());
      if (target_sn.empty()) {
        RCLCPP_WARN(logger_, "  camera_id=%zu has empty SN, skipped", cid);
        continue;
      }
      bool found = false;
      for (uint32_t i = 0; i < device_count; i++) {
        if (sn_by_index[i] == target_sn) {
          if (add_camera(device_list[i], static_cast<uint8_t>(cid), target_sn, name_by_index[i])) {
            found = true;
            RCLCPP_INFO(logger_, "  camera_id=%zu -> device index %u opened (SN: %s)",
                        cid, i, target_sn.c_str());
          }
          break;
        }
      }
      if (!found) {
        RCLCPP_ERROR(logger_, "  No device found with SN '%s' for camera_id=%zu",
                     target_sn.c_str(), cid);
      }
    }
    if (cameras_.empty()) {
      RCLCPP_ERROR(logger_,
                   "Failed to open any camera according to camera_id_sn_map.\n"
                   "  Check that the SNs in camera_id_sn_map match the detected devices above.");
      return false;
    }
  } else if (!serial_number_.empty()) {
    // ── 情况2：仅指定单个 SN（兼容原逻辑），camera_id=0 ──
    for (uint32_t i = 0; i < device_count; i++) {
      if (sn_by_index[i] == serial_number_) {
        if (add_camera(device_list[i], 0, serial_number_, name_by_index[i])) {
          RCLCPP_INFO(logger_, "Selected device by serial number: %s (camera_id=0)", serial_number_.c_str());
        }
        break;
      }
    }
    if (cameras_.empty()) {
      RCLCPP_ERROR(logger_, "No device found with serial number '%s'", serial_number_.c_str());
      return false;
    }
  } else {
    // ── 情况3：未配置映射，逐个打开所有设备，camera_id 按枚举顺序分配（0,1,2,...） ──
    for (uint32_t i = 0; i < device_count; i++) {
      if (!add_camera(device_list[i], static_cast<uint8_t>(i), sn_by_index[i], name_by_index[i])) {
        RCLCPP_WARN(logger_,
                    "Failed to open device index %u.\n"
                    "  Hint: Check USB permissions (udev rules) or if device is already in use.",
                    i);
        continue;
      }
    }
    if (cameras_.empty()) {
      RCLCPP_ERROR(logger_,
                   "Failed to open any of the %u detected device(s).\n"
                   "  Common causes:\n"
                   "  1. Missing USB permissions\n"
                   "  2. Device already in use by another berxel process\n"
                   "  3. Insufficient power to USB port",
                   device_count);
      return false;
    }
    RCLCPP_INFO(logger_, "Opened %zu camera(s), camera_id assigned by enumeration order", cameras_.size());
  }

  // 汇总打印每个相机的 camera_id / SN 对应关系
  RCLCPP_INFO(logger_, "Camera mapping (camera_id -> device):");
  for (const auto &cam : cameras_) {
    RCLCPP_INFO(logger_, "  camera_id=%u -> %s (SN: %s)",
                cam.camera_id, cam.device_name.c_str(), cam.serial_number.c_str());
  }

  // 主相机（index 0）句柄，保持原有单相机逻辑兼容
  device_ = cameras_[0].device;

  return device_ != nullptr;
}

bool CameraDriverNode::startStreams() {
  if (cameras_.empty()) {
    RCLCPP_ERROR(logger_, "No camera opened");
    return false;
  }

  // 重新武装关闭标记（stop() 置位后，本节点复位/重开流程需要复位）
  shutting_down_ = false;

  // 分配每相机最新数据槽位（按 camera_id 索引）
  latest_cam_data_by_cam_.assign(cameras_.size(), nullptr);

  // ── 归一化图像配准参数（全局，一次） ──
  if (align_mode_ == "OFF") {
    enable_align_ = false;
  } else if (align_mode_ == "SW") {
    // Berxel SDK 无节点内软件对齐管线；SW 与 HW 一致地走设备内 D2C 对齐。
    // 仅关闭对齐（"OFF"）时才真正禁用。
    enable_align_ = false;
  } else if (align_mode_ != "HW") {
    RCLCPP_WARN(logger_, "Unknown align_mode '%s', defaulting to 'HW'", align_mode_.c_str());
    align_mode_ = "HW";
  }
  // Berxel SDK 仅支持 深度->彩色（D2C）设备内对齐。
  if (align_target_stream_str_ == "DEPTH") {
    RCLCPP_WARN(logger_,
                "Berxel SDK supports depth->color alignment only; "
                "forcing align_target_stream=COLOR");
    align_target_stream_str_ = "COLOR";
  } else {
    align_target_stream_str_ = "COLOR";
  }
  depth_registration_ = (align_mode_ != "OFF");
  RCLCPP_INFO(logger_,
              "Image registration: align_mode=%s, align_target_stream=%s, enable_align=%d",
              align_mode_.c_str(), align_target_stream_str_.c_str(), enable_align_ ? 1 : 0);

  // 所有相机统一启动完整数据流（D2C 对齐 + 融合）
  stream_flags_ = berxel::BERXEL_HAWK_COLOR_STREAM | berxel::BERXEL_HAWK_DEPTH_STREAM;
  bool all_ok = true;
  for (size_t i = 0; i < cameras_.size(); ++i) {
    if (!startStreamForCamera(i)) {
      RCLCPP_ERROR(logger_, "Failed to start streams for camera index %zu (camera_id=%u)",
                   i, cameras_[i].camera_id);
      all_ok = false;
    }
  }
  RCLCPP_INFO(logger_, "Started %zu camera stream(s)", cameras_.size());
  return all_ok;
}

// ============================================================================
// 主机时间同步（Berxel：setSystemClock() 将相机内部时钟同步到主机系统时间）
// ============================================================================
void CameraDriverNode::syncHostTimeOnce(size_t cam_index) {
  if (cam_index >= cameras_.size()) {
    return;
  }
  CameraStreamState &cam = cameras_[cam_index];
  if (!cam.device) {
    return;
  }
  try {
    // setSystemClock() 成功后，BerxelHawkFrame::getTimeStamp() 返回与主机对齐的
    // 全局时间戳（Unix 微秒）。
    if (cam.device->setSystemClock() == 0) {
      cam.time_synced_ = true;
      RCLCPP_INFO(logger_, "[Camera %zu] host time synced (SN=%s)",
                  cam_index, cam.serial_number.c_str());
    } else {
      cam.time_synced_ = false;
      RCLCPP_WARN(logger_, "[Camera %zu] setSystemClock() failed", cam_index);
    }
  } catch (const std::exception &e) {
    cam.time_synced_ = false;
    RCLCPP_WARN(logger_, "[Camera %zu] host time sync failed: %s",
                cam_index, e.what());
  }
}

void CameraDriverNode::periodicTimeSyncCb() {
  if (!enable_host_time_sync_) {
    return;
  }
  for (size_t i = 0; i < cameras_.size(); ++i) {
    auto &cam = cameras_[i];
    if (!cam.device || !cam.time_synced_) {
      continue;
    }
    try {
      if (cam.device->setSystemClock() == 0) {
        RCLCPP_INFO(logger_, "[Camera %zu] periodic host time resync ok", i);
      } else {
        RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 10000,
          "[Camera %zu] periodic host time resync failed", i);
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 10000,
        "[Camera %zu] periodic time resync failed: %s", i, e.what());
    }
  }
}

// ============================================================================
// 单相机启动完整数据流
// ============================================================================
bool CameraDriverNode::startStreamForCamera(size_t cam_index) {
  if (cam_index >= cameras_.size()) {
    return false;
  }
  CameraStreamState &cam = cameras_[cam_index];
  berxel::BerxelHawkDevice * device = cam.device;
  if (!device) {
    RCLCPP_ERROR(logger_, "[Camera %zu] Device is null", cam_index);
    return false;
  }

  try {
    // ── 深度到彩色注册（设备内 D2C 对齐；等效 camera_driver 的 HW D2C） ──
    {
      const int32_t r = device->setRegistrationEnable(depth_registration_);
      if (r != 0) {
        RCLCPP_WARN(logger_, "[Camera %zu] setRegistrationEnable(%d) returned %d",
                    cam_index, depth_registration_ ? 1 : 0, r);
      }
    }

    // ── 流标志模式：MIX_HD（彩色 + 深度同开） ──
    {
      const int32_t r = device->setStreamFlagMode(berxel::BERXEL_HAWK_MIX_HD_STREAM_FLAG_MODE);
      if (r != 0) {
        RCLCPP_WARN(logger_, "[Camera %zu] setStreamFlagMode(MIX_HD) returned %d",
                    cam_index, r);
      }
    }
    RCLCPP_INFO(logger_, "[Camera %zu] Using MIX_HD stream flag mode (color + depth)", cam_index);

    // ── 选择彩色 / 深度帧模式并记录活动分辨率 ──
    if (!applyFrameMode(cam_index, berxel::BERXEL_HAWK_COLOR_STREAM,
                        color_width_, color_height_, color_fps_)) {
      return false;
    }
    if (!applyFrameMode(cam_index, berxel::BERXEL_HAWK_DEPTH_STREAM,
                        depth_width_, depth_height_, depth_fps_)) {
      return false;
    }

    // ── 内参与 CameraInfo 预构建 ──
    readIntrinsics(cam_index);
    updateIntrinsicsForResolution(cam_index);
    buildCameraInfoMessages(cam_index);

    // ── 主机时间同步：流启动前将相机时钟同步到系统时间 ──
    if (enable_host_time_sync_) {
      syncHostTimeOnce(cam_index);
    }

    // ── 启动帧桥（本相机处理线程），再启动 SDK 回调流 ──
    {
      std::lock_guard<std::mutex> lock(cam.frame_mutex_);
      cam.process_running_ = true;
    }
    cam.process_thread_ = std::thread(&CameraDriverNode::processLoop, this, cam_index);

    const int ret = device->startStreams(
        stream_flags_, &CameraDriverNode::onNewFrameCallback, &cam.callback_ctx);
    if (ret != 0) {
      RCLCPP_ERROR(logger_, "[Camera %zu] startStreams() failed with %d", cam_index, ret);
      {
        std::lock_guard<std::mutex> lock(cam.frame_mutex_);
        cam.process_running_ = false;
        cam.frame_cv_.notify_all();
      }
      if (cam.process_thread_.joinable()) {
        cam.process_thread_.join();
      }
      return false;
    }

    RCLCPP_INFO(logger_, "[Camera %zu] Streams started successfully (camera_id=%u)",
                cam_index, cam.camera_id);
    return true;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "[Camera %zu] Failed to start streams: %s", cam_index, e.what());
    return false;
  }
}

// ============================================================================
// 帧模式选择与设置（Berxel：getSupportFrameModes / getCurrentFrameMode / setFrameMode）
//   先在设备支持列表中自动挑选可用模式（分辨率精确命中优先，否则像素面积最近；
//   帧率在所选分辨率内取最接近请求值），绝不把不支持的 (w,h)/(fps) 强塞给 SDK ——
//   否则 SDK 报 “Frame mode invalid”，流不投递任何帧，节点会卡在 IDLE 无帧输出。
//   无支持列表时回退当前帧模式并尽力覆盖请求值。setFrameMode 失败视为致命错误
//   （返回 false，让上层可见地报错），而不是静默继续。
//   成功后把活动分辨率记录到 cam.color/depth_width_/height_。
// ============================================================================
bool CameraDriverNode::applyFrameMode(
    size_t cam_index, berxel::BerxelHawkStreamType stream_type,
    int32_t want_w, int32_t want_h, int32_t want_fps) {
  if (cam_index >= cameras_.size()) {
    return false;
  }
  CameraStreamState &cam = cameras_[cam_index];
  if (cam.device == nullptr) {
    return false;
  }
  const bool is_color = (stream_type == berxel::BERXEL_HAWK_COLOR_STREAM);
  const char * stream_name = is_color ? "Color" : "Depth";
  try {
    berxel::BerxelHawkStreamFrameMode mode{};
    bool have_mode = false;

    // ── 1) 读取 SDK 支持列表并记录 ──
    const berxel::BerxelHawkStreamFrameMode * modes = nullptr;
    uint32_t count = 0;
    const bool have_list =
        (cam.device->getSupportFrameModes(stream_type, &modes, &count) == 0 &&
         modes != nullptr && count > 0);

    if (have_list) {
      std::string supported;
      for (uint32_t i = 0; i < count; ++i) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%dx%d@%d ",
                      modes[i].resolutionX, modes[i].resolutionY, modes[i].framerate);
        supported += buf;
      }
      RCLCPP_INFO(logger_, "[Camera %zu] Supported %s modes: [%s]",
                  cam_index, stream_name, supported.c_str());

      // ── 2) 在支持列表中挑选最佳模式（绝不强制不支持的 (w,h)/(fps)） ──
      //      分辨率：精确命中优先，否则像素面积差最小；同差取更大 X。
      const int64_t want_area = static_cast<int64_t>(want_w) * want_h;
      int32_t res_idx = -1;
      int64_t best_area_delta = 0;
      for (uint32_t i = 0; i < count; ++i) {
        const int64_t area =
            static_cast<int64_t>(modes[i].resolutionX) * modes[i].resolutionY;
        const int64_t d = (area > want_area) ? (area - want_area) : (want_area - area);
        if (res_idx < 0 ||
            d < best_area_delta ||
            (d == best_area_delta &&
             modes[i].resolutionX > modes[res_idx].resolutionX)) {
          res_idx = static_cast<int32_t>(i);
          best_area_delta = d;
        }
      }
      //      帧率：在该分辨率内取最接近请求值（同差取更高帧率；未指定取最高）。
      const int32_t res_x = modes[res_idx].resolutionX;
      const int32_t res_y = modes[res_idx].resolutionY;
      int32_t fps_idx = -1;
      int32_t best_fps_delta = 0;
      for (uint32_t i = 0; i < count; ++i) {
        if (modes[i].resolutionX != res_x || modes[i].resolutionY != res_y) {
          continue;
        }
        if (want_fps <= 0) {
          if (fps_idx < 0 || modes[i].framerate > modes[fps_idx].framerate) {
            fps_idx = static_cast<int32_t>(i);
          }
          continue;
        }
        const int32_t diff = static_cast<int32_t>(modes[i].framerate) - want_fps;
        const int32_t d = (diff < 0) ? -diff : diff;
        if (fps_idx < 0 ||
            d < best_fps_delta ||
            (d == best_fps_delta &&
             modes[i].framerate > modes[fps_idx].framerate)) {
          fps_idx = static_cast<int32_t>(i);
          best_fps_delta = d;
        }
      }
      mode = modes[static_cast<uint32_t>(fps_idx >= 0 ? fps_idx : res_idx)];
      have_mode = true;
    }

    // ── 3) 请求与所选模式不一致时给出明确提示 ──
    if (have_mode) {
      const bool exact =
          (static_cast<int32_t>(mode.resolutionX) == want_w &&
           static_cast<int32_t>(mode.resolutionY) == want_h &&
           (want_fps <= 0 ||
            static_cast<int32_t>(mode.framerate) == want_fps));
      if (!exact) {
        RCLCPP_WARN(logger_,
                    "[Camera %zu] Requested %s mode %dx%d@%d is not exactly supported; "
                    "auto-selected nearest supported %dx%d@%d.",
                    cam_index, stream_name, want_w, want_h, want_fps,
                    static_cast<int32_t>(mode.resolutionX),
                    static_cast<int32_t>(mode.resolutionY),
                    static_cast<int32_t>(mode.framerate));
      }
    } else {
      // 无支持列表：以当前帧模式为基准，仅当请求值有效时才覆盖（尽力而为）。
      if (cam.device->getCurrentFrameMode(stream_type, &mode) != 0) {
        RCLCPP_ERROR(logger_, "[Camera %zu] getCurrentFrameMode(%s) failed",
                     cam_index, stream_name);
        return false;
      }
      if (want_w > 0) {
        mode.resolutionX = static_cast<int16_t>(want_w);
      }
      if (want_h > 0) {
        mode.resolutionY = static_cast<int16_t>(want_h);
      }
      if (want_fps > 0) {
        mode.framerate = static_cast<int8_t>(want_fps);
      }
      RCLCPP_WARN(logger_,
                  "[Camera %zu] SDK returned no %s mode list; applying current mode "
                  "(%dx%d@%d) with requested override.",
                  cam_index, stream_name,
                  static_cast<int32_t>(mode.resolutionX),
                  static_cast<int32_t>(mode.resolutionY),
                  static_cast<int32_t>(mode.framerate));
    }

    // ── 4) 设置帧模式：失败视为致命（否则流不投递任何帧，节点卡在 IDLE） ──
    const int32_t r = cam.device->setFrameMode(stream_type, &mode);
    if (r != 0) {
      RCLCPP_ERROR(logger_,
                   "[Camera %zu] setFrameMode(%s) %dx%d@%d FAILED (ret=%d). "
                   "The %s stream will NOT deliver frames; pick a supported mode from "
                   "the list above or lower the requested resolution/fps.",
                   cam_index, stream_name,
                   static_cast<int32_t>(mode.resolutionX),
                   static_cast<int32_t>(mode.resolutionY),
                   static_cast<int32_t>(mode.framerate), r, stream_name);
      return false;
    }

    // ── 5) 读回实际生效的帧模式并记录活动分辨率 ──
    berxel::BerxelHawkStreamFrameMode active{};
    if (cam.device->getCurrentFrameMode(stream_type, &active) == 0) {
      if (is_color) {
        cam.color_width_ = active.resolutionX;
        cam.color_height_ = active.resolutionY;
      } else {
        cam.depth_width_ = active.resolutionX;
        cam.depth_height_ = active.resolutionY;
      }
      RCLCPP_INFO(logger_, "[Camera %zu] Active %s mode: %d x %d @ %d fps",
                  cam_index, stream_name, active.resolutionX, active.resolutionY,
                  active.framerate);
      if (active.resolutionX != mode.resolutionX ||
          active.resolutionY != mode.resolutionY ||
          active.framerate != mode.framerate) {
        RCLCPP_WARN(logger_,
                    "[Camera %zu] Device did not adopt requested %s mode %dx%d@%d; "
                    "it is running %dx%d@%d.",
                    cam_index, stream_name,
                    static_cast<int32_t>(mode.resolutionX),
                    static_cast<int32_t>(mode.resolutionY),
                    static_cast<int32_t>(mode.framerate),
                    active.resolutionX, active.resolutionY, active.framerate);
      }
    } else {
      // 读回失败：按请求值记录活动分辨率
      if (is_color) {
        cam.color_width_ = want_w;
        cam.color_height_ = want_h;
      } else {
        cam.depth_width_ = want_w;
        cam.depth_height_ = want_h;
      }
    }
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "[Camera %zu] applyFrameMode(%s) error: %s",
                 cam_index, stream_name, e.what());
    return false;
  }
}

// ============================================================================
// Berxel 帧桥：SDK 回调 -> 有界队列 -> 本相机处理线程
// ============================================================================
void CameraDriverNode::onNewFrameCallback(
  berxel::BerxelHawkStreamType stream_type, berxel::BerxelHawkFrame * p_frame,
  void * p_user_data)
{
  if (p_user_data == nullptr || p_frame == nullptr) {
    return;
  }
  auto * ctx = static_cast<CameraStreamState::CallbackCtx *>(p_user_data);
  if (ctx->node) {
    ctx->node->pushFrame(ctx->cam_index, stream_type, p_frame);
  }
}

void CameraDriverNode::pushFrame(
  size_t cam_index, berxel::BerxelHawkStreamType stream_type,
  berxel::BerxelHawkFrame * p_frame)
{
  if (cam_index >= cameras_.size()) {
    return;
  }
  CameraStreamState &cam = cameras_[cam_index];

  std::lock_guard<std::mutex> lock(cam.frame_mutex_);
  if (shutting_down_ || !cam.process_running_) {
    // 关闭/未启动期间收到的帧直接归还 SDK
    if (cam.device) {
      berxel::BerxelHawkFrame * f = p_frame;
      cam.device->releaseFrame(f);
    }
    return;
  }

  const bool is_color = (stream_type == berxel::BERXEL_HAWK_COLOR_STREAM);
  auto & q = is_color ? cam.color_queue_ : cam.depth_queue_;

  // Bounded queue: drop the oldest frame when full.
  if (q.size() >= kFrameQueueSize && cam.device) {
    berxel::BerxelHawkFrame * old = q.front();
    q.pop();
    cam.device->releaseFrame(old);
  }
  q.push(p_frame);
  cam.frame_cv_.notify_all();
}

void CameraDriverNode::processLoop(size_t cam_index) {
  if (cam_index >= cameras_.size()) {
    return;
  }
  CameraStreamState &cam = cameras_[cam_index];
  RCLCPP_INFO(logger_, "[Camera %zu] Process thread started.", cam_index);
  while (rclcpp::ok() && cam.process_running_) {
    berxel::BerxelHawkFrame * color_frame = nullptr;
    berxel::BerxelHawkFrame * depth_frame = nullptr;

    {
      std::unique_lock<std::mutex> lock(cam.frame_mutex_);
      const bool got = cam.frame_cv_.wait_for(
        lock, std::chrono::milliseconds(200),
        [&cam]() {
          return !cam.process_running_ ||
                 (!cam.color_queue_.empty() && !cam.depth_queue_.empty());
        });
      if (!cam.process_running_) {
        break;
      }
      if (!got) {
        continue;
      }
      color_frame = cam.color_queue_.front();
      cam.color_queue_.pop();
      depth_frame = cam.depth_queue_.front();
      cam.depth_queue_.pop();
    }

    handleFramePair(cam_index, color_frame, depth_frame);

    if (cam.device) {
      berxel::BerxelHawkFrame * f = depth_frame;
      cam.device->releaseFrame(f);
      f = color_frame;
      cam.device->releaseFrame(f);
    }
  }
  RCLCPP_INFO(logger_, "[Camera %zu] Process thread stopped.", cam_index);
}

// ============================================================================
// 本相机处理线程取到一对彩色/深度帧后构建 SyncedRobotVision 并提交融合。
// 对应 camera_driver 的 onFrameSet()。
// ============================================================================
void CameraDriverNode::handleFramePair(
  size_t cam_index, berxel::BerxelHawkFrame * p_color,
  berxel::BerxelHawkFrame * p_depth)
{
  if (!p_color || !p_depth || cam_index >= cameras_.size()) {
    return;
  }

  CameraStreamState &cam = cameras_[cam_index];
  const uint8_t camera_id = cam.camera_id;

  try {
    // Update frame rate counters
    fps_frame_count_++;

    auto rgbd_msg = std::make_shared<com_interfaces::msg::SyncedRobotVision>();

    // 相机时间戳：优先使用相机报告的全局时间戳（主机时间同步后为 Unix 微秒），
    // 同步失败或数值无效时回退到系统时间 now()。
    rclcpp::Time color_ts;
    bool use_camera_ts = false;
    constexpr uint64_t kMinValidGlobalUs = 1600000000000000ULL;  // 约 2020-09-13 UTC
    if (enable_host_time_sync_ && cam.time_synced_) {
      try {
        const uint64_t global_us = p_color->getTimeStamp();
        if (global_us > kMinValidGlobalUs) {
          color_ts = fromUsToROSTime(global_us);
          use_camera_ts = true;
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 10000,
          "[Camera %zu] getTimeStamp failed: %s, fallback to now()",
          cam_index, e.what());
      }
    }
    if (!use_camera_ts) {
      color_ts = now();
    }

    // 诊断：打印相机帧时间戳（SDK 原始 us、转换后 ROS 时间、系统时间）
    // 时间节流每 2 秒打印一次。use_camera_ts=1 表示使用了相机全局时间戳。
    {
      const uint64_t sdk_us = p_color->getTimeStamp();
      const rclcpp::Time sys_now = now();
      RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 2000,
        "[Camera %zu] cam_ts diag: sdk_us=%llu, color_ts=%.9f, now=%.9f, use_camera_ts=%d",
        cam_index, static_cast<unsigned long long>(sdk_us),
        color_ts.seconds(), sys_now.seconds(), static_cast<int>(use_camera_ts));
    }

    // 非主相机使用带 camera_id 的 frame_id 前缀，避免多相机 frame_id 冲突
    std::string fid = frame_id_;
    std::string color_fid = color_frame_id_;
    std::string depth_fid = depth_frame_id_;
    if (camera_id != 0) {
      fid = "camera_" + std::to_string(camera_id) + "_link";
      color_fid = "camera_" + std::to_string(camera_id) + "_color_optical_frame";
      depth_fid = "camera_" + std::to_string(camera_id) + "_depth_optical_frame";
    }

    rgbd_msg->header.stamp = color_ts;
    rgbd_msg->header.frame_id = fid;

    rgbd_msg->camera_id = camera_id;

    // Use pre-built CameraInfo (only update timestamp per frame)
    rgbd_msg->camera_info = cam.rgb_camera_info_;
    rgbd_msg->camera_info.header.stamp = color_ts;

    // Color Image (RGB8)
    {
      const uint32_t cw = p_color->getWidth();
      const uint32_t ch = p_color->getHeight();
      const size_t expected = static_cast<size_t>(cw) * ch * 3;
      const uint8_t * data = static_cast<const uint8_t *>(p_color->getData());
      const size_t avail = (data != nullptr)
          ? std::min<size_t>(p_color->getDataSize(), expected) : 0;

      sensor_msgs::msg::Image rgb_msg;
      rgb_msg.header.stamp = color_ts;
      rgb_msg.header.frame_id = color_fid;
      rgb_msg.height = ch;
      rgb_msg.width = cw;
      rgb_msg.encoding = sensor_msgs::image_encodings::RGB8;
      rgb_msg.is_bigendian = false;
      rgb_msg.step = cw * 3;
      rgb_msg.data.resize(expected, 0);
      if (avail > 0) {
        std::memcpy(rgb_msg.data.data(), data, avail);
      }
      if (avail < expected) {
        RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 5000,
          "[Camera %zu] Color frame data size (%zu) < expected (%zu).",
          cam_index, static_cast<size_t>(p_color->getDataSize()), expected);
      }
      // 新增话题 /vision/rgb_raw：仅当存在订阅者时发布主相机（camera_id==0）的 RGB
      // 原始数据，无订阅者时不发布（get_subscription_count() == 0）。
      if (camera_id == 0 && rgb_raw_pub_ && rgb_raw_pub_->get_subscription_count() > 0) {
        rgb_raw_pub_->publish(rgb_msg);
      }
      rgbd_msg->camera_rgbd_color = std::move(rgb_msg);
    }

    // Point Cloud（RGB 点云，单位米）
    {
      const uint32_t depth_w = p_depth->getWidth();
      const uint32_t depth_h = p_depth->getHeight();
      const uint32_t color_w = p_color->getWidth();
      const uint32_t color_h = p_color->getHeight();

      // registration（设备内 D2C）实际生效时，深度帧分辨率 == 彩色分辨率；
      // 否则点云位于深度光学坐标系，frame_id 必须相应切换。
      const bool depth_aligned = depth_registration_ &&
          (depth_w == color_w && depth_h == color_h);
      if (depth_registration_ && !depth_aligned) {
        RCLCPP_WARN_THROTTLE(
            logger_, *get_clock(), 5000,
            "[Camera %zu] D2C alignment not in effect (depth %ux%u != color %ux%u). "
            "Publishing point cloud in depth optical frame.",
            cam_index, depth_w, depth_h, color_w, color_h);
      }
      const std::string cloud_fid = depth_aligned ? color_fid : depth_fid;

      buildPointCloud(cam_index, p_depth, p_color, cloud_fid, rgbd_msg);

      // 头部位姿变换：将相机系点云从相机坐标系变换到机器人基坐标系
      // （T = M_end × M_fixed，P' = T × P）；尚未收到头部位姿时保持原样。
      if (rgbd_msg->camera_rgbd_pointcloud.width > 0 &&
          rgbd_msg->camera_rgbd_pointcloud.height > 0) {
        applyPointCloudTransform(rgbd_msg->camera_rgbd_pointcloud);
      }
    }

    // Transition to RUNNING on first valid frame
    if (curr_st_ == CameraNodeSt::IDLE) {
      RCLCPP_INFO(logger_, "First valid frame pair received (camera_id=%u), transition IDLE -> RUNNING",
                  camera_id);
      change_status(CameraNodeSt::RUNNING, 0);
    }

    // 内部时间同步融合（发布 /fusion/synced_robot_vision）
    handle_camera_data(camera_id, rgbd_msg);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "[Camera %zu] handleFramePair error: %s", cam_index, e.what());
  }
}

// ============================================================================
// 时间同步融合（原 robot_time_sync_node）
// ============================================================================
CameraNodeSt CameraDriverNode::get_current_status() const {
  std::lock_guard<std::mutex> lock(status_mtx_);
  return curr_st_;
}

// ----------------------------------------------------------
// 相机数据内部回调：保存数据并触发延迟融合定时器（相机优先模式）
// 由 handleFramePair 内部直接调用，不再经 ROS topic 中转
// ----------------------------------------------------------
void CameraDriverNode::handle_camera_data(uint8_t camera_id, const SyncedRobotVision::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(logger_, "[TimeSync] received null camera message");
    return;
  }

  if (camera_id >= latest_cam_data_by_cam_.size()) {
    RCLCPP_WARN(logger_, "[TimeSync] camera_id=%u out of range (size=%zu), ignored",
                camera_id, latest_cam_data_by_cam_.size());
    return;
  }

  // 相机优先模式：仅当节点处于 RUNNING 状态时进行融合
  if (get_current_status() != CameraNodeSt::RUNNING) {
    RCLCPP_DEBUG(logger_,
      "[TimeSync] camera frame ignored, not in RUNNING state (current:%s)",
      node_state_to_string(get_current_status()));
    return;
  }

  // 保存最新的相机数据 (SharedPtr 零拷贝，避免每帧 ~4.6MB 深拷贝)
  {
    std::lock_guard<std::mutex> lock(cam_mtx_);
    latest_cam_data_by_cam_[camera_id] = msg;
  }

  // 重置定时器：reset() 即可重新计时，无需先 cancel()（cancel+reset 在 30Hz 下开销大）
  fusion_timer_->reset();

  RCLCPP_DEBUG(logger_,
    "[TimeSync] camera frame received (camera_id=%u), stamp:%d.%09d, fusion timer reset",
    camera_id, msg->header.stamp.sec, msg->header.stamp.nanosec);
}

// ----------------------------------------------------------
// 运控回调：数据入缓存，按时间戳排序
// ----------------------------------------------------------
void CameraDriverNode::motor_callback(const MotorStatus::SharedPtr msg)
{
  if (!msg) return;

  {
    std::lock_guard<std::mutex> lock(motor_mtx_);
    motor_cache_.push_back(*msg);
    if (motor_cache_.size() > MAX_MOTOR_CACHE) {
      motor_cache_.pop_front();
    }
  }

  // 标记已收到运控数据
  if (!motor_available_) {
    motor_available_ = true;
    RCLCPP_INFO(logger_,
      "[TimeSync] first motor frame received, motor sync enabled");
  }

  // 运控帧也可触发 IDLE→RUNNING（向后兼容）
  if (get_current_status() == CameraNodeSt::IDLE) {
    RCLCPP_INFO(logger_,
      "[TimeSync] motor frame received, transition IDLE -> RUNNING");
    change_status(CameraNodeSt::RUNNING, 0);
  }

  RCLCPP_DEBUG(logger_,
    "[TimeSync] motor frame: cache_size=%zu", motor_cache_.size());
}

// ----------------------------------------------------------
// 头部位姿回调（/Control/HeadPose，眼在手标定）
//   取 end_effector_pose[0]（头部末端位姿）构建 M_end 4×4，
//   与固定标定矩阵 kFixedTransform_（相机→头部末端）相乘得到
//   点云变换矩阵 T = M_end × M_fixed（行主序，P' = T × P）。
// ----------------------------------------------------------
void CameraDriverNode::head_pose_callback(const MotorStatus::SharedPtr msg)
{
  if (!msg || msg->end_effector_pose.empty()) {
    return;
  }

  // 头部末端位姿：只取 end_effector_pose[0]
  const auto &pose = msg->end_effector_pose[0];

  float M_end[16];
  poseToMat4(pose, M_end);

  // T = M_end × M_fixed
  float T[16];
  mat4Mul(M_end, kFixedTransform_.data(), T);

  {
    std::lock_guard<std::mutex> lock(head_pose_mtx_);
    for (int i = 0; i < 16; ++i) {
      head_pose_transform_[i] = T[i];
    }
    if (!head_pose_available_) {
      head_pose_available_ = true;
      RCLCPP_INFO(logger_, "[HeadPose] first head pose received, point cloud transform enabled "
                  "(frame_id -> %s)", pointcloud_base_frame_id_.c_str());
    }
  }
}

// 四元数位姿 -> 4×4 变换矩阵（行主序）：旋转 + 平移
void CameraDriverNode::poseToMat4(const geometry_msgs::msg::Pose &pose, float M[16])
{
  const double qx = pose.orientation.x;
  const double qy = pose.orientation.y;
  const double qz = pose.orientation.z;
  const double qw = pose.orientation.w;
  const double x2 = qx + qx, y2 = qy + qy, z2 = qz + qz;
  const double xx = qx * x2, xy = qx * y2, xz = qx * z2;
  const double yy = qy * y2, yz = qy * z2, zz = qz * z2;
  const double wx = qw * x2, wy = qw * y2, wz = qw * z2;

  M[0] = static_cast<float>(1.0 - (yy + zz));
  M[1] = static_cast<float>(xy - wz);
  M[2] = static_cast<float>(xz + wy);
  M[3] = static_cast<float>(pose.position.x);
  M[4] = static_cast<float>(xy + wz);
  M[5] = static_cast<float>(1.0 - (xx + zz));
  M[6] = static_cast<float>(yz - wx);
  M[7] = static_cast<float>(pose.position.y);
  M[8] = static_cast<float>(xz - wy);
  M[9] = static_cast<float>(yz + wx);
  M[10] = static_cast<float>(1.0 - (xx + yy));
  M[11] = static_cast<float>(pose.position.z);
  M[12] = 0.0f; M[13] = 0.0f; M[14] = 0.0f; M[15] = 1.0f;
}

// 4×4 矩阵相乘（行主序）：out = A * B
void CameraDriverNode::mat4Mul(const float A[16], const float B[16], float out[16])
{
  float tmp[16];
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      tmp[r * 4 + c] =
          A[r * 4 + 0] * B[0 * 4 + c] +
          A[r * 4 + 1] * B[1 * 4 + c] +
          A[r * 4 + 2] * B[2 * 4 + c] +
          A[r * 4 + 3] * B[3 * 4 + c];
    }
  }
  for (int i = 0; i < 16; ++i) {
    out[i] = tmp[i];
  }
}

// 用 4×4 齐次变换矩阵变换三维点（最后一行固定为 [0 0 0 1]）
void CameraDriverNode::mat4TransformPoint(const float M[16], float x, float y, float z,
                                          float &ox, float &oy, float &oz)
{
  ox = M[0] * x + M[1] * y + M[2] * z + M[3];
  oy = M[4] * x + M[5] * y + M[6] * z + M[7];
  oz = M[8] * x + M[9] * y + M[10] * z + M[11];
}

// 应用已保存的变换矩阵 T 到点云：P' = T × P（原地修改 xyz），并更新 frame_id
void CameraDriverNode::applyPointCloudTransform(sensor_msgs::msg::PointCloud2 &pc_msg)
{
  float T[16];
  {
    std::lock_guard<std::mutex> lock(head_pose_mtx_);
    if (!head_pose_available_) {
      return;  // 尚未收到头部位姿，点云保持原样
    }
    for (int i = 0; i < 16; ++i) {
      T[i] = head_pose_transform_[i];
    }
  }

  int32_t xoff = -1, yoff = -1, zoff = -1;
  for (const auto &f : pc_msg.fields) {
    if (f.name == "x") xoff = f.offset;
    else if (f.name == "y") yoff = f.offset;
    else if (f.name == "z") zoff = f.offset;
  }
  if (xoff < 0 || yoff < 0 || zoff < 0 || pc_msg.point_step == 0 || pc_msg.data.empty()) {
    return;
  }

  const size_t num_points = static_cast<size_t>(pc_msg.width) * pc_msg.height;
  uint8_t *data = pc_msg.data.data();
  for (size_t i = 0; i < num_points; ++i) {
    uint8_t *p = data + i * pc_msg.point_step;
    float *px = reinterpret_cast<float *>(p + xoff);
    float *py = reinterpret_cast<float *>(p + yoff);
    float *pz = reinterpret_cast<float *>(p + zoff);
    float ox, oy, oz;
    mat4TransformPoint(T, *px, *py, *pz, ox, oy, oz);
    *px = ox;
    *py = oy;
    *pz = oz;
  }

  // 变换后点云位于机器人基坐标系
  pc_msg.header.frame_id = pointcloud_base_frame_id_;
}

// ----------------------------------------------------------
// 延迟融合回调（由 fusion_timer_ 触发）
//   相机优先：运控数据缺失或同步超时仍发布纯视觉帧
//   多相机：遍历所有相机，为每个相机发布带 camera_id 的融合数据
// ----------------------------------------------------------
void CameraDriverNode::fusion_cb()
{
  // 停止定时器，确保只执行一次
  fusion_timer_->cancel();

  if (get_current_status() != CameraNodeSt::RUNNING) {
    RCLCPP_DEBUG(logger_,
      "[TimeSync] fusion skipped, not RUNNING (state:%s)",
      node_state_to_string(get_current_status()));
    return;
  }

  // 取出所有相机的最新数据（SharedPtr 零拷贝，仅拷贝智能指针）
  std::vector<SyncedRobotVision::SharedPtr> cam_datas;
  {
    std::lock_guard<std::mutex> lock(cam_mtx_);
    cam_datas = latest_cam_data_by_cam_;
  }

  for (size_t cid = 0; cid < cam_datas.size(); ++cid) {
    const auto &cam_data = cam_datas[cid];
    if (!cam_data) {
      continue;  // 该相机尚未收到数据
    }

    // 拼装融合消息（视觉数据始终填充），camera_id 标识来源相机
    SyncedRobotVision fusion_out;
    fusion_out.header = cam_data->header;
    fusion_out.camera_id = static_cast<uint8_t>(cid);
    fusion_out.camera_rgbd_color = cam_data->camera_rgbd_color;
    fusion_out.camera_rgbd_pointcloud = cam_data->camera_rgbd_pointcloud;
    fusion_out.camera_info = cam_data->camera_info;
    fusion_out.target_label = cam_data->target_label;
    fusion_out.seg_confidence = cam_data->seg_confidence;
    fusion_out.data_valid = cam_data->data_valid;

    // 尝试匹配运控帧（尽力而为，不做硬性要求）
    {
      std::lock_guard<std::mutex> lock(motor_mtx_);

      if (!motor_cache_.empty()) {
        const rclcpp::Time cam_ts(cam_data->header.stamp);
        int64_t min_diff_ns = LLONG_MAX;
        const MotorStatus* best_motor = nullptr;

        for (const auto& mot : motor_cache_) {
          int64_t diff = std::abs((cam_ts - rclcpp::Time(mot.header.stamp)).nanoseconds());
          if (diff < min_diff_ns) {
            min_diff_ns = diff;
            best_motor = &mot;
          }
        }

        if (best_motor && min_diff_ns <= MAX_OFFSET_NS) {
          // ● 时间同步成功：填充运控数据
          fusion_out.joint_names = best_motor->joint_names;
          fusion_out.joint_positions = best_motor->joint_positions;
          fusion_out.joint_velocities = best_motor->joint_velocities;
          fusion_out.joint_efforts = best_motor->joint_efforts;
          fusion_out.end_effector_pose = best_motor->end_effector_pose;

          double diff_ms = static_cast<double>(min_diff_ns) / 1e6;
          RCLCPP_DEBUG(logger_,
            "[TimeSync] fusion: publish (motor synced), camera_id=%zu, diff:%.3fms, label:%s",
            cid, diff_ms, cam_data->target_label.c_str());
        } else {
          // ● 时间同步超阈值：发布纯视觉帧（运控字段留空），不丢弃
          double diff_ms = static_cast<double>(min_diff_ns) / 1e6;
          RCLCPP_WARN(logger_,
            "[TimeSync] fusion: sync diff %.3fms > %.1fms, publish camera-only (camera_id=%zu, cam_ts:%d.%09d, cache:%zu)",
            diff_ms, MAX_OFFSET_NS / 1e6,
            cid, cam_data->header.stamp.sec, cam_data->header.stamp.nanosec,
            motor_cache_.size());
            return;
        }
      } else {
        // ● 无运控数据：发布纯视觉帧（运控字段留空）
        //   使用 WARN 级别以便运维人员能直接发现运控数据缺失
        RCLCPP_WARN(logger_,
          "[TimeSync] fusion: publish camera-only (motor unavailable), camera_id=%zu, label:%s, cam_ts:%d.%09d",
          cid, cam_data->target_label.c_str(),
          cam_data->header.stamp.sec, cam_data->header.stamp.nanosec);
      }
    }

    // ● 始终发布，保证视觉数据不丢失
    fusion_pub_->publish(fusion_out);
  }
}

// ============================================================================
// RGB 点云（手动反投影；字段布局与 camera_driver 完全一致）
// ============================================================================
// 预计算像素->射线查找表（fxcx/fycy），缓存键=(w,h,fx,fy,cx,cy)
void CameraDriverNode::ensureXyTables(CameraStreamState & st, uint32_t width, uint32_t height)
{
  const float fx = st.aligned_depth_intrinsic_.fxParam;
  const float fy = st.aligned_depth_intrinsic_.fyParam;
  const float cx = st.aligned_depth_intrinsic_.cxParam;
  const float cy = st.aligned_depth_intrinsic_.cyParam;
  if (width == st.table_w_ && height == st.table_h_ &&
      std::fabs(fx - st.table_fx_) < 1e-6f && std::fabs(fy - st.table_fy_) < 1e-6f &&
      std::fabs(cx - st.table_cx_) < 1e-6f && std::fabs(cy - st.table_cy_) < 1e-6f) {
    return;
  }

  // Offsets used by the official SDK driver for certain scaled/un-distorted modes.
  int xStep = 0;
  int yStep = 0;
  if (width == 284) { xStep = 25; yStep = 12; }
  else if (width == 568) { xStep = 50; yStep = 25; }
  else if (width == 1136) { xStep = 100; yStep = 50; }

  st.fxcx_table_.resize(width);
  st.fycy_table_.resize(height);
  for (uint32_t i = 0; i < width; ++i) {
    st.fxcx_table_[i] = (static_cast<float>(static_cast<int32_t>(i) + xStep) - cx) / fx;
  }
  for (uint32_t j = 0; j < height; ++j) {
    st.fycy_table_[j] = (static_cast<float>(static_cast<int32_t>(j) + yStep) - cy) / fy;
  }
  st.table_w_ = width;
  st.table_h_ = height;
  st.table_fx_ = fx;
  st.table_fy_ = fy;
  st.table_cx_ = cx;
  st.table_cy_ = cy;
}

float CameraDriverNode::depthToMeters(uint16_t raw, berxel::BerxelHawkPixelType pixel_type)
{
  float v = 0.0f;
  switch (pixel_type) {
    case berxel::BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_12I_4D:
      v = static_cast<float>(raw) / 16000.0f;
      if (v >= 4.095f) v = 0.0f;
      break;
    case berxel::BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_14I_2D:
      v = static_cast<float>(raw) / 4000.0f;
      if (v >= 16.383f) v = 0.0f;
      break;
    default:  // DEP_16BIT_13I_3D and any other
      v = static_cast<float>(raw) / 8000.0f;
      if (v >= 8.191f) v = 0.0f;
      break;
  }
  return v;
}

void CameraDriverNode::buildPointCloud(
  size_t cam_index, berxel::BerxelHawkFrame * p_depth,
  berxel::BerxelHawkFrame * p_color, const std::string & cloud_fid,
  const SyncedRobotVision::SharedPtr & rgbd_msg)
{
  if (cam_index >= cameras_.size() || !rgbd_msg || !p_depth) {
    return;
  }
  CameraStreamState &cam = cameras_[cam_index];

  const uint32_t depth_w = p_depth->getWidth();
  const uint32_t depth_h = p_depth->getHeight();
  const size_t total = static_cast<size_t>(depth_w) * depth_h;
  if (total == 0) {
    return;
  }

  // 彩色数据源（RGB888）。registration 开启且彩色/深度分辨率一致时深度与彩色
  // 逐像素对齐，直接从彩色帧取色。
  const bool has_rgb = (pointcloud_format_ == "rgb_point");
  const uint8_t * color_data = nullptr;
  if (has_rgb && p_color) {
    const uint32_t cw = p_color->getWidth();
    const uint32_t ch = p_color->getHeight();
    if (cw == depth_w && ch == depth_h) {
      color_data = static_cast<const uint8_t *>(p_color->getData());
    } else if (!cam.color_cloud_mismatch_warned_) {
      RCLCPP_WARN(logger_,
        "[Camera %zu] RGB point cloud requested but color (%ux%u) and depth (%ux%u) "
        "resolutions differ (registration off?). Publishing xyz-only cloud.",
        cam_index, cw, ch, depth_w, depth_h);
      cam.color_cloud_mismatch_warned_ = true;
    }
  }

  const uint16_t * depth_data = static_cast<const uint16_t *>(p_depth->getData());
  if (depth_data == nullptr) {
    return;
  }
  const berxel::BerxelHawkPixelType pixel_type = p_depth->getPixelType();

  ensureXyTables(cam, depth_w, depth_h);

  // pc_keep_invalid_points=true：保留所有像素（width=深度分辨率），无效深度点用
  // NaN 表示；false 时沿用旧逻辑只统计有效点。
  const bool keep_invalid = pc_keep_invalid_points_;
  size_t valid = keep_invalid ? total : 0;
  if (!keep_invalid) {
    for (size_t i = 0; i < total; ++i) {
      if (depthToMeters(depth_data[i], pixel_type) > 0.0f) {
        ++valid;
      }
    }
  }
  if (valid == 0) {
    return;
  }

  sensor_msgs::msg::PointCloud2 pc_msg;
  pc_msg.header.stamp = rgbd_msg->header.stamp;
  pc_msg.header.frame_id = cloud_fid;
  pc_msg.height = 1;
  pc_msg.width = static_cast<uint32_t>(valid);
  pc_msg.fields.resize(has_rgb ? 4 : 3);
  pc_msg.fields[0].name = "x";
  pc_msg.fields[0].offset = 0;
  pc_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pc_msg.fields[0].count = 1;
  pc_msg.fields[1].name = "y";
  pc_msg.fields[1].offset = 4;
  pc_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pc_msg.fields[1].count = 1;
  pc_msg.fields[2].name = "z";
  pc_msg.fields[2].offset = 8;
  pc_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pc_msg.fields[2].count = 1;
  if (has_rgb) {
    // ROS 标准：rgb 字段以 FLOAT32 声明，实际存放打包的 uint32(R<<16|G<<8|B)
    pc_msg.fields[3].name = "rgb";
    pc_msg.fields[3].offset = 12;
    pc_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc_msg.fields[3].count = 1;
  }
  pc_msg.is_bigendian = false;
  pc_msg.point_step = has_rgb ? 16 : 12;
  pc_msg.row_step = pc_msg.point_step * pc_msg.width;
  pc_msg.is_dense = false;
  pc_msg.data.resize(pc_msg.row_step);

  const float nan = std::numeric_limits<float>::quiet_NaN();
  uint8_t * dst = pc_msg.data.data();
  size_t idx = 0;
  for (uint32_t v = 0; v < depth_h; ++v) {
    const size_t row = static_cast<size_t>(v) * depth_w;
    const uint8_t * color_row = color_data ? color_data + row * 3 : nullptr;
    const float fycy = cam.fycy_table_[v];
    for (uint32_t u = 0; u < depth_w; ++u) {
      const size_t px = row + u;
      const float z = depthToMeters(depth_data[px], pixel_type);
      const bool invalid = (z <= 0.0f);
      if (invalid && !keep_invalid) {
        continue;  // 不保留无效点
      }
      float * out = reinterpret_cast<float *>(dst + idx * pc_msg.point_step);
      if (invalid) {
        // 保留所有像素：无效深度点写 NaN，RGB 置 0
        out[0] = nan;
        out[1] = nan;
        out[2] = nan;
        if (has_rgb) {
          const uint32_t rgb = 0;
          std::memcpy(dst + idx * pc_msg.point_step + 12, &rgb, sizeof(rgb));
        }
      } else {
        const float fxcx = cam.fxcx_table_[u];
        out[0] = fxcx * z;
        out[1] = fycy * z;
        out[2] = z;
        if (has_rgb) {
          uint8_t r = 0, g = 0, b = 0;
          if (color_row) {
            r = color_row[u * 3 + 0];
            g = color_row[u * 3 + 1];
            b = color_row[u * 3 + 2];
          }
          const uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                               (static_cast<uint32_t>(g) << 8) |
                               static_cast<uint32_t>(b);
          std::memcpy(dst + idx * pc_msg.point_step + 12, &rgb, sizeof(rgb));
        }
      }
      ++idx;
    }
  }

  rgbd_msg->camera_rgbd_pointcloud = std::move(pc_msg);
}

// ============================================================================
// 相机配置/内参
// ============================================================================
void CameraDriverNode::readIntrinsics(size_t cam_index)
{
  if (cam_index >= cameras_.size()) {
    return;
  }
  CameraStreamState &cam = cameras_[cam_index];
  if (cam.device == nullptr) {
    return;
  }
  berxel::BerxelHawkDeviceIntrinsicParams device_intr;
  std::memset(&device_intr, 0, sizeof(device_intr));
  if (cam.device->getDeviceIntriscParams(&device_intr) != 0) {
    RCLCPP_WARN(logger_, "[Camera %zu] getDeviceIntriscParams() failed; using identity.",
                cam_index);
    return;
  }
  std::memcpy(&cam.color_intrinsic_, device_intr.colorIntrinsicParams,
              sizeof(berxel::BerxelHawkCameraIntrinsic));
  std::memcpy(&cam.depth_intrinsic_, device_intr.liteIrIntrinsicParams,
              sizeof(berxel::BerxelHawkCameraIntrinsic));
  RCLCPP_INFO(logger_, "[Camera %zu] Color intrinsics: fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
              cam_index, cam.color_intrinsic_.fxParam, cam.color_intrinsic_.fyParam,
              cam.color_intrinsic_.cxParam, cam.color_intrinsic_.cyParam);
}

void CameraDriverNode::updateIntrinsicsForResolution(size_t cam_index)
{
  if (cam_index >= cameras_.size()) {
    return;
  }
  CameraStreamState &cam = cameras_[cam_index];

  // Color intrinsic scaled to the active color resolution.
  const int color_scale = pixelScaleForWidth(static_cast<uint32_t>(cam.color_width_), false);
  if (color_scale > 1) {
    cam.color_intrinsic_.fxParam /= color_scale;
    cam.color_intrinsic_.fyParam /= color_scale;
    cam.color_intrinsic_.cxParam /= color_scale;
    cam.color_intrinsic_.cyParam /= color_scale;
  }

  // Registered depth uses the (scaled) color intrinsic so that the point cloud
  // is expressed in color coordinates. Un-registered depth uses the depth one.
  const int depth_scale = pixelScaleForWidth(static_cast<uint32_t>(cam.depth_width_), true);
  if (depth_registration_) {
    cam.aligned_depth_intrinsic_ = cam.color_intrinsic_;
    RCLCPP_INFO(logger_,
                "[Camera %zu] Depth registration ENABLED: depth (point cloud) uses color intrinsics.",
                cam_index);
  } else {
    if (depth_scale > 1) {
      cam.depth_intrinsic_.fxParam /= depth_scale;
      cam.depth_intrinsic_.fyParam /= depth_scale;
      cam.depth_intrinsic_.cxParam /= depth_scale;
      cam.depth_intrinsic_.cyParam /= depth_scale;
    }
    cam.aligned_depth_intrinsic_ = cam.depth_intrinsic_;
    RCLCPP_INFO(logger_,
                "[Camera %zu] Depth registration DISABLED: depth (point cloud) uses depth intrinsics.",
                cam_index);
  }
}

void CameraDriverNode::buildCameraInfoMessages(size_t cam_index)
{
  if (cam_index >= cameras_.size()) {
    return;
  }
  CameraStreamState &cam = cameras_[cam_index];

  // 与 camera_driver 一致：CameraInfo 的 frame_id 使用基准 frame_id（全局成员）。
  cam.rgb_camera_info_ = convertToCameraInfo(cam.color_intrinsic_);
  cam.rgb_camera_info_.header.frame_id = color_frame_id_;
  cam.rgb_camera_info_.width = static_cast<uint32_t>(cam.color_width_);
  cam.rgb_camera_info_.height = static_cast<uint32_t>(cam.color_height_);

  cam.depth_camera_info_ = convertToCameraInfo(cam.depth_intrinsic_);
  cam.depth_camera_info_.header.frame_id = depth_frame_id_;
  cam.depth_camera_info_.width = static_cast<uint32_t>(cam.depth_width_);
  cam.depth_camera_info_.height = static_cast<uint32_t>(cam.depth_height_);

  cam.aligned_depth_camera_info_ = convertToCameraInfo(cam.aligned_depth_intrinsic_);
  cam.aligned_depth_camera_info_.header.frame_id =
      depth_registration_ ? color_frame_id_ : depth_frame_id_;
  if (depth_registration_) {
    cam.aligned_depth_camera_info_.width = static_cast<uint32_t>(cam.color_width_);
    cam.aligned_depth_camera_info_.height = static_cast<uint32_t>(cam.color_height_);
  } else {
    cam.aligned_depth_camera_info_.width = static_cast<uint32_t>(cam.depth_width_);
    cam.aligned_depth_camera_info_.height = static_cast<uint32_t>(cam.depth_height_);
  }
}

// ============================================================================
// Heartbeat & Status
// ============================================================================
void CameraDriverNode::heartbeat_cb() {
  auto hb_msg = com_interfaces::msg::HeartBeat();
  hb_msg.header.stamp = now();
  hb_msg.node_id = CAMERA_DRIVER_NODE_ID;
  hb_pub_->publish(hb_msg);

  // Throttle logging: only log every 30 heartbeats (~30s)
  static int hb_count = 0;
  hb_count++;
  if (hb_count % 30 == 1) {
    RCLCPP_INFO(logger_, "[Heartbeat] #%d, node:%d, state:%s, cameras:%zu",
                hb_count, CAMERA_DRIVER_NODE_ID,
                node_state_to_string(curr_st_), cameras_.size());
  } else {
    RCLCPP_DEBUG(logger_, "[Heartbeat] #%d", hb_count);
  }
}

void CameraDriverNode::change_status(CameraNodeSt new_st, int32_t err) {
  {
    std::lock_guard<std::mutex> lock(status_mtx_);
    curr_st_ = new_st;
    err_code_ = err;
  }
  publish_status(new_st, err);
}

void CameraDriverNode::publish_status(CameraNodeSt st, int32_t err) {
  com_interfaces::msg::NodeStatus st_msg;
  st_msg.header.stamp = now();
  st_msg.node_id = CAMERA_DRIVER_NODE_ID;
  st_msg.status = static_cast<int32_t>(st);
  st_msg.error_flag = err;
  status_pub_->publish(st_msg);

  if (err != 0) {
    RCLCPP_WARN(logger_, "[Status] changed -> %s (node:%d, err:%d)",
                node_state_to_string(st), CAMERA_DRIVER_NODE_ID, err);
  } else {
    RCLCPP_INFO(logger_, "[Status] changed -> %s (node:%d)",
                node_state_to_string(st), CAMERA_DRIVER_NODE_ID);
  }
}

void CameraDriverNode::checkFrameRate() {
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      now - fps_last_check_).count();

  if (elapsed > 0) {
    actual_color_fps_ = static_cast<double>(fps_frame_count_) / elapsed;
  }

  RCLCPP_DEBUG(logger_, "[FPS Monitor] Actual frame rate: %.1f fps (expected: %d fps)",
               actual_color_fps_, color_fps_);

  if (fps_frame_count_ == 0) {
    RCLCPP_WARN(logger_, "[FPS Monitor] No frames received in the last %.0f seconds!",
                elapsed);
  } else if (actual_color_fps_ < static_cast<double>(color_fps_) * 0.5) {
    RCLCPP_WARN(logger_, "[FPS Monitor] Frame rate too low: %.1f fps (expected: %d fps)",
                actual_color_fps_, color_fps_);
  }

  // Reset counters
  fps_frame_count_ = 0;
  fps_last_check_ = now;
}

void CameraDriverNode::stop() {
  try {
    // 停止主机时间同步周期定时器（与 camera_driver 一致，仅停止该定时器）
    if (time_sync_timer_) {
      time_sync_timer_->cancel();
      time_sync_timer_.reset();
    }
    // 置位关闭标记：SDK 回调线程将直接丢弃并释放帧，不再触碰相机状态
    shutting_down_ = true;

    // 停止并释放所有相机的流与设备
    for (auto &cam : cameras_) {
      {
        std::lock_guard<std::mutex> lock(cam.frame_mutex_);
        cam.process_running_ = false;
      }
      cam.frame_cv_.notify_all();
      if (cam.process_thread_.joinable()) {
        cam.process_thread_.join();
      }
      if (cam.device) {
        cam.device->stopStreams(stream_flags_);
        // Give the SDK callback worker a moment to stop before we release frames.
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        std::vector<berxel::BerxelHawkFrame *> leftover;
        {
          std::lock_guard<std::mutex> lock(cam.frame_mutex_);
          while (!cam.color_queue_.empty()) {
            leftover.push_back(cam.color_queue_.front());
            cam.color_queue_.pop();
          }
          while (!cam.depth_queue_.empty()) {
            leftover.push_back(cam.depth_queue_.front());
            cam.depth_queue_.pop();
          }
        }
        for (auto * f : leftover) {
          if (f != nullptr) {
            berxel::BerxelHawkFrame * ff = f;
            cam.device->releaseFrame(ff);
          }
        }
        if (context_) {
          context_->closeDevice(cam.device);
        }
        cam.device = nullptr;
      }
    }
    cameras_.clear();
    latest_cam_data_by_cam_.clear();
    device_ = nullptr;
  } catch (const std::exception &e) {
    RCLCPP_WARN(logger_, "Error stopping camera streams: %s", e.what());
  }
}

// ============================================================================
// 工具函数
// ============================================================================
rclcpp::Time CameraDriverNode::fromUsToROSTime(uint64_t us) {
  // Safely convert microseconds to ROS time (seconds + nanoseconds)
  uint32_t sec = static_cast<uint32_t>(us / 1000000);
  uint32_t nsec = static_cast<uint32_t>((us % 1000000) * 1000);
  return rclcpp::Time(sec, nsec);
}

sensor_msgs::msg::CameraInfo CameraDriverNode::convertToCameraInfo(
    const berxel::BerxelHawkCameraIntrinsic & intrinsic) {
  sensor_msgs::msg::CameraInfo info;
  // 注意：BerxelHawkCameraIntrinsic 不含 width/height，由调用方设置。

  info.distortion_model = "plumb_bob";
  info.d.resize(5, 0.0);
  info.d[0] = intrinsic.k1Param;
  info.d[1] = intrinsic.k2Param;
  info.d[2] = intrinsic.p1Param;
  info.d[3] = intrinsic.p2Param;
  info.d[4] = intrinsic.k3Param;

  info.k.fill(0.0);
  info.k[0] = intrinsic.fxParam;
  info.k[2] = intrinsic.cxParam;
  info.k[4] = intrinsic.fyParam;
  info.k[5] = intrinsic.cyParam;
  info.k[8] = 1.0;

  info.r.fill(0.0);
  info.r[0] = 1;
  info.r[4] = 1;
  info.r[8] = 1;

  info.p.fill(0.0);
  info.p[0] = intrinsic.fxParam;
  info.p[2] = intrinsic.cxParam;
  info.p[5] = intrinsic.fyParam;
  info.p[6] = intrinsic.cyParam;
  info.p[10] = 1.0;

  return info;
}

}  // namespace berxel_camera_driver

// ============================================================================
// main
// ============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<berxel_camera_driver::CameraDriverNode>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
