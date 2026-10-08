/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include "camera_driver/camera_driver_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

// 时间融合固定参数（原 robot_time_sync_node）
// 缓存需覆盖“相机帧打标时刻(cam_ts) → 融合执行时刻”的处理延迟：
// 点云构建/软件对齐等耗时可达 100~300ms，10 帧(100ms)窗口会被挤到
// 处理完成时刻附近，导致匹配到的 motor 帧与 cam_ts 相差约一个处理延迟。
// 100 帧 @100Hz = 1s 窗口，保证能匹配到与 cam_ts 真正接近的运控帧。
constexpr size_t MAX_MOTOR_CACHE = 100;
constexpr auto CAM_DELAY_AFTER = 2ms;          // 相机帧到达后的延迟融合时间
constexpr int64_t MAX_OFFSET_NS = 10'000'000;  // 10ms 最大同步误差

// ============================================================================
// 初始化
// ============================================================================
void CameraDriverNode::init() {
  logger_ = get_logger();
  declareParameters();
  getParameters();

  // --- 融合结果发布器（原 robot_time_sync_node 的话题名称） ---
  // 注：原 camera_driver 对外发布 /vision/camera_data 的话题已移除，
  //     该话题原先仅发送给 time_sync 节点，融合已在本节点内部完成。
  // 多相机时所有相机都通过该话题发布，用 camera_id 字段区分来源相机
  // （0=头部相机，1=腰部相机，2+预留）。
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

  // Initialize SDK context
  try {
    ctx_ = std::make_unique<ob::Context>();
  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "Failed to create Orbbec SDK context: %s", e.getMessage());
    publish_status(CameraNodeSt::EXCEPTION, 0);
    return;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Failed to create Orbbec SDK context: %s", e.what());
    publish_status(CameraNodeSt::EXCEPTION, 0);
    return;
  }

  // Try to acquire cross-process device lock
  if (!acquireDeviceLock()) {
    RCLCPP_ERROR(logger_,
                 "Device is occupied by another process (orbbec_camera_node may be running).\n"
                 "Please stop orbbec_camera_node first, or set parameter bypass_lock:=true to force access.");
    publish_status(CameraNodeSt::EXCEPTION, 0);
    return;
  }

  // Query and open device(s)（多相机时逐个打开所有相机）
  try {
    if (!openDevice()) {
      RCLCPP_ERROR(logger_, "Failed to open Orbbec device");
      cleanupAfterInitFailure();
      publish_status(CameraNodeSt::EXCEPTION, 0);
      return;
    }
  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "Unexpected error opening device: %s", e.getMessage());
    cleanupAfterInitFailure();
    publish_status(CameraNodeSt::EXCEPTION, 0);
    return;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Unexpected error opening device: %s", e.what());
    cleanupAfterInitFailure();
    publish_status(CameraNodeSt::EXCEPTION, 0);
    return;
  }

  // Create Pipeline（每个相机独立 Pipeline）
  try {
    for (auto &cam : cameras_) {
      cam.pipeline = std::make_shared<ob::Pipeline>(cam.device);
    }
    pipeline_ = cameras_.empty() ? nullptr : cameras_[0].pipeline;
  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "Failed to create pipeline: %s", e.getMessage());
    cleanupAfterInitFailure();
    publish_status(CameraNodeSt::EXCEPTION, 0);
    return;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Failed to create pipeline: %s", e.what());
    cleanupAfterInitFailure();
    publish_status(CameraNodeSt::EXCEPTION, 0);
    return;
  }

  // Configure and start streams（所有相机统一启动完整数据流）
  try {
    if (!startStreams()) {
      RCLCPP_ERROR(logger_, "Failed to start streams");
      cleanupAfterInitFailure();
      publish_status(CameraNodeSt::EXCEPTION, 0);
      return;
    }
  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "Unexpected error starting streams: %s", e.getMessage());
    cleanupAfterInitFailure();
    publish_status(CameraNodeSt::EXCEPTION, 0);
    return;
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
  ctx_.reset();
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
  // align_mode: "OFF" | "HW" (pipeline hardware D2C) | "SW" (in-node reprojection)
  declare_parameter<std::string>("align_mode", "HW");
  // align_target_stream: "COLOR" (D2C, depth -> color) | "DEPTH" (C2D, color -> depth)
  declare_parameter<std::string>("align_target_stream", "COLOR");
  // 点云输出格式："rgb_point"（默认，RGB 点云，每点 xyz(float32)+rgb(uint32)）| "point"（仅坐标）
  declare_parameter<std::string>("pointcloud_format", "rgb_point");
  // 是否保留点云所有像素：true 时无效深度点用 NaN 表示（width=图像分辨率），false 时过滤无效点
  declare_parameter<bool>("pc_keep_invalid_points", true);
  // 点云坐标变换后的坐标系 frame_id（机器人基坐标系）
  declare_parameter<std::string>("pointcloud_base_frame_id", "base_link");
  // 主机时间同步：将相机内部时钟同步到主机系统时间（timerSyncWithHost + global timestamp）
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
    error_msg = "No Orbbec device connected";
    RCLCPP_ERROR(logger_, "[HardwareCheck] %s", error_msg.c_str());
  } else {
    RCLCPP_INFO(logger_, "[HardwareCheck] %zu camera(s) connected", cameras_.size());
  }

  // --- Step 2: Verify firmware version (主相机，index 0) ---
  if (all_ok && !send_feedback(30.0f, "Verifying firmware version...")) return;
  if (all_ok && !cameras_.empty()) {
    try {
      auto dev_info = cameras_[0].device->getDeviceInfo();
      if (dev_info) {
        RCLCPP_INFO(logger_, "[HardwareCheck] Device: %s, SN: %s, FW: %s",
                    dev_info->name(), dev_info->serialNumber(), dev_info->firmwareVersion());
      }
    } catch (const std::exception &e) {
      all_ok = false;
      error_msg = "Failed to read device info: " + std::string(e.what());
      RCLCPP_ERROR(logger_, "[HardwareCheck] %s", error_msg.c_str());
    }
  }

  // --- Step 3: Test image sensor data validity ---
  if (all_ok && !send_feedback(50.0f, "Testing image sensor...")) return;
  if (all_ok && !cameras_.empty() && cameras_[0].pipeline) {
    try {
      // Check if pipeline is actively producing frames by verifying frame count
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
          now - fps_last_check_).count();
      if (elapsed >= 5 && fps_frame_count_ == 0) {
        all_ok = false;
        error_msg = "No frames received from camera pipeline";
        RCLCPP_ERROR(logger_, "[HardwareCheck] %s", error_msg.c_str());
      } else {
        RCLCPP_INFO(logger_, "[HardwareCheck] Pipeline active, frames received: %u",
                    fps_frame_count_);
      }
    } catch (const std::exception &e) {
      all_ok = false;
      error_msg = "Pipeline check failed: " + std::string(e.what());
    }
  }

  // --- Step 4: Validate calibration data (主相机，index 0) ---
  if (all_ok && !send_feedback(70.0f, "Validating calibration data...")) return;
  if (all_ok && !cameras_.empty()) {
    const auto &cam0 = cameras_[0];
    if (cam0.color_intrinsic_.fx <= 0 || cam0.color_intrinsic_.fy <= 0 ||
        cam0.depth_intrinsic_.fx <= 0 || cam0.depth_intrinsic_.fy <= 0) {
      all_ok = false;
      error_msg = "Invalid camera intrinsics (fx/fy are zero or negative)";
      RCLCPP_ERROR(logger_, "[HardwareCheck] %s", error_msg.c_str());
    } else {
      RCLCPP_INFO(logger_, "[HardwareCheck] Calibration valid: Color fx=%.2f, Depth fx=%.2f",
                  cam0.color_intrinsic_.fx, cam0.depth_intrinsic_.fx);
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
    RCLCPP_WARN(logger_, "[HardwareReset] Error stopping pipelines: %s", e.what());
  }

  // --- Step 3: Re-open device(s) and restart streams ---
  if (!send_feedback(60.0f, "Re-initializing sensor...")) return;
  bool reopen_ok = false;
  try {
    // 重新打开所有相机（多相机时逐个打开）
    if (openDevice()) {
      for (auto &cam : cameras_) {
        cam.pipeline = std::make_shared<ob::Pipeline>(cam.device);
      }
      pipeline_ = cameras_.empty() ? nullptr : cameras_[0].pipeline;
      reopen_ok = startStreams();
    }
  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "[HardwareReset] Re-init failed: %s", e.getMessage());
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
      auto dev_info = cameras_[0].device->getDeviceInfo();
      if (dev_info) {
        RCLCPP_INFO(logger_, "[HardwareReset] Device reconnected: %s (SN: %s)",
                    dev_info->name(), dev_info->serialNumber());
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
// 设备锁
// ============================================================================
bool CameraDriverNode::acquireDeviceLock() {
  if (bypass_lock_) {
    RCLCPP_WARN(logger_, "Bypassing device lock check via bypass_lock parameter");
    return true;
  }

  shm_fd_ = shm_open(ORB_DEVICE_LOCK_NAME, O_RDWR, 0666);
  if (shm_fd_ < 0) {
    RCLCPP_INFO(logger_,
                 "No device lock detected (orbbec_camera_node not running), accessing device directly");
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
                 "Device is occupied by orbbec_camera_node.\n"
                 "Please stop orbbec_camera_node, or set parameter --ros-args -p bypass_lock:=true");
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
// 设备操作
// ============================================================================
bool CameraDriverNode::openDevice() {
  std::shared_ptr<ob::DeviceList> device_list;
  try {
    device_list = ctx_->queryDeviceList();
  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "Failed to query device list: %s", e.getMessage());
    return false;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Failed to query device list: %s", e.what());
    return false;
  }

  if (device_list == nullptr || device_list->deviceCount() == 0) {
    RCLCPP_ERROR(logger_, "No Orbbec device found");
    return false;
  }

  const uint32_t device_count = device_list->deviceCount();
  RCLCPP_INFO(logger_, "Found %d device(s)", device_count);

  // 打印所有检测到的设备信息（含 SN 码），便于多相机场景识别
  for (uint32_t i = 0; i < device_count; i++) {
    try {
      auto dev = device_list->getDevice(i);
      auto dev_info = dev->getDeviceInfo();
      if (dev_info) {
        RCLCPP_INFO(logger_, "  Device[%u]: %s (SN: %s, FW: %s)",
                    i, dev_info->name(), dev_info->serialNumber(),
                    dev_info->firmwareVersion());
      }
    } catch (const ob::Error &e) {
      RCLCPP_WARN(logger_, "Failed to read device info for index %u: %s", i, e.getMessage());
    }
  }

  // 清空旧设备状态
  cameras_.clear();
  device_.reset();
  pipeline_.reset();

  // 建立 设备枚举索引 -> SN 的查询表
  std::vector<std::string> sn_by_index(device_count, "");
  std::vector<std::string> name_by_index(device_count, "");
  for (uint32_t i = 0; i < device_count; i++) {
    try {
      auto dev = device_list->getDevice(i);
      auto dev_info = dev->getDeviceInfo();
      if (dev_info) {
        sn_by_index[i] = dev_info->serialNumber();
        name_by_index[i] = dev_info->name();
      }
    } catch (const ob::Error &e) {
      RCLCPP_WARN(logger_, "Failed to read device info for index %u: %s", i, e.getMessage());
    }
  }

  auto add_camera = [this](const std::shared_ptr<ob::Device> &dev, uint8_t camera_id,
                           const std::string &sn, const std::string &name) {
    CameraStreamState cam;
    cam.device = dev;
    cam.camera_id = camera_id;
    cam.serial_number = sn;
    cam.device_name = name;
    cameras_.push_back(std::move(cam));
  };

  if (!camera_id_sn_map_.empty()) {
    // ── 情况1：配置了 camera_id_sn_map（多相机 SN -> camera_id 映射） ──
    RCLCPP_INFO(logger_, "Using camera_id_sn_map (%zu entry/ies) to map SN -> camera_id:", camera_id_sn_map_.size());
    for (size_t cid = 0; cid < camera_id_sn_map_.size(); ++cid) {
      const std::string &target_sn = camera_id_sn_map_[cid];
      RCLCPP_INFO(logger_, "  camera_id=%zu -> SN=%s", cid, target_sn.c_str());
      if (target_sn.empty()) {
        RCLCPP_WARN(logger_, "  camera_id=%zu has empty SN, skipped", cid);
        continue;
      }
      bool found = false;
      for (uint32_t i = 0; i < device_count; i++) {
        if (sn_by_index[i] == target_sn) {
          try {
            auto dev = device_list->getDevice(i);
            add_camera(dev, static_cast<uint8_t>(cid), target_sn, name_by_index[i]);
            found = true;
            RCLCPP_INFO(logger_, "  camera_id=%zu -> device index %u opened (SN: %s)",
                        cid, i, target_sn.c_str());
            break;
          } catch (const ob::Error &e) {
            RCLCPP_WARN(logger_, "  Failed to open device index %u (SN %s): %s",
                        i, target_sn.c_str(), e.getMessage());
          }
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
        try {
          auto dev = device_list->getDevice(i);
          add_camera(dev, 0, serial_number_, name_by_index[i]);
          RCLCPP_INFO(logger_, "Selected device by serial number: %s (camera_id=0)", serial_number_.c_str());
          break;
        } catch (const ob::Error &e) {
          RCLCPP_WARN(logger_, "Failed to open device index %d (SN filter): %s", i, e.getMessage());
        }
      }
    }
    if (cameras_.empty()) {
      RCLCPP_ERROR(logger_, "No device found with serial number '%s'", serial_number_.c_str());
      return false;
    }
  } else {
    // ── 情况3：未配置映射，逐个打开所有设备，camera_id 按枚举顺序分配（0,1,2,...） ──
    for (uint32_t i = 0; i < device_count; i++) {
      try {
        auto dev = device_list->getDevice(i);
        add_camera(dev, static_cast<uint8_t>(i), sn_by_index[i], name_by_index[i]);
      } catch (const ob::Error &e) {
        RCLCPP_WARN(logger_,
                    "Failed to open device index %d: %s\n"
                    "  Hint: Check USB permissions (udev rules) or if device is already in use.",
                    i, e.getMessage());
        continue;
      }
    }
    if (cameras_.empty()) {
      RCLCPP_ERROR(logger_,
                   "Failed to open any of the %d detected device(s).\n"
                   "  Common causes:\n"
                   "  1. Missing USB permissions (run: sudo cp SDK/scripts/99-obsensor-libusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger)\n"
                   "  2. Device already in use by orbbec_camera_node or another process\n"
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

  // 分配每相机最新数据槽位（按 camera_id 索引）
  latest_cam_data_by_cam_.assign(cameras_.size(), nullptr);

  // ── 归一化图像配准参数（全局，一次） ──
  if (align_mode_ == "OFF") {
    enable_align_ = false;
  } else if (align_mode_ == "SW") {
    enable_align_ = false;  // software alignment replaces HW alignment
  } else if (align_mode_ != "HW") {
    RCLCPP_WARN(logger_, "Unknown align_mode '%s', defaulting to 'HW'", align_mode_.c_str());
    align_mode_ = "HW";
  }
  if (align_target_stream_str_ == "DEPTH") {
    align_target_stream_ = OB_STREAM_DEPTH;
  } else {
    align_target_stream_str_ = "COLOR";
    align_target_stream_ = OB_STREAM_COLOR;
  }
  depth_registration_ = (align_mode_ != "OFF");
  RCLCPP_INFO(logger_,
              "Image registration: align_mode=%s, align_target_stream=%s, enable_align=%d",
              align_mode_.c_str(), align_target_stream_str_.c_str(), enable_align_ ? 1 : 0);

  // 所有相机统一启动完整数据流（D2C 对齐 + 融合）
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
// Stream profile 选择（健壮版：捕获 SDK 异常 + fps 逐级降级）
// ============================================================================
std::shared_ptr<ob::VideoStreamProfile> CameraDriverNode::selectVideoStreamProfile(
    const std::shared_ptr<ob::StreamProfileList> &list,
    int width, int height, int fps,
    const std::vector<OBFormat> &format_candidates,
    const char *stream_name) {
  if (!list) {
    RCLCPP_WARN(logger_, "[%s] No stream profile list available", stream_name);
    return nullptr;
  }
  if (list->count() == 0) {
    RCLCPP_WARN(logger_, "[%s] Empty stream profile list", stream_name);
    return nullptr;
  }

  // fps 逐级降级：优先用户指定帧率，再尝试常见帧率，最后 ANY。
  // SDK 的 getVideoStreamProfile() 在无匹配时抛异常（而非返回 nullptr），
  // 例如请求 1280x720@3fps 而设备不支持 3fps 时会抛
  // "No matched video stream profile found!"，因此每个候选都必须 try-catch。
  const std::vector<int> fps_candidates = {fps, 5, 10, 15, 30, OB_FPS_ANY};
  for (auto fmt : format_candidates) {
    for (int f : fps_candidates) {
      try {
        auto profile = list->getVideoStreamProfile(width, height, fmt, f);
        if (profile) {
          RCLCPP_INFO(logger_, "[%s] Selected stream profile: %dx%d@%d fmt=0x%X",
                      stream_name, width, height, profile->fps(),
                      static_cast<unsigned>(profile->format()));
          return profile;
        }
      } catch (const ob::Error &e) {
        RCLCPP_DEBUG(logger_, "[%s] No profile match %dx%d fmt=0x%X fps=%d: %s",
                     stream_name, width, height, static_cast<unsigned>(fmt), f,
                     e.getMessage());
      } catch (const std::exception &e) {
        RCLCPP_DEBUG(logger_, "[%s] No profile match %dx%d fmt=0x%X fps=%d: %s",
                     stream_name, width, height, static_cast<unsigned>(fmt), f,
                     e.what());
      }
    }
  }
  return nullptr;
}

// ============================================================================
// 主机时间同步
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
    // 使 getGlobalTimeStampUs() 返回与主机对齐的全局时间戳（Unix 微秒）
    cam.device->enableGlobalTimestamp(true);
    // 将相机内部时钟同步到主机系统时间
    cam.device->timerSyncWithHost();
    cam.time_synced_ = true;
    RCLCPP_INFO(logger_, "[Camera %zu] host time synced (SN=%s)",
                cam_index, cam.serial_number.c_str());
  } catch (const ob::Error &e) {
    cam.time_synced_ = false;
    RCLCPP_WARN(logger_, "[Camera %zu] host time sync failed: %s",
                cam_index, e.what());
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
      cam.device->timerSyncWithHost();
      RCLCPP_INFO(logger_, "[Camera %zu] periodic host time resync ok", i);
    } catch (const ob::Error &e) {
      RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 10000,
        "[Camera %zu] periodic time resync failed: %s", i, e.what());
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 10000,
        "[Camera %zu] periodic time resync failed: %s", i, e.what());
    }
  }
}

bool CameraDriverNode::startStreamForCamera(size_t cam_index) {
  if (cam_index >= cameras_.size()) {
    return false;
  }
  CameraStreamState &cam = cameras_[cam_index];
  auto &pipeline = cam.pipeline;
  auto &device = cam.device;
  if (!pipeline || !device) {
    RCLCPP_ERROR(logger_, "[Camera %zu] Pipeline or device is null", cam_index);
    return false;
  }

  try {
    // ── Discover available color sensor ──
    // Gemini 336L and similar stereo cameras may expose color via
    // OB_SENSOR_COLOR_LEFT/OB_SENSOR_COLOR_RIGHT instead of OB_SENSOR_COLOR.
    // Try MAIN first, then LEFT, then RIGHT.
    std::shared_ptr<ob::Sensor> color_sensor;
    {
      constexpr OBSensorType color_sensor_candidates[] = {
          OB_SENSOR_COLOR, OB_SENSOR_COLOR_LEFT, OB_SENSOR_COLOR_RIGHT};
      for (auto st : color_sensor_candidates) {
        color_sensor = device->getSensor(st);
        if (color_sensor) {
          RCLCPP_INFO(logger_, "[Camera %zu] Using color sensor type: %d",
                      cam_index, static_cast<int>(st));
          break;
        }
      }
    }
    if (!color_sensor) {
      RCLCPP_ERROR(logger_, "[Camera %zu] No color sensor found (tried COLOR, COLOR_LEFT, COLOR_RIGHT)", cam_index);
      return false;
    }

    auto depth_sensor = device->getSensor(OB_SENSOR_DEPTH);
    if (!depth_sensor) {
      RCLCPP_ERROR(logger_, "[Camera %zu] Depth sensor not available", cam_index);
      return false;
    }

    auto color_profiles = color_sensor->getStreamProfileList();
    auto depth_profiles = depth_sensor->getStreamProfileList();

    if (!color_profiles || color_profiles->count() == 0) {
      RCLCPP_ERROR(logger_, "[Camera %zu] No available Color stream profiles", cam_index);
      return false;
    }
    if (!depth_profiles || depth_profiles->count() == 0) {
      RCLCPP_ERROR(logger_, "[Camera %zu] No available Depth stream profiles", cam_index);
      return false;
    }

    // ── Select color profile ──
    // Gemini 336L may prefer MJPG; try RGB888 first, then MJPG, then any.
    // SDK 的 getVideoStreamProfile() 在无匹配时抛异常（如请求 1280x720@3fps 而设备
    // 不支持该帧率），selectVideoStreamProfile() 内部逐级降级 fps 并捕获异常，
    // 避免直接导致 "Failed to start streams"。
    const std::vector<OBFormat> color_format_candidates = {
        OB_FORMAT_RGB888, OB_FORMAT_MJPG, OB_FORMAT_ANY};
    std::shared_ptr<ob::VideoStreamProfile> color_profile =
        selectVideoStreamProfile(color_profiles, color_width_, color_height_, color_fps_,
                                 color_format_candidates, "Color");
    if (!color_profile) {
      // Last resort: pick the first profile
      color_profile = color_profiles->getProfile(0)->as<ob::VideoStreamProfile>();
      RCLCPP_WARN(logger_, "[Camera %zu] Using fallback color profile: %dx%d fmt=0x%X fps=%d",
                  cam_index, color_profile->width(), color_profile->height(),
                  static_cast<unsigned>(color_profile->format()),
                  color_profile->fps());
    }

    // ── Select depth profile ──
    // When HW D2C alignment is requested, query compatible depth profiles
    // from the pipeline.  Not all depth profiles support hardware D2C;
    // the SDK returns only those that do for the selected color profile.
    std::shared_ptr<ob::VideoStreamProfile> depth_profile;
    // Only the HW alignment path needs D2C-compatible depth profiles. In SW mode
    // the in-node reprojection can use any ordinary depth profile.
    if (enable_align_ && align_mode_ == "HW") {
      auto d2c_depth_list = pipeline->getD2CDepthProfileList(color_profile, ALIGN_D2C_HW_MODE);
      if (d2c_depth_list && d2c_depth_list->count() > 0) {
        RCLCPP_INFO(logger_, "[Camera %zu] Found %u D2C-compatible depth profile(s)",
                    cam_index, d2c_depth_list->count());
        const std::vector<OBFormat> depth_format_candidates = {
            OB_FORMAT_Y16, OB_FORMAT_Y12, OB_FORMAT_Y11, OB_FORMAT_ANY};
        depth_profile = selectVideoStreamProfile(
            d2c_depth_list, depth_width_, depth_height_, depth_fps_,
            depth_format_candidates, "Depth(D2C)");
        if (!depth_profile) {
          depth_profile = d2c_depth_list->getProfile(0)->as<ob::VideoStreamProfile>();
          RCLCPP_WARN(logger_,
                      "[Camera %zu] No exact D2C depth match for %dx%d@%d; using fallback: %dx%d fmt=0x%X fps=%d",
                      cam_index, depth_width_, depth_height_, depth_fps_,
                      depth_profile->width(), depth_profile->height(),
                      static_cast<unsigned>(depth_profile->format()),
                      depth_profile->fps());
        }
      } else {
        RCLCPP_WARN(logger_,
                    "[Camera %zu] No D2C-compatible depth profiles found for the selected color profile. "
                    "Falling back to normal depth profile selection and disabling HW D2C alignment.",
                    cam_index);
        enable_align_ = false;
      }
    }

    if (!depth_profile) {
      // Non-D2C path (or D2C fallback): select depth profile from sensor directly
      const std::vector<OBFormat> depth_format_candidates = {
          OB_FORMAT_Y16, OB_FORMAT_Y12, OB_FORMAT_Y11, OB_FORMAT_ANY};
      depth_profile = selectVideoStreamProfile(
          depth_profiles, depth_width_, depth_height_, depth_fps_,
          depth_format_candidates, "Depth");
    }
    if (!depth_profile) {
      depth_profile = depth_profiles->getProfile(0)->as<ob::VideoStreamProfile>();
      RCLCPP_WARN(logger_, "[Camera %zu] Using fallback depth profile: %dx%d fmt=0x%X fps=%d",
                  cam_index, depth_profile->width(), depth_profile->height(),
                  static_cast<unsigned>(depth_profile->format()),
                  depth_profile->fps());
    }

    color_width_   = color_profile->width();
    color_height_  = color_profile->height();
    depth_width_   = depth_profile->width();
    depth_height_  = depth_profile->height();

    RCLCPP_INFO(logger_, "[Camera %zu] Color stream: %dx%d@%d fmt=0x%X",
                cam_index, color_width_, color_height_, color_profile->fps(),
                static_cast<unsigned>(color_profile->format()));
    RCLCPP_INFO(logger_, "[Camera %zu] Depth stream: %dx%d@%d fmt=0x%X",
                cam_index, depth_width_, depth_height_, depth_profile->fps(),
                static_cast<unsigned>(depth_profile->format()));

    cam.color_intrinsic_  = color_profile->getIntrinsic();
    cam.color_distortion_ = color_profile->getDistortion();
    cam.depth_intrinsic_  = depth_profile->getIntrinsic();
    cam.depth_distortion_ = depth_profile->getDistortion();

    RCLCPP_INFO(logger_, "[Camera %zu] Color intrinsics: fx=%f, fy=%f, cx=%f, cy=%f",
                cam_index, cam.color_intrinsic_.fx, cam.color_intrinsic_.fy,
                cam.color_intrinsic_.cx, cam.color_intrinsic_.cy);
    RCLCPP_INFO(logger_, "[Camera %zu] Depth intrinsics: fx=%f, fy=%f, cx=%f, cy=%f",
                cam_index, cam.depth_intrinsic_.fx, cam.depth_intrinsic_.fy,
                cam.depth_intrinsic_.cx, cam.depth_intrinsic_.cy);

    // Pre-build CameraInfo messages (constant for device lifetime)
    cam.rgb_camera_info_ = convertToCameraInfo(cam.color_intrinsic_, cam.color_distortion_);
    cam.rgb_camera_info_.header.frame_id = color_frame_id_;
    cam.depth_camera_info_ = convertToCameraInfo(cam.depth_intrinsic_, cam.depth_distortion_);
    cam.depth_camera_info_.header.frame_id = depth_frame_id_;

    // ── D2C Alignment Setup ──
    if (enable_align_) {
      // HW mode: use pipeline-level hardware D2C alignment so that the SDK
      // delivers already-warped depth frames in the FrameSet callback.
      cam.aligned_depth_intrinsic_ = cam.color_intrinsic_;
      cam.aligned_depth_camera_info_ = convertToCameraInfo(cam.aligned_depth_intrinsic_, cam.color_distortion_);
      cam.aligned_depth_camera_info_.header.frame_id = color_frame_id_;

      RCLCPP_INFO(logger_, "[Camera %zu] D2C alignment enabled (HW mode): depth -> color", cam_index);
      RCLCPP_INFO(logger_, "[Camera %zu] Aligned depth intrinsics (== color intrinsics): "
                  "fx=%f, fy=%f, cx=%f, cy=%f, %dx%d",
                  cam_index, cam.aligned_depth_intrinsic_.fx, cam.aligned_depth_intrinsic_.fy,
                  cam.aligned_depth_intrinsic_.cx, cam.aligned_depth_intrinsic_.cy,
                  cam.aligned_depth_intrinsic_.width, cam.aligned_depth_intrinsic_.height);
    } else if (align_mode_ == "SW") {
      // SW mode: in-node software D2C. Queries the depth->color extrinsic and
      // pre-builds CameraInfo using the color intrinsics. The per-frame warping
      // is performed by softwareAlignD2C() inside the FrameSet callback.
      syncSoftwareAlignment(cam_index, depth_profile, color_profile);
    } else {
      RCLCPP_INFO(logger_, "[Camera %zu] D2C alignment disabled", cam_index);
    }

    // ── RGB 点云生成器（SDK PointCloudFilter，替代原 depth 深度图） ──
    // process() 会自动从帧集的 stream profile 读取相机参数生成点云。
    // HW D2C 对齐路径下 pipeline 输出对齐帧集，点云位于彩色光学坐标系；
    // SW / 关闭对齐路径下输出为深度光学坐标系点云。
    cam.pc_filter_ = std::make_shared<ob::PointCloudFilter>();
    if (pointcloud_format_ == "point") {
      cam.pc_filter_->setCreatePointFormat(OB_FORMAT_POINT);  // 仅 xyz
    } else {
      cam.pc_filter_->setCreatePointFormat(OB_FORMAT_RGB_POINT);  // xyz + rgb
      cam.pc_filter_->setColorDataNormalization(true);  // 颜色归一化到 [0,1]
    }
    // 右手坐标系，坐标值 * coordinateScale = mm（onFrameSet 统一换算为米）
    cam.pc_filter_->setCoordinateSystem(OB_RIGHT_HAND_COORDINATE_SYSTEM);
    RCLCPP_INFO(logger_, "[Camera %zu] PointCloudFilter created (format=%s)",
                cam_index, pointcloud_format_.c_str());

    auto config = std::make_shared<ob::Config>();
    config->enableStream(color_profile);
    config->enableStream(depth_profile);

    // Alignment at pipeline level
    if (enable_align_) {
      // HW D2C alignment (depth → color)
      config->setAlignMode(ALIGN_D2C_HW_MODE);
      config->setDepthScaleRequire(true);
    } else if (align_mode_ == "SW") {
      // SW D2C: no hardware alignment; still request mm-scale depth so the
      // in-node reprojection math (which assumes millimeters) is correct.
      config->setDepthScaleRequire(true);
    }

    // Frame aggregate mode: require a full frameset (color + depth) for every output
    // so the callback always receives both streams. Avoids partial framesets that
    // would otherwise produce empty/all-zero point clouds (matches orbbec_camera
    // "full_frame" mode).
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_FULL_FRAME_REQUIRE);

    // Enable health monitor to detect pipeline errors
    pipeline->enableHealthMonitor([this, cam_index](OBPipelineStatus status) {
      if (status.issue != OB_PIPELINE_ISSUE_NONE) {
        RCLCPP_WARN(logger_, "[Camera %zu] Pipeline issue: 0x%X (sdk=0x%lX, dev=0x%lX, drv=0x%lX)",
                    cam_index,
                    static_cast<unsigned>(status.issue),
                    static_cast<unsigned long>(status.sdkStatus),
                    static_cast<unsigned long>(status.devStatus),
                    static_cast<unsigned long>(status.drvStatus));
      }
    }, 3000);

    // ── 主机时间同步：流启动前将相机时钟同步到系统时间 ──
    if (enable_host_time_sync_) {
      syncHostTimeOnce(cam_index);
    }

    pipeline->start(config, [this, cam_index](std::shared_ptr<ob::FrameSet> frame_set) {
      onFrameSet(cam_index, frame_set);
    });
    pipeline->enableFrameSync();

    // 仅主相机（index 0）等待首帧做快速自检；其余相机并行出帧，由 FPS 监控统一观测
    if (cam_index == 0) {
      RCLCPP_INFO(logger_, "[Camera %zu] Waiting for first frameset...", cam_index);
      auto first_fs = pipeline->waitForFrameset(2000);
      if (first_fs) {
        RCLCPP_INFO(logger_, "[Camera %zu] Pipeline producing frames (initial frameset received)", cam_index);
        onFrameSet(cam_index, first_fs);
        has_received_first_frame_ = true;
      } else {
        RCLCPP_WARN(logger_, "[Camera %zu] No frameset received within 2 seconds after pipeline start - check device compatibility", cam_index);
      }
    }

    RCLCPP_INFO(logger_, "[Camera %zu] Pipeline started successfully (camera_id=%u)",
                cam_index, cam.camera_id);
    return true;

  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "[Camera %zu] Failed to start streams: %s", cam_index, e.getMessage());
    return false;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "[Camera %zu] Failed to start streams: %s", cam_index, e.what());
    return false;
  }
}

void CameraDriverNode::onFrameSet(size_t cam_index, std::shared_ptr<ob::FrameSet> frame_set) {
  if (!frame_set || cam_index >= cameras_.size()) {
    return;
  }

  CameraStreamState &cam = cameras_[cam_index];
  const uint8_t camera_id = cam.camera_id;

  try {
    auto color_frame = frame_set->getFrame(OB_FRAME_COLOR);
    auto depth_frame = frame_set->getFrame(OB_FRAME_DEPTH);

    // Log missing streams to help diagnose callback silence
    static std::atomic<uint32_t> missing_frame_log_count{0};
    if (!color_frame || !depth_frame) {
      uint32_t cnt = missing_frame_log_count.fetch_add(1);
      if (cnt < 5 || cnt % 50 == 0) {
        RCLCPP_WARN(logger_,
                    "[Camera %zu] Incomplete FrameSet: color=%s, depth=%s (logged %u times)",
                    cam_index,
                    color_frame ? "yes" : "MISSING",
                    depth_frame ? "yes" : "MISSING",
                    cnt + 1);
      }
      return;
    }

    auto color_video = color_frame->as<ob::VideoFrame>();
    auto depth_video = depth_frame->as<ob::VideoFrame>();

    if (!color_video || !depth_video) {
      return;
    }

    // Update frame rate counters
    fps_frame_count_++;

    // 头部相机（camera_id==0）的点云需依赖 /Control/HeadPose 头部位姿做手眼变换
    // （T = M_end × kFixedTransform_0）。未收到头部位姿前直接丢弃该相机帧，
    // 不发布任何数据（/vision/rgb_raw 与 /fusion/synced_robot_vision 均不发布），
    // 避免下游拿到未变换到基坐标系的无效视觉数据。
    if (camera_id == 0 && !is_head_pose_available()) {
      RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 5000,
        "[Camera 0] head pose not received yet, camera 0 data not published "
        "(waiting for /Control/HeadPose)");
      return;
    }

    auto rgbd_msg = std::make_shared<com_interfaces::msg::SyncedRobotVision>();

    // 相机时间戳：优先使用相机报告的全局时间戳（主机时间同步后为 Unix 微秒），
    // 同步失败或数值无效时回退到系统时间 now()。
    rclcpp::Time color_ts;
    bool use_camera_ts = false;
    constexpr uint64_t kMinValidGlobalUs = 1600000000000000ULL;  // 约 2020-09-13 UTC
    if (enable_host_time_sync_ && cam.time_synced_) {
      try {
        const uint64_t global_us = color_frame->getGlobalTimeStampUs();
        if (global_us > kMinValidGlobalUs) {
          color_ts = fromUsToROSTime(global_us);
          use_camera_ts = true;
        }
      } catch (const ob::Error &e) {
        RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 10000,
          "[Camera %zu] getGlobalTimeStampUs failed: %s, fallback to now()",
          cam_index, e.what());
      } catch (const std::exception &e) {
        RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 10000,
          "[Camera %zu] getGlobalTimeStampUs failed: %s, fallback to now()",
          cam_index, e.what());
      }
    }
    if (!use_camera_ts) {
      color_ts = now();
    }

    // 诊断：打印相机帧时间戳（SDK 原始 us、全局 us、转换后 ROS 时间、系统时间）
    // 时间节流每 2 秒打印一次，避免“每 N 帧”节流在低帧率下打印间隔过大。
    // use_camera_ts=1 表示使用了相机全局时间戳，=0 表示回退到 now()。
    {
      const uint64_t sdk_us = color_frame->getTimeStampUs();
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
    // 注：SyncedRobotVision.msg 仅含单一 camera_info（彩色内参），无独立的深度内参字段
    rgbd_msg->camera_info = cam.rgb_camera_info_;
    rgbd_msg->camera_info.header.stamp = color_ts;

    // Color Image (RGB888)
    {
      sensor_msgs::msg::Image rgb_msg;
      rgb_msg.header.stamp = color_ts;
      rgb_msg.header.frame_id = color_fid;
      rgb_msg.height = color_video->getHeight();
      rgb_msg.width = color_video->getWidth();
      rgb_msg.encoding = sensor_msgs::image_encodings::RGB8;
      rgb_msg.is_bigendian = false;
      rgb_msg.step = rgb_msg.width * 3;
      rgb_msg.data.resize(rgb_msg.step * rgb_msg.height);
      memcpy(rgb_msg.data.data(), color_video->getData(), rgb_msg.data.size());
      // 新增话题 /vision/rgb_raw：仅当存在订阅者时发布主相机（camera_id==0）的 RGB 原始数据，
      // 无订阅者时不发布（get_subscription_count() == 0），避免无效拷贝与传输开销。
      if (camera_id == 0 && rgb_raw_pub_ && rgb_raw_pub_->get_subscription_count() > 0) {
        rgb_raw_pub_->publish(rgb_msg);
      }
      rgbd_msg->camera_rgbd_color = std::move(rgb_msg);
    }

    // Point Cloud（RGB 点云，单位米）
    {
      if (cam.sw_align_d2c_) {
        // SW D2C 模式：发布未对齐深度用于诊断，构建对齐深度缓冲区，
        // 并用对齐深度 + 彩色图 + 彩色内参构建与彩图逐像素对齐的 RGB 点云
        // （参考 orbbec_camera 的 publishColoredPointCloud()：对齐后深度
        //   位于彩色光学坐标系，必须用彩色内参反投影）。
        publishRawDepthImage(cam_index, depth_frame);
        softwareAlignD2C(cam_index, depth_video);
        buildAlignedPointCloud(cam_index, color_video, color_ts, color_fid, rgbd_msg);
      } else if (cam.pc_filter_) {
        // HW D2C / OFF 模式：使用 SDK PointCloudFilter 生成点云。
        // HW D2C 对齐实际生效时，pipeline 输出彩色光学坐标系的对齐深度帧；
        // 若深度帧分辨率与彩色不一致，说明 HW D2C 未生效（回退），
        // 此时点云位于深度光学坐标系，frame_id 必须相应切换。
        const bool depth_aligned = enable_align_ &&
            (depth_video->getWidth() == color_video->getWidth() &&
             depth_video->getHeight() == color_video->getHeight());
        // 请求了 HW D2C 但实际未对齐（深度帧分辨率 != 彩色分辨率）：
        // 不再跳过整帧，仍照常生成点云（位于深度光学坐标系），仅打印告警提示改用 SW 模式。
        if (enable_align_ && !depth_aligned) {
          RCLCPP_WARN_THROTTLE(
              logger_, *get_clock(), 5000,
              "[Camera %zu] HW D2C alignment not in effect (depth %ux%u != color %ux%u). "
              "Publishing point cloud in depth optical frame; set align_mode=SW for a pixel-aligned point cloud.",
              cam_index, depth_video->getWidth(), depth_video->getHeight(),
              color_video->getWidth(), color_video->getHeight());
        }
        // 与 orbbec_camera 参考实现一致：process 前用当前深度帧的 value scale
        // 修正 PointCloudFilter 坐标尺度（坐标值 × scale = mm）。
        cam.pc_filter_->setPositionDataScaled(depth_frame->as<ob::DepthFrame>()->getValueScale());
        auto pc_frame = cam.pc_filter_->process(frame_set);
        if (pc_frame) {
          auto points = pc_frame->as<ob::PointsFrame>();
          if (points && points->getDataSize() > 0) {
            const float scale = points->getCoordinateValueScale();  // 坐标值 × scale = mm
            const bool has_rgb = (pointcloud_format_ == "rgb_point");
            // OBColorPoint = 6×float32（xyz+rgb，rgb 已归一化到 [0,1]）
            // OBPoint = 3×float32（仅 xyz）
            const size_t src_step = has_rgb ? 24 : 12;
            const size_t num_points = points->getDataSize() / src_step;
            const uint8_t *src = static_cast<const uint8_t *>(points->getData());

            // pc_keep_invalid_points=true：保留所有点，无效深度（xyz 全 0）用 NaN 表示，
            // width = SDK 输出的全部点数（深度/对齐分辨率）；
            // pc_keep_invalid_points=false：沿用旧的防御性过滤，只发布有效点。
            const bool keep_invalid = pc_keep_invalid_points_;
            size_t out_total = num_points;
            if (!keep_invalid) {
              out_total = 0;
              for (size_t i = 0; i < num_points; ++i) {
                const float *p = reinterpret_cast<const float *>(src + i * src_step);
                if (p[0] != 0.0f || p[1] != 0.0f || p[2] != 0.0f) {
                  ++out_total;
                }
              }
            }

            sensor_msgs::msg::PointCloud2 pc_msg;
            pc_msg.header.stamp = color_ts;
            // HW D2C 实际生效（对齐帧分辨率 == 彩色分辨率）时点云位于彩色光学坐标系；
            // 对齐关闭/回退时为深度光学坐标系
            pc_msg.header.frame_id = depth_aligned ? color_fid : depth_fid;
            pc_msg.height = 1;
            pc_msg.width = static_cast<uint32_t>(out_total);
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

            const float to_m = scale / 1000.0f;  // mm -> m
            const float nan = std::numeric_limits<float>::quiet_NaN();
            uint8_t *dst = pc_msg.data.data();
            size_t out_idx = 0;
            for (size_t i = 0; i < num_points; ++i) {
              const float *p = reinterpret_cast<const float *>(src + i * src_step);
              const bool invalid = (p[0] == 0.0f && p[1] == 0.0f && p[2] == 0.0f);
              if (invalid && !keep_invalid) {
                continue;  // 不保留无效点（xyz 全 0，深度无效）
              }
              float *out = reinterpret_cast<float *>(dst + out_idx * pc_msg.point_step);
              if (invalid) {
                // 保留所有像素：无效深度点写 NaN，RGB 置 0
                out[0] = nan;
                out[1] = nan;
                out[2] = nan;
                if (has_rgb) {
                  const uint32_t rgb = 0;
                  memcpy(dst + out_idx * pc_msg.point_step + 12, &rgb, sizeof(rgb));
                }
              } else {
                out[0] = p[0] * to_m;
                out[1] = p[1] * to_m;
                out[2] = p[2] * to_m;
                if (has_rgb) {
                  uint8_t r = static_cast<uint8_t>(std::max(0.0f, std::min(1.0f, p[3])) * 255.0f + 0.5f);
                  uint8_t g = static_cast<uint8_t>(std::max(0.0f, std::min(1.0f, p[4])) * 255.0f + 0.5f);
                  uint8_t b = static_cast<uint8_t>(std::max(0.0f, std::min(1.0f, p[5])) * 255.0f + 0.5f);
                  uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                                 (static_cast<uint32_t>(g) << 8) |
                                 static_cast<uint32_t>(b);
                  memcpy(dst + out_idx * pc_msg.point_step + 12, &rgb, sizeof(rgb));
                }
              }
              ++out_idx;
            }
            rgbd_msg->camera_rgbd_pointcloud = std::move(pc_msg);
          }
        }
      }
    }

    // 点云坐标变换：按各相机外参将相机系点云变换到机器人基座标系（P' = T × P）。
    //   0 号（头部）相机：T = M_end × kFixedTransform_0，两次转换（相机→头部→基座）；
    //   1 号（底盘）相机：T = kFixedTransform_1，一次转换（相机→基座）。
    if (rgbd_msg->camera_rgbd_pointcloud.width > 0 &&
        rgbd_msg->camera_rgbd_pointcloud.height > 0) {
      applyPointCloudTransform(rgbd_msg->camera_rgbd_pointcloud, camera_id);
    }

    // Transition to RUNNING on first valid frame
    if (curr_st_ == CameraNodeSt::IDLE) {
      RCLCPP_INFO(logger_, "First valid frameset received (camera_id=%u), transition IDLE -> RUNNING", camera_id);
      change_status(CameraNodeSt::RUNNING, 0);
    }

    // 内部时间同步融合（替代原 time_sync 的 topic 订阅，发布 /fusion/synced_robot_vision）
    handle_camera_data(camera_id, rgbd_msg);

  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "[Camera %zu] onFrameSet error: %s", cam_index, e.getMessage());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "[Camera %zu] onFrameSet error: %s", cam_index, e.what());
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
// 由 onFrameSet 内部直接调用，不再经 ROS topic 中转
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
//   与头部相机固定标定矩阵 kFixedTransform_0（相机→头部末端）相乘得到
//   头部相机点云变换矩阵 T = M_end × kFixedTransform_0（行主序，P' = T × P）。
//   头部相机需两次转换：相机→头部末端（kFixedTransform_0）→基座（M_end）。
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

  // T = M_end × kFixedTransform_0（头部相机：相机→头部末端→基座）
  float T[16];
  mat4Mul(M_end, kFixedTransform_0.data(), T);

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

// 按 camera_id 选择对应相机的外参变换矩阵并应用到点云：P' = T × P（原地修改 xyz），
// 并更新 frame_id 为基座标系。
//   0 号（头部）相机：T = M_end × kFixedTransform_0（相机→头部末端→基座，两次转换），
//     尚未收到头部位姿时点云保持原样；
//   1 号（底盘）相机：T = kFixedTransform_1（相机→基座，一次转换，与头部位姿无关）。
void CameraDriverNode::applyPointCloudTransform(sensor_msgs::msg::PointCloud2 &pc_msg,
                                                uint8_t camera_id)
{
  float T[16];
  if (camera_id == 1) {
    // 底盘相机：固定外参直接变换到基座标系
    for (int i = 0; i < 16; ++i) {
      T[i] = kFixedTransform_1[i];
    }
  } else {
    // 头部相机：使用 M_end × kFixedTransform_0（头部末端位姿 × 相机→头部外参）
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

// 线程安全地查询是否已收到头部位姿（头部相机点云坐标变换依赖该位姿）
bool CameraDriverNode::is_head_pose_available() const {
  std::lock_guard<std::mutex> lock(head_pose_mtx_);
  return head_pose_available_;
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
// Image registration (self-implemented software D2C alignment)
// ============================================================================
void CameraDriverNode::syncSoftwareAlignment(
    size_t cam_index,
    std::shared_ptr<ob::VideoStreamProfile> depth_profile,
    std::shared_ptr<ob::VideoStreamProfile> color_profile) {
  if (cam_index >= cameras_.size()) {
    return;
  }
  CameraStreamState &cam = cameras_[cam_index];
  if (align_mode_ != "SW" || align_target_stream_ != OB_STREAM_COLOR) {
    RCLCPP_WARN(logger_, "[Camera %zu] syncSoftwareAlignment: only SW D2C (depth->color) is supported "
                "(align_mode=%s, target=%s)",
                cam_index, align_mode_.c_str(), align_target_stream_str_.c_str());
    return;
  }
  try {
    // depth -> color extrinsic (mm). getExtrinsicTo() returns the transform that
    // maps a 3D point in the depth camera frame into the color camera frame.
    cam.depth_to_color_extrinsic_ = depth_profile->getExtrinsicTo(color_profile);
    cam.depth_to_color_extrinsic_valid_ = true;
    RCLCPP_INFO(logger_, "[Camera %zu] Software D2C alignment enabled (self-implemented, in-node): depth -> color", cam_index);
    RCLCPP_INFO(logger_, "[Camera %zu] depth->color extrinsic trans=(%f, %f, %f) mm",
                cam_index,
                cam.depth_to_color_extrinsic_.trans[0],
                cam.depth_to_color_extrinsic_.trans[1],
                cam.depth_to_color_extrinsic_.trans[2]);

    // The aligned depth image lives in the color camera's coordinate space, so it
    // uses the color camera's intrinsics and distortion (resolution included).
    cam.aligned_depth_intrinsic_ = cam.color_intrinsic_;
    cam.aligned_depth_camera_info_ = convertToCameraInfo(cam.color_intrinsic_, cam.color_distortion_);
    cam.aligned_depth_camera_info_.header.frame_id = color_frame_id_;
    RCLCPP_INFO(logger_, "[Camera %zu] Aligned depth intrinsics (== color intrinsics): "
                "fx=%f, fy=%f, cx=%f, cy=%f, %dx%d",
                cam_index, cam.aligned_depth_intrinsic_.fx, cam.aligned_depth_intrinsic_.fy,
                cam.aligned_depth_intrinsic_.cx, cam.aligned_depth_intrinsic_.cy,
                color_width_, color_height_);

    // Publisher for the raw (un-aligned) depth, useful for verification.
    if (!depth_unaligned_pub_) {
      depth_unaligned_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image_unaligned", 10);
    }
    depth_registration_ = true;
    cam.sw_align_d2c_ = true;
  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "[Camera %zu] syncSoftwareAlignment error: %s", cam_index, e.getMessage());
    cam.depth_to_color_extrinsic_valid_ = false;
    cam.sw_align_d2c_ = false;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "[Camera %zu] syncSoftwareAlignment error: %s", cam_index, e.what());
    cam.depth_to_color_extrinsic_valid_ = false;
    cam.sw_align_d2c_ = false;
  }
}

void CameraDriverNode::publishRawDepthImage(size_t cam_index, const std::shared_ptr<ob::Frame>& depth_frame) {
  if (!depth_unaligned_pub_ || !depth_frame || cam_index >= cameras_.size()) {
    return;
  }
  const uint8_t camera_id = cameras_[cam_index].camera_id;
  std::string depth_fid = depth_frame_id_;
  if (camera_id != 0) {
    depth_fid = "camera_" + std::to_string(camera_id) + "_depth_optical_frame";
  }
  auto depth_video = depth_frame->as<ob::VideoFrame>();
  if (!depth_video) {
    return;
  }
  sensor_msgs::msg::Image msg;
  msg.header.stamp = now();  // 深度图时间戳同步改为系统时间
  msg.header.frame_id = depth_fid;
  msg.height = depth_video->getHeight();
  msg.width = depth_video->getWidth();
  msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  msg.is_bigendian = false;
  msg.step = msg.width * 2;
  msg.data.resize(msg.step * msg.height);
  memcpy(msg.data.data(), depth_video->getData(), msg.data.size());
  depth_unaligned_pub_->publish(msg);
}

void CameraDriverNode::softwareAlignD2C(size_t cam_index, const std::shared_ptr<ob::VideoFrame>& depth_video) {
  if (cam_index >= cameras_.size()) {
    return;
  }
  CameraStreamState &cam = cameras_[cam_index];
  if (!depth_video || !cam.depth_to_color_extrinsic_valid_) {
    return;
  }
  const uint32_t dw = depth_video->getWidth();
  const uint32_t dh = depth_video->getHeight();
  const uint16_t *src = reinterpret_cast<const uint16_t *>(depth_video->getData());
  if (!src) {
    return;
  }

  const uint32_t cw = static_cast<uint32_t>(color_width_);
  const uint32_t ch = static_cast<uint32_t>(color_height_);
  cam.aligned_depth_buffer_.assign(static_cast<size_t>(cw) * ch, 0);
  auto *dst = cam.aligned_depth_buffer_.data();

  // Depth camera intrinsics (pixel units)
  const float fx_d = cam.depth_intrinsic_.fx;
  const float fy_d = cam.depth_intrinsic_.fy;
  const float cx_d = cam.depth_intrinsic_.cx;
  const float cy_d = cam.depth_intrinsic_.cy;
  // Color camera intrinsics (pixel units)
  const float fx_c = cam.color_intrinsic_.fx;
  const float fy_c = cam.color_intrinsic_.fy;
  const float cx_c = cam.color_intrinsic_.cx;
  const float cy_c = cam.color_intrinsic_.cy;
  // depth -> color extrinsic: P_color = R * P_depth + t (mm)
  const float *R = cam.depth_to_color_extrinsic_.rot;
  const float *t = cam.depth_to_color_extrinsic_.trans;

  for (uint32_t v = 0; v < dh; ++v) {
    const uint16_t *row = src + static_cast<size_t>(v) * dw;
    for (uint32_t u = 0; u < dw; ++u) {
      const uint16_t z_raw = row[u];
      if (z_raw == 0) {
        continue;
      }
      const float z = static_cast<float>(z_raw);
      // 1) Back-project the depth pixel into the depth camera 3D frame (mm)
      const float x_d = (static_cast<float>(u) - cx_d) / fx_d * z;
      const float y_d = (static_cast<float>(v) - cy_d) / fy_d * z;
      // 2) Transform into the color camera 3D frame (mm)
      const float x_c = R[0] * x_d + R[1] * y_d + R[2] * z + t[0];
      const float y_c = R[3] * x_d + R[4] * y_d + R[5] * z + t[1];
      const float z_c = R[6] * x_d + R[7] * y_d + R[8] * z + t[2];
      if (z_c <= 0.0f) {
        continue;
      }
      // 3) Project into the color image plane
      const float u_c = fx_c * x_c / z_c + cx_c;
      const float v_c = fy_c * y_c / z_c + cy_c;
      const int uc = static_cast<int>(std::lround(u_c));
      const int vc = static_cast<int>(std::lround(v_c));
      if (uc < 0 || vc < 0 || uc >= static_cast<int>(cw) || vc >= static_cast<int>(ch)) {
        continue;
      }
      // Store the transformed depth (z in the color frame) so that a point cloud
      // reconstructed with the color intrinsics matches the original geometry.
      dst[static_cast<size_t>(vc) * cw + uc] =
          static_cast<uint16_t>(std::min(z_c, 65535.0f));
    }
  }
}

void CameraDriverNode::buildAlignedPointCloud(
    size_t cam_index,
    const std::shared_ptr<ob::VideoFrame>& color_video,
    const rclcpp::Time& color_ts,
    const std::string& color_fid,
    const SyncedRobotVision::SharedPtr& rgbd_msg) {
  if (cam_index >= cameras_.size()) {
    return;
  }
  CameraStreamState &cam = cameras_[cam_index];
  if (!color_video || cam.aligned_depth_buffer_.empty()) {
    return;
  }

  const uint32_t cw = color_video->getWidth();
  const uint32_t ch = color_video->getHeight();
  const size_t buf_len = static_cast<size_t>(cw) * ch;
  if (cam.aligned_depth_buffer_.size() != buf_len) {
    RCLCPP_WARN_ONCE(logger_,
                     "[Camera %zu] Aligned depth buffer (%zu) != color image (%ux%u); "
                     "skip aligned point cloud",
                     cam_index, cam.aligned_depth_buffer_.size(), cw, ch);
    return;
  }

  const float fx = cam.color_intrinsic_.fx;
  const float fy = cam.color_intrinsic_.fy;
  const float cx = cam.color_intrinsic_.cx;
  const float cy = cam.color_intrinsic_.cy;
  if (fx <= 0.0f || fy <= 0.0f) {
    return;
  }
  // 彩色图像数据（RGB888），与 onFrameSet 发布的 rgb 图同源
  const uint8_t *color_data = static_cast<const uint8_t *>(color_video->getData());
  if (!color_data) {
    return;
  }
  const size_t color_step = static_cast<size_t>(cw) * 3;

  const uint16_t *depth = cam.aligned_depth_buffer_.data();

  // pc_keep_invalid_points=true：保留所有像素（width=彩色分辨率），
  // 无效深度（0）用 NaN 表示；false 时沿用旧逻辑只统计有效点。
  const bool keep_invalid = pc_keep_invalid_points_;
  size_t valid = keep_invalid ? buf_len : 0;
  if (!keep_invalid) {
    for (size_t i = 0; i < buf_len; ++i) {
      if (depth[i] != 0) {
        ++valid;
      }
    }
  }
  if (valid == 0) {
    return;
  }

  sensor_msgs::msg::PointCloud2 pc_msg;
  pc_msg.header.stamp = color_ts;
  pc_msg.header.frame_id = color_fid;  // 对齐后点云位于彩色光学坐标系
  pc_msg.height = 1;
  pc_msg.width = static_cast<uint32_t>(valid);
  pc_msg.fields.resize(4);
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
  // ROS 标准：rgb 字段以 FLOAT32 声明，实际存放打包的 uint32(R<<16|G<<8|B)
  pc_msg.fields[3].name = "rgb";
  pc_msg.fields[3].offset = 12;
  pc_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pc_msg.fields[3].count = 1;
  pc_msg.is_bigendian = false;
  pc_msg.point_step = 16;
  pc_msg.row_step = pc_msg.point_step * pc_msg.width;
  pc_msg.is_dense = false;
  pc_msg.data.resize(pc_msg.row_step);

  constexpr float mm_to_m = 0.001f;  // mm -> m
  const float nan = std::numeric_limits<float>::quiet_NaN();
  uint8_t *dst = pc_msg.data.data();
  size_t idx = 0;
  for (uint32_t v = 0; v < ch; ++v) {
    const uint8_t *color_row = color_data + static_cast<size_t>(v) * color_step;
    const uint16_t *depth_row = depth + static_cast<size_t>(v) * cw;
    for (uint32_t u = 0; u < cw; ++u) {
      const uint16_t z_mm = depth_row[u];
      const bool invalid = (z_mm == 0);
      if (invalid && !keep_invalid) {
        continue;
      }
      float *out = reinterpret_cast<float *>(dst + idx * pc_msg.point_step);
      if (invalid) {
        // 保留所有像素：无效深度点写 NaN，RGB 置 0
        out[0] = nan;
        out[1] = nan;
        out[2] = nan;
        const uint32_t rgb = 0;
        memcpy(dst + idx * pc_msg.point_step + 12, &rgb, sizeof(rgb));
      } else {
        // 对齐后深度位于彩色坐标系，用彩色内参反投影（单位米）
        const float z = static_cast<float>(z_mm) * mm_to_m;
        const float x = (static_cast<float>(u) - cx) / fx * z;
        const float y = (static_cast<float>(v) - cy) / fy * z;
        out[0] = x;
        out[1] = y;
        out[2] = z;
        float piont[4] = {};
        piont[0] = x;
        piont[1] = y;
        piont[2] = z; 
        piont[3] = 1; 
        // 从彩色图像取 RGB 打包（与 HW 路径相同的 ROS rgb 布局）
        const uint8_t r = color_row[u * 3 + 0];
        const uint8_t g = color_row[u * 3 + 1];
        const uint8_t b = color_row[u * 3 + 2];
        const uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                             (static_cast<uint32_t>(g) << 8) |
                             static_cast<uint32_t>(b);
        memcpy(dst + idx * pc_msg.point_step + 12, &rgb, sizeof(rgb));
      }
      ++idx;
    }
  }

  rgbd_msg->camera_rgbd_pointcloud = std::move(pc_msg);
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
    // 停止主机时间同步周期定时器
    if (time_sync_timer_) {
      time_sync_timer_->cancel();
      time_sync_timer_.reset();
    }
    // 停止并释放所有相机的 Pipeline 与 Device
    for (auto &cam : cameras_) {
      if (cam.pipeline) {
        cam.pipeline->stop();
      }
      cam.pipeline.reset();
      cam.device.reset();
    }
    cameras_.clear();
    latest_cam_data_by_cam_.clear();
    pipeline_.reset();
    device_.reset();
  } catch (const std::exception &e) {
    RCLCPP_WARN(logger_, "Error stopping pipelines: %s", e.what());
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
    const OBCameraIntrinsic &intrinsic,
    const OBCameraDistortion &distortion) {
  sensor_msgs::msg::CameraInfo info;
  info.width = intrinsic.width;
  info.height = intrinsic.height;

  info.distortion_model = "plumb_bob";
  info.d.resize(5, 0.0);
  info.d[0] = distortion.k1;
  info.d[1] = distortion.k2;
  info.d[2] = distortion.p1;
  info.d[3] = distortion.p2;
  info.d[4] = distortion.k3;

  info.k.fill(0.0);
  info.k[0] = intrinsic.fx;
  info.k[2] = intrinsic.cx;
  info.k[4] = intrinsic.fy;
  info.k[5] = intrinsic.cy;
  info.k[8] = 1.0;

  info.r.fill(0.0);
  info.r[0] = 1;
  info.r[4] = 1;
  info.r[8] = 1;

  info.p.fill(0.0);
  info.p[0] = intrinsic.fx;
  info.p[2] = intrinsic.cx;
  info.p[5] = intrinsic.fy;
  info.p[6] = intrinsic.cy;
  info.p[10] = 1.0;

  return info;
}

// ============================================================================
// main
// ============================================================================
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
