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

// ============================================================================
// 初始化
// ============================================================================
void CameraDriverNode::init() {
  logger_ = get_logger();
  declareParameters();
  getParameters();

  // --- Create publisher with RELIABLE QoS for compatibility with all subscribers ---
  rclcpp::QoS qos(rclcpp::KeepLast(5));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  rgbd_pub_ = create_publisher<com_interfaces::msg::CameraDriver>("/vision/camera_data", qos);

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

  // Initialize SDK context
  try {
    ctx_ = std::make_unique<ob::Context>();
  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "Failed to create Orbbec SDK context: %s", e.getMessage());
    return;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Failed to create Orbbec SDK context: %s", e.what());
    return;
  }

  // Try to acquire cross-process device lock
  if (!acquireDeviceLock()) {
    RCLCPP_ERROR(logger_,
                 "Device is occupied by another process (orbbec_camera_node may be running).\n"
                 "Please stop orbbec_camera_node first, or set parameter bypass_lock:=true to force access.");
    return;
  }

  // Query and open device
  try {
    if (!openDevice()) {
      RCLCPP_ERROR(logger_, "Failed to open Orbbec device");
      cleanupAfterInitFailure();
      return;
    }
  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "Unexpected error opening device: %s", e.getMessage());
    cleanupAfterInitFailure();
    return;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Unexpected error opening device: %s", e.what());
    cleanupAfterInitFailure();
    return;
  }

  // Create Pipeline
  try {
    pipeline_ = std::make_shared<ob::Pipeline>(device_);
  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "Failed to create pipeline: %s", e.getMessage());
    cleanupAfterInitFailure();
    return;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Failed to create pipeline: %s", e.what());
    cleanupAfterInitFailure();
    return;
  }

  // Configure and start streams
  try {
    if (!startStreams()) {
      RCLCPP_ERROR(logger_, "Failed to start streams");
      cleanupAfterInitFailure();
      return;
    }
  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "Unexpected error starting streams: %s", e.getMessage());
    cleanupAfterInitFailure();
    return;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Unexpected error starting streams: %s", e.what());
    cleanupAfterInitFailure();
    return;
  }

  // Start frame rate monitoring timer (every 5 seconds)
  fps_last_check_ = std::chrono::steady_clock::now();
  fps_frame_count_ = 0;
  fps_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&CameraDriverNode::checkFrameRate, this));

  RCLCPP_INFO(logger_, "CameraDriverNode initialized successfully");
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
  declare_parameter<int>("color_width", 1280);
  declare_parameter<int>("color_height", 720);
  declare_parameter<int>("color_fps", 30);
  declare_parameter<int>("depth_width", 1280);
  declare_parameter<int>("depth_height", 720);
  declare_parameter<int>("depth_fps", 30);
  declare_parameter<std::string>("serial_number", "");
  declare_parameter<std::string>("frame_id", "camera_link");
  declare_parameter<std::string>("color_frame_id", "camera_color_optical_frame");
  declare_parameter<std::string>("depth_frame_id", "camera_depth_optical_frame");
  declare_parameter<bool>("bypass_lock", false);
}

void CameraDriverNode::getParameters() {
  color_width_ = get_parameter("color_width").as_int();
  color_height_ = get_parameter("color_height").as_int();
  color_fps_ = get_parameter("color_fps").as_int();
  depth_width_ = get_parameter("depth_width").as_int();
  depth_height_ = get_parameter("depth_height").as_int();
  depth_fps_ = get_parameter("depth_fps").as_int();
  serial_number_ = get_parameter("serial_number").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  color_frame_id_ = get_parameter("color_frame_id").as_string();
  depth_frame_id_ = get_parameter("depth_frame_id").as_string();
  bypass_lock_ = get_parameter("bypass_lock").as_bool();
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
  if (!device_) {
    all_ok = false;
    error_msg = "No Orbbec device connected";
    RCLCPP_ERROR(logger_, "[HardwareCheck] %s", error_msg.c_str());
  }

  // --- Step 2: Verify firmware version ---
  if (all_ok && !send_feedback(30.0f, "Verifying firmware version...")) return;
  if (all_ok && device_) {
    try {
      auto dev_info = device_->getDeviceInfo();
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
  if (all_ok && pipeline_) {
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

  // --- Step 4: Validate calibration data ---
  if (all_ok && !send_feedback(70.0f, "Validating calibration data...")) return;
  if (all_ok) {
    if (color_intrinsic_.fx <= 0 || color_intrinsic_.fy <= 0 ||
        depth_intrinsic_.fx <= 0 || depth_intrinsic_.fy <= 0) {
      all_ok = false;
      error_msg = "Invalid camera intrinsics (fx/fy are zero or negative)";
      RCLCPP_ERROR(logger_, "[HardwareCheck] %s", error_msg.c_str());
    } else {
      RCLCPP_INFO(logger_, "[HardwareCheck] Calibration valid: Color fx=%.2f, Depth fx=%.2f",
                  color_intrinsic_.fx, depth_intrinsic_.fx);
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

  // --- Step 1: Stop camera streams ---
  if (!send_feedback(10.0f, "Stopping camera streams...")) return;
  try {
    if (pipeline_) {
      pipeline_->stop();
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN(logger_, "[HardwareReset] Error stopping pipeline: %s", e.what());
  }

  // --- Step 2: Release device ---
  if (!send_feedback(30.0f, "Resetting device...")) return;
  pipeline_.reset();
  device_.reset();

  // Small delay to let hardware settle
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // --- Step 3: Re-open device and restart streams ---
  if (!send_feedback(60.0f, "Re-initializing sensor...")) return;
  bool reopen_ok = false;
  try {
    // Re-query device list
    auto device_list = ctx_->queryDeviceList();
    if (device_list && device_list->deviceCount() > 0) {
      if (!serial_number_.empty()) {
        for (uint32_t i = 0; i < device_list->deviceCount(); i++) {
          try {
            auto dev = device_list->getDevice(i);
            auto dev_info = dev->getDeviceInfo();
            if (dev_info && dev_info->serialNumber() == serial_number_) {
              device_ = dev;
              break;
            }
          } catch (const ob::Error &e) {
            RCLCPP_WARN(logger_, "[HardwareReset] Failed to open device index %d: %s", i, e.getMessage());
            continue;
          }
        }
      } else {
        for (uint32_t i = 0; i < device_list->deviceCount(); i++) {
          try {
            device_ = device_list->getDevice(i);
            break;
          } catch (const ob::Error &e) {
            RCLCPP_WARN(logger_, "[HardwareReset] Failed to open device index %d: %s", i, e.getMessage());
            continue;
          }
        }
      }
    }

    if (device_) {
      pipeline_ = std::make_shared<ob::Pipeline>(device_);
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

  // --- Step 4: Verify device status ---
  if (!send_feedback(80.0f, "Verifying device status...")) return;
  try {
    auto dev_info = device_->getDeviceInfo();
    if (dev_info) {
      RCLCPP_INFO(logger_, "[HardwareReset] Device reconnected: %s (SN: %s)",
                  dev_info->name(), dev_info->serialNumber());
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

  RCLCPP_INFO(logger_, "Found %d device(s)", device_list->deviceCount());

  if (!serial_number_.empty()) {
    for (uint32_t i = 0; i < device_list->deviceCount(); i++) {
      try {
        auto dev = device_list->getDevice(i);
        auto dev_info = dev->getDeviceInfo();
        if (dev_info && dev_info->serialNumber() == serial_number_) {
          device_ = dev;
          RCLCPP_INFO(logger_, "Selected device by serial number: %s", serial_number_.c_str());
          break;
        }
      } catch (const ob::Error &e) {
        RCLCPP_WARN(logger_, "Failed to open device index %d (SN filter): %s", i, e.getMessage());
        continue;
      }
    }
    if (!device_) {
      RCLCPP_ERROR(logger_, "No device found with serial number '%s'", serial_number_.c_str());
      return false;
    }
  } else {
    // Try to open each available device until one succeeds
    for (uint32_t i = 0; i < device_list->deviceCount(); i++) {
      try {
        auto dev = device_list->getDevice(i);
        auto dev_info = dev->getDeviceInfo();
        if (dev_info) {
          RCLCPP_INFO(logger_, "Selected device: %s (SN: %s)", dev_info->name(),
                      dev_info->serialNumber());
        }
        device_ = dev;
        break;
      } catch (const ob::Error &e) {
        RCLCPP_WARN(logger_,
                    "Failed to open device index %d: %s\n"
                    "  Hint: Check USB permissions (udev rules) or if device is already in use.",
                    i, e.getMessage());
        // Try next device if available
        if (i + 1 < device_list->deviceCount()) {
          RCLCPP_INFO(logger_, "Trying next device...");
        }
        continue;
      }
    }
    if (!device_) {
      RCLCPP_ERROR(logger_,
                   "Failed to open any of the %d detected device(s).\n"
                   "  Common causes:\n"
                   "  1. Missing USB permissions (run: sudo cp SDK/scripts/99-obsensor-libusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger)\n"
                   "  2. Device already in use by orbbec_camera_node or another process\n"
                   "  3. Insufficient power to USB port",
                   device_list->deviceCount());
      return false;
    }
  }

  return device_ != nullptr;
}

bool CameraDriverNode::startStreams() {
  if (!pipeline_) {
    RCLCPP_ERROR(logger_, "Pipeline is null");
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
        color_sensor = device_->getSensor(st);
        if (color_sensor) {
          //color_sensor_type_ = st;
          RCLCPP_INFO(logger_, "Using color sensor type: %d", static_cast<int>(st));
          break;
        }
      }
    }
    if (!color_sensor) {
      RCLCPP_ERROR(logger_, "No color sensor found (tried COLOR, COLOR_LEFT, COLOR_RIGHT)");
      return false;
    }

    auto depth_sensor = device_->getSensor(OB_SENSOR_DEPTH);
    if (!depth_sensor) {
      RCLCPP_ERROR(logger_, "Depth sensor not available");
      return false;
    }

    auto color_profiles = color_sensor->getStreamProfileList();
    auto depth_profiles = depth_sensor->getStreamProfileList();

    if (!color_profiles || color_profiles->count() == 0) {
      RCLCPP_ERROR(logger_, "No available Color stream profiles");
      return false;
    }
    if (!depth_profiles || depth_profiles->count() == 0) {
      RCLCPP_ERROR(logger_, "No available Depth stream profiles");
      return false;
    }

    // ── Select color profile ──
    // Gemini 336L may prefer MJPG; try RGB888 first, then MJPG, then any.
    std::shared_ptr<ob::VideoStreamProfile> color_profile;
    {
      constexpr OBFormat color_format_candidates[] = {
          OB_FORMAT_RGB888, OB_FORMAT_MJPG, OB_FORMAT_ANY};
      for (auto fmt : color_format_candidates) {
        color_profile = color_profiles->getVideoStreamProfile(
            color_width_, color_height_, fmt, color_fps_);
        if (color_profile) break;
      }
    }
    if (!color_profile) {
      // Last resort: pick the first profile
      color_profile = color_profiles->getProfile(0)->as<ob::VideoStreamProfile>();
      RCLCPP_WARN(logger_, "Using fallback color profile: %dx%d fmt=0x%X fps=%d",
                  color_profile->width(), color_profile->height(),
                  static_cast<unsigned>(color_profile->format()),
                  color_profile->fps());
    }

    // ── Select depth profile ──
    std::shared_ptr<ob::VideoStreamProfile> depth_profile;
    {
      constexpr OBFormat depth_format_candidates[] = {
          OB_FORMAT_Y16, OB_FORMAT_Y12, OB_FORMAT_Y11, OB_FORMAT_ANY};
      for (auto fmt : depth_format_candidates) {
        depth_profile = depth_profiles->getVideoStreamProfile(
            depth_width_, depth_height_, fmt, depth_fps_);
        if (depth_profile) break;
      }
    }
    if (!depth_profile) {
      depth_profile = depth_profiles->getProfile(0)->as<ob::VideoStreamProfile>();
      RCLCPP_WARN(logger_, "Using fallback depth profile: %dx%d fmt=0x%X fps=%d",
                  depth_profile->width(), depth_profile->height(),
                  static_cast<unsigned>(depth_profile->format()),
                  depth_profile->fps());
    }

    color_width_   = color_profile->width();
    color_height_  = color_profile->height();
    depth_width_   = depth_profile->width();
    depth_height_  = depth_profile->height();

    RCLCPP_INFO(logger_, "Color stream: %dx%d@%d fmt=0x%X",
                color_width_, color_height_, color_profile->fps(),
                static_cast<unsigned>(color_profile->format()));
    RCLCPP_INFO(logger_, "Depth stream: %dx%d@%d fmt=0x%X",
                depth_width_, depth_height_, depth_profile->fps(),
                static_cast<unsigned>(depth_profile->format()));

    color_intrinsic_  = color_profile->getIntrinsic();
    color_distortion_ = color_profile->getDistortion();
    depth_intrinsic_  = depth_profile->getIntrinsic();
    depth_distortion_ = depth_profile->getDistortion();

    RCLCPP_INFO(logger_, "Color intrinsics: fx=%f, fy=%f, cx=%f, cy=%f",
                color_intrinsic_.fx, color_intrinsic_.fy,
                color_intrinsic_.cx, color_intrinsic_.cy);
    RCLCPP_INFO(logger_, "Depth intrinsics: fx=%f, fy=%f, cx=%f, cy=%f",
                depth_intrinsic_.fx, depth_intrinsic_.fy,
                depth_intrinsic_.cx, depth_intrinsic_.cy);

    // Pre-build CameraInfo messages (constant for device lifetime)
    rgb_camera_info_ = convertToCameraInfo(color_intrinsic_, color_distortion_);
    rgb_camera_info_.header.frame_id = color_frame_id_;
    depth_camera_info_ = convertToCameraInfo(depth_intrinsic_, depth_distortion_);
    depth_camera_info_.header.frame_id = depth_frame_id_;

    auto config = std::make_shared<ob::Config>();
    config->enableStream(color_profile);
    config->enableStream(depth_profile);

    // Set frame aggregate mode: output framesets even if some streams are missing
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);

    // Enable health monitor to detect pipeline errors
    pipeline_->enableHealthMonitor([this](OBPipelineStatus status) {
      if (status.issue != OB_PIPELINE_ISSUE_NONE) {
        RCLCPP_WARN(logger_, "Pipeline issue: 0x%X (sdk=0x%lX, dev=0x%lX, drv=0x%lX)",
                    static_cast<unsigned>(status.issue),
                    static_cast<unsigned long>(status.sdkStatus),
                    static_cast<unsigned long>(status.devStatus),
                    static_cast<unsigned long>(status.drvStatus));
      }
    }, 3000);

    pipeline_->start(config, [this](std::shared_ptr<ob::FrameSet> frame_set) {
      onFrameSet(frame_set);
    });

    // Quick sanity check: wait briefly for first frameset
    RCLCPP_INFO(logger_, "Waiting for first frameset...");
    auto first_fs = pipeline_->waitForFrameset(2000);
    if (first_fs) {
      RCLCPP_INFO(logger_, "Pipeline producing frames (initial frameset received)");
      onFrameSet(first_fs);
      has_received_first_frame_ = true;
    } else {
      RCLCPP_WARN(logger_, "No frameset received within 2 seconds after pipeline start - check device compatibility");
    }

    RCLCPP_INFO(logger_, "Pipeline started successfully");
    return true;

  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "Failed to start streams: %s", e.getMessage());
    return false;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Failed to start streams: %s", e.what());
    return false;
  }
}

void CameraDriverNode::onFrameSet(std::shared_ptr<ob::FrameSet> frame_set) {
  if (!frame_set) {
    return;
  }

  try {
    auto color_frame = frame_set->getFrame(OB_FRAME_COLOR);
    auto depth_frame = frame_set->getFrame(OB_FRAME_DEPTH);

    // Log missing streams to help diagnose callback silence
    static int missing_frame_log_count = 0;
    if (!color_frame || !depth_frame) {
      if (missing_frame_log_count < 5 || missing_frame_log_count % 50 == 0) {
        RCLCPP_WARN(logger_,
                    "Incomplete FrameSet: color=%s, depth=%s (logged %d times)",
                    color_frame ? "yes" : "MISSING",
                    depth_frame ? "yes" : "MISSING",
                    missing_frame_log_count + 1);
      }
      missing_frame_log_count++;
      return;
    }

    auto color_video = color_frame->as<ob::VideoFrame>();
    auto depth_video = depth_frame->as<ob::VideoFrame>();

    if (!color_video || !depth_video) {
      return;
    }

    // Update frame rate counters
    fps_frame_count_++;

    auto rgbd_msg = std::make_unique<com_interfaces::msg::CameraDriver>();

    auto color_ts = fromUsToROSTime(color_frame->getTimeStampUs());
    auto depth_ts = fromUsToROSTime(depth_frame->getTimeStampUs());

    rgbd_msg->header.stamp = color_ts;
    rgbd_msg->header.frame_id = frame_id_;

    // Use pre-built CameraInfo (only update timestamp per frame)
    rgbd_msg->rgb_camera_info = rgb_camera_info_;
    rgbd_msg->rgb_camera_info.header.stamp = color_ts;

    rgbd_msg->depth_camera_info = depth_camera_info_;
    rgbd_msg->depth_camera_info.header.stamp = depth_ts;

    // Color Image (RGB888) - use zero-copy via pointer reference where possible
    {
      sensor_msgs::msg::Image rgb_msg;
      rgb_msg.header.stamp = color_ts;
      rgb_msg.header.frame_id = color_frame_id_;
      rgb_msg.height = color_video->getHeight();
      rgb_msg.width = color_video->getWidth();
      rgb_msg.encoding = sensor_msgs::image_encodings::RGB8;
      rgb_msg.is_bigendian = false;
      rgb_msg.step = rgb_msg.width * 3;
      // Pre-allocate and copy (Orbbec SDK buffer is owned by SDK)
      rgb_msg.data.resize(rgb_msg.step * rgb_msg.height);
      memcpy(rgb_msg.data.data(), color_video->getData(), rgb_msg.data.size());
      rgbd_msg->rgb = std::move(rgb_msg);
    }

    // Depth Image (Y16)
    {
      sensor_msgs::msg::Image depth_msg;
      depth_msg.header.stamp = depth_ts;
      depth_msg.header.frame_id = depth_frame_id_;
      depth_msg.height = depth_video->getHeight();
      depth_msg.width = depth_video->getWidth();
      depth_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      depth_msg.is_bigendian = false;
      depth_msg.step = depth_msg.width * 2;
      depth_msg.data.resize(depth_msg.step * depth_msg.height);
      memcpy(depth_msg.data.data(), depth_video->getData(), depth_msg.data.size());
      rgbd_msg->depth = std::move(depth_msg);
    }

    rgbd_pub_->publish(std::move(rgbd_msg));

  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "onFrameSet error: %s", e.getMessage());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "onFrameSet error: %s", e.what());
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
    if (pipeline_) {
      pipeline_->stop();
      pipeline_.reset();
    }
    device_.reset();
  } catch (const std::exception &e) {
    RCLCPP_WARN(logger_, "Error stopping pipeline: %s", e.what());
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
