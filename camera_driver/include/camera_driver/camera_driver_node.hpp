#ifndef CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_
#define CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/header.hpp"

#include "com_interfaces/msg/camera_driver.hpp"
#include "com_interfaces/action/hardware_check.hpp"
#include "com_interfaces/action/hardware_reset.hpp"

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Device.hpp"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "libobsensor/hpp/Context.hpp"
#include "libobsensor/hpp/Sensor.hpp"

static constexpr const char *CAMERA_DRIVER_NODE_NAME = "camera_driver_node";
static constexpr const char *ORB_DEVICE_LOCK_NAME = "orbbec_device_lock";

class CameraDriverNode : public rclcpp::Node {
 public:
  using HardwareCheck = com_interfaces::action::HardwareCheck;
  using GoalHandleHardwareCheck = rclcpp_action::ServerGoalHandle<HardwareCheck>;
  using HardwareReset = com_interfaces::action::HardwareReset;
  using GoalHandleHardwareReset = rclcpp_action::ServerGoalHandle<HardwareReset>;

  explicit CameraDriverNode(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node(CAMERA_DRIVER_NODE_NAME, node_options) {
    init();
  }

  CameraDriverNode(const std::string &node_name, const std::string &ns,
                   const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node(node_name, ns, node_options) {
    init();
  }

  ~CameraDriverNode() override {
    stop();
    cleanupDeviceLock();
    // Signal any active action thread to finish
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      action_cancelled_ = true;
      if (action_promise_) {
        action_promise_->set_value();
      }
    }
    // Give the action thread a chance to complete
    if (action_thread_.joinable()) {
      action_thread_.join();
    }
  }

 private:
  void init();

  // --- HardwareCheck Action Server (自检) ---
  rclcpp_action::GoalResponse handle_hw_check_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const HardwareCheck::Goal> goal);

  rclcpp_action::CancelResponse handle_hw_check_cancel(
    const std::shared_ptr<GoalHandleHardwareCheck> goal_handle);

  void handle_hw_check_accepted(
    const std::shared_ptr<GoalHandleHardwareCheck> goal_handle);

  void execute_hw_check(const std::shared_ptr<GoalHandleHardwareCheck> goal_handle);

  // --- HardwareReset Action Server (复位) ---
  rclcpp_action::GoalResponse handle_hw_reset_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const HardwareReset::Goal> goal);

  rclcpp_action::CancelResponse handle_hw_reset_cancel(
    const std::shared_ptr<GoalHandleHardwareReset> goal_handle);

  void handle_hw_reset_accepted(
    const std::shared_ptr<GoalHandleHardwareReset> goal_handle);

  void execute_hw_reset(const std::shared_ptr<GoalHandleHardwareReset> goal_handle);

  // --- Camera functions ---
  void declareParameters();
  void getParameters();
  void cleanupAfterInitFailure();
  bool acquireDeviceLock();
  void releaseDeviceLock();
  void cleanupDeviceLock();
  bool openDevice();
  bool startStreams();
  void onFrameSet(std::shared_ptr<ob::FrameSet> frame_set);
  void stop();

  // --- Frame rate monitoring ---
  void checkFrameRate();

  static rclcpp::Time fromUsToROSTime(uint64_t us);
  static sensor_msgs::msg::CameraInfo convertToCameraInfo(
      const OBCameraIntrinsic &intrinsic,
      const OBCameraDistortion &distortion);

  // --- Members ---
  rclcpp::Logger logger_{rclcpp::get_logger(CAMERA_DRIVER_NODE_NAME)};
  rclcpp::Publisher<com_interfaces::msg::CameraDriver>::SharedPtr rgbd_pub_{nullptr};

  // Action servers
  rclcpp_action::Server<HardwareCheck>::SharedPtr hw_check_server_{nullptr};
  rclcpp_action::Server<HardwareReset>::SharedPtr hw_reset_server_{nullptr};

  // Action execution management (replaces detach threads)
  std::mutex action_mutex_;
  std::atomic<bool> action_cancelled_{false};
  std::thread action_thread_;
  std::unique_ptr<std::promise<void>> action_promise_{nullptr};

  // Orbbec SDK
  std::unique_ptr<ob::Context> ctx_{nullptr};
  std::shared_ptr<ob::Device> device_{nullptr};
  std::shared_ptr<ob::Pipeline> pipeline_{nullptr};

  // Cross-process device lock
  int shm_fd_ = -1;
  uint8_t *shm_addr_ = nullptr;
  pthread_mutex_t *device_lock_ = nullptr;
  bool lock_acquired_ = false;
  bool bypass_lock_ = false;

  // Camera intrinsics (cached from StreamProfile at startup)
  OBCameraIntrinsic color_intrinsic_{};
  OBCameraDistortion color_distortion_{};
  OBCameraIntrinsic depth_intrinsic_{};
  OBCameraDistortion depth_distortion_{};

  // Pre-built CameraInfo messages (avoid reconstructing every frame)
  sensor_msgs::msg::CameraInfo rgb_camera_info_;
  sensor_msgs::msg::CameraInfo depth_camera_info_;

  // Frame rate monitoring
  std::chrono::steady_clock::time_point fps_last_check_{};
  uint32_t fps_frame_count_ = 0;
  double actual_color_fps_ = 0.0;
  double actual_depth_fps_ = 0.0;
  bool has_received_first_frame_ = false;
  rclcpp::TimerBase::SharedPtr fps_timer_{nullptr};

  // Parameters
  std::string serial_number_;
  int color_width_ = 1280;
  int color_height_ = 720;
  int color_fps_ = 30;
  int depth_width_ = 1280;
  int depth_height_ = 720;
  int depth_fps_ = 30;
  std::string frame_id_{"camera_link"};
  std::string color_frame_id_{"camera_color_optical_frame"};
  std::string depth_frame_id_{"camera_depth_optical_frame"};
};

#endif  // CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_
