#ifndef CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_
#define CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <deque>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <climits>
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
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "com_interfaces/msg/heart_beat.hpp"
#include "com_interfaces/msg/node_status.hpp"
#include "com_interfaces/msg/motor_status.hpp"
#include "com_interfaces/msg/synced_robot_vision.hpp"
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
#include "libobsensor/hpp/Filter.hpp"

static constexpr const char *CAMERA_DRIVER_NODE_NAME = "camera_driver_node";
static constexpr const char *ORB_DEVICE_LOCK_NAME = "orbbec_device_lock";
static constexpr int32_t CAMERA_DRIVER_NODE_ID = 4;

// 相机驱动节点状态枚举
enum class CameraNodeSt : int32_t {
  UNKNOWN     = 0,
  INIT        = 1,
  CHECK       = 2,
  RESET       = 3,
  IDLE        = 5,
  RESET_DONE  = 6,
  RUNNING     = 7,
  EXCEPTION   = 8
};

// 状态枚举转可读字符串
inline const char* node_state_to_string(CameraNodeSt st) {
  switch (st) {
    case CameraNodeSt::UNKNOWN:    return "UNKNOWN";
    case CameraNodeSt::INIT:       return "INIT";
    case CameraNodeSt::CHECK:      return "CHECK";
    case CameraNodeSt::RESET:      return "RESET";
    case CameraNodeSt::IDLE:       return "IDLE";
    case CameraNodeSt::RESET_DONE: return "RESET_DONE";
    case CameraNodeSt::RUNNING:    return "RUNNING";
    case CameraNodeSt::EXCEPTION:  return "EXCEPTION";
    default:                       return "UNKNOWN";
  }
}

// 每个相机独立的流状态（设备、管线、内参、对齐状态、CameraInfo 等）
struct CameraStreamState {
  std::shared_ptr<ob::Device> device{nullptr};
  std::shared_ptr<ob::Pipeline> pipeline{nullptr};
  std::string serial_number;      // 设备序列号
  std::string device_name;        // 设备名称
  uint8_t camera_id = 0;          // 0=头部相机，1=腰部相机，2+预留

  // 相机内参/畸变（从 StreamProfile 缓存）
  OBCameraIntrinsic color_intrinsic_{};
  OBCameraDistortion color_distortion_{};
  OBCameraIntrinsic depth_intrinsic_{};
  OBCameraDistortion depth_distortion_{};
  OBCameraIntrinsic aligned_depth_intrinsic_{};  // D2C 对齐后的深度内参

  // 预构建的 CameraInfo 消息（避免每帧重建）
  sensor_msgs::msg::CameraInfo rgb_camera_info_;
  sensor_msgs::msg::CameraInfo depth_camera_info_;
  sensor_msgs::msg::CameraInfo aligned_depth_camera_info_;

  // SW D2C 对齐状态
  OBExtrinsic depth_to_color_extrinsic_{};
  bool depth_to_color_extrinsic_valid_ = false;
  bool sw_align_d2c_ = false;
  std::vector<uint16_t> aligned_depth_buffer_;  // 对齐深度缓冲区（彩色分辨率）

  // RGB 点云生成器（SDK PointCloudFilter）。
  // HW D2C 对齐路径下 pipeline 输出对齐帧集，process() 生成位于彩色光学坐标系的 RGB 点云。
  std::shared_ptr<ob::PointCloudFilter> pc_filter_{nullptr};

  // 主机时间同步状态：是否已完成首次 timerSyncWithHost() 同步
  bool time_synced_ = false;
};

class CameraDriverNode : public rclcpp::Node {
 public:
  using HardwareCheck = com_interfaces::action::HardwareCheck;
  using GoalHandleHardwareCheck = rclcpp_action::ServerGoalHandle<HardwareCheck>;
  using HardwareReset = com_interfaces::action::HardwareReset;
  using GoalHandleHardwareReset = rclcpp_action::ServerGoalHandle<HardwareReset>;

  using MotorStatus = com_interfaces::msg::MotorStatus;
  using SyncedRobotVision = com_interfaces::msg::SyncedRobotVision;

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

  // --- Heartbeat & Status ---
  void heartbeat_cb();
  void change_status(CameraNodeSt new_st, int32_t err);
  void publish_status(CameraNodeSt st, int32_t err);

  bool acquireDeviceLock();
  void releaseDeviceLock();
  void cleanupDeviceLock();
  bool openDevice();
  bool startStreams();
  // 为单个相机启动完整数据流（D2C 对齐 + 帧回调）
  bool startStreamForCamera(size_t cam_index);
  // 在 StreamProfileList 中按 (width, height, format, fps) 选择 video stream profile。
  // SDK 的 getVideoStreamProfile() 在无匹配时抛异常（而非返回 nullptr），例如请求
  // 1280x720@3fps 而设备不支持该帧率时会导致启动失败。该辅助函数对每个候选格式 +
  // 帧率做 try-catch 匹配，fps 逐级降级（精确 → 5 → 10 → 15 → 30 → ANY），
  // 全部失败返回 nullptr，由调用方 fallback 到 getProfile(0)。
  std::shared_ptr<ob::VideoStreamProfile> selectVideoStreamProfile(
      const std::shared_ptr<ob::StreamProfileList> &list,
      int width, int height, int fps,
      const std::vector<OBFormat> &format_candidates,
      const char *stream_name);
  void onFrameSet(size_t cam_index, std::shared_ptr<ob::FrameSet> frame_set);
  // 主机时间同步：将相机内部时钟同步到主机（系统）时间。
  // 首次在启动流前调用（enableGlobalTimestamp + timerSyncWithHost），
  // 之后由 time_sync_timer_ 每 time_sync_period_ 秒周期重同步以补偿设备时钟漂移。
  void syncHostTimeOnce(size_t cam_index);
  void periodicTimeSyncCb();
  void stop();

  // --- Image registration (D2C/C2D alignment) ---
  // Software alignment is implemented in-node (self-implemented pixel reprojection),
  // it does NOT depend on the SDK's ob::Align filter.
  void syncSoftwareAlignment(size_t cam_index,
                             std::shared_ptr<ob::VideoStreamProfile> depth_profile,
                             std::shared_ptr<ob::VideoStreamProfile> color_profile);
  void publishRawDepthImage(size_t cam_index, const std::shared_ptr<ob::Frame>& depth_frame);
  void softwareAlignD2C(size_t cam_index, const std::shared_ptr<ob::VideoFrame>& depth_video);
  // SW D2C 模式下，基于 softwareAlignD2C() 生成的对齐深度缓冲区（彩色分辨率、mm）+ 彩色图像
  // + 彩色内参，构建与彩图逐像素对齐的 RGB 点云（位于彩色光学坐标系）。
  // 参考 orbbec_camera 的 publishColoredPointCloud()：对齐后深度必须用彩色内参反投影。
  void buildAlignedPointCloud(size_t cam_index,
                              const std::shared_ptr<ob::VideoFrame>& color_video,
                              const rclcpp::Time& color_ts,
                              const std::string& color_fid,
                              const SyncedRobotVision::SharedPtr& rgbd_msg);

  // --- 头部位姿订阅与点云坐标变换（/Control/HeadPose，眼在手标定） ---
  // 订阅头部末端位姿（MotorStatus，取 end_effector_pose[0]）构建 M_end 4×4，
  // 与头部相机固定标定矩阵 kFixedTransform_0（相机→头部末端）相乘得到
  // 头部相机点云变换矩阵 T = M_end × kFixedTransform_0（行主序，P' = T × P）。
  // 头部相机需两次转换：相机→头部末端→基座。
  void head_pose_callback(const MotorStatus::SharedPtr msg);
  // 按 camera_id 选择对应相机的外参变换应用到点云（原地修改 xyz），并更新
  // frame_id 为基座标系：
  //   0 号（头部）相机：T = M_end × kFixedTransform_0（相机→头部→基座，两次转换），
  //     尚未收到头部位姿时保持原样；
  //   1 号（底盘）相机：T = kFixedTransform_1（相机→基座，一次转换，与头部位姿无关）。
  void applyPointCloudTransform(sensor_msgs::msg::PointCloud2 &pc_msg, uint8_t camera_id);
  // 工具函数：四元数位姿 -> 4×4 矩阵（行主序）；4×4 相乘（行主序）；齐次变换点
  static void poseToMat4(const geometry_msgs::msg::Pose &pose, float M[16]);
  static void mat4Mul(const float A[16], const float B[16], float out[16]);
  static void mat4TransformPoint(const float M[16], float x, float y, float z,
                                 float &ox, float &oy, float &oz);

  // --- Frame rate monitoring ---
  void checkFrameRate();

  // --- Time fusion (from robot_time_sync_node) ---
  // 由 onFrameSet 内部直接调用，替代原 time_sync 通过 topic 订阅的方式
  void handle_camera_data(uint8_t camera_id, const SyncedRobotVision::SharedPtr msg);
  void motor_callback(const MotorStatus::SharedPtr msg);
  void fusion_cb();
  CameraNodeSt get_current_status() const;

  static rclcpp::Time fromUsToROSTime(uint64_t us);
  static sensor_msgs::msg::CameraInfo convertToCameraInfo(
      const OBCameraIntrinsic &intrinsic,
      const OBCameraDistortion &distortion);

  // --- Members ---
  rclcpp::Logger logger_{rclcpp::get_logger(CAMERA_DRIVER_NODE_NAME)};

  // 融合已在本节点内部完成，直接发布 /fusion/synced_robot_vision）
  // 多相机时所有相机都通过该话题发布，用 camera_id 字段区分来源相机
  rclcpp::Publisher<SyncedRobotVision>::SharedPtr fusion_pub_{nullptr};

  // 原始 RGB 数据发布器（仅主相机 camera_id==0）：
  // 仅当检测到订阅者（get_subscription_count() > 0）时才发布
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_raw_pub_{nullptr};

  // Heartbeat & Status publishers
  rclcpp::Publisher<com_interfaces::msg::HeartBeat>::SharedPtr hb_pub_{nullptr};
  rclcpp::Publisher<com_interfaces::msg::NodeStatus>::SharedPtr status_pub_{nullptr};
  rclcpp::TimerBase::SharedPtr hb_timer_{nullptr};

  // Node status tracking
  mutable std::mutex status_mtx_;
  CameraNodeSt curr_st_{CameraNodeSt::INIT};
  int32_t err_code_{0};

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
  // device_/pipeline_ 为主相机（index 0）句柄，保持原有单相机逻辑兼容
  std::shared_ptr<ob::Device> device_{nullptr};
  std::shared_ptr<ob::Pipeline> pipeline_{nullptr};

  // --- 多相机支持：所有相机统一保留融合逻辑，通过 camera_id 区分 ---
  // camera_id 映射：由启动参数 camera_id_sn_map（下标=camera_id，值=SN）指定，
  // 例如 camera_id_sn_map=[头部SN, 腰部SN] => camera_id 0=头部，1=腰部，2+预留。
  // 未配置映射时按设备枚举顺序分配 camera_id（0,1,2,...）。
  std::vector<CameraStreamState> cameras_;

  // Cross-process device lock
  int shm_fd_ = -1;
  uint8_t *shm_addr_ = nullptr;
  pthread_mutex_t *device_lock_ = nullptr;
  bool lock_acquired_ = false;
  bool bypass_lock_ = false;

  // Frame rate monitoring
  std::chrono::steady_clock::time_point fps_last_check_{};
  uint32_t fps_frame_count_ = 0;
  double actual_color_fps_ = 0.0;
  double actual_depth_fps_ = 0.0;
  bool has_received_first_frame_ = false;
  rclcpp::TimerBase::SharedPtr fps_timer_{nullptr};

  // Parameters
  std::string serial_number_;
  std::vector<std::string> camera_id_sn_map_;  // 下标=camera_id，值=对应SN（多相机映射）
  int color_width_ = 1280;
  int color_height_ = 720;
  int color_fps_ = 30;
  int depth_width_ = 1280;
  int depth_height_ = 720;
  int depth_fps_ = 30;
  std::string frame_id_{"camera_link"};
  std::string color_frame_id_{"camera_color_optical_frame"};
  std::string depth_frame_id_{"camera_depth_optical_frame"};
  bool enable_align_ = true;

  // --- 主机时间同步（相机内部时钟同步到系统时间） ---
  rclcpp::TimerBase::SharedPtr time_sync_timer_{nullptr};
  bool enable_host_time_sync_ = true;  // 总开关（默认开启）
  double time_sync_period_ = 60.0;     // 周期重同步间隔（秒），默认 1 分钟

  // --- Image registration state (self-implemented, no SDK Align filter) ---
  // depth_registration_:   whether any alignment (D2C/C2D) is active
  // align_mode_:           "OFF" | "HW" (pipeline hardware D2C) | "SW" (in-node reprojection)
  // align_target_stream_:  OB_STREAM_COLOR (D2C) or OB_STREAM_DEPTH (C2D)
  // sw_align_d2c_:         true when in-node software D2C (depth -> color) is active
  bool depth_registration_ = false;
  std::string align_mode_{"HW"};
  std::string align_target_stream_str_{"COLOR"};
  OBStreamType align_target_stream_{OB_STREAM_COLOR};
  // 点云输出格式："rgb_point"（默认，RGB 点云，每点 xyz(float32)+rgb(uint32)）| "point"（仅坐标）
  std::string pointcloud_format_{"rgb_point"};
  // 是否保留点云所有像素：true 时无效深度点用 NaN 表示（width=图像分辨率），false 时过滤无效点
  bool pc_keep_invalid_points_ = true;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_unaligned_pub_{nullptr};

  // --- 头部位姿与点云坐标变换状态 ---
  rclcpp::Subscription<MotorStatus>::SharedPtr sub_head_pose_{nullptr};
  std::mutex head_pose_mtx_;              // 保护 head_pose_transform_ / head_pose_available_
  bool head_pose_available_ = false;      // 是否已收到头部位姿并构建变换矩阵（头部相机）
  std::array<float, 16> head_pose_transform_{};  // 头部相机 T = M_end × kFixedTransform_0（行主序）
  // 固定标定矩阵 M_fixed（行主序，相机→头部末端，手眼标定结果） //141主机的
  // 0 号相机（头部相机）外参：相机 → 头部末端（还需再乘头部末端在基座系下的实时位姿 M_end）
  const std::array<float, 16> kFixedTransform_0{
    0.01571449f, -0.30814851f,  0.9512087f,  0.10762206f,
    0.00216534f, -0.95131259f, -0.3082185f,  0.12872836f,
    0.99987466f,  0.00690311f, -0.01428272f, -0.01844746f,
    0.0f,         0.0f,         0.0f,         1.0f
  };

  // 1 号相机（底盘相机）外参：相机 → 基座标系（直接一次旋转到基座，与头部位姿无关）
  const std::array<float, 16> kFixedTransform_1{
    0.0f,  0.0f,  1.0f,  0.0f,
    -1.0f, 0.0f,  0.0f,  0.0f,
    0.0f, -1.0f,  0.0f,  0.0f,
    0.0f,  0.0f,  0.0f,  1.0f
  };

  // 固定标定矩阵 M_fixed（行主序，相机→头部末端，手眼标定结果） //136主机的
  // const std::array<float, 16> kFixedTransform_{
  //   -1.4207706586713147e-02, -3.0468173843290935e-01, 9.5234824478183744e-01,  1.0791357510450654e-01,
  //   5.5389667691170047e-03, -9.5245374778143488e-01, -3.0463285801802209e-01,  1.2887697895527106e-01,
  //   9.9988372370024903e-01, 9.4689101708163163e-04, 1.5219805410230502e-02,    -2.7674147492936205e-02,
  //   0.0f,         0.0f,          0.0f,         1.0f
  // };
  // 点云变换后的坐标系 frame_id（机器人基坐标系）
  std::string pointcloud_base_frame_id_{"base_link"};

  // --- Time fusion (from robot_time_sync_node) ---
  rclcpp::Subscription<MotorStatus>::SharedPtr sub_motor_{nullptr};
  rclcpp::TimerBase::SharedPtr fusion_timer_{nullptr};  // 延迟融合定时器

  std::deque<MotorStatus> motor_cache_;
  std::mutex motor_mtx_;

  // 每相机最新相机数据（按 camera_id 索引）
  std::vector<SyncedRobotVision::SharedPtr> latest_cam_data_by_cam_;
  std::mutex cam_mtx_;

  bool motor_available_ = false;  // 是否已收到过运控数据
};

#endif  // CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_
