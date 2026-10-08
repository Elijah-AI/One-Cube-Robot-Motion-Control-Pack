/*******************************************************************************
 * camera_driver_node.hpp
 *
 * berxel_camera_driver : ROS2 driver node for Berxel Hawk (iHawk) depth cameras,
 * functionally aligned with camera_driver (Orbbec based). 除底层获取图片原始
 * 数据来源不同（Berxel Hawk SDK 替代 Orbbec SDK）之外，其余功能完全保持一致：
 *   * 发布话题：/fusion/synced_robot_vision、/vision/rgb_raw(仅主相机)、
 *     /SafeNode/Heartbeat、/SafeNode/status
 *   * 订阅话题：/Control/MotorInfo、/Control/HeadPose
 *   * Action 服务：/vision/Check_self(HardwareCheck)、/vision/Reset(HardwareReset)
 *   * 状态机、时间同步融合、多相机支持、主机时间同步、头部位姿点云坐标变换等
 *******************************************************************************/
#ifndef BERXEL_CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_
#define BERXEL_CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <deque>
#include <queue>
#include <chrono>
#include <thread>
#include <future>
#include <atomic>
#include <mutex>
#include <condition_variable>
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

// Berxel Hawk SDK
#include <BerxelHawkContext.h>
#include <BerxelHawkDefines.h>
#include <BerxelHawkDevice.h>
#include <BerxelHawkFrame.h>

namespace berxel_camera_driver
{

// 与 camera_driver 保持一致的节点/锁/节点ID 常量。
// 设备锁名使用 berxel 专属（Berxel 与 Orbbec 是物理上独立的不同设备，
// 避免跨进程与 orbbec 锁相互误伤）。
static constexpr const char * CAMERA_DRIVER_NODE_NAME = "camera_driver_node";
static constexpr const char * BERXEL_DEVICE_LOCK_NAME = "berxel_device_lock";
static constexpr int32_t CAMERA_DRIVER_NODE_ID = 4;

// 相机驱动节点状态枚举（与 camera_driver 一致）
enum class CameraNodeSt : int32_t
{
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
inline const char * node_state_to_string(CameraNodeSt st)
{
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

class CameraDriverNode;  // forward decl（CameraStreamState 持有其指针）

// 每个相机独立的流状态（设备、内参、对齐状态、CameraInfo、帧桥等）。
//
// 注意：Berxel SDK 没有 pipeline/frameset/PointCloudFilter，彩色与深度帧通过
// 各自的流回调（startStreams 的 onNewFrameCallback）独立到达，因此本结构内置
// “有界队列 + 本相机处理线程”的帧桥。SDK 回调线程把帧压入队列（帧所有权在
// releaseFrame 前有效），本相机处理线程取出配对的彩色/深度帧后统一处理。
struct CameraStreamState
{
  // 设备句柄（由 context_->openDevice(deviceInfo) 打开，closeDevice 关闭）
  berxel::BerxelHawkDevice * device{nullptr};
  std::string serial_number;      // 设备序列号
  std::string device_name;        // 设备名称（设备地址/类型）
  uint8_t camera_id = 0;          // 0=头部相机，1=腰部相机，2+预留

  // 相机内参/畸变（BerxelHawkCameraIntrinsic：fx/fy/cx/cy + 5 畸变系数 k1,k2,p1,p2,k3）
  berxel::BerxelHawkCameraIntrinsic color_intrinsic_{};
  berxel::BerxelHawkCameraIntrinsic depth_intrinsic_{};
  berxel::BerxelHawkCameraIntrinsic aligned_depth_intrinsic_{};  // registration 对齐后深度内参(=彩色内参)

  // 预构建的 CameraInfo 消息（避免每帧重建）
  sensor_msgs::msg::CameraInfo rgb_camera_info_;
  sensor_msgs::msg::CameraInfo depth_camera_info_;
  sensor_msgs::msg::CameraInfo aligned_depth_camera_info_;

  // 活动分辨率（applyFrameMode 成功后记录；registration 打开时深度分辨率=彩色分辨率）
  int32_t color_width_ = 0;
  int32_t color_height_ = 0;
  int32_t depth_width_ = 0;
  int32_t depth_height_ = 0;

  // 反投影查找表（快速点云生成；按当前活动内参与分辨率缓存）
  std::vector<float> fxcx_table_;
  std::vector<float> fycy_table_;
  uint32_t table_w_ = 0;
  uint32_t table_h_ = 0;
  float table_fx_ = -1.0f;
  float table_fy_ = -1.0f;
  float table_cx_ = -1.0f;
  float table_cy_ = -1.0f;

  // ---- Berxel 帧桥（SDK 回调线程 -> 有界队列 -> 本相机处理线程）----
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::queue<berxel::BerxelHawkFrame *> color_queue_;
  std::queue<berxel::BerxelHawkFrame *> depth_queue_;
  std::thread process_thread_;
  std::atomic<bool> process_running_{false};

  // SDK 回调 userData：每个相机 startStreams 使用独立 userData 以便区分回调
  // 来自哪个相机。node 指针 + 相机下标（相机打开期间地址保持稳定）。
  struct CallbackCtx
  {
    CameraDriverNode * node{nullptr};
    size_t cam_index{0};
  } callback_ctx;

  // registration 打开时深度已对齐到彩色；若实际彩色/深度分辨率不一致，仅告警一次
  bool color_cloud_mismatch_warned_ = false;

  // 主机时间同步状态：是否已完成首次 setSystemClock() 同步
  bool time_synced_ = false;
};

class CameraDriverNode : public rclcpp::Node
{
public:
  using HardwareCheck = com_interfaces::action::HardwareCheck;
  using GoalHandleHardwareCheck = rclcpp_action::ServerGoalHandle<HardwareCheck>;
  using HardwareReset = com_interfaces::action::HardwareReset;
  using GoalHandleHardwareReset = rclcpp_action::ServerGoalHandle<HardwareReset>;

  using MotorStatus = com_interfaces::msg::MotorStatus;
  using SyncedRobotVision = com_interfaces::msg::SyncedRobotVision;

  explicit CameraDriverNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node(CAMERA_DRIVER_NODE_NAME, node_options)
  {
    init();
  }

  CameraDriverNode(
    const std::string & node_name, const std::string & ns,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node(node_name, ns, node_options)
  {
    init();
  }

  ~CameraDriverNode() override
  {
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
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const HardwareCheck::Goal> goal);

  rclcpp_action::CancelResponse handle_hw_check_cancel(
    const std::shared_ptr<GoalHandleHardwareCheck> goal_handle);

  void handle_hw_check_accepted(
    const std::shared_ptr<GoalHandleHardwareCheck> goal_handle);

  void execute_hw_check(const std::shared_ptr<GoalHandleHardwareCheck> goal_handle);

  // --- HardwareReset Action Server (复位) ---
  rclcpp_action::GoalResponse handle_hw_reset_goal(
    const rclcpp_action::GoalUUID & uuid,
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
  // 为单个相机启动完整数据流（registration 对齐 + 帧回调桥）
  bool startStreamForCamera(size_t cam_index);
  // 在当前设备的支持帧模式列表中按 (w,h) 精确选择帧模式并设置到设备
  // （需要时覆盖帧率；找不到精确(w,h)时回退当前帧模式并覆盖分辨率/帧率）。
  // 对应 camera_driver 的 selectVideoStreamProfile()。
  bool applyFrameMode(
    size_t cam_index, berxel::BerxelHawkStreamType stream_type,
    int32_t want_w, int32_t want_h, int32_t want_fps);

  // --- Berxel 帧桥：SDK 回调 -> 有界队列 -> 本相机处理线程 ---
  static void onNewFrameCallback(
    berxel::BerxelHawkStreamType stream_type, berxel::BerxelHawkFrame * p_frame,
    void * p_user_data);
  void pushFrame(
    size_t cam_index, berxel::BerxelHawkStreamType stream_type,
    berxel::BerxelHawkFrame * p_frame);
  void processLoop(size_t cam_index);
  // 本相机处理线程取到一对彩色/深度帧后构建 SyncedRobotVision 并提交融合；
  // 对应 camera_driver 的 onFrameSet()（构建 RGB 图 + RGB 点云 + CameraInfo）。
  void handleFramePair(
    size_t cam_index, berxel::BerxelHawkFrame * p_color,
    berxel::BerxelHawkFrame * p_depth);

  // --- 相机配置/内参 ---
  void readIntrinsics(size_t cam_index);
  void updateIntrinsicsForResolution(size_t cam_index);
  void buildCameraInfoMessages(size_t cam_index);

  // 主机时间同步：将相机内部时钟同步到主机（系统）时间。
  // 首次在启动流前调用 setSystemClock()，之后由 time_sync_timer_ 每
  // time_sync_period_ 秒周期重新 setSystemClock() 以补偿设备时钟漂移
  // （对应 camera_driver 的 enableGlobalTimestamp + timerSyncWithHost）。
  void syncHostTimeOnce(size_t cam_index);
  void periodicTimeSyncCb();
  void stop();

  // --- RGB 点云（手动反投影；字段布局与 camera_driver 完全一致）---
  // 预计算像素->射线查找表（fxcx/fycy），缓存键=(w,h,fx,fy,cx,cy)
  static void ensureXyTables(CameraStreamState & st, uint32_t width, uint32_t height);
  static float depthToMeters(uint16_t raw, berxel::BerxelHawkPixelType pixel_type);
  // 基于深度帧（registration 开启时已与彩色逐像素对齐）+ 彩色帧 + 彩色内参，
  // 生成 camera_driver 同款字段布局（x,y,z FLOAT32 + rgb FLOAT32 打包 uint32
  // (R<<16|G<<8|B)，point_step=16）的 RGB 点云并填入 rgbd_msg。
  void buildPointCloud(
    size_t cam_index, berxel::BerxelHawkFrame * p_depth,
    berxel::BerxelHawkFrame * p_color, const std::string & cloud_fid,
    const SyncedRobotVision::SharedPtr & rgbd_msg);

  // --- 头部位姿订阅与点云坐标变换（/Control/HeadPose，眼在手标定） ---
  // 订阅头部末端位姿（MotorStatus，取 end_effector_pose[0]）构建 M_end 4×4，
  // 与固定标定矩阵 kFixedTransform_（相机→头部末端）相乘得到点云变换矩阵
  // T = M_end × M_fixed，将相机系点云变换到机器人基坐标系（P' = T × P）。
  void head_pose_callback(const MotorStatus::SharedPtr msg);
  // 应用已保存的变换矩阵 T 到点云（原地修改 xyz），并更新 frame_id 为基坐标系；
  // 若尚未收到头部位姿则保持原样。
  void applyPointCloudTransform(sensor_msgs::msg::PointCloud2 & pc_msg);
  // 工具函数：四元数位姿 -> 4×4 矩阵（行主序）；4×4 相乘（行主序）；齐次变换点
  static void poseToMat4(const geometry_msgs::msg::Pose & pose, float M[16]);
  static void mat4Mul(const float A[16], const float B[16], float out[16]);
  static void mat4TransformPoint(
    const float M[16], float x, float y, float z,
    float & ox, float & oy, float & oz);

  // --- Frame rate monitoring ---
  void checkFrameRate();

  // --- Time fusion (from robot_time_sync_node) ---
  // 由 handleFramePair 内部直接调用，替代原 time_sync 通过 topic 订阅的方式
  void handle_camera_data(uint8_t camera_id, const SyncedRobotVision::SharedPtr msg);
  void motor_callback(const MotorStatus::SharedPtr msg);
  void fusion_cb();
  CameraNodeSt get_current_status() const;

  static rclcpp::Time fromUsToROSTime(uint64_t us);
  static sensor_msgs::msg::CameraInfo convertToCameraInfo(
    const berxel::BerxelHawkCameraIntrinsic & intrinsic);

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

  // Berxel SDK
  berxel::BerxelHawkContext * context_{nullptr};
  // device_ 为主相机（index 0）句柄，保持原有单相机逻辑兼容
  berxel::BerxelHawkDevice * device_{nullptr};
  int32_t stream_flags_{0};  // COLOR | DEPTH 位掩码，传给 startStreams

  // --- 多相机支持：所有相机统一保留融合逻辑，通过 camera_id 区分 ---
  // camera_id 映射：由启动参数 camera_id_sn_map（下标=camera_id，值=SN）指定，
  // 例如 camera_id_sn_map=[头部SN, 腰部SN] => camera_id 0=头部，1=腰部，2+预留。
  // 未配置映射时按设备枚举顺序分配 camera_id（0,1,2,...）。
  // 注意：CameraStreamState 含 std::mutex/cv/thread/atomic，不可移动/拷贝，
  // 只能用 std::deque（emplace_back 原地构造，不会搬移已有元素）。
  std::deque<CameraStreamState> cameras_;

  // Cross-process device lock
  int shm_fd_ = -1;
  uint8_t * shm_addr_ = nullptr;
  pthread_mutex_t * device_lock_ = nullptr;
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

  // --- Image registration state ---
  // depth_registration_:   whether any alignment (D2C) is active
  // align_mode_:           "OFF" | "HW" | "SW"
  //   Berxel SDK 只提供深度到彩色的硬件/设备内对齐（setRegistrationEnable），
  //   因此 HW 与 SW 均映射为 setRegistrationEnable(true)（SW 在设备内完成等效
  //   的深度→彩色对齐），OFF 映射为 setRegistrationEnable(false)。
  // align_target_stream_str_: "COLOR"（默认；Berxel 仅支持 D2C，若请求 "DEPTH" 告警并回退 COLOR）
  bool depth_registration_ = false;
  std::string align_mode_{"HW"};
  std::string align_target_stream_str_{"COLOR"};
  // 点云输出格式："rgb_point"（默认，RGB 点云，每点 xyz(float32)+rgb(uint32)）| "point"（仅坐标）
  std::string pointcloud_format_{"rgb_point"};
  // 是否保留点云所有像素：true 时无效深度点用 NaN 表示（width=图像分辨率），false 时过滤无效点
  bool pc_keep_invalid_points_ = true;

  // --- 头部位姿与点云坐标变换状态 ---
  rclcpp::Subscription<MotorStatus>::SharedPtr sub_head_pose_{nullptr};
  std::mutex head_pose_mtx_;              // 保护 head_pose_transform_ / head_pose_available_
  bool head_pose_available_ = false;      // 是否已收到头部位姿并构建变换矩阵
  std::array<float, 16> head_pose_transform_{};  // T = M_end × M_fixed（行主序）

  // 固定标定矩阵 M_fixed（行主序，相机→头部末端，手眼标定结果） //136主机的
  // const std::array<float, 16> kFixedTransform_{
  //   -1.4207706586713147e-02, -3.0468173843290935e-01, 9.5234824478183744e-01,  1.0791357510450654e-01,
  //   5.5389667691170047e-03, -9.5245374778143488e-01, -3.0463285801802209e-01,  1.2887697895527106e-01,
  //   9.9988372370024903e-01, 9.4689101708163163e-04, 1.5219805410230502e-02,    -2.7674147492936205e-02,
  //   0.0f,         0.0f,          0.0f,         1.0f
  // };

  const std::array<float, 16> kFixedTransform_{
    -6.0959534262902187e-03, -3.1532727029043589e-01, 9.4896340918025257e-01,  1.0326640201986360e-01,
    -7.5426986988484370e-03, -9.4893954620563736e-01, -3.1536779376368951e-01, 1.3104302245482674e-01,
    9.9995297241828418e-01, -9.0802124546141508e-03, 3.4062726867034676e-03,    -2.0292211299454920e-02,
    0.0f,         0.0f,          0.0f,         1.0f
  };

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

  // 关闭流程标记：置位后 SDK 回调直接丢弃并释放帧，不再触碰相机状态
  std::atomic<bool> shutting_down_{false};
};

}  // namespace berxel_camera_driver

#endif  // BERXEL_CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_
