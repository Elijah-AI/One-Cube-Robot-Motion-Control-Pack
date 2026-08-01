#include "rclcpp/rclcpp.hpp"
#include <deque>
#include <mutex>
#include <climits>
#include <memory>
#include <string>

#include "com_interfaces/msg/motor_info.hpp"
#include "com_interfaces/msg/camera_driver.hpp"
#include "com_interfaces/msg/synced_robot_vision.hpp"
#include "com_interfaces/msg/node_status.hpp"
#include "com_interfaces/msg/heart_beat.hpp"

namespace robot_time_sync_node
{

using MotorInfo = com_interfaces::msg::MotorInfo;
using CameraDriver = com_interfaces::msg::CameraDriver;
using SyncedRobotVision = com_interfaces::msg::SyncedRobotVision;
using NodeStatus = com_interfaces::msg::NodeStatus;
using HeartBeat = com_interfaces::msg::HeartBeat;

using namespace std::chrono_literals;

// ============================================================
// 固定参数
// ============================================================
constexpr int32_t NODE_ID = 8;
constexpr size_t MAX_MOTOR_CACHE = 10;
constexpr auto CAM_DELAY_AFTER = 2ms;
constexpr int64_t MAX_OFFSET_NS = 10'000'000;   // 10ms 最大同步误差
constexpr auto HB_PERIOD = 1000ms;

// ============================================================
// 节点状态枚举 (enum class 提供类型安全)
// ============================================================
enum class NodeSt : int32_t
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

// ----------------------------------------------------------
// 将节点状态枚举转换为可读字符串
// ----------------------------------------------------------
const char* node_state_to_string(NodeSt st)
{
  switch (st)
  {
    case NodeSt::UNKNOWN:    return "UNKNOWN";
    case NodeSt::INIT:       return "INIT";
    case NodeSt::CHECK:      return "CHECK";
    case NodeSt::RESET:      return "RESET";
    case NodeSt::IDLE:       return "IDLE";
    case NodeSt::RESET_DONE: return "RESET_DONE";
    case NodeSt::RUNNING:    return "RUNNING";
    case NodeSt::EXCEPTION:  return "EXCEPTION";
    default:                 return "UNKNOWN";
  }
}

// ============================================================
// TimeSyncFusion 节点
// ============================================================
class TimeSyncFusion : public rclcpp::Node
{
public:
  TimeSyncFusion() : Node("time_sync_fusion"), curr_st_(NodeSt::INIT), err_code_(0)
  {
    RCLCPP_INFO(get_logger(),
      "[TimeSync] ======== Node Initializing (node_id:%d) ========", NODE_ID);

    // ---- 发布器 ----
    pub_fusion_ = create_publisher<SyncedRobotVision>("/fusion/synced_robot_vision", rclcpp::QoS(10));
    pub_status_ = create_publisher<NodeStatus>("/SafeNode/status", rclcpp::QoS(10));
    pub_hb_     = create_publisher<HeartBeat>("/SafeNode/Heartbeat", rclcpp::QoS(10));
    RCLCPP_INFO(get_logger(),
      "[TimeSync] publishers: /fusion/synced_robot_vision, /SafeNode/status, /SafeNode/Heartbeat");

    // ---- 订阅器 ----
    sub_motor_ = create_subscription<MotorInfo>(
      "/Control/MotorInfo", rclcpp::QoS(20),
      std::bind(&TimeSyncFusion::motor_callback, this, std::placeholders::_1));

    auto cam_qos = rclcpp::QoS(20).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    sub_camera_ = create_subscription<CameraDriver>(
      "/vision/camera_data", cam_qos,
      std::bind(&TimeSyncFusion::camera_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(),
      "[TimeSync] subscriptions: /Control/MotorInfo (MotorInfo), /vision/camera_data (CameraDriver)");

    // ---- 定时器 ----
    hb_timer_ = create_wall_timer(HB_PERIOD, std::bind(&TimeSyncFusion::heartbeat_cb, this));
    RCLCPP_INFO(get_logger(),
      "[TimeSync] heartbeat timer: %ldms period", HB_PERIOD.count());

    // 延迟融合定时器：只创建一次，由 camera_callback 触发
    fusion_timer_ = create_wall_timer(CAM_DELAY_AFTER, std::bind(&TimeSyncFusion::fusion_cb, this));
    fusion_timer_->cancel();  // 初始为停止状态
    RCLCPP_INFO(get_logger(),
      "[TimeSync] fusion delay timer: %ldms, max sync offset: %ldms",
      CAM_DELAY_AFTER.count(), MAX_OFFSET_NS / 1000000);

    change_status(NodeSt::IDLE, 0);
    RCLCPP_INFO(get_logger(),
      "[TimeSync] ======== Node Startup Complete (node_id:%d) ========", NODE_ID);
  }

private:
  // ----------------------------------------------------------
  // 变更节点状态并发布状态报文
  // ----------------------------------------------------------
  void change_status(NodeSt new_st, int32_t err)
  {
    {
      std::lock_guard<std::mutex> lock(status_mtx_);
      curr_st_ = new_st;
      err_code_ = err;
    }
    NodeStatus st_msg;
    st_msg.header.stamp = now();
    st_msg.node_id = NODE_ID;
    st_msg.status = static_cast<int32_t>(new_st);
    st_msg.error_flag = err;
    pub_status_->publish(st_msg);

    // 标准日志输出状态变更
    if (err != 0)
    {
      RCLCPP_WARN(get_logger(),
        "[TimeSync] status changed -> %s (node:%d, err:%d)",
        node_state_to_string(new_st), NODE_ID, err);
    }
    else
    {
      RCLCPP_INFO(get_logger(),
        "[TimeSync] status changed -> %s (node:%d)",
        node_state_to_string(new_st), NODE_ID);
    }
  }

  // ----------------------------------------------------------
  // 获取当前状态（线程安全）
  // ----------------------------------------------------------
  NodeSt get_current_status() const
  {
    std::lock_guard<std::mutex> lock(status_mtx_);
    return curr_st_;
  }

  // ----------------------------------------------------------
  // 心跳回调 1Hz
  // ----------------------------------------------------------
  void heartbeat_cb()
  {
    HeartBeat hb_msg;
    hb_msg.header.stamp = now();
    hb_msg.node_id = NODE_ID;
    pub_hb_->publish(hb_msg);

    // 每 30 次心跳（约 30 秒）打印一次 INFO 日志，避免刷屏
    static int hb_count = 0;
    hb_count++;
    if (hb_count % 30 == 1)
    {
      RCLCPP_INFO(get_logger(),
        "[TimeSync] heartbeat #%d, node:%d, state:%s, motor:%s",
        hb_count, NODE_ID,
        node_state_to_string(get_current_status()),
        motor_available_ ? "OK" : "MISSING");
    }
    else
    {
      RCLCPP_DEBUG(get_logger(), "[TimeSync] heartbeat #%d", hb_count);
    }
  }

  // ----------------------------------------------------------
  // 运控回调：数据入缓存，按时间戳排序
  // ----------------------------------------------------------
  void motor_callback(const MotorInfo::SharedPtr msg)
  {
    if (!msg) return;

    {
      std::lock_guard<std::mutex> lock(motor_mtx_);
      motor_cache_.push_back(*msg);
      if (motor_cache_.size() > MAX_MOTOR_CACHE)
      {
        motor_cache_.pop_front();
      }
    }

    // 标记已收到运控数据
    if (!motor_available_)
    {
      motor_available_ = true;
      RCLCPP_INFO(get_logger(),
        "[TimeSync] first motor frame received, motor sync enabled");
    }

    // 运控帧也可触发 IDLE→RUNNING（向后兼容）
    if (get_current_status() == NodeSt::IDLE)
    {
      RCLCPP_INFO(get_logger(),
        "[TimeSync] motor frame received, transition IDLE -> RUNNING");
      change_status(NodeSt::RUNNING, 0);
    }

    RCLCPP_DEBUG(get_logger(),
      "[TimeSync] motor frame: cache_size=%zu", motor_cache_.size());
  }

  // ----------------------------------------------------------
  // 相机回调：保存数据并触发延迟融合定时器（相机优先模式）
  // ----------------------------------------------------------
  void camera_callback(const CameraDriver::SharedPtr msg)
  {

    if (!msg)
    {
      RCLCPP_WARN(get_logger(), "[TimeSync] received null camera message");
      return;
    }

    // ● 相机优先模式：首次收到相机帧亦可触发 IDLE→RUNNING
    //   确保即使没有运控数据，视觉数据也能被处理和转发
    if (get_current_status() == NodeSt::IDLE)
    {
      RCLCPP_INFO(get_logger(),
        "[TimeSync] first camera frame received, transition IDLE -> RUNNING (camera-first mode)");
      change_status(NodeSt::RUNNING, 0);
    }

    if (get_current_status() != NodeSt::RUNNING)
    {
      RCLCPP_DEBUG(get_logger(),
        "[TimeSync] camera frame ignored, not in RUNNING state (current:%s)",
        node_state_to_string(get_current_status()));
      return;
    }

    // 保存最新的相机数据
    {
      std::lock_guard<std::mutex> lock(cam_mtx_);
      latest_cam_data_ = *msg;
    }

    // 重置定时器：取消后重新激活，确保每次相机数据都重新计时
    fusion_timer_->cancel();
    fusion_timer_->reset();

    RCLCPP_DEBUG(get_logger(),
      "[TimeSync] camera frame received, stamp:%d.%09d, fusion timer reset",
      msg->header.stamp.sec, msg->header.stamp.nanosec);
  }

  // ----------------------------------------------------------
  // 延迟融合回调（由 fusion_timer_ 触发）
  //   相机优先：运控数据缺失或同步超时仍发布纯视觉帧
  // ----------------------------------------------------------
  void fusion_cb()
  {
    // 停止定时器，确保只执行一次
    fusion_timer_->cancel();

    if (get_current_status() != NodeSt::RUNNING)
    {
      RCLCPP_DEBUG(get_logger(),
        "[TimeSync] fusion skipped, not RUNNING (state:%s)",
        node_state_to_string(get_current_status()));
      return;
    }

    // 取出相机数据
    CameraDriver cam_data;
    {
      std::lock_guard<std::mutex> lock(cam_mtx_);
      cam_data = latest_cam_data_;
    }

    // 拼装融合消息（视觉数据始终填充）
    SyncedRobotVision fusion_out;
    fusion_out.header = cam_data.header;
    fusion_out.camera_rgbd_color = cam_data.rgb;
    fusion_out.camera_rgbd_depth = cam_data.depth;
    fusion_out.camera_info = cam_data.rgb_camera_info;
    fusion_out.target_label = cam_data.target_label;
    fusion_out.seg_confidence = cam_data.seg_confidence;
    fusion_out.data_valid = cam_data.data_valid;

    // 尝试匹配运控帧（尽力而为，不做硬性要求）
    {
      std::lock_guard<std::mutex> lock(motor_mtx_);

      if (!motor_cache_.empty())
      {
        const rclcpp::Time cam_ts(cam_data.header.stamp);
        int64_t min_diff_ns = LLONG_MAX;
        const MotorInfo* best_motor = nullptr;

        for (const auto& mot : motor_cache_)
        {
          int64_t diff = std::abs((cam_ts - rclcpp::Time(mot.header.stamp)).nanoseconds());
          if (diff < min_diff_ns)
          {
            min_diff_ns = diff;
            best_motor = &mot;
          }
        }

        if (best_motor && min_diff_ns <= MAX_OFFSET_NS)
        {
          // ● 时间同步成功：填充运控数据
          fusion_out.joint_names = best_motor->joint_names;
          fusion_out.joint_positions = best_motor->joint_positions;
          fusion_out.joint_velocities = best_motor->joint_velocities;
          fusion_out.joint_efforts = best_motor->joint_efforts;
          fusion_out.end_effector_pose = best_motor->end_effector_pose;

          double diff_ms = static_cast<double>(min_diff_ns) / 1e6;
          RCLCPP_DEBUG(get_logger(),
            "[TimeSync] fusion: publish (motor synced), diff:%.3fms, label:%s",
            diff_ms, cam_data.target_label.c_str());
        }
        else
        {
          // ● 时间同步超阈值：发布纯视觉帧（运控字段留空），不丢弃
          double diff_ms = static_cast<double>(min_diff_ns) / 1e6;
          RCLCPP_WARN(get_logger(),
            "[TimeSync] fusion: sync diff %.3fms > %.1fms, publish camera-only (cam_ts:%d.%09d, cache:%zu)",
            diff_ms, MAX_OFFSET_NS / 1e6,
            cam_data.header.stamp.sec, cam_data.header.stamp.nanosec,
            motor_cache_.size());
        }
      }
      else
      {
        // ● 无运控数据：发布纯视觉帧（运控字段留空）
        //   使用 WARN 级别以便运维人员能直接发现运控数据缺失
        RCLCPP_WARN(get_logger(),
          "[TimeSync] fusion: publish camera-only (motor unavailable), label:%s, cam_ts:%d.%09d",
          cam_data.target_label.c_str(),
          cam_data.header.stamp.sec, cam_data.header.stamp.nanosec);
      }
    }

    // ● 始终发布，保证视觉数据不丢失
    pub_fusion_->publish(fusion_out);
  }

  // ---- 发布订阅 ----
  rclcpp::Publisher<SyncedRobotVision>::SharedPtr pub_fusion_;
  rclcpp::Publisher<NodeStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<HeartBeat>::SharedPtr pub_hb_;

  rclcpp::Subscription<MotorInfo>::SharedPtr sub_motor_;
  rclcpp::Subscription<CameraDriver>::SharedPtr sub_camera_;

  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp::TimerBase::SharedPtr fusion_timer_;   // 单例延迟融合定时器

  // ---- 数据缓存 ----
  std::deque<MotorInfo> motor_cache_;
  std::mutex motor_mtx_;

  CameraDriver latest_cam_data_;
  std::mutex cam_mtx_;

  // ---- 运控可用性标志 ----
  bool motor_available_ = false;   // 是否已收到过运控数据

  // ---- 状态 ----
  mutable std::mutex status_mtx_;
  NodeSt curr_st_;
  int32_t err_code_;
};

}  // namespace robot_time_sync_node

// ============================================================
// 主函数
// ============================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_time_sync_node::TimeSyncFusion>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  RCLCPP_INFO(rclcpp::get_logger("time_sync_fusion"), "[TimeSync] Node shutdown complete.");
  return 0;
}
