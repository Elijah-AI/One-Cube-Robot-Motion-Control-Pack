#include "rclcpp/rclcpp.hpp"
#include <deque>
#include <mutex>
#include <climits>

#include "robot_time_sync_node/msg/motor_info.hpp"
#include "robot_time_sync_node/msg/camera_raw_data.hpp"
#include "robot_time_sync_node/msg/synced_robot_vision.hpp"
#include "robot_time_sync_node/msg/node_status.hpp"
#include "robot_time_sync_node/msg/heart_beat.hpp"

using namespace robot_time_sync_node::msg;
using namespace std::chrono_literals;

//固定参数
constexpr int32_t NODE_ID = 8;
constexpr size_t MAX_MOTOR_CACHE = 10;
constexpr std::chrono::milliseconds CAM_DELAY_AFTER = 2ms;
constexpr int64_t MAX_OFFSET_NS = 10'000'000; //10ms最大同步误差
constexpr std::chrono::milliseconds HB_PERIOD = 1000ms;

//节点状态枚举
enum NodeSt
{
  ST_UNKNOWN     = 0,
  ST_INIT        = 1,
  ST_CHECK       = 2,
  ST_RESET       = 3,
  ST_IDLE        = 5,
  ST_RESET_DONE  = 6,
  ST_RUNNING     = 7,
  ST_EXCEPTION   = 8
};

class TimeSyncFusion : public rclcpp::Node
{
public:
  TimeSyncFusion() : Node("time_sync_fusion"), curr_st_(ST_INIT), err_code_(0)
  {
    //发布器
    pub_fusion_ = create_publisher<SyncedRobotVision>("/fusion/synced_robot_vision", rclcpp::QoS(10));
    pub_status_ = create_publisher<NodeStatus>("/SafeNode/status", rclcpp::QoS(10));
    pub_hb_     = create_publisher<HeartBeat>("/SafeNode/Heartbeat", rclcpp::QoS(10));

    //订阅器
    sub_motor_ = create_subscription<MotorInfo>(
      "/Control/MotorInfo", rclcpp::QoS(20),
      std::bind(&TimeSyncFusion::motor_callback, this, std::placeholders::_1));

    sub_camera_ = create_subscription<CameraRawData>(
      "/vision/camera_data", rclcpp::QoS(20),
      std::bind(&TimeSyncFusion::camera_callback, this, std::placeholders::_1));

    //1秒心跳定时器
    hb_timer_ = create_wall_timer(HB_PERIOD, std::bind(&TimeSyncFusion::heartbeat_cb, this));

    change_status(ST_IDLE, 0);
    RCLCPP_INFO(get_logger(), "[time_sync_fusion] node startup complete, node_id:8");
  }

private:
  //变更节点状态并发布状态报文
  void change_status(int32_t new_st, int32_t err)
  {
    curr_st_ = new_st;
    err_code_ = err;
    NodeStatus st_msg;
    st_msg.header.stamp = now();
    st_msg.node_id = NODE_ID;
    st_msg.status = curr_st_;
    st_msg.error_code = err_code_;
    pub_status_->publish(st_msg);
  }

  //心跳回调1Hz
  void heartbeat_cb()
  {
    HeartBeat hb_msg;
    hb_msg.header.stamp = now();
    hb_msg.node_id = NODE_ID;
    pub_hb_->publish(hb_msg);
    RCLCPP_INFO(get_logger(), "Pub Heartbeat");
  }

  //运控回调：数据入缓存，上限10帧
  void motor_callback(const MotorInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(motor_mtx_);
    motor_cache_.push_back(*msg);
    while (motor_cache_.size() > MAX_MOTOR_CACHE)
    {
      motor_cache_.pop_front();
    }
    //收到首个运控帧从空闲切运行
    if (curr_st_ == ST_IDLE)
    {
      change_status(ST_RUNNING, 0);
    }
  }

  //相机回调：收到相机延时1ms再匹配
  void camera_callback(const CameraRawData::SharedPtr msg)
  {
    if (!msg || curr_st_ != ST_RUNNING)
      return;

    CameraRawData cam_data = *msg;
    auto delay_timer = create_wall_timer(CAM_DELAY_AFTER,
      [this, cam_data]() {
        std::lock_guard<std::mutex> lock(motor_mtx_);
        if (motor_cache_.empty())
        {
          RCLCPP_WARN(get_logger(), "[Sync] motor cache empty, discard camera frame");
          return;
        }

        rclcpp::Time cam_ts(cam_data.header.stamp);
        int64_t min_diff_ns = LLONG_MAX;
        const MotorInfo* best_motor = nullptr;

        //遍历找最近时间运控帧
        for (const auto& mot : motor_cache_)
        {
          int64_t diff = std::abs((cam_ts - rclcpp::Time(mot.header.stamp)).nanoseconds());
          if (diff < min_diff_ns)
          {
            min_diff_ns = diff;
            best_motor = &mot;
          }
        }

        //超过10ms丢弃
        if (!best_motor || min_diff_ns > MAX_OFFSET_NS)
        {
          double diff_ms = static_cast<double>(min_diff_ns) / 1e6;
          RCLCPP_WARN(get_logger(), "[Sync] sync diff:%.3fms >10ms, skip fusion", diff_ms);
          return;
        }

        //拼装融合消息
        SyncedRobotVision fusion_out;
        fusion_out.header = cam_data.header;

        fusion_out.camera_rgbd_color = cam_data.camera_rgbd_color;
        fusion_out.camera_rgbd_depth = cam_data.camera_rgbd_depth;
        fusion_out.camera_info = cam_data.camera_info;
        fusion_out.target_label = cam_data.target_label;
        fusion_out.seg_confidence = cam_data.seg_confidence;
        fusion_out.data_valid = cam_data.data_valid;

        fusion_out.joint_names = best_motor->joint_names;
        fusion_out.joint_positions = best_motor->joint_positions;
        fusion_out.joint_velocities = best_motor->joint_velocities;
        fusion_out.joint_efforts = best_motor->joint_efforts;
        fusion_out.end_effector_pose = best_motor->end_effector_pose;

        pub_fusion_->publish(fusion_out);
        double diff_ms = static_cast<double>(min_diff_ns) / 1e6;
        RCLCPP_DEBUG(get_logger(), "[Sync] publish fusion ok, diff:%.3fms", diff_ms);
      });
    //延长智能指针生命周期，定时器不提前析构
    (void)delay_timer;
  }

  //发布订阅
  rclcpp::Publisher<SyncedRobotVision>::SharedPtr pub_fusion_;
  rclcpp::Publisher<NodeStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<HeartBeat>::SharedPtr pub_hb_;

  rclcpp::Subscription<MotorInfo>::SharedPtr sub_motor_;
  rclcpp::Subscription<CameraRawData>::SharedPtr sub_camera_;

  rclcpp::TimerBase::SharedPtr hb_timer_;

  std::deque<MotorInfo> motor_cache_;
  std::mutex motor_mtx_;

  int32_t curr_st_;
  int32_t err_code_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeSyncFusion>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}