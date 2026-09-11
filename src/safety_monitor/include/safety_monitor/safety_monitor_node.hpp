#ifndef SAFETY_MONITOR_NODE_HPP
#define SAFETY_MONITOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <com_interfaces/msg/node_status.hpp>
#include <com_interfaces/msg/heart_beat.hpp>
#include <com_interfaces/srv/get_system_status.hpp>
#include <std_msgs/msg/header.hpp>

#include <unordered_map>
#include <chrono>
#include <string>
#include <cstdint>
#include <functional>
#include <climits>

namespace safety_monitor
{

// 整机FSM状态枚举（扩展自检SELF_CHECK）
enum class SystemFsmState : int8_t
{
  UNKNOWN     = 0,    // 上电初始未知
  INIT        = 1,    // 初始化，等待全节点上报
  SELF_CHECK  = 2,    // 自检状态
  RESET       = 3,    // 复位状态
  RESET_DONE  = 4,    // 复位完成状态
  IDLE        = 5,    // 空闲状态
  RUN         = 6,    // 运行状态
  ABNORMAL    = 7,    // 任意节点异常
  EMERGENCY   = 8     // 任意节点急停，最高优先级
};

// 状态转字符串工具
inline std::string fsm_state_to_str(SystemFsmState s)
{
  switch(s)
  {
    case SystemFsmState::UNKNOWN:      return "UNKNOWN";
    case SystemFsmState::INIT:          return "INIT";
    case SystemFsmState::SELF_CHECK:    return "SELF_CHECK";
    case SystemFsmState::RESET:         return "RESET";
    case SystemFsmState::RESET_DONE:    return "RESET_DONE";
    case SystemFsmState::IDLE:          return "IDLE";
    case SystemFsmState::RUN:           return "RUN";
    case SystemFsmState::ABNORMAL:      return "ABNORMAL";
    case SystemFsmState::EMERGENCY:     return "EMERGENCY";
    default: return "INVALID_STATE";
  }
}

// 状态机类
class SafetyStateMachine
{
public:
  using StateChangeCb = std::function<void(SystemFsmState old_st, SystemFsmState new_st)>;

  explicit SafetyStateMachine(rclcpp::Logger logger);

  void set_state_change_callback(StateChangeCb cb);
  SystemFsmState get_current_state() const;

  /**
   * @brief FSM核心驱动更新函数
   * @param all_node_reported 是否所有节点都上报过NodeStatus
   * @param any_node_emergency 是否存在任意节点急停
   * @param any_node_abnormal 是否存在任意节点异常
   * @param task_mgr_reset_ok 任务管理节点(node_id=1)是否复位完成
   */
  void update_fsm(
    bool all_node_reported,
    bool any_node_emergency,
    bool any_node_abnormal,
    bool task_mgr_reset_ok);

private:
  void transition_to(SystemFsmState new_state);

  SystemFsmState current_state_;
  StateChangeCb state_change_callback_;
  rclcpp::Logger logger_;
};

// 单节点运行时信息结构体
struct NodeRuntimeInfo
{
  bool online{false};
  bool status_reported{false};         // 是否上报过/status话题
  int32_t node_status{0};              // 节点自身状态值
  int64_t error_flag{0};               // 错误标志位（可自定义位定义：异常/急停等位）
  rclcpp::Time last_heartbeat_time;
  uint32_t heartbeat_lost_cnt{0};
  std::string node_process_name;
  std::string command;
};


struct NodeInfo
{
    std::string name;
    std::string command;
};

class SafetyMonitorNode : public rclcpp::Node
{
public:
  explicit SafetyMonitorNode(const rclcpp::NodeOptions & options);
  ~SafetyMonitorNode() override = default;

private:
  void init_monitored_node_map();

  // 话题回调
  void node_status_callback(const com_interfaces::msg::NodeStatus::SharedPtr msg);
  void heartbeat_callback(const com_interfaces::msg::HeartBeat::SharedPtr msg);

  // 1Hz定时巡检
  void heartbeat_check_timer_callback();

  // 系统状态查询服务
  void get_system_status_srv_callback(
    const std::shared_ptr<com_interfaces::srv::GetSystemStatus::Request> req,
    std::shared_ptr<com_interfaces::srv::GetSystemStatus::Response> res);

  // 工具函数
  bool is_ros_node_running(const std::string & node_name);
  void restart_fault_node(int32_t node_id);

  // 汇总全局判定条件，供给FSM决策
  void collect_system_condition(
    bool & all_node_reported,
    bool & any_emergency,
    bool & any_abnormal,
    bool & task_manager_reset_done);

  // 状态跳转进入动作回调
  void on_global_state_changed(SystemFsmState old_st, SystemFsmState new_st);

  // 配置常量
  const uint32_t HEARTBEAT_LOST_THRESHOLD_{3};
  const std::chrono::milliseconds TIMER_PERIOD_{1000};
  const int32_t TASK_MGR_NODE_ID = 1;

  std::unordered_map<int32_t, NodeRuntimeInfo> node_info_map_;
  size_t total_monitor_node_cnt_{0};

  // 通信接口
  rclcpp::Subscription<com_interfaces::msg::NodeStatus>::SharedPtr sub_node_status_;
  rclcpp::Subscription<com_interfaces::msg::HeartBeat>::SharedPtr sub_heartbeat_;
  rclcpp::Service<com_interfaces::srv::GetSystemStatus>::SharedPtr srv_get_system_status_;
  rclcpp::TimerBase::SharedPtr heartbeat_check_timer_;

  SafetyStateMachine fsm_;
  int64_t global_total_error_flag_{0};

  bool all_node_online_; //所有节点成功连接
  bool error_;           //有无节点反馈异常
  bool emergency_;       //有无节点反馈急停
  bool reset_done_;      //是否复位完成
};

} // namespace safety_monitor

#endif // SAFETY_MONITOR_NODE_HPP