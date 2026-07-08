#include "safety_monitor/safety_monitor_node.hpp"
#include <iostream>
#include <cstdio>
#include <memory>
#include <sstream>

namespace safety_monitor
{

//==================== FSM 状态机实现 ====================
SafetyStateMachine::SafetyStateMachine(rclcpp::Logger logger)
: logger_(logger), current_state_(SystemFsmState::UNKNOWN)
{
  // 需求1：上电默认未知状态
  RCLCPP_INFO(logger_, "[FSM POWER-ON] Initial state: UNKNOWN");
}

void SafetyStateMachine::set_state_change_callback(StateChangeCb cb)
{
  state_change_callback_ = cb;
}

SystemFsmState SafetyStateMachine::get_current_state() const
{
  return current_state_;
}

void SafetyStateMachine::transition_to(SystemFsmState new_state)
{
  if (new_state == current_state_)
    return;

  SystemFsmState old = current_state_;
  current_state_ = new_state;
  RCLCPP_INFO(logger_, "[FSM TRANSITION] %s --> %s",
    fsm_state_to_str(old).c_str(), fsm_state_to_str(new_state).c_str());

  if (state_change_callback_)
    state_change_callback_(old, new_state);
}

void SafetyStateMachine::update_fsm(
  bool all_node_reported,
  bool any_node_emergency,
  bool any_node_abnormal,
  bool task_mgr_reset_ok)
{
  // 规则6：任意节点急停，最高优先级直接进入急停
  if (any_node_emergency)
  {
    transition_to(SystemFsmState::EMERGENCY);
    return;
  }

  // 规则5：任意节点异常，切入异常状态
  if (any_node_abnormal)
  {
    transition_to(SystemFsmState::ABNORMAL);
    return;
  }

  switch (current_state_)
  {
    case SystemFsmState::UNKNOWN:
      // UNKNOWN 自动切入 INIT（初始化通信接口、等待节点上报）
      transition_to(SystemFsmState::INIT);
      break;

    case SystemFsmState::INIT:
      // 规则3：全部节点上报完成 → 自检状态
      if (all_node_reported)
      {
        transition_to(SystemFsmState::SELF_CHECK);
      }
      break;

    case SystemFsmState::SELF_CHECK:
      // 规则4：自检态收到任务管理节点复位完成 → 空闲
      if (task_mgr_reset_ok)
      {
        transition_to(SystemFsmState::IDLE);
      }
      break;

    case SystemFsmState::IDLE:
      // 无异常无急停，保持空闲
      break;

    case SystemFsmState::ABNORMAL:
    case SystemFsmState::EMERGENCY:
    default:
      break;
  }
}

//==================== 安全监控节点主体 ====================
SafetyMonitorNode::SafetyMonitorNode(const rclcpp::NodeOptions & options)
: Node("safety_monitor_node", options),
  fsm_(this->get_logger()),
  all_node_online_(false),
  error_(false),
  emergency_(false),
  reset_done_(false)
{
  RCLCPP_INFO(this->get_logger(), "==== Safety Monitor Node New FSM Logic Startup ====");

  // 注册所有需要监控的节点ID
  init_monitored_node_map();
  total_monitor_node_cnt_ = node_info_map_.size();
  RCLCPP_INFO(this->get_logger(), "Total monitored nodes: %zu", total_monitor_node_cnt_);

  // 绑定状态跳转回调
  fsm_.set_state_change_callback(
    std::bind(&SafetyMonitorNode::on_global_state_changed, this,
      std::placeholders::_1, std::placeholders::_2));

  // 【需求2】初始化所有通信接口
  sub_node_status_ = this->create_subscription<com_interfaces::msg::NodeStatus>(
    "/SafeNode/status",
    rclcpp::QoS(10),
    std::bind(&SafetyMonitorNode::node_status_callback, this, std::placeholders::_1));

  sub_heartbeat_ = this->create_subscription<com_interfaces::msg::Heartbeat>(
    "/SafeNode/Heartbeat",
    rclcpp::QoS(10),
    std::bind(&SafetyMonitorNode::heartbeat_callback, this, std::placeholders::_1));

  srv_get_system_status_ = this->create_service<com_interfaces::srv::GetSystemStatus>(
    "/Safety/get_system_status",
    std::bind(&SafetyMonitorNode::get_system_status_srv_callback,
      this, std::placeholders::_1, std::placeholders::_2));

  heartbeat_check_timer_ = this->create_wall_timer(
    TIMER_PERIOD_,
    std::bind(&SafetyMonitorNode::heartbeat_check_timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Communication interfaces initialized, wait state transition...");
}

void SafetyMonitorNode::init_monitored_node_map()
{
  std::unordered_map<int32_t, NodeInfo> node_name_mapping = {
    {1,  {"task_manager_node", "ros2 launch task_manager task_manager.launch.py&"}},
  };

  for (auto & pair : node_name_mapping)
  {
    NodeRuntimeInfo info;
    info.node_process_name = pair.second.name;
    info.command = pair.second.command;
    info.last_heartbeat_time = this->now();
    node_info_map_[pair.first] = info;
    RCLCPP_INFO(this->get_logger(), "Register node_id:%d | proc_name:%s",
      pair.first, pair.second.name.c_str());
  }
}

void SafetyMonitorNode::node_status_callback(const com_interfaces::msg::NodeStatus::SharedPtr msg)
{
  int32_t nid = msg->node_id;

  if (node_info_map_.find(nid) == node_info_map_.end())
  {
    RCLCPP_WARN(this->get_logger(), "Recv status unknown node_id: %d", nid);
    return;
  }

  auto & info = node_info_map_[nid];
  info.node_status = msg->status;
  info.error_flag = msg->error_flag;
  info.status_reported = true;

  RCLCPP_DEBUG(this->get_logger(),
    "node_id[%d] report status=%d, error_flag=%ld",
    nid, msg->status, msg->error_flag);
}

void SafetyMonitorNode::heartbeat_callback(const com_interfaces::msg::Heartbeat::SharedPtr msg)
{
  int32_t nid = msg->node_id;
  if (node_info_map_.find(nid) == node_info_map_.end())
  {
    RCLCPP_WARN(this->get_logger(), "Recv heartbeat unknown node_id: %d", nid);
    return;
  }
  //RCLCPP_INFO(this->get_logger(), "Recv heartbeat node_id: %d", nid);


  //RCLCPP_INFO(this->get_logger(), "node_info_map_[%d] online:%s, heartbeat_lost_cnt:%ld, ", nid, node_info_map_[nid].online?"true":"false", node_info_map_[nid].heartbeat_lost_cnt);
  auto & info = node_info_map_[nid];
  info.online = true;
  info.heartbeat_lost_cnt = 0;
  info.last_heartbeat_time = this->now();


  bool all_reported, any_emergency, any_abnormal, task_reset_ok;
  int64_t total_err;
  collect_system_condition(all_node_online_, emergency_, error_, reset_done_);
  // RCLCPP_INFO(this->get_logger(), "all_node_online_:%s, emergency_:%s, error_: %s, reset_done_: %s", 
  // all_node_online_?"true":"false", 
  // emergency_?"true":"false", 
  // error_?"true":"false", 
  // reset_done_?"true":"false");
  fsm_.update_fsm(all_node_online_, emergency_, error_, reset_done_);

//   bool flag = true;
//   for (auto & item : node_info_map_)
//   {
//     if(item.second.online == false)
//     {
//         flag = false;
//     }
//   }
//   if (flag)
//   {
//     all_node_online_ = true;
//   }
//   else 
//   {
//     all_node_online_ = false;
//   }

  
  //RCLCPP_INFO(this->get_logger(), "node_info_map_[%d] online:%s, heartbeat_lost_cnt:%ld, ", nid, node_info_map_[nid].online?"true":"false", node_info_map_[nid].heartbeat_lost_cnt);
}

void SafetyMonitorNode::heartbeat_check_timer_callback()
{
  // 心跳超时检测 + 节点重启逻辑保留
  for (auto & entry : node_info_map_)
  {
    auto & info = entry.second;
    if (!info.online) continue;

    info.heartbeat_lost_cnt++;
    
    if (info.heartbeat_lost_cnt >= HEARTBEAT_LOST_THRESHOLD_)
    {
      RCLCPP_ERROR(this->get_logger(),
        "node_id[%d] heartbeat timeout(%d/%d)",
        entry.first, info.heartbeat_lost_cnt, HEARTBEAT_LOST_THRESHOLD_);

      bool alive = is_ros_node_running(info.node_process_name);
      if (!alive)
      {
        RCLCPP_ERROR(this->get_logger(), "node_id[%d] dead, trigger restart", entry.first);
        restart_fault_node(entry.first);
      }
      info.heartbeat_lost_cnt = 0;
      info.online = false;
    }
  }

  // 汇总条件驱动FSM刷新
  bool all_reported, any_emergency, any_abnormal, task_reset_ok;
  int64_t total_err;
  collect_system_condition(all_node_online_, emergency_, error_, reset_done_);
  // RCLCPP_INFO(this->get_logger(), "all_node_online_:%s, emergency_:%s, error_: %s, reset_done_: %s", 
  // all_node_online_?"true":"false", 
  // emergency_?"true":"false", 
  // error_?"true":"false", 
  // reset_done_?"true":"false");
  fsm_.update_fsm(all_node_online_, emergency_, error_, reset_done_);
}

void SafetyMonitorNode::collect_system_condition(
  bool & all_node_reported,
  bool & any_emergency,
  bool & any_abnormal,
  bool & task_manager_reset_done)
{
  all_node_reported = true;
  any_emergency = false;
  any_abnormal = false;
  task_manager_reset_done = false;
  size_t reported_cnt = 0;

  for (auto & entry : node_info_map_)
  {
    int32_t nid = entry.first;
    auto & info = entry.second;

    // 判断是否全部节点上报状态
    if (!info.online)
      all_node_reported = false;
    else
      reported_cnt++;

    // ========== 位定义约定（你可自行修改掩码）==========
    // error_flag bit0: 异常标志
    // error_flag bit1: 急停标志
    if (info.error_flag == 0 && info.online)
    {
        //上线了并且没有异常
        
    }
    else
    {
        if (info.online)
        {
            if (info.error_flag  & (1LL << 0) == 1 )
            {
                any_emergency = true;
            }
            else if(info.error_flag != 0)
            {
                any_abnormal = true;
            }
        }
        else
        {
            
        }
    }

    
  }
  
  if (!all_node_reported)
  {
    RCLCPP_DEBUG(this->get_logger(), "Init waiting: reported %zu/%zu nodes",
      reported_cnt, total_monitor_node_cnt_);

  }

  if (node_info_map_[1].online)
  {
    if(node_info_map_[1].node_status == static_cast<int32_t>(SystemFsmState::UNKNOWN) || 
          node_info_map_[1].node_status == static_cast<int32_t>(SystemFsmState::INIT) || 
          node_info_map_[1].node_status == static_cast<int32_t>(SystemFsmState::SELF_CHECK) || 
          node_info_map_[1].node_status == static_cast<int32_t>(SystemFsmState::RESET))
    {
        task_manager_reset_done = false;
    }
    else
    {
        task_manager_reset_done = true;
    }
  }
}

void SafetyMonitorNode::on_global_state_changed(SystemFsmState old_st, SystemFsmState new_st)
{
  // 各状态进入动作，可扩展业务逻辑
  if (old_st == new_st)
  {
    return;
  }

  switch (new_st)
  {
    case SystemFsmState::UNKNOWN:
      RCLCPP_INFO(this->get_logger(), "System in initial unknown, prepare enter init");
      break;

    case SystemFsmState::INIT:
      RCLCPP_INFO(this->get_logger(), "System INIT: all communication ready, waiting all node status report");
      break;

    case SystemFsmState::SELF_CHECK:
      RCLCPP_INFO(this->get_logger(), "All nodes reported done, enter SELF_CHECK state");
      break;

    case SystemFsmState::IDLE:
      RCLCPP_INFO(this->get_logger(), "Task manager reset complete, system enter IDLE ready for mission");
      break;

    case SystemFsmState::ABNORMAL:
      RCLCPP_WARN(this->get_logger(), "System ABNORMAL: node fault or offline detected");
      break;

    case SystemFsmState::EMERGENCY:
      RCLCPP_FATAL(this->get_logger(), "SYSTEM EMERGENCY STOP TRIGGERED");
      // 可扩展：下发急停指令、继电器、上报远端平台
      break;

    default:
      RCLCPP_WARN(this->get_logger(), "Undefined system state");
      break;
  }
}

bool SafetyMonitorNode::is_ros_node_running(const std::string & node_name)
{
  std::string cmd = "ros2 node list 2>/dev/null | grep -q \"/" + node_name + "\"";
  int ret = std::system(cmd.c_str());
  return (ret == 0);
}

void SafetyMonitorNode::restart_fault_node(int32_t node_id)
{
  auto & info = node_info_map_[node_id];
  std::string proc = info.node_process_name;

  std::stringstream kill_cmd;
  kill_cmd << "pkill -f " << proc << " 2>/dev/null";
  std::system(kill_cmd.str().c_str());
  //std::system(info.command.c_str());
  // 替换为你实际 ros2 run / launch 启动指令
  RCLCPP_INFO(this->get_logger(), "Restart trigger node_id:%d, process:%s", node_id, proc.c_str());
}

void SafetyMonitorNode::get_system_status_srv_callback(
  const std::shared_ptr<com_interfaces::srv::GetSystemStatus::Request> req,
  std::shared_ptr<com_interfaces::srv::GetSystemStatus::Response> res)
{
  SystemFsmState curr = fsm_.get_current_state();
  res->status = static_cast<int8_t>(curr);
  res->error_flag = global_total_error_flag_;

  RCLCPP_DEBUG(this->get_logger(),
    "ReqID:%s | FSM State:%s, ErrFlag:%ld",
    req->request_id.c_str(), fsm_state_to_str(curr).c_str(), res->error_flag);
}

} // namespace safety_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<safety_monitor::SafetyMonitorNode>(opts);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}