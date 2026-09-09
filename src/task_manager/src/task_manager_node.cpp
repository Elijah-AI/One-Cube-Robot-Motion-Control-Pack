// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Author: gene.wang
// Date: 2026
//
// Description:
// 任务管理器 ROS2 节点实现。
// 系统核心节点，负责初始化配置、启动 Modbus 从站、
// 管理有限状态机（FSM）、调度行为树执行任务、
// 发布心跳和状态信息，以及处理 Modbus 寄存器触发任务。
// 订阅 /Safety/device_safety_state 获取设备安全状态，响应急停/停止/恢复指令。

#include "task_manager/task_manager_node.hpp"
#include "task_manager/log_helper.hpp"
#include <sstream>
#include <uuid/uuid.h>

namespace task_mgr
{
TaskManagerNode::TaskManagerNode()
: Node("task_manager_node"), current_state_(task_fsm::SystemState::UNKNOWN), error_flag_(0)
{
  hb_pub_ = this->create_publisher<HeartbeatMsg>("/SafeNode/Heartbeat",
    rclcpp::QoS(1).best_effort().durability_volatile());
  state_pub_ = this->create_publisher<NodeStatusMsg>("/SafeNode/status", 10);
  sys_state_client_ = this->create_client<GetSystemStatusSrv>("/Safety/get_system_status");

  // ==================== 订阅设备安全状态 ====================
  // 话题: /Safety/device_safety_state，消息类型: com_interfaces/msg/SafetyState
  // QoS 与 /SafeNode/status 状态话题保持一致（RELIABLE | TRANSIENT_LOCAL | KEEP_LAST / 1），
  // 采用 /SafeNode/Heartbeat 的链式 QoS 写法
  safety_state_sub_ = this->create_subscription<SafetyStateMsg>(
    "/Safety/device_safety_state",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&TaskManagerNode::safety_state_callback, this, std::placeholders::_1));

  LOG_INFO(this->get_logger(), task_log::Tag::SYSTEM,
    "Subscribed to /Safety/device_safety_state (SafetyState)");
}

TaskManagerNode::~TaskManagerNode()
{
  if(modbus_slave_) modbus_slave_->stop();
}

void TaskManagerNode::run()
{
  LOG_DEBUG(this->get_logger(), task_log::Tag::SYSTEM, "Task manager node starting...");
  action_client_ = std::make_unique<task_act::TaskActionClient>(shared_from_this());
  bt_engine_ = std::make_unique<BehaviorTreeEngine>(this->get_logger());
  precondition_mgr_.init(shared_from_this());
  // 注入前置条件管理器供前置条件门评估；条件判定失败/中止时置 ERR_PRECONDITION_FAIL 错误位
  bt_engine_->setPreconditionManager(&precondition_mgr_);
  bt_engine_->setPreconditionFailCb([this]()
  {
    set_error_flag(task_fsm::ERR_PRECONDITION_FAIL);
  });

  if(!init_config())
  {
    set_error_flag(task_fsm::ERR_CONFIG_PARSE_FAIL);
    switch_state(task_fsm::SystemState::ERROR);
    return;
  }
  switch_state(task_fsm::SystemState::INIT);
  
  if(!init_modbus())
  {
    set_error_flag(task_fsm::ERR_MODBUS_FAULT);
    switch_state(task_fsm::SystemState::ERROR);
    return;
  }

  // 所有初始化完成后，再创建定时器，避免 context 未就绪时触发回调
  hb_timer_ = this->create_wall_timer(std::chrono::milliseconds(HEARTBEAT_INTERVAL_MS),
    std::bind(&TaskManagerNode::heartbeat_timer_cb, this));
  bt_tick_timer_ = this->create_wall_timer(std::chrono::milliseconds(BT_TICK_INTERVAL_MS),
    std::bind(&TaskManagerNode::bt_tick_cb, this));

  //boot_check_system_state();
  rclcpp::spin(shared_from_this());
}

bool TaskManagerNode::init_config()
{
  task_cfg::TaskConfigParser parser;
  std::string cfg_path = this->declare_parameter<std::string>("config_path", "./config/task_config.yaml");
  LOG_WARN(this->get_logger(), task_log::Tag::CONFIG, "Config path: " << cfg_path);
  return parser.load_config(cfg_path, cfg_);
}

bool TaskManagerNode::init_modbus()
{
  modbus_slave_ = std::make_unique<task_modbus::ModbusTcpSlave>(cfg_.modbus_listen_ip, cfg_.modbus_port, this->get_logger());
  modbus_slave_->register_write_callback(std::bind(&TaskManagerNode::on_modbus_reg_write, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  modbus_slave_->start();
  LOG_INFO(this->get_logger(), task_log::Tag::MODBUS, "Modbus slave initialized on " << cfg_.modbus_listen_ip << ":" << cfg_.modbus_port);
  return true;
}

void TaskManagerNode::boot_check_system_state()
{
  // 避免重复请求
  if (boot_state_req_pending_)
  {
    return;
  }

  auto req = std::make_shared<GetSystemStatusSrv::Request>();
  LOG_INFO(this->get_logger(), task_log::Tag::FSM, "Waiting for safety server...");
  if(!sys_state_client_->wait_for_service(std::chrono::milliseconds(SERVICE_TIMEOUT_MS)))
  {
    LOG_WARN(this->get_logger(), task_log::Tag::FSM, "System state service not online, will retry later");
    return;
  }

  boot_state_req_pending_ = true;
  req->header.stamp = this->now();
  req->request_id = gen_request_id();
  auto fut = sys_state_client_->async_send_request(req,
  std::bind(&TaskManagerNode::request_result_callback, this, std::placeholders::_1));
}

void TaskManagerNode::request_result_callback(rclcpp::Client<GetSystemStatusSrv>::SharedFuture result_future)
{
  boot_state_req_pending_ = false;
  auto response = result_future.get();
  LOG_INFO_TITLE(this->get_logger(), task_log::Tag::FSM, "===== Service Response =====");
  LOG_INFO(this->get_logger(), task_log::Tag::FSM, "Global status: " << static_cast<int>(response->status));
  LOG_INFO(this->get_logger(), task_log::Tag::FSM, "Error flag: " << response->error_flag);

  // 如果安全节点返回 Init 状态，则进入自检流程
  if (response->status == static_cast<int8_t>(task_fsm::SystemState::INIT) ||
      response->status == static_cast<int8_t>(task_fsm::SystemState::SELF_CHECK) ||
      response->status == static_cast<int8_t>(task_fsm::SystemState::RESET))
  {
    LOG_INFO(this->get_logger(), task_log::Tag::FSM, "Safety node reports INIT state, starting self-check...");
    // 根据 YAML 配置决定是否执行 self_check
    if (!cfg_.boot_tasks.self_check_seq.empty())
    {
      run_boot_task(cfg_.boot_tasks.self_check_seq);
      switch_state(task_fsm::SystemState::SELF_CHECK);
    }
    else
    {
      LOG_WARN(this->get_logger(), task_log::Tag::FSM, "No self_check tasks configured in boot_task, skip to reset");
      // 没有自检任务，直接进入复位
      if (!cfg_.boot_tasks.reset_seq.empty())
      {
        boot_reset_system_state();
        switch_state(task_fsm::SystemState::RESET);
      }
      else
      {
        switch_state(task_fsm::SystemState::RESET_DONE);
      }
    }
  }
  else if (response->status == static_cast<int8_t>(task_fsm::SystemState::IDLE) || 
           response->status == static_cast<int8_t>(task_fsm::SystemState::ERROR))
  {
    LOG_INFO(this->get_logger(), task_log::Tag::FSM, "Safety node reports IDLE state, entering idle directly");
    switch_state(task_fsm::SystemState::IDLE);
  }
  else
  {
    LOG_INFO(this->get_logger(), task_log::Tag::FSM, "Safety node reports state: " << static_cast<int>(response->status) << ", will retry later");
  }
}


void TaskManagerNode::boot_reset_system_state()
{
  LOG_INFO(this->get_logger(), task_log::Tag::FSM, "Starting boot reset task...");
  if (!cfg_.boot_tasks.reset_seq.empty())
  {
    run_boot_task(cfg_.boot_tasks.reset_seq);
  }
  else
  {
    LOG_WARN(this->get_logger(), task_log::Tag::FSM, "No reset tasks configured in boot_task");
  }
}

// task_manager_node.cpp

BTNode::Ptr TaskManagerNode::create_action_node(const task_cfg::AtomicAction& action)
{
    std::string name = action.action_type; // 用于日志和调试
    return nullptr;
}



// run root task

void TaskManagerNode::run_boot_task(const std::vector<task_cfg::AtomicAction>& seq)
{
  action_client_->reset_all_states();
  bt_engine_->load_action_group(seq, *action_client_);
  bt_engine_->start();

}





void TaskManagerNode::heartbeat_timer_cb()
{
  HeartbeatMsg msg;
  msg.header.stamp = this->now();
  msg.node_id = cfg_.node_id; // nodeId → node_id
  hb_pub_->publish(msg);

  NodeStatusMsg status_msg;
  status_msg.header.stamp = this->now();
  status_msg.node_id = cfg_.node_id; // nodeId → node_id
  status_msg.status = static_cast<int32_t>(current_state_);
  state_pub_->publish(status_msg);
}

void TaskManagerNode::publish_state_change()
{
  NodeStatusMsg msg;
  msg.header.stamp = this->now();
  msg.node_id = cfg_.node_id;       // nodeId → node_id
  msg.status = static_cast<int32_t>(current_state_);
  msg.error_flag = error_flag_;     // errorFlag → error_flag
  state_pub_->publish(msg);
  LOG_INFO(this->get_logger(), task_log::Tag::FSM, "FSM switch to " << task_fsm::state_to_str(current_state_) << ", errFlag: " << error_flag_ << ", node_id: " << msg.node_id);
}


std::string TaskManagerNode::gen_request_id()
{
  std::stringstream ss;
  ss << this->get_name() << "_" << req_id_counter_++;
  return ss.str();
}

void TaskManagerNode::on_modbus_reg_write(uint32_t addr, uint16_t old_val, uint16_t new_val)
{

  LOG_INFO(this->get_logger(), task_log::Tag::MODBUS, "Modbus reg write: addr=" << addr << ", new_val=" << new_val << ", old_val=" << old_val);
  if(new_val == 1 && old_val == 0)
  {
    handle_trigger_task(addr);
  }
}

void TaskManagerNode::handle_trigger_task(uint32_t trigger_addr)
{
  // 紧急状态时拒绝常规任务触发（不缓存，紧急态恢复后应由操作员明确触发）
  if (current_state_ == task_fsm::SystemState::EMERGENCY)
  {
    LOG_WARN(this->get_logger(), task_log::Tag::FSM, "System in EMERGENCY mode, ignore normal task reg " << trigger_addr);
    return;
  }

  // 只在 IDLE 状态才能触发新任务，防止任务重叠导致状态混乱。
  // 系统忙时先将触发地址缓存到待执行队列，当前任务完成后自动执行（FIFO 顺序）。
  if(current_state_ != task_fsm::SystemState::IDLE)
  {
    if (current_state_ == task_fsm::SystemState::RUNNING)
    {
      std::lock_guard<std::mutex> lock(pending_trigger_mutex_);
      pending_trigger_regs_.push_back(trigger_addr);
    }

    LOG_WARN(this->get_logger(), task_log::Tag::FSM, "System busy (state="
      << task_fsm::state_to_str(current_state_) << "), cache pending task reg " << trigger_addr
      << ", pending=" << pending_trigger_regs_.size());
    return;
  }

  start_task_by_reg(trigger_addr);
}

void TaskManagerNode::start_task_by_reg(uint32_t trigger_addr)
{
  for(auto& [name, item] : cfg_.task_dict)
  {
    if(item.trigger_reg == trigger_addr)
    {
      LOG_INFO(this->get_logger(), task_log::Tag::FSM, "Trigger task: " << name << " (reg=" << trigger_addr << ")");
      switch_state(task_fsm::SystemState::RUNNING);
      // 重置所有状态并释放所有资源锁，确保新任务从干净状态开始
      action_client_->reset_all_states();
      if (bt_engine_->isTaskRunning())
      {
        bt_engine_->abort();
      }
      bt_engine_->load_parallel_task(item.action_seq, *action_client_);
      bt_engine_->start();
    }
  }
}

void TaskManagerNode::try_process_pending_trigger()
{
  uint32_t trigger_addr = 0;
  {
    std::lock_guard<std::mutex> lock(pending_trigger_mutex_);
    if (pending_trigger_regs_.empty())
    {
      return;
    }
    trigger_addr = pending_trigger_regs_.front();
    pending_trigger_regs_.pop_front();
  }

  // 仅在安全状态下才执行缓存的触发（防止紧急态下自动执行普通任务）；
  // 若状态不允许，重新放回队首等待下一次机会。
  if (current_state_ != task_fsm::SystemState::IDLE)
  {
    LOG_WARN(this->get_logger(), task_log::Tag::FSM,
      "Skip pending task reg " << trigger_addr
      << " (state=" << task_fsm::state_to_str(current_state_) << "), re-queue");
    {
      std::lock_guard<std::mutex> lock(pending_trigger_mutex_);
      pending_trigger_regs_.push_front(trigger_addr);
    }
    return;
  }

  LOG_INFO(this->get_logger(), task_log::Tag::FSM,
    "Execute pending trigger reg " << trigger_addr
    << ", remaining=" << pending_trigger_regs_.size());
  start_task_by_reg(trigger_addr);
}

void TaskManagerNode::write_task_status_reg(uint32_t status_addr, uint16_t result)
{
  modbus_slave_->set_reg(status_addr, result);
}

void TaskManagerNode::switch_state(task_fsm::SystemState new_state, uint64_t err_mask)
{
  if(err_mask != 0) set_error_flag(err_mask);
  if(current_state_ == new_state) return;
  current_state_ = new_state;
  publish_state_change();
}

void TaskManagerNode::clear_error_flag(uint64_t mask)
{
  error_flag_ &= (~mask);
}

void TaskManagerNode::set_error_flag(uint64_t mask)
{
  error_flag_ |= mask;
}

// ============================================================================
// 设备安全状态处理（替代原 AddTask Action Server）
// ============================================================================

void TaskManagerNode::safety_state_callback(const SafetyStateMsg::SharedPtr msg)
{
  if (!msg)
  {
    return;
  }

  // 转发到前置条件管理器（缓存机器人状态/安全状态/电池电量，供 preconditions 检测）
  precondition_mgr_.safety_state_cb(msg);

  const int8_t status = msg->status;

  // 状态码含义（com_interfaces/msg/SafetyState）:
  //   SAFETY_STATE_NORMAL    (0) -> 恢复正常，task_manager 切换为 IDLE
  //   SAFETY_STATE_HALT      (1) -> 功能受限/停止当前任务，task_manager 切换为 ERROR
  //   SAFETY_STATE_EMERGENCY (2) -> 急停，task_manager 立即停止任务并进入 EMERGENCY
  switch (status)
  {
    case SAFETY_STATE_EMERGENCY:
    {
      // 已在急停状态则跳过，避免周期消息重复处理
      if (current_state_ == task_fsm::SystemState::EMERGENCY)
      {
        break;
      }

      LOG_ERROR(this->get_logger(), task_log::Tag::SYSTEM,
        "Safety state = EMERGENCY (2), halting current modbus task immediately"
        << (msg->error_msg.empty() ? "" : ", error_msg=" + msg->error_msg));

      // 1. 立刻停止当前执行的 modbus 任务并发送取消指令
      halt_current_modbus_task();

      // 2. task_manager 进入急停状态
      switch_state(task_fsm::SystemState::EMERGENCY, task_fsm::ERR_EMERGENCY);
      break;
    }

    case SAFETY_STATE_HALT:
    {
      // 已在异常状态则跳过，避免周期消息重复处理
      if (current_state_ == task_fsm::SystemState::ERROR)
      {
        break;
      }

      LOG_ERROR(this->get_logger(), task_log::Tag::SYSTEM,
        "Safety state = HALT (1), halting current modbus task immediately"
        << (msg->error_msg.empty() ? "" : ", error_msg=" + msg->error_msg));

      // 1. 立刻停止当前执行的 modbus 任务并发送取消指令
      halt_current_modbus_task();

      // 2. task_manager 进入异常状态
      switch_state(task_fsm::SystemState::ERROR);
      break;
    }

    case SAFETY_STATE_NORMAL:
    default:
    {
      // 恢复正常：仅从 EMERGENCY/ERROR 状态切换为 IDLE（不影响正在运行的任务）
      if (current_state_ == task_fsm::SystemState::EMERGENCY ||
          current_state_ == task_fsm::SystemState::ERROR)
      {
        LOG_INFO(this->get_logger(), task_log::Tag::SYSTEM,
          "Safety state = NORMAL (0), switching task_manager to IDLE");
        // 清除全部错误标志（急停/故障等），恢复正常运行
        clear_error_flag(task_fsm::ERR_EMERGENCY | task_fsm::ERR_MODBUS_FAULT |
                         task_fsm::ERR_VISION_TIMEOUT | task_fsm::ERR_ARM_PLAN_FAIL |
                         task_fsm::ERR_GRIPPER_FAULT | task_fsm::ERR_CONFIG_PARSE_FAIL |
                         task_fsm::ERR_BT_EXEC_ERROR | task_fsm::ERR_SERVICE_TIMEOUT);
        switch_state(task_fsm::SystemState::IDLE);
      }
      break;
    }
  }
}

void TaskManagerNode::halt_current_modbus_task()
{
  // 1. 中止行为树，停止当前正在执行的 modbus 任务
  if (bt_engine_ && bt_engine_->isTaskRunning())
  {
    LOG_WARN(this->get_logger(), task_log::Tag::SYSTEM,
      "Halting current task: aborting behavior tree");
    bt_engine_->abort();
  }

  // 2. 取消所有远端 Action（机械臂/夹爪/吸盘/视觉等），并重置客户端状态
  if (action_client_)
  {
    action_client_->cancel_all_actions();
    action_client_->reset_all_states();
  }

  // 3. 复位 Modbus 触发/状态寄存器
  if (modbus_slave_)
  {
    // 触发寄存器清 0，防止重复触发
    modbus_slave_->set_reg(modbus_slave_->get_regaddr(), 0);
    // 任务状态寄存器置为 FAILED，通知 PLC 当前任务已被中止
    modbus_slave_->set_reg(modbus_slave_->get_regaddr() + MODBUS_STATUS_OFFSET, TASK_STATUS_FAILED);
  }
}

void TaskManagerNode::bt_tick_cb()
{
  
  switch (current_state_)
  {
  case task_fsm::SystemState::UNKNOWN:
    break;

  case task_fsm::SystemState::INIT:
      switch_state(task_fsm::SystemState::IDLE);
      //boot_check_system_state();
  break;
    

  case task_fsm::SystemState::SELF_CHECK:
    if(rclcpp::ok() && bt_engine_->isTaskRunning())
    {
      bt_engine_->tick_once();
    }
    else
    {
      if (bt_engine_->isTaskSuccess())
      {
        
        LOG_SUCCESS(this->get_logger(), task_log::Tag::FSM, "Self-check task success");
        boot_reset_system_state();
        switch_state(task_fsm::SystemState::RESET);
      }
      else
      {
        switch_state(task_fsm::SystemState::INIT);
        LOG_ERROR(this->get_logger(), task_log::Tag::FSM, "Self-check task failed");
      }
      
    }
  break;

  case task_fsm::SystemState::RESET:
    if(rclcpp::ok() && bt_engine_->isTaskRunning())
    {
      bt_engine_->tick_once();
    }
    else
    {
      if (bt_engine_->isTaskSuccess())
      {
        
        LOG_SUCCESS(this->get_logger(), task_log::Tag::FSM, "Reset task success");
        switch_state(task_fsm::SystemState::RESET_DONE);
      }
      else
      {
        boot_reset_system_state();
        switch_state(task_fsm::SystemState::RESET);
        LOG_ERROR(this->get_logger(), task_log::Tag::FSM, "Reset task failed, retrying...");
      }
      
    }
  break;

  case task_fsm::SystemState::RESET_DONE:
  boot_check_system_state();
  break;

  case task_fsm::SystemState::IDLE:
  break;

  case task_fsm::SystemState::RUNNING:
    if (modbus_slave_->get_reg(modbus_slave_->get_regaddr() + MODBUS_STATUS_OFFSET) != TASK_STATUS_RUNNING)
    {
      modbus_slave_->set_reg(modbus_slave_->get_regaddr() + MODBUS_STATUS_OFFSET, TASK_STATUS_RUNNING);
    }
    
    if(rclcpp::ok() && bt_engine_->isTaskRunning())
    {
      bt_engine_->tick_once();
    }
    else
    {

      if (modbus_slave_)
      {
        modbus_slave_->set_reg(modbus_slave_->get_regaddr(), 0);
        LOG_DEBUG(this->get_logger(), task_log::Tag::MODBUS, "Set running reg to 0, addr=" << modbus_slave_->get_regaddr());
      }

      if (bt_engine_->isTaskSuccess())
      {
        if (modbus_slave_)
        {
          modbus_slave_->set_reg(modbus_slave_->get_regaddr() + MODBUS_STATUS_OFFSET, TASK_STATUS_SUCCESS);
          LOG_DEBUG(this->get_logger(), task_log::Tag::MODBUS, "Set status reg to " << TASK_STATUS_SUCCESS << " (success), addr=" << (modbus_slave_->get_regaddr() + MODBUS_STATUS_OFFSET));
        }
        LOG_SUCCESS(this->get_logger(), task_log::Tag::FSM, "Task execution success");
      }
      else
      {
        // 任务失败：先中止行为树，确保所有节点状态被重置
        bt_engine_->abort();

        if (modbus_slave_)
        {
          modbus_slave_->set_reg(modbus_slave_->get_regaddr() + MODBUS_STATUS_OFFSET, TASK_STATUS_FAILED);
          LOG_DEBUG(this->get_logger(), task_log::Tag::MODBUS, "Set status reg to " << TASK_STATUS_FAILED << " (failed), addr=" << (modbus_slave_->get_regaddr() + MODBUS_STATUS_OFFSET));
        }


        LOG_ERROR(this->get_logger(), task_log::Tag::FSM, "Task execution failed");
      }
      // 无论成功还是失败，都重置所有 action 客户端状态并释放所有资源锁，确保下次任务从干净状态开始
      action_client_->reset_all_states();
      switch_state(task_fsm::SystemState::IDLE);

      // 任务已完成，检查是否有缓存的待执行触发，按 FIFO 顺序自动执行
      try_process_pending_trigger();
    }
  break;

  case task_fsm::SystemState::ERROR:
  {
    LOG_ERROR(this->get_logger(), task_log::Tag::FSM, "System in ERROR state");
    break;
  }

  case task_fsm::SystemState::EMERGENCY:
  {
    // 紧急状态：不做任何行为树调度，等待 safety_state_callback 收到恢复状态后切回 IDLE
    break;
  }
  
  
  default:
    break;
  }


  
}

}  // namespace task_mgr

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<task_mgr::TaskManagerNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
