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

#include "task_manager/task_manager_node.hpp"
#include "task_manager/log_helper.hpp"
#include <sstream>
#include <uuid/uuid.h>

namespace task_mgr
{
TaskManagerNode::TaskManagerNode()
: Node("task_manager_node"), current_state_(task_fsm::SystemState::UNKNOWN), error_flag_(0)
{
  hb_pub_ = this->create_publisher<HeartbeatMsg>("/SafeNode/Heartbeat", 10);
  state_pub_ = this->create_publisher<NodeStatusMsg>("/SafeNode/status", 10);
  sys_state_client_ = this->create_client<GetSystemStatusSrv>("/Safety/get_system_status");
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
  // 只在 IDLE 状态才能触发新任务，防止任务重叠导致状态混乱
  if(current_state_ != task_fsm::SystemState::IDLE)
  {
    LOG_WARN(this->get_logger(), task_log::Tag::FSM, "System busy (state="
      << task_fsm::state_to_str(current_state_) << "), ignore task reg " << trigger_addr);
    return;
  }

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

void TaskManagerNode::bt_tick_cb()
{
  
  switch (current_state_)
  {
  case task_fsm::SystemState::UNKNOWN:
    break;

  case task_fsm::SystemState::INIT:
      boot_check_system_state();
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
    }
  break;

  case task_fsm::SystemState::ERROR:
  {
    LOG_ERROR(this->get_logger(), task_log::Tag::FSM, "System in ERROR state");
    break;
  }
  
  
  default:
    break;
  }


  
}

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<task_mgr::TaskManagerNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
