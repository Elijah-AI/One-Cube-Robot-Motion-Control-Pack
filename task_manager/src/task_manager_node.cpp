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
// 新增 /Task_manager/add_task Action Server，接收 safety_monitor 的安全控制任务。

#include "task_manager/task_manager_node.hpp"
#include "task_manager/log_helper.hpp"
#include <sstream>
#include <uuid/uuid.h>
#include <thread>

namespace task_mgr
{
TaskManagerNode::TaskManagerNode()
: Node("task_manager_node"), current_state_(task_fsm::SystemState::UNKNOWN), error_flag_(0)
{
  hb_pub_ = this->create_publisher<HeartbeatMsg>("/SafeNode/Heartbeat", 10);
  state_pub_ = this->create_publisher<NodeStatusMsg>("/SafeNode/status", 10);
  sys_state_client_ = this->create_client<GetSystemStatusSrv>("/Safety/get_system_status");

  // ==================== 初始化 AddTask Action Server ====================
  // 服务名称: /Task_manager/add_task，接收 safety_monitor 下发的安全控制任务
  add_task_action_server_ = rclcpp_action::create_server<AddTaskAction>(
    this,
    "/Task_manager/add_task",
    std::bind(&TaskManagerNode::handle_add_task_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&TaskManagerNode::handle_add_task_cancel, this,
      std::placeholders::_1),
    std::bind(&TaskManagerNode::handle_add_task_accepted, this,
      std::placeholders::_1));

  LOG_INFO(this->get_logger(), task_log::Tag::SYSTEM,
    "AddTask Action Server created at /Task_manager/add_task");
}

TaskManagerNode::~TaskManagerNode()
{
  // 等待安全任务线程结束
  safety_task_running_.store(false);
  if (safety_task_thread_.joinable())
  {
    safety_task_thread_.join();
  }
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
  // 紧急状态或安全任务执行中，拒绝常规任务触发
  if (current_state_ == task_fsm::SystemState::EMERGENCY || safety_task_running_.load())
  {
    LOG_WARN(this->get_logger(), task_log::Tag::FSM, "System in EMERGENCY/safety task mode, ignore normal task reg " << trigger_addr);
    return;
  }

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
      start_normal_task_timing(name);
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

void TaskManagerNode::start_normal_task_timing(const std::string& task_name)
{
  current_task_id_ = task_name;
  normal_task_start_time_ = std::chrono::steady_clock::now();
  task_running_.store(true);
  action_client_->begin_task_timing();
}

void TaskManagerNode::log_normal_task_total_time(const char* result)
{
  if (!task_running_.exchange(false))
  {
    return;
  }

  const auto end_time = std::chrono::steady_clock::now();
  const double execution_time =
    std::chrono::duration<double>(end_time - normal_task_start_time_).count();

  action_client_->log_task_timing_summary(current_task_id_, result);

  LOG_INFO_TITLE(this->get_logger(), task_log::Tag::FSM,
    "===== Task Complete: " << current_task_id_
    << ", result=" << result
    << ", total_time=" << execution_time << "s =====");

  current_task_id_.clear();
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
// AddTask Action Server 实现
// ============================================================================

rclcpp_action::GoalResponse TaskManagerNode::handle_add_task_goal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const AddTaskAction::Goal> goal)
{
  (void)uuid; // 未直接使用 UUID，但可用于日志追踪

  LOG_INFO_TITLE(this->get_logger(), task_log::Tag::SYSTEM,
    "===== AddTask Goal Received =====");
  LOG_INFO(this->get_logger(), task_log::Tag::SYSTEM,
    "task_id: " << goal->task_id
    << ", task_type: " << goal->task_type
    << ", target_node: " << goal->target_node
    << ", priority: " << goal->priority
    << ", timeout_ms: " << goal->timeout_ms);

  // ========== 参数校验 ==========
  if (goal->task_id.empty())
  {
    LOG_ERROR(this->get_logger(), task_log::Tag::SYSTEM, "AddTask: task_id is empty, rejecting");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->task_type.empty())
  {
    LOG_ERROR(this->get_logger(), task_log::Tag::SYSTEM, "AddTask: task_type is empty, rejecting");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->priority < 1 || goal->priority > SAFETY_TASK_PRIORITY_MAX)
  {
    LOG_ERROR(this->get_logger(), task_log::Tag::SYSTEM,
      "AddTask: invalid priority " << goal->priority << " (valid: 1-" << SAFETY_TASK_PRIORITY_MAX << "), rejecting");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // ========== 优先级与状态检查 ==========
  // 紧急任务（priority >= 20）始终接受，可抢占当前任务
  if (goal->priority >= SAFETY_TASK_PRIORITY_EMERGENCY)
  {
    LOG_WARN(this->get_logger(), task_log::Tag::SYSTEM,
      "AddTask: EMERGENCY task received! task_id=" << goal->task_id
      << ", priority=" << goal->priority << ", will preempt current task");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 非紧急任务：如果当前正在执行安全任务或处于紧急状态，拒绝
  if (current_state_ == task_fsm::SystemState::EMERGENCY)
  {
    LOG_WARN(this->get_logger(), task_log::Tag::SYSTEM,
      "AddTask: system in EMERGENCY state, rejecting non-emergency task " << goal->task_id);
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (safety_task_running_.load())
  {
    LOG_WARN(this->get_logger(), task_log::Tag::SYSTEM,
      "AddTask: safety task already running, rejecting task " << goal->task_id);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskManagerNode::handle_add_task_cancel(
  const std::shared_ptr<GoalHandleAddTask> goal_handle)
{
  (void)goal_handle;
  LOG_INFO(this->get_logger(), task_log::Tag::SYSTEM, "AddTask: cancel request received");
  // 安全任务一旦接受通常不允许取消，但这里返回 ACCEPT 以符合协议
  // 实际取消逻辑在 execute 线程中通过 safety_task_running_ 标志处理
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskManagerNode::handle_add_task_accepted(
  const std::shared_ptr<GoalHandleAddTask> goal_handle)
{
  auto goal = goal_handle->get_goal();

  // 构建优先级任务结构体
  PrioritizedTask task;
  task.task_id = goal->task_id;
  task.task_type = goal->task_type;
  task.target_node = goal->target_node;
  task.parameters_json = goal->parameters_json;
  task.priority = goal->priority;
  task.timeout_ms = goal->timeout_ms > 0 ? goal->timeout_ms : 5000;
  task.arrival_time = this->now();
  task.goal_handle = goal_handle;

  LOG_INFO_FLOW(this->get_logger(), task_log::Tag::SYSTEM,
    "AddTask: task accepted, task_id=" << task.task_id
    << ", priority=" << task.priority
    << ", pushing to priority queue");

  // ========== 紧急任务：立刻抢占 ==========
  if (task.priority >= SAFETY_TASK_PRIORITY_EMERGENCY)
  {
    // 立即中止当前行为树
    if (bt_engine_ && bt_engine_->isTaskRunning())
    {
      LOG_WARN(this->get_logger(), task_log::Tag::SYSTEM,
        "AddTask: EMERGENCY preemption - aborting current behavior tree");
      bt_engine_->abort();
    }

    // 如果已有安全任务线程在运行，先停止
    if (safety_task_running_.load())
    {
      LOG_WARN(this->get_logger(), task_log::Tag::SYSTEM,
        "AddTask: previous safety task is being interrupted by higher priority task");
      safety_task_running_.store(false);
      if (safety_task_thread_.joinable())
      {
        safety_task_thread_.join();
      }
    }

    // 进入紧急停止状态
    switch_state(task_fsm::SystemState::EMERGENCY, task_fsm::ERR_EMERGENCY);

    // 立即在新线程中执行紧急任务
    safety_task_running_.store(true);
    safety_task_thread_ = std::thread(
      &TaskManagerNode::safety_task_execution_thread,
      this, goal_handle);
  }
  else
  {
    // ========== 普通安全任务：加入优先级队列 ==========
    {
      std::lock_guard<std::mutex> lock(task_queue_mutex_);
      safety_task_queue_.push(task);
    }

    LOG_INFO(this->get_logger(), task_log::Tag::SYSTEM,
      "AddTask: task queued, queue size=" << safety_task_queue_.size());

    // 如果当前没有安全任务在执行，启动队列处理
    if (!safety_task_running_.load())
    {
      safety_task_running_.store(true);
      safety_task_thread_ = std::thread(
        &TaskManagerNode::safety_task_execution_thread,
        this, nullptr);  // nullptr 表示从队列取任务
    }
  }
}

void TaskManagerNode::execute_safety_task(
  const PrioritizedTask& task,
  std::shared_ptr<GoalHandleAddTask> goal_handle)
{
  auto logger = this->get_logger();

  LOG_INFO_TITLE(logger, task_log::Tag::SYSTEM,
    "===== Execute Safety Task: " << task.task_type << " (id=" << task.task_id << ") =====");

  // ========== 发送初始反馈 ==========
  auto feedback = std::make_shared<AddTaskAction::Feedback>();
  feedback->current_phase = "task_received";
  feedback->progress_percent = 0.0f;
  feedback->active_node = this->get_name();
  feedback->header.stamp = this->now();
  goal_handle->publish_feedback(feedback);

  safety_task_start_time_ = this->now();

  // ========== 根据 task_type 分发执行 ==========
  bool success = false;
  uint8_t status_code = 0;
  std::string error_message;

  if (task.task_type == "emergency_stop")
  {
    // ========== 紧急停止任务 ==========
    LOG_WARN(logger, task_log::Tag::SYSTEM,
      "EMERGENCY STOP: immediately halting all operations");

    feedback->current_phase = "emergency_stop_executing";
    feedback->progress_percent = 30.0f;
    feedback->header.stamp = this->now();
    goal_handle->publish_feedback(feedback);

    // 1. 中止行为树
    if (bt_engine_ && bt_engine_->isTaskRunning())
    {
      bt_engine_->abort();
      LOG_WARN(logger, task_log::Tag::SYSTEM, "EMERGENCY STOP: behavior tree aborted");
    }

    // 2. 重置所有 Action 客户端状态
    if (action_client_)
    {
      action_client_->reset_all_states();
    }

    // 3. 写入紧急停止标志到 Modbus（如果有特殊寄存器）
    // 通知 PLC 紧急状态

    // 4. 切换到紧急状态
    switch_state(task_fsm::SystemState::EMERGENCY, task_fsm::ERR_EMERGENCY);

    feedback->current_phase = "emergency_stop_complete";
    feedback->progress_percent = 100.0f;
    feedback->header.stamp = this->now();
    goal_handle->publish_feedback(feedback);

    success = true;
    status_code = 0;
    LOG_SUCCESS(logger, task_log::Tag::SYSTEM,
      "EMERGENCY STOP: completed successfully");
  }
  else if (task.task_type == "safety_halt")
  {
    // ========== 安全暂停任务 ==========
    LOG_WARN(logger, task_log::Tag::SYSTEM,
      "SAFETY HALT: pausing current operations");

    feedback->current_phase = "safety_halt_executing";
    feedback->progress_percent = 30.0f;
    feedback->header.stamp = this->now();
    goal_handle->publish_feedback(feedback);

    // 中止行为树但不改变状态为 EMERGENCY，允许后续恢复
    if (bt_engine_ && bt_engine_->isTaskRunning())
    {
      bt_engine_->abort();
    }

    if (action_client_)
    {
      action_client_->reset_all_states();
    }

    // 回到 IDLE 状态，等待后续指令
    switch_state(task_fsm::SystemState::IDLE);

    feedback->current_phase = "safety_halt_complete";
    feedback->progress_percent = 100.0f;
    feedback->header.stamp = this->now();
    goal_handle->publish_feedback(feedback);

    success = true;
    status_code = 0;
    LOG_SUCCESS(logger, task_log::Tag::SYSTEM,
      "SAFETY HALT: completed successfully");
  }
  else if (task.task_type == "resume")
  {
    // ========== 恢复任务 ==========
    LOG_INFO(logger, task_log::Tag::SYSTEM,
      "RESUME: attempting to resume from previous state");

    feedback->current_phase = "resume_executing";
    feedback->progress_percent = 30.0f;
    feedback->header.stamp = this->now();
    goal_handle->publish_feedback(feedback);

    // 清除紧急标志，恢复到 IDLE
    clear_error_flag(task_fsm::ERR_EMERGENCY);
    if (current_state_ == task_fsm::SystemState::EMERGENCY)
    {
      switch_state(task_fsm::SystemState::IDLE);
    }

    feedback->current_phase = "resume_complete";
    feedback->progress_percent = 100.0f;
    feedback->header.stamp = this->now();
    goal_handle->publish_feedback(feedback);

    success = true;
    status_code = 0;
    LOG_SUCCESS(logger, task_log::Tag::SYSTEM,
      "RESUME: completed successfully");
  }
  else if (task.task_type == "hardware_reset")
  {
    // ========== 硬件复位任务 ==========
    LOG_INFO(logger, task_log::Tag::SYSTEM,
      "HARDWARE RESET: executing hardware reset sequence");

    feedback->current_phase = "hardware_reset_executing";
    feedback->progress_percent = 30.0f;
    feedback->header.stamp = this->now();
    goal_handle->publish_feedback(feedback);

    // 执行开机复位序列
    if (!cfg_.boot_tasks.reset_seq.empty())
    {
      action_client_->reset_all_states();
      bt_engine_->load_action_group(cfg_.boot_tasks.reset_seq, *action_client_);
      bt_engine_->start();

      // 轮询等待复位完成（带超时）
      auto start_wait = this->now();
      rclcpp::Duration timeout(std::chrono::milliseconds(task.timeout_ms));
      bool timed_out = false;

      while (rclcpp::ok() && bt_engine_->isTaskRunning())
      {
        if ((this->now() - start_wait) > timeout)
        {
          timed_out = true;
          bt_engine_->abort();
          break;
        }
        bt_engine_->tick_once();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 更新反馈
        auto elapsed = this->now() - start_wait;
        float progress = std::min(95.0f,
          static_cast<float>(elapsed.nanoseconds()) /
          static_cast<float>(timeout.nanoseconds()) * 100.0f);
        feedback->current_phase = "hardware_reset_in_progress";
        feedback->progress_percent = 30.0f + progress * 0.65f;
        feedback->header.stamp = this->now();
        goal_handle->publish_feedback(feedback);
      }

      if (timed_out)
      {
        success = false;
        status_code = 2;  // TIMEOUT
        error_message = "Hardware reset timed out after " + std::to_string(task.timeout_ms) + "ms";
      }
      else if (bt_engine_->isTaskSuccess())
      {
        success = true;
        status_code = 0;
      }
      else
      {
        success = false;
        status_code = 1;  // FAILED
        error_message = "Hardware reset sequence failed";
      }
    }
    else
    {
      success = false;
      status_code = 1;  // FAILED
      error_message = "No hardware reset tasks configured";
    }

    feedback->current_phase = success ? "hardware_reset_complete" : "hardware_reset_failed";
    feedback->progress_percent = 100.0f;
    feedback->header.stamp = this->now();
    goal_handle->publish_feedback(feedback);
  }
  else
  {
    // ========== 未知任务类型 ==========
    LOG_ERROR(logger, task_log::Tag::SYSTEM,
      "Unknown safety task type: " << task.task_type);
    success = false;
    status_code = 4;  // INVALID_PARAM
    error_message = "Unknown task type: " + task.task_type;
  }

  // ========== 计算执行时间 ==========
  auto end_time = this->now();
  double execution_time = (end_time - safety_task_start_time_).seconds();

  // ========== 返回最终结果 ==========
  auto result = std::make_shared<AddTaskAction::Result>();
  result->success = success;
  result->status_code = status_code;
  result->error_message = error_message;
  result->execution_time = execution_time;

  if (success)
  {
    goal_handle->succeed(result);
  }
  else
  {
    goal_handle->abort(result);
  }

  LOG_INFO_TITLE(logger, task_log::Tag::SYSTEM,
    "===== Safety Task Complete: " << task.task_type
    << " (id=" << task.task_id << ") success=" << (success ? "true" : "false")
    << ", time=" << execution_time << "s =====");
}

void TaskManagerNode::safety_task_execution_thread(
  std::shared_ptr<GoalHandleAddTask> goal_handle)
{
  // 如果传入了 goal_handle，直接处理该紧急任务
  if (goal_handle)
  {
    auto goal = goal_handle->get_goal();
    PrioritizedTask task;
    task.task_id = goal->task_id;
    task.task_type = goal->task_type;
    task.target_node = goal->target_node;
    task.parameters_json = goal->parameters_json;
    task.priority = goal->priority;
    task.timeout_ms = goal->timeout_ms > 0 ? goal->timeout_ms : 5000;
    task.arrival_time = this->now();
    task.goal_handle = goal_handle;

    execute_safety_task(task, goal_handle);

    // 检查队列中是否还有待处理任务
    process_queued_tasks();
    return;
  }

  // 否则从优先级队列中取任务处理
  process_queued_tasks();
}

void TaskManagerNode::process_queued_tasks()
{
  while (rclcpp::ok() && safety_task_running_.load())
  {
    PrioritizedTask task;
    bool has_task = false;

    {
      std::lock_guard<std::mutex> lock(task_queue_mutex_);
      if (!safety_task_queue_.empty())
      {
        task = safety_task_queue_.top();
        safety_task_queue_.pop();
        has_task = true;
      }
    }

    if (!has_task)
    {
      // 队列为空，退出安全任务模式
      break;
    }

    LOG_INFO(this->get_logger(), task_log::Tag::SYSTEM,
      "Processing queued safety task: " << task.task_id
      << " (priority=" << task.priority << "), remaining=" << safety_task_queue_.size());

    execute_safety_task(task, task.goal_handle);
  }

  safety_task_running_.store(false);
  LOG_INFO(this->get_logger(), task_log::Tag::SYSTEM,
    "Safety task execution thread finished");
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
      const bool task_success = bt_engine_->isTaskSuccess();

      if (modbus_slave_)
      {
        modbus_slave_->set_reg(modbus_slave_->get_regaddr(), 0);
        LOG_DEBUG(this->get_logger(), task_log::Tag::MODBUS, "Set running reg to 0, addr=" << modbus_slave_->get_regaddr());
      }

      if (task_success)
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
      log_normal_task_total_time(task_success ? "SUCCESS" : "FAILURE");
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

  case task_fsm::SystemState::EMERGENCY:
  {
    // 紧急状态：等待安全任务执行完成，不做任何行为树调度
    // 安全任务执行线程会在完成后切换状态
    if (!safety_task_running_.load())
    {
      // 安全任务已完成，检查是否可以恢复到 IDLE
      LOG_INFO(this->get_logger(), task_log::Tag::FSM,
        "Safety task completed, staying in EMERGENCY until explicit resume");
    }
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
