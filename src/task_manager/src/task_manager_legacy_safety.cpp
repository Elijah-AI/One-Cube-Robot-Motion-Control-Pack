#include "task_manager/task_manager_node.hpp"
#include "task_manager/log_helper.hpp"

namespace task_mgr
{
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

  if ((goal->task_type == "resume" || goal->task_type == "hardware_reset") &&
      device_safety_status_.load() != SAFETY_STATE_NORMAL) {
    LOG_WARN(this->get_logger(), task_log::Tag::SYSTEM,
      "AddTask cannot resume/reset while device SafetyState requests a stop");
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

    // The worker applies the requested stop type; resume/reset are not stop commands.

    // 立即在新线程中执行紧急任务
    safety_task_running_.store(true);
    if (safety_task_thread_.joinable()) { safety_task_thread_.join(); }
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
      if (safety_task_thread_.joinable()) { safety_task_thread_.join(); }
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

  if ((task.task_type == "resume" || task.task_type == "hardware_reset") &&
      device_safety_status_.load() != SAFETY_STATE_NORMAL) {
    auto result = std::make_shared<AddTaskAction::Result>();
    result->success = false;
    result->error_message = "Device safety stop is active";
    goal_handle->abort(result);
    return;
  }

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
      action_client_->cancel_all_actions();
      action_client_->reset_all_states();
    }

    // 3. 写入紧急停止标志到 Modbus（如果有特殊寄存器）
    // 通知 PLC 紧急状态

    // 4. Keep the legacy stop latched until an explicit resume.
    legacy_stop_status_.store(SAFETY_STATE_EMERGENCY);
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
      action_client_->cancel_all_actions();
      action_client_->reset_all_states();
    }

    legacy_stop_status_.store(std::max(legacy_stop_status_.load(), SAFETY_STATE_HALT));
    const auto status = effective_safety_status(device_safety_status_.load(), legacy_stop_status_.load());
    switch_state(status == SAFETY_STATE_EMERGENCY ? task_fsm::SystemState::EMERGENCY : task_fsm::SystemState::ERROR);

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

    legacy_stop_status_.store(SAFETY_STATE_NORMAL);
    auto current = std::make_shared<SafetyStateMsg>();
    current->status = device_safety_status_.load();
    safety_state_callback(current);

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
      action_client_->cancel_all_actions();
      action_client_->reset_all_states();
      bt_engine_->load_action_group(cfg_.boot_tasks.reset_seq, *action_client_);
      bt_engine_->start();

      // 轮询等待复位完成（带超时）
      auto start_wait = this->now();
      rclcpp::Duration timeout(std::chrono::milliseconds(task.timeout_ms));
      bool timed_out = false;

      while (rclcpp::ok() && safety_task_running_.load() && !goal_handle->is_canceling() && bt_engine_->isTaskRunning())
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

      if (!safety_task_running_.load() || goal_handle->is_canceling())
      {
        halt_current_modbus_task();
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

  if (goal_handle->is_canceling())
  {
    goal_handle->canceled(result);
  }
  else if (success)
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

}
