#include "task_manager/task_action_client.hpp"
#include "task_manager/log_helper.hpp"
#include <algorithm>
#include <random>
#include <sstream>
#include <uuid/uuid.h>
#include <cmath>
#include <iomanip>
#include <thread>
#include <chrono>

// 使用 com_interfaces 中的 Action 定义
#include "com_interfaces/action/detection_pose_cloud.hpp"
#include "com_interfaces/action/control_gripper.hpp"
#include "com_interfaces/action/move_pose.hpp"
#include "com_interfaces/action/move_joints.hpp"
#include "com_interfaces/action/control_suck.hpp"

namespace task_act
{

// ============================================================================
// call_control_suck
// ============================================================================

ActionCallResult TaskActionClient::call_control_suck(
  const std::string& task_id, const task_cfg::SuckParam& param,
  const std::string& state_key)
{
  if (task_id.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "task_id is empty (control_suck)");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  auto goal = CtrlSuck::Goal();
  goal.header.stamp = node_->now();
  goal.task_id = task_id;

  // suck_id 直接使用 uint8，与 com_interfaces 保持一致
  if (param.suck_id != CtrlSuck::Goal::SUCK_LEFT && param.suck_id != CtrlSuck::Goal::SUCK_RIGHT &&
      param.suck_id != CtrlSuck::Goal::SUCK_BOTH)
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "invalid suck_id: " << static_cast<int>(param.suck_id));
    return ActionCallResult::FAIL_PARAM_INVALID;
  }
  goal.suck_id = param.suck_id;

  goal.active = param.active;
  goal.timeout_ms = param.timeout_ms;

  // ========== 参数打印（实际发送的 Goal） ==========
  {
    std::ostringstream oss;
    oss << "ControlSuck Goal: task_id=" << goal.task_id
        << ", suck_id=" << static_cast<int>(goal.suck_id)
        << ", active=[";
    for (size_t i = 0; i < goal.active.size(); ++i)
      oss << (i ? "," : "") << (goal.active[i] ? "true" : "false");
    oss << "], timeout_ms=" << goal.timeout_ms;
    LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, oss.str());
  }

  // 等待服务端就绪
  if (!control_suck_client_->wait_for_action_server(std::chrono::seconds(2))) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck action server not available");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }

  auto send_goal_options = rclcpp_action::Client<CtrlSuck>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this, state_key](GoalHandleCtrlSuck::SharedPtr goal_handle)
    {
      if (goal_handle) {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_suck_goal_[state_key] = goal_handle;
      }
      auto goal = goal_handle.get();
      if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck goal rejected by server [" << state_key << "]");
        finish_action_timing(state_key, "GOAL_REJECTED");
        complete_action_context(state_key, ActionRunState::FAILURE, ActionCallResult::FAIL_GOAL_REJECTED);
      } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck goal accepted by server [" << state_key << "], waiting for result...");
      }
    };

  send_goal_options.result_callback =
    [this, state_key](const GoalHandleCtrlSuck::WrappedResult & result)
    {
      {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_suck_goal_.erase(state_key);
      }

      const bool succeeded = result.result && result.result->success;
      finish_action_timing(
        state_key, succeeded ? "SUCCESS" : "FAILURE",
        false, 0.0, 0.0, 0.0, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      if (!result.result)
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION,
          "ControlSuck returned an empty result [" << state_key << "]");
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GENERAL);
        return;
      }
      if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck succeeded [" << state_key << "]");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
      }
      else
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck failed [" << state_key << "]: " << result.result->error_message);
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GENERAL;
      }

      complete_action_context(state_key, get_ctx(state_key).state, get_ctx(state_key).call_result);
    };

  control_suck_client_->async_send_goal(goal, send_goal_options);
  return ActionCallResult::SUCCESS;
}


// ============================================================================
// control_suck 回调（默认 key 版本）
// ============================================================================

void TaskActionClient::control_suck_goal_response_callback(GoalHandleCtrlSuck::SharedPtr goal_handle)
{
    auto goal = goal_handle.get();
    if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck goal rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck goal accepted by server, waiting for result...");
    }
}


void TaskActionClient::control_suck_feedback_callback(GoalHandleCtrlSuck::SharedPtr, const std::shared_ptr<const CtrlSuck::Feedback> feedback)
{
  LOG_DEBUG(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck feedback: progress=" << (feedback ? std::to_string(feedback->progress_rate) : "N/A"));
}


void TaskActionClient::control_suck_result_callback(const GoalHandleCtrlSuck::WrappedResult & result)
{
  if(result.result->success)
  {
    LOG_SUCCESS(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck succeeded");
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::SUCCESS;
    ctx.call_result = ActionCallResult::SUCCESS;
  }
  else
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck failed: " << result.result->error_message);
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::FAILURE;
    ctx.call_result = ActionCallResult::FAIL_GENERAL;
  }

  {
    std::lock_guard<std::mutex> lock(ctx_mutex_);
    for (auto& [key, ctx] : action_contexts_)
    {
      if (!ctx.acquired_resources.empty())
      {
        for (const auto& res : ctx.acquired_resources)
        {
          release_resource(res);
        }
        ctx.acquired_resources.clear();
      }
    }
  }
}



// ============================================================================
// call_gripper
// ============================================================================

ActionCallResult TaskActionClient::call_gripper(
  const std::string& task_id, const task_cfg::GripperParam& param,
  const std::string& state_key)
{

  LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::GRIPPER, "===== Gripper Control Start =====");

  // 在访问动态数组前完成校验和整体赋值。
  CtrlGripper::Goal goal;
  std::string goal_error;
  const auto build_result = detail::build_gripper_goal(
    task_id, param, goal, goal_error);
  if (!action_call_result_is_ok(build_result))
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER,
      "Invalid gripper goal: " << goal_error);
    return build_result;
  }
  goal.header.stamp = node_->now();

  // 1. 等待 Server 就绪
  if(!gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper action server not available");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }

  LOG_INFO(node_->get_logger(), task_log::Tag::GRIPPER, "Goal: task_id=" << task_id
    << ", gripper_id=" << (int)goal.gripper_id
    << ", position=[" << (int)goal.position[0] << "," << (int)goal.position[1] << "]"
    << ", block=" << (goal.block_until_complete ? "true" : "false")
    << ", timeout=" << goal.timeout_ms << "ms");
  
  // 增加等待时间和重试机制
  int max_retries = 3;
  int retry_count = 0;
  bool server_ready = false;
  
  while (retry_count < max_retries && !server_ready) {
    LOG_INFO(node_->get_logger(), task_log::Tag::GRIPPER, "Waiting for gripper action server (attempt " << (retry_count + 1) << "/" << max_retries << ")...");
    
    server_ready = gripper_client_->wait_for_action_server(std::chrono::seconds(5));
    
    if (!server_ready) {
      retry_count++;
      if (retry_count < max_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  
  if (!server_ready) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "gripper action server timeout after " << max_retries << " attempts");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }
  
  LOG_INFO(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper action server connected successfully");



  // 5. 配置异步回调选项 (SendGoalOptions)
  auto send_goal_options = rclcpp_action::Client<CtrlGripper>::SendGoalOptions();
  
  // 目标响应回调：使用 lambda 捕获 state_key，更新正确的 context
  send_goal_options.goal_response_callback =
    [this, state_key](GoalHandleCtrlGripper::SharedPtr goal_handle)
    {
      if (goal_handle) {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_gripper_goal_[state_key] = goal_handle;
      }
      auto goal = goal_handle.get();
      if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "Goal was rejected by server [" << state_key << "]");
        finish_action_timing(state_key, "GOAL_REJECTED");
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GOAL_REJECTED);
      } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::GRIPPER, "Goal accepted by server [" << state_key << "], waiting for result...");
      }
    };
    
  // 反馈回调：服务器周期性发送进度时触发
  send_goal_options.feedback_callback =
    std::bind(&TaskActionClient::gripper_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

  // 结果回调：使用 lambda 捕获 state_key，更新正确的 context
  send_goal_options.result_callback =
    [this, state_key](const GoalHandleCtrlGripper::WrappedResult & result)
    {
      {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_gripper_goal_.erase(state_key);
      }

      const bool succeeded = result.result && result.result->success;
      finish_action_timing(
        state_key, succeeded ? "SUCCESS" : "FAILURE",
        false, 0.0, 0.0, 0.0, true);
      if (!result.result)
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER,
          "Gripper control returned an empty result [" << state_key << "]");
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GENERAL);
      }
      else if (result.result->success &&
        (result.result->final_position.size() < 2U ||
        result.result->final_torque.size() < 2U))
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER,
          "Gripper control returned incomplete final arrays [" << state_key
          << "]: position_size=" << result.result->final_position.size()
          << ", torque_size=" << result.result->final_torque.size());
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GENERAL);
      }
      else if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper control succeeded [" << state_key << "]: final_position=["
          << (int)result.result->final_position[0] << "," << (int)result.result->final_position[1]
          << "], final_torque=[" << (int)result.result->final_torque[0] << "," << (int)result.result->final_torque[1] << "]");
        complete_action_context(
          state_key, ActionRunState::SUCCESS,
          ActionCallResult::SUCCESS);
      }
      else
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper control failed [" << state_key << "]: " << result.result->error_message);
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GENERAL);
      }
    };
  gripper_send_future_ = gripper_client_->async_send_goal(goal, send_goal_options);
  return ActionCallResult::SUCCESS;
}



void TaskActionClient::gripper_goal_response_callback(GoalHandleCtrlGripper::SharedPtr goal_handle)
{
    auto goal = goal_handle.get();
    if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "Goal was rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::GRIPPER, "Goal accepted by server, waiting for result...");
    }
}


void TaskActionClient::gripper_feedback_callback(GoalHandleCtrlGripper::SharedPtr, const std::shared_ptr<const GoalHandleCtrlGripper::Feedback> feedback)
{
  LOG_DEBUG(node_->get_logger(), task_log::Tag::GRIPPER, "Feedback: progress=" << (feedback ? std::to_string(feedback->progress_rate) : "N/A"));
}


void TaskActionClient::gripper_result_callback(const GoalHandleCtrlGripper::WrappedResult & result)
{
  //StopTimer();
  if(result.result->success)
  {
    LOG_SUCCESS(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper control succeeded: final_position=["
      << (int)result.result->final_position[0] << "," << (int)result.result->final_position[1]
      << "], final_torque=[" << (int)result.result->final_torque[0] << "," << (int)result.result->final_torque[1] << "]");
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::SUCCESS;
    ctx.call_result = ActionCallResult::SUCCESS;
    
  }
  else
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper control failed: " << result.result->error_message);
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::FAILURE;
    ctx.call_result = ActionCallResult::FAIL_GENERAL;
  }

  // 释放该 action 占用的资源（遍历所有 context）
  {
    std::lock_guard<std::mutex> lock(ctx_mutex_);
    for (auto& [key, ctx] : action_contexts_)
    {
      if (!ctx.acquired_resources.empty())
      {
        for (const auto& res : ctx.acquired_resources)
        {
          release_resource(res);
        }
        ctx.acquired_resources.clear();
      }
    }
  }
}

} // namespace task_act
