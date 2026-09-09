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
// call_control_check_self - 调用运控自检（/Control/Checkself）
// ============================================================================
ActionCallResult TaskActionClient::call_control_check_self(
    const std::string& task_id, const task_cfg::ControlCheckSelfParam& param,
    const std::string& state_key)
{
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::ARM, "===== Control CheckSelf Start =====");

    if (task_id.empty())
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "task_id is empty");
        return ActionCallResult::FAIL_PARAM_INVALID;
    }

    // 使用成员变量 control_check_client_（已在构造函数中初始化）
    if (!control_check_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Control CheckSelf action server not available");
        return ActionCallResult::FAIL_SERVER_TIMEOUT;
    }

    auto goal = HardWareCheck::Goal();
    goal.header.stamp = node_->now();
    goal.task_id = task_id;
    goal.timeout_ms = param.timeout_ms > 0 ? param.timeout_ms : 15000;
    goal.priority = 1;
    // 设置 com_port 参数（如果配置了）
    if (param.com_port.size() >= 2)
    {
        goal.com_port[0] = param.com_port[0];
        goal.com_port[1] = param.com_port[1];
    }

    LOG_INFO(node_->get_logger(), task_log::Tag::ARM,
        "ControlCheckSelf Goal: task_id=" << goal.task_id
        << ", timeout_ms=" << goal.timeout_ms << ", priority=" << (int)goal.priority
        << ", com_port=[" << (int)goal.com_port[0] << "," << (int)goal.com_port[1] << "]");

    auto send_goal_options = rclcpp_action::Client<HardWareCheck>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this, state_key](GoalHandleHardWareCheck::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Control CheckSelf goal rejected [" << state_key << "]");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
            }
            else
            {
                LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Control CheckSelf goal accepted [" << state_key << "]");
                // 保存活动的 goal handle，供抢占/急停时取消远端动作
                std::lock_guard<std::mutex> lock(active_goal_mutex_);
                active_control_check_goal_ = goal_handle;
            }
        };

    send_goal_options.feedback_callback =
        [this](GoalHandleHardWareCheck::SharedPtr, const std::shared_ptr<const HardWareCheck::Feedback> feedback)
        {
            if (feedback)
            {
                LOG_DEBUG(node_->get_logger(), task_log::Tag::ARM,
                    "Control CheckSelf feedback: " << feedback->progress_percent << "% - " << feedback->current_step);
            }
        };

    send_goal_options.result_callback =
        [this, state_key](const GoalHandleHardWareCheck::WrappedResult& result)
        {
            if (result.result->success)
            {
                LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM,
                    "Control CheckSelf succeeded [" << state_key << "], execution_time=" << result.result->execution_time << "s");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::SUCCESS;
                ctx.call_result = ActionCallResult::SUCCESS;
            }
            else
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
                    "Control CheckSelf failed [" << state_key << "]: " << result.result->error_message);
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GENERAL;
            }

            // 释放资源
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

            // 清除活动的 Control CheckSelf goal handle
            {
                std::lock_guard<std::mutex> lock(active_goal_mutex_);
                active_control_check_goal_.reset();
            }
        };

    control_check_client_->async_send_goal(goal, send_goal_options);
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::ARM, "===== Control CheckSelf End =====");
    return ActionCallResult::SUCCESS;
}


// ============================================================================
// call_control_reset - 调用运控复位（/Control/Reset）
// ============================================================================
ActionCallResult TaskActionClient::call_control_reset(
    const std::string& task_id, const task_cfg::ControlResetParam& param,
    const std::string& state_key)
{
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::ARM, "===== Control Reset Start =====");

    if (task_id.empty())
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "task_id is empty");
        return ActionCallResult::FAIL_PARAM_INVALID;
    }

    // 使用成员变量 control_reset_client_（已在构造函数中初始化）
    if (!control_reset_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Control Reset action server not available");
        return ActionCallResult::FAIL_SERVER_TIMEOUT;
    }

    auto goal = HardWareReset::Goal();
    goal.header.stamp = node_->now();
    goal.task_id = task_id;
    goal.timeout_ms = param.timeout_ms > 0 ? param.timeout_ms : 15000;
    goal.priority = 1;

    LOG_INFO(node_->get_logger(), task_log::Tag::ARM,
        "ControlReset Goal: task_id=" << goal.task_id
        << ", timeout_ms=" << goal.timeout_ms << ", priority=" << (int)goal.priority);

    auto send_goal_options = rclcpp_action::Client<HardWareReset>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this, state_key](GoalHandleHardWareReset::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Control Reset goal rejected [" << state_key << "]");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
            }
            else
            {
                LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Control Reset goal accepted [" << state_key << "]");
                // 保存活动的 goal handle，供抢占/急停时取消远端动作
                std::lock_guard<std::mutex> lock(active_goal_mutex_);
                active_control_reset_goal_ = goal_handle;
            }
        };

    send_goal_options.feedback_callback =
        [this](GoalHandleHardWareReset::SharedPtr, const std::shared_ptr<const HardWareReset::Feedback> feedback)
        {
            if (feedback)
            {
                LOG_DEBUG(node_->get_logger(), task_log::Tag::ARM,
                    "Control Reset feedback: " << feedback->progress_percent << "% - " << feedback->current_step);
            }
        };

    send_goal_options.result_callback =
        [this, state_key](const GoalHandleHardWareReset::WrappedResult& result)
        {
            if (result.result->success)
            {
                LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM,
                    "Control Reset succeeded [" << state_key << "], duration=" << result.result->duration_sec << "s");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::SUCCESS;
                ctx.call_result = ActionCallResult::SUCCESS;
            }
            else
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
                    "Control Reset failed [" << state_key << "]: " << result.result->error_message);
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GENERAL;
            }

            // 释放资源
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

            // 清除活动的 Control Reset goal handle
            {
                std::lock_guard<std::mutex> lock(active_goal_mutex_);
                active_control_reset_goal_.reset();
            }
        };

    control_reset_client_->async_send_goal(goal, send_goal_options);
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::ARM, "===== Control Reset End =====");
    return ActionCallResult::SUCCESS;
}

} // namespace task_act
