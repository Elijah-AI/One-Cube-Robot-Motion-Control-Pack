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
// call_vision_detect
// ============================================================================

ActionCallResult
TaskActionClient::call_vision_detect(
  const std::string& task_id, const task_cfg::VisionActionParam& param,
  const std::string& state_key)
{

  LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::VISION, "===== Vision Detect Start =====");
  
  // 参数校验
  if (task_id.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "task_id is empty");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }
  if (param.target_obj_name.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "target_obj_name is empty");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  auto goal = VisionDet::Goal();
  goal.header.stamp = node_->now();
  goal.task_id = task_id;
  goal.target_obj_name = param.target_obj_name;
  goal.expected_frame_id = param.expected_frame_id;
  goal.priority = param.priority;
  goal.type = param.type;

  LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Goal: task_id=" << goal.task_id
    << ", target=" << goal.target_obj_name
    << ", frame_id=" << goal.expected_frame_id
    << ", priority=" << goal.priority
    << ", type=" << goal.type);
  
  // 增加等待时间和重试机制
  int max_retries = 3;
  int retry_count = 0;
  bool server_ready = false;
  
  while (retry_count < max_retries && !server_ready) {
    LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Waiting for vision action server (attempt " << (retry_count + 1) << "/" << max_retries << ")...");
    
    server_ready = vision_client_->wait_for_action_server(std::chrono::seconds(5));
    
    if (!server_ready) {
      retry_count++;
      if (retry_count < max_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  
  if (!server_ready) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision action server timeout after " << max_retries << " attempts");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }
  
  LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Vision action server connected successfully");

  // 5. 配置异步回调选项 (SendGoalOptions)
  auto send_goal_options = rclcpp_action::Client<VisionDet>::SendGoalOptions();
  
  // 目标响应回调：使用 lambda 捕获 state_key，更新正确的 context
  send_goal_options.goal_response_callback =
    [this, state_key](GoalHandleVisionDetection::SharedPtr goal_handle)
    {
      auto goal = goal_handle.get();
      if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Goal was rejected by server [" << state_key << "]");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
      } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Goal accepted by server [" << state_key << "], waiting for result...");
        // 保存活动的 goal handle，供抢占/急停时取消远端动作
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_vision_goal_ = goal_handle;
      }
    };
    
  // 反馈回调：服务器周期性发送进度时触发
  send_goal_options.feedback_callback =
    std::bind(&TaskActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    
  // 结果回调：使用 lambda 捕获 state_key，更新正确的 context
  send_goal_options.result_callback =
    [this, state_key](const GoalHandleVisionDetection::WrappedResult & result)
    {
      auto& ctx = get_ctx(state_key);

      if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION, "Vision detect succeeded [" << state_key << "]");
        print_vision_result(node_->get_logger(), *result.result);

        // ========== 保存视觉检测结果到 context，供后续动作使用 ==========
        ctx.vision_result = *result.result;
        ctx.has_vision_result = true;

        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
      }
      else
      {
        // 失败时标记无有效结果
        ctx.has_vision_result = false;

        if (result.result->status_code == 3)
        {
          LOG_WARN(node_->get_logger(), task_log::Tag::VISION, "Vision detect server busy [" << state_key << "]: " << result.result->error_message);
          ctx.call_result = ActionCallResult::FAIL_SERVER_BUSY;
        }
        else
        {
          LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision detect failed [" << state_key << "]: " << result.result->error_message);
          ctx.call_result = ActionCallResult::FAIL_GENERAL;
        }
        ctx.state = ActionRunState::FAILURE;
      }

      // 释放该 action 占用的资源（遍历所有 context）
      {
        std::lock_guard<std::mutex> lock(ctx_mutex_);
        for (auto& [key, ctx_inner] : action_contexts_)
        {
          if (!ctx_inner.acquired_resources.empty())
          {
            for (const auto& res : ctx_inner.acquired_resources)
            {
              release_resource(res);
            }
            ctx_inner.acquired_resources.clear();
          }
        }
      }

      // 清除活动的 vision goal handle
      {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_vision_goal_.reset();
      }
    };
  auto send_goal_future = vision_client_->async_send_goal(goal, send_goal_options);
  LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::VISION, "===== Vision Detect End =====");
  return ActionCallResult::SUCCESS;

}

void TaskActionClient::goal_response_callback(GoalHandleVisionDetection::SharedPtr goal_handle)
{
    auto goal = goal_handle.get();
    if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Goal was rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Goal accepted by server, waiting for result...");
    }
}


void TaskActionClient::feedback_callback(GoalHandleVisionDetection::SharedPtr, const std::shared_ptr<const VisionDet::Feedback> feedback)
{
  LOG_DEBUG(node_->get_logger(), task_log::Tag::VISION, "Feedback: progress=" << (feedback ? std::to_string(feedback->progress_rate) : "N/A"));
}


void TaskActionClient::result_callback(const GoalHandleVisionDetection::WrappedResult & result)
{
  if(result.result->success)
  {
    LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION, "Vision detect succeeded");
    print_vision_result(node_->get_logger(), *result.result);
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::SUCCESS;
    ctx.call_result = ActionCallResult::SUCCESS;
    
  }
  else
  {
    // 根据错误码区分失败原因
    if (result.result->status_code == 3)
    {
      LOG_WARN(node_->get_logger(), task_log::Tag::VISION, "Vision detect server busy: " << result.result->error_message);
      auto& ctx = get_ctx("default");
      ctx.call_result = ActionCallResult::FAIL_SERVER_BUSY;
    }
    else
    {
      LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision detect failed: " << result.result->error_message);
      auto& ctx = get_ctx("default");
      ctx.call_result = ActionCallResult::FAIL_GENERAL;
    }
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::FAILURE;
  }

  // 释放该 action 占用的资源（遍历所有 context，释放 vision 类型的资源）
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


void TaskActionClient::print_vision_result(rclcpp::Logger logger, const VisionDet::Result & res)
{
  LOG_INFO_TITLE(logger, task_log::Tag::VISION, "===== DetectionPoseCloud Result =====");
  LOG_INFO(logger, task_log::Tag::VISION, "success: " << (res.success ? "true" : "false"));

  // status_code 文字解析
  std::string status_str;
  switch (res.status_code)
  {
    case 0: status_str = "OK"; break;
    case 1: status_str = "FAILED (hardware fault)"; break;
    case 2: status_str = "TIMEOUT"; break;
    case 3: status_str = "BUSY (high priority task running)"; break;
    default: status_str = "Unknown(" + std::to_string(res.status_code) + ")";
  }
  LOG_INFO(logger, task_log::Tag::VISION, "status_code: " << status_str);
  LOG_INFO(logger, task_log::Tag::VISION, "error_message: " << res.error_message);
  LOG_INFO(logger, task_log::Tag::VISION, "execution_time: " << res.execution_time << " s");
  LOG_INFO(logger, task_log::Tag::VISION, "total_detected_count: " << res.total_detected_count);

  // 多目标检测结果（detected_targets 数组）
  for (size_t i = 0; i < res.detected_targets.size(); ++i)
  {
    const auto & target = res.detected_targets[i];
    LOG_INFO(logger, task_log::Tag::VISION, "detected_targets[" << i << "].obj_name: " << target.obj_name);
    LOG_INFO(logger, task_log::Tag::VISION, "detected_targets[" << i << "].tool_type: " << target.tool_type);
    const auto & pose = target.pose;
    LOG_INFO(logger, task_log::Tag::VISION, "detected_targets[" << i << "].pose.frame_id: " << pose.header.frame_id);
    LOG_INFO(logger, task_log::Tag::VISION, "detected_targets[" << i << "].pose.position: x=" << pose.pose.position.x
      << ", y=" << pose.pose.position.y << ", z=" << pose.pose.position.z);
    LOG_INFO(logger, task_log::Tag::VISION, "detected_targets[" << i << "].pose.orientation: qx=" << pose.pose.orientation.x
      << ", qy=" << pose.pose.orientation.y << ", qz=" << pose.pose.orientation.z << ", qw=" << pose.pose.orientation.w);
    LOG_INFO(logger, task_log::Tag::VISION, "detected_targets[" << i << "].confidence: " << target.confidence);
  }

  // 额外打印 XYZABC 格式（四元数转欧拉角，取第一个检测目标）
  if (!res.detected_targets.empty())
  {
    print_vision_result_xyzabc(logger, res);
  }
}


// ============================================================================
// 四元数转欧拉角（静态工具函数）
// ============================================================================
void TaskActionClient::quaternion_to_rpy(double qx, double qy, double qz, double qw,
                                          double& roll, double& pitch, double& yaw)
{
    // roll (绕 X 轴)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (绕 Y 轴)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    // yaw (绕 Z 轴)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    // 转换为度
    roll  = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw   = yaw * 180.0 / M_PI;
}


// ============================================================================
// print_vision_result_xyzabc - 打印视觉检测结果的 XYZABC 格式
// ============================================================================
void TaskActionClient::print_vision_result_xyzabc(rclcpp::Logger logger, const VisionDet::Result & res)
{
  if (res.detected_targets.empty())
  {
    LOG_WARN(logger, task_log::Tag::VISION, "No detected target, skip XYZABC print");
    return;
  }
  const auto & target = res.detected_targets[0];
  const auto & pose = target.pose;
  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double z = pose.pose.position.z;
  double qx = pose.pose.orientation.x;
  double qy = pose.pose.orientation.y;
  double qz = pose.pose.orientation.z;
  double qw = pose.pose.orientation.w;

  double roll_deg, pitch_deg, yaw_deg;
  quaternion_to_rpy(qx, qy, qz, qw, roll_deg, pitch_deg, yaw_deg);

  LOG_INFO_TITLE(logger, task_log::Tag::VISION, "===== 6D Pose (XYZABC) =====");
  LOG_INFO(logger, task_log::Tag::VISION, "Frame ID: " << pose.header.frame_id);
  LOG_INFO(logger, task_log::Tag::VISION, "X (mm): " << std::fixed << std::setprecision(2) << x * 1000.0
    << ", Y (mm): " << y * 1000.0 << ", Z (mm): " << z * 1000.0);
  LOG_INFO(logger, task_log::Tag::VISION, "A (RotX/deg): " << std::fixed << std::setprecision(2) << roll_deg
    << ", B (RotY/deg): " << pitch_deg << ", C (RotZ/deg): " << yaw_deg);
  LOG_INFO(logger, task_log::Tag::VISION, "XYZABC: [" << std::fixed << std::setprecision(2)
    << x * 1000.0 << ", " << y * 1000.0 << ", " << z * 1000.0 << ", "
    << roll_deg << ", " << pitch_deg << ", " << yaw_deg << "]");
  LOG_INFO(logger, task_log::Tag::VISION, "Confidence: " << std::fixed << std::setprecision(2) << target.confidence * 100.0 << "%");
}


// ============================================================================
// call_vision_check_self - 调用视觉自检（/vision/Checkself）
// ============================================================================
ActionCallResult TaskActionClient::call_vision_check_self(
    const std::string& task_id, const task_cfg::VisionCheckSelfParam& param,
    const std::string& state_key)
{
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::VISION, "===== Vision CheckSelf Start =====");

    if (task_id.empty())
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "task_id is empty");
        return ActionCallResult::FAIL_PARAM_INVALID;
    }

    // 等待 server 就绪
    if (!hw_check_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision CheckSelf action server not available");
        return ActionCallResult::FAIL_SERVER_TIMEOUT;
    }

    auto goal = HardWareCheck::Goal();
    goal.header.stamp = node_->now();
    goal.task_id = task_id;
    goal.timeout_ms = param.timeout_ms > 0 ? param.timeout_ms : 15000;
    goal.priority = 1;

    LOG_INFO(node_->get_logger(), task_log::Tag::VISION,
        "VisionCheckSelf Goal: task_id=" << goal.task_id
        << ", timeout_ms=" << goal.timeout_ms << ", priority=" << (int)goal.priority);

    auto send_goal_options = rclcpp_action::Client<HardWareCheck>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this, state_key](GoalHandleHardWareCheck::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision CheckSelf goal rejected [" << state_key << "]");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
            }
            else
            {
                LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Vision CheckSelf goal accepted [" << state_key << "]");
                // 保存活动的 goal handle，供抢占/急停时取消远端动作
                std::lock_guard<std::mutex> lock(active_goal_mutex_);
                active_vision_check_goal_ = goal_handle;
            }
        };

    send_goal_options.feedback_callback =
        std::bind(&TaskActionClient::hw_check_feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
        [this, state_key](const GoalHandleHardWareCheck::WrappedResult& result)
        {
            if (result.result->success)
            {
                LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION,
                    "Vision CheckSelf succeeded [" << state_key << "], execution_time=" << result.result->execution_time << "s");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::SUCCESS;
                ctx.call_result = ActionCallResult::SUCCESS;
            }
            else
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
                    "Vision CheckSelf failed [" << state_key << "]: " << result.result->error_message);
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

            // 清除活动的 Vision CheckSelf goal handle
            {
                std::lock_guard<std::mutex> lock(active_goal_mutex_);
                active_vision_check_goal_.reset();
            }
        };

    hw_check_client_->async_send_goal(goal, send_goal_options);
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::VISION, "===== Vision CheckSelf End =====");
    return ActionCallResult::SUCCESS;
}


// ============================================================================
// call_vision_reset - 调用视觉复位（/vision/Reset）
// ============================================================================
ActionCallResult TaskActionClient::call_vision_reset(
    const std::string& task_id, const task_cfg::VisionResetParam& param,
    const std::string& state_key)
{
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::VISION, "===== Vision Reset Start =====");

    if (task_id.empty())
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "task_id is empty");
        return ActionCallResult::FAIL_PARAM_INVALID;
    }

    if (!hw_reset_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision Reset action server not available");
        return ActionCallResult::FAIL_SERVER_TIMEOUT;
    }

    auto goal = HardWareReset::Goal();
    goal.header.stamp = node_->now();
    goal.task_id = task_id;
    goal.timeout_ms = param.timeout_ms > 0 ? param.timeout_ms : 15000;
    goal.priority = 1;

    LOG_INFO(node_->get_logger(), task_log::Tag::VISION,
        "VisionReset Goal: task_id=" << goal.task_id
        << ", timeout_ms=" << goal.timeout_ms << ", priority=" << (int)goal.priority);

    auto send_goal_options = rclcpp_action::Client<HardWareReset>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this, state_key](GoalHandleHardWareReset::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision Reset goal rejected [" << state_key << "]");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
            }
            else
            {
                LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Vision Reset goal accepted [" << state_key << "]");
                // 保存活动的 goal handle，供抢占/急停时取消远端动作
                std::lock_guard<std::mutex> lock(active_goal_mutex_);
                active_vision_reset_goal_ = goal_handle;
            }
        };

    send_goal_options.feedback_callback =
        std::bind(&TaskActionClient::hw_reset_feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
        [this, state_key](const GoalHandleHardWareReset::WrappedResult& result)
        {
            if (result.result->success)
            {
                LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION,
                    "Vision Reset succeeded [" << state_key << "], duration=" << result.result->duration_sec << "s");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::SUCCESS;
                ctx.call_result = ActionCallResult::SUCCESS;
            }
            else
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
                    "Vision Reset failed [" << state_key << "]: " << result.result->error_message);
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

            // 清除活动的 Vision Reset goal handle
            {
                std::lock_guard<std::mutex> lock(active_goal_mutex_);
                active_vision_reset_goal_.reset();
            }
        };

    hw_reset_client_->async_send_goal(goal, send_goal_options);
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::VISION, "===== Vision Reset End =====");
    return ActionCallResult::SUCCESS;
}


// ============================================================================
// HardwareCheck 回调函数
// ============================================================================
void TaskActionClient::hw_check_goal_response_callback(GoalHandleHardWareCheck::SharedPtr goal_handle)
{
    if (!goal_handle)
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "HardwareCheck goal was rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    }
    else
    {
        LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "HardwareCheck goal accepted by server, waiting for result...");
    }
}


void TaskActionClient::hw_check_feedback_callback(
    GoalHandleHardWareCheck::SharedPtr,
    const std::shared_ptr<const HardWareCheck::Feedback> feedback)
{
    if (feedback)
    {
        LOG_DEBUG(node_->get_logger(), task_log::Tag::VISION,
            "HardwareCheck feedback: " << feedback->progress_percent << "% - " << feedback->current_step);
    }
}


void TaskActionClient::hw_check_result_callback(const GoalHandleHardWareCheck::WrappedResult& result)
{
    if (result.result->success)
    {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION,
            "HardwareCheck succeeded, execution_time=" << result.result->execution_time << "s");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
    }
    else
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
            "HardwareCheck failed: " << result.result->error_message);
        auto& ctx = get_ctx("default");
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
}


// ============================================================================
// HardwareReset 回调函数
// ============================================================================
void TaskActionClient::hw_reset_goal_response_callback(GoalHandleHardWareReset::SharedPtr goal_handle)
{
    if (!goal_handle)
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "HardwareReset goal was rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    }
    else
    {
        LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "HardwareReset goal accepted by server, waiting for result...");
    }
}


void TaskActionClient::hw_reset_feedback_callback(
    GoalHandleHardWareReset::SharedPtr,
    const std::shared_ptr<const HardWareReset::Feedback> feedback)
{
    if (feedback)
    {
        LOG_DEBUG(node_->get_logger(), task_log::Tag::VISION,
            "HardwareReset feedback: " << feedback->progress_percent << "% - " << feedback->current_step);
    }
}


void TaskActionClient::hw_reset_result_callback(const GoalHandleHardWareReset::WrappedResult& result)
{
    if (result.result->success)
    {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION,
            "HardwareReset succeeded, duration=" << result.result->duration_sec << "s");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
    }
    else
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
            "HardwareReset failed: " << result.result->error_message);
        auto& ctx = get_ctx("default");
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
}

} // namespace task_act
