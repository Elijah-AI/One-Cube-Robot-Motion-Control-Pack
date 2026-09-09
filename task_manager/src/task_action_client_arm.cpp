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
// call_move_pose
// ============================================================================

ActionCallResult TaskActionClient::call_move_pose(
  const std::string& task_id, const task_cfg::MovePoseParam& param,
  const geometry_msgs::msg::PoseStamped& target_pose,
  const sensor_msgs::msg::PointCloud2& cloud, const std::string& frame_id,
  const std::string& state_key)
{
  // 参数校验
  if (task_id.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "task_id is empty");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }
  if (param.arm_id != MovePose::Goal::ARM_LEFT && param.arm_id != MovePose::Goal::ARM_RIGHT &&
      param.arm_id != MovePose::Goal::ARM_BOTH && param.arm_id != MovePose::Goal::HEAD &&
      param.arm_id != MovePose::Goal::LEG && param.arm_id != MovePose::Goal::WHOLE_BODY_LEFT && 
      param.arm_id != MovePose::Goal::WHOLE_BODY_RIGHT && param.arm_id != MovePose::Goal::WHOLE_BODY_BOTH)
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "invalid arm_id: " << static_cast<int>(param.arm_id));
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  auto goal = MovePose::Goal();
  goal.header.stamp = node_->now();
  goal.header.frame_id = param.frame_id;

  goal.task_id = task_id;

  // arm_id 直接使用 uint8，与 com_interfaces 保持一致
  goal.arm_id = param.arm_id;

  goal.enable_collision_check = param.enable_collision_check;
  goal.enable_point_cloud_check = param.enable_point_cloud_check;

  //   com_interfaces/action/MovePose.action 定义：
  //   ARM_LEFT/ARM_RIGHT/HEAD/LEG → 使用 target_pose + bound_size
  //   ARM_BOTH → 使用 left_target_pose/left_bound_size + right_target_pose/right_bound_size + synchronization_mode
  if (param.arm_id == MovePose::Goal::ARM_BOTH)
  {
    // 填充左臂目标位姿（YAML 中为 [x, y, z, qx, qy, qz, qw] 浮点数组）

    if (param.use_config_pose)
    {
      if (param.left_target_pose.size() >= 7)
      {
        goal.left_target_pose.header.stamp = node_->now();
        goal.left_target_pose.header.frame_id = param.frame_id;
        goal.left_target_pose.pose.position.x    = param.left_target_pose[0];
        goal.left_target_pose.pose.position.y    = param.left_target_pose[1];
        goal.left_target_pose.pose.position.z    = param.left_target_pose[2];
        goal.left_target_pose.pose.orientation.x = param.left_target_pose[3];
        goal.left_target_pose.pose.orientation.y = param.left_target_pose[4];
        goal.left_target_pose.pose.orientation.z = param.left_target_pose[5];
        goal.left_target_pose.pose.orientation.w = param.left_target_pose[6];
      }
      // 填充右臂目标位姿
      if (param.right_target_pose.size() >= 7)
      {
        goal.right_target_pose.header.stamp = node_->now();
        goal.right_target_pose.header.frame_id = param.frame_id;
        goal.right_target_pose.pose.position.x    = param.right_target_pose[0];
        goal.right_target_pose.pose.position.y    = param.right_target_pose[1];
        goal.right_target_pose.pose.position.z    = param.right_target_pose[2];
        goal.right_target_pose.pose.orientation.x = param.right_target_pose[3];
        goal.right_target_pose.pose.orientation.y = param.right_target_pose[4];
        goal.right_target_pose.pose.orientation.z = param.right_target_pose[5];
        goal.right_target_pose.pose.orientation.w = param.right_target_pose[6];
      }
    }
    else 
    {
      // 如果不使用配置位姿，则直接使用传入的 target_pose
      goal.left_target_pose = target_pose;
      goal.right_target_pose = target_pose;
      // 强制使用配置文件中定义的 frame_id（配置了非空 frame_id 时）
      if (!param.frame_id.empty())
      {
        goal.left_target_pose.header.frame_id = param.frame_id;
        goal.right_target_pose.header.frame_id = param.frame_id;
      }
    }
    
    // 填充左右包围盒尺寸（固定长度 3）
    for (size_t i = 0; i < param.left_bound_size.size() && i < goal.left_bound_size.size(); ++i)
    {
      goal.left_bound_size[i] = param.left_bound_size[i];
    }
      
    for (size_t i = 0; i < param.right_bound_size.size() && i < goal.right_bound_size.size(); ++i)
    {
      goal.right_bound_size[i] = param.right_bound_size[i];
    }

    goal.synchronization_mode = param.synchronization_mode;
  }
  else
  {
    if (param.use_config_pose)
    {
      goal.target_pose = target_pose;
      if (param.config_pose.size() >= 7)
      {
        goal.target_pose.pose.position.x    = param.config_pose[0];
        goal.target_pose.pose.position.y    = param.config_pose[1];
        goal.target_pose.pose.position.z    = param.config_pose[2];
        goal.target_pose.pose.orientation.x = param.config_pose[3];
        goal.target_pose.pose.orientation.y = param.config_pose[4];
        goal.target_pose.pose.orientation.z = param.config_pose[5];
        goal.target_pose.pose.orientation.w = param.config_pose[6];
      }
      else
      {
        LOG_WARN(node_->get_logger(), task_log::Tag::ARM, "config_pose size < 7, using target_pose directly");
      }
    }
    else
    {
      goal.target_pose = target_pose;
    }
    // 若配置文件中定义了 frame_id 则强制使用，否则保留 target_pose 自带的 frame_id
    if (!param.frame_id.empty())
    {
      goal.target_pose.header.frame_id = param.frame_id;
    }
    if (!param.bound_size.empty())
    {
      goal.bound_size.resize(param.bound_size.size());
      for (size_t i = 0; i < param.bound_size.size(); ++i)
      {
        goal.bound_size[i] = param.bound_size[i];
      }
    }
  }
  goal.use_force_control = param.use_force_control;
  
  goal.speed = param.speed;
  if (!param.hand_action_type.empty())
  {
    goal.hand_action_type = param.hand_action_type;
  }
  goal.env_point_cloud = cloud;
  goal.point_cloud_frame_id = frame_id;
  goal.timeout_ms = param.timeout_ms;
  goal.enable_move_line = false;
 
  // ========== 参数打印（实际发送的 Goal） ==========
  {
    std::ostringstream oss;
    oss << "MovePose Goal: task_id=" << goal.task_id
        << "\n arm_id=" << static_cast<int>(goal.arm_id)
        << "\n collision_check=" << (goal.enable_collision_check ? "true" : "false")
        << "\n point_cloud_check=" << (goal.enable_point_cloud_check ? "true" : "false")
        << "\n force_control=" << (goal.use_force_control ? "true" : "false")
        << "\n enable_move_line=" << (goal.enable_move_line ? "true" : "false")
        << "\n speed=" << goal.speed
        << "\n timeout_ms=" << goal.timeout_ms
        << "\n cloud_frame=" << goal.point_cloud_frame_id
        << "\n cloud_size=" << goal.env_point_cloud.data.size()
        << "\n Target.head.frame_id=" << goal.target_pose.header.frame_id
        << "\n";
    if (goal.arm_id == MovePose::Goal::ARM_BOTH)
    {
      
      oss << "\n  [ARM_BOTH] sync_mode=" << static_cast<int>(goal.synchronization_mode);
      oss << "\n  left_target_pose=("
          << goal.left_target_pose.pose.position.x << ","
          << goal.left_target_pose.pose.position.y << ","
          << goal.left_target_pose.pose.position.z << ","
          << goal.left_target_pose.pose.orientation.x << ","
          << goal.left_target_pose.pose.orientation.y << ","
          << goal.left_target_pose.pose.orientation.z << ","
          << goal.left_target_pose.pose.orientation.w << ")";
      oss << "\n  right_target_pose=("
          << goal.right_target_pose.pose.position.x << ","
          << goal.right_target_pose.pose.position.y << ","
          << goal.right_target_pose.pose.position.z << ","
          << goal.right_target_pose.pose.orientation.x << ","
          << goal.right_target_pose.pose.orientation.y << ","
          << goal.right_target_pose.pose.orientation.z << ","
          << goal.right_target_pose.pose.orientation.w << ")";
      oss << "\n  left_bound_size=(";
      for (size_t i = 0; i < goal.left_bound_size.size(); ++i)
        oss << (i ? "," : "") << goal.left_bound_size[i];
      oss << ")\nright_bound_size=(";
      for (size_t i = 0; i < goal.right_bound_size.size(); ++i)
        oss << (i ? "," : "") << goal.right_bound_size[i];
      oss << ")";
    }
    else
    {
      oss << "\n  use_config_pose=" << (param.use_config_pose ? "true" : "false");
      oss << "\n  frame_id=(";
      oss << goal.target_pose.header.frame_id << ")";
      oss << "\n  target_pose=("
          << goal.target_pose.pose.position.x << ","
          << goal.target_pose.pose.position.y << ","
          << goal.target_pose.pose.position.z << ","
          << goal.target_pose.pose.orientation.x << ","
          << goal.target_pose.pose.orientation.y << ","
          << goal.target_pose.pose.orientation.z << ","
          << goal.target_pose.pose.orientation.w << ")";
      oss << "\n  bound_size=(";
      for (size_t i = 0; i < goal.bound_size.size(); ++i)
        oss << (i ? "," : "") << goal.bound_size[i];
      oss << ")";
    }
    if (!goal.hand_action_type.empty())
    {
      oss << "\n  hand_action_type=[";
      for (size_t i = 0; i < goal.hand_action_type.size(); ++i)
        oss << (i ? "," : "") << static_cast<int>(goal.hand_action_type[i]);
      oss << "]";
    }
    LOG_INFO(node_->get_logger(), task_log::Tag::ARM, oss.str());
  }

  // 等待服务端就绪
  int max_retries = 3;
  int retry_count = 0;
  bool server_ready = false;
  
  while (retry_count < max_retries && !server_ready) {
    LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Waiting for MovePose action server (attempt " << (retry_count + 1) << "/" << max_retries << ")...");

    server_ready = arm_client_->wait_for_action_server(std::chrono::seconds(5));

    if (!server_ready) {
      retry_count++;
      if (retry_count < max_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }

  if (!server_ready) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MovePose action server timeout after " << max_retries << " attempts");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }

  LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "MovePose action server connected successfully");

  auto send_goal_options = rclcpp_action::Client<MovePose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this, state_key](GoalHandleMovePose::SharedPtr goal_handle)
    {
      auto goal = goal_handle.get();
      if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Goal was rejected by server [" << state_key << "]");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
      } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Goal accepted by server [" << state_key << "], waiting for result...");
        // 保存活动的 goal handle，供抢占/急停时取消远端动作
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_arm_goal_ = goal_handle;
      }
    };

  send_goal_options.feedback_callback =
    std::bind(&TaskActionClient::arm_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback =
    [this, state_key](const GoalHandleMovePose::WrappedResult & result)
    {
      
      if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM, "MovePose succeeded [" << state_key << "]");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
      }
      else
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MovePose failed [" << state_key << "]: " << result.result->error_message);
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GENERAL;
      }

      // 释放该 action 占用的资源
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

      // 清除活动的 MovePose goal handle
      {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_arm_goal_.reset();
      }
    };

  arm_client_->async_send_goal(goal, send_goal_options);
  return ActionCallResult::SUCCESS;
}


// ============================================================================
// call_move_joints
// ============================================================================

ActionCallResult TaskActionClient::call_move_joints(
  const std::string& task_id, const task_cfg::MoveJointsParam& param,
  const std::string& state_key)
{
  if (task_id.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "task_id is empty");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }
  if (param.arm_id != MoveJoints::Goal::ARM_LEFT && param.arm_id != MoveJoints::Goal::ARM_RIGHT &&
      param.arm_id != MoveJoints::Goal::ARM_BOTH && param.arm_id != MoveJoints::Goal::HEAD &&
      param.arm_id != MoveJoints::Goal::LEG)
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "invalid arm_id: " << static_cast<int>(param.arm_id));
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  auto goal = MoveJoints::Goal();
  goal.header.stamp = node_->now();
  goal.task_id = task_id;

  // arm_id 直接使用 uint8，与 com_interfaces 保持一致
  goal.arm_id = param.arm_id;

  // com_interfaces/action/MoveJoints.action 定义：
  //   ARM_LEFT/ARM_RIGHT/HEAD/LEG → 使用 joint_name + position（可变长 vector）
  //   ARM_BOTH → 使用 left_joint_name/left_position + right_joint_name/right_position（固定长 array<7>）
  if (param.arm_id == MoveJoints::Goal::ARM_BOTH)
  {
    for (size_t i = 0; i < param.left_joint_name.size() && i < goal.left_joint_name.size(); ++i)
      goal.left_joint_name[i] = param.left_joint_name[i];
    for (size_t i = 0; i < param.left_position.size() && i < goal.left_position.size(); ++i)
      goal.left_position[i] = param.left_position[i];
    for (size_t i = 0; i < param.right_joint_name.size() && i < goal.right_joint_name.size(); ++i)
      goal.right_joint_name[i] = param.right_joint_name[i];
    for (size_t i = 0; i < param.right_position.size() && i < goal.right_position.size(); ++i)
      goal.right_position[i] = param.right_position[i];
    goal.synchronization_mode = param.synchronization_mode;
  }
  else
  {
    goal.joint_name = param.joint_name;
    goal.position = param.position;
  }
  goal.enable_collision_check = param.enable_collision_check;
  goal.speed = param.speed;
  goal.timeout_ms = param.timeout_ms;

  // ========== 参数打印（实际发送的 Goal） ==========
  {
    std::ostringstream oss;
    oss << "MoveJoints Goal: task_id=" << goal.task_id
        << ", arm_id=" << static_cast<int>(goal.arm_id)
        << ", collision_check=" << (goal.enable_collision_check ? "true" : "false")
        << ", speed=" << goal.speed
        << ", timeout_ms=" << goal.timeout_ms;
    if (goal.arm_id == MoveJoints::Goal::ARM_BOTH)
    {
      oss << "\n  [ARM_BOTH] sync_mode=" << static_cast<int>(goal.synchronization_mode);
      oss << "\n  left_joint_name=[";
      for (size_t i = 0; i < goal.left_joint_name.size(); ++i)
        oss << (i ? "," : "") << goal.left_joint_name[i];
      oss << "]";
      oss << "\n  left_position=[";
      for (size_t i = 0; i < goal.left_position.size(); ++i)
        oss << (i ? "," : "") << goal.left_position[i];
      oss << "]";
      oss << "\n  right_joint_name=[";
      for (size_t i = 0; i < goal.right_joint_name.size(); ++i)
        oss << (i ? "," : "") << goal.right_joint_name[i];
      oss << "]";
      oss << "\n  right_position=[";
      for (size_t i = 0; i < goal.right_position.size(); ++i)
        oss << (i ? "," : "") << goal.right_position[i];
      oss << "]";
    }
    else
    {
      oss << "\n  joint_name=[";
      for (size_t i = 0; i < goal.joint_name.size(); ++i)
        oss << (i ? "," : "") << goal.joint_name[i];
      oss << "]";
      oss << "\n  position=[";
      for (size_t i = 0; i < goal.position.size(); ++i)
        oss << (i ? "," : "") << goal.position[i];
      oss << "]";
    }
    LOG_INFO(node_->get_logger(), task_log::Tag::ARM, oss.str());
  }

  // 等待服务端就绪
  int max_retries = 3;
  int retry_count = 0;
  bool server_ready = false;

  while (retry_count < max_retries && !server_ready) {
    LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Waiting for MoveJoints action server (attempt " << (retry_count + 1) << "/" << max_retries << ")...");

    server_ready = move_joints_client_->wait_for_action_server(std::chrono::seconds(5));

    if (!server_ready) {
      retry_count++;
      if (retry_count < max_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }

  if (!server_ready) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MoveJoints action server timeout after " << max_retries << " attempts");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }

  LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "MoveJoints action server connected successfully");

  auto send_goal_options = rclcpp_action::Client<MoveJoints>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this, state_key](GoalHandleMoveJoints::SharedPtr goal_handle)
    {
      auto goal = goal_handle.get();
      if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MoveJoints goal rejected by server [" << state_key << "]");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
      } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "MoveJoints goal accepted by server [" << state_key << "], waiting for result...");
        // 保存活动的 goal handle，供抢占/急停时取消远端动作
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_move_joints_goal_ = goal_handle;
      }
    };

  send_goal_options.feedback_callback =
    std::bind(&TaskActionClient::move_joints_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback =
    [this, state_key](const GoalHandleMoveJoints::WrappedResult & result)
    {
      if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM, "MoveJoints succeeded [" << state_key << "]");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
      }
      else
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MoveJoints failed [" << state_key << "]: " << result.result->error_message);
        auto& ctx = get_ctx(state_key);
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

      // 清除活动的 MoveJoints goal handle
      {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_move_joints_goal_.reset();
      }
    };

  move_joints_client_->async_send_goal(goal, send_goal_options);
  return ActionCallResult::SUCCESS;
}


// ============================================================================
// arm 回调（默认 key 版本，保留兼容性）
// ============================================================================

void TaskActionClient::arm_goal_response_callback(GoalHandleMovePose::SharedPtr goal_handle)
{
    auto goal = goal_handle.get();
    if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Goal was rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Goal accepted by server, waiting for result...");
    }
}


void TaskActionClient::arm_feedback_callback(GoalHandleMovePose::SharedPtr, const std::shared_ptr<const MovePose::Feedback> feedback)
{
  LOG_DEBUG(node_->get_logger(), task_log::Tag::ARM, "Feedback: progress=" << (feedback ? std::to_string(feedback->progress_rate) : "N/A"));
}


void TaskActionClient::arm_result_callback(const GoalHandleMovePose::WrappedResult & result)
{
  if(result.result->success)
  {
    LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM, "MovePose succeeded");
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::SUCCESS;
    ctx.call_result = ActionCallResult::SUCCESS;
  }
  else
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MovePose failed: " << result.result->error_message);
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
// move_joints 回调（默认 key 版本）
// ============================================================================

void TaskActionClient::move_joints_goal_response_callback(GoalHandleMoveJoints::SharedPtr goal_handle)
{
    auto goal = goal_handle.get();
    if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MoveJoints goal rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "MoveJoints goal accepted by server, waiting for result...");
    }
}


void TaskActionClient::move_joints_feedback_callback(GoalHandleMoveJoints::SharedPtr, const std::shared_ptr<const MoveJoints::Feedback> feedback)
{
  LOG_DEBUG(node_->get_logger(), task_log::Tag::ARM, "MoveJoints feedback: progress=" << (feedback ? std::to_string(feedback->progress_rate) : "N/A"));
}


void TaskActionClient::move_joints_result_callback(const GoalHandleMoveJoints::WrappedResult & result)
{
  if(result.result->success)
  {
    LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM, "MoveJoints succeeded");
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::SUCCESS;
    ctx.call_result = ActionCallResult::SUCCESS;
  }
  else
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MoveJoints failed: " << result.result->error_message);
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

} // namespace task_act
