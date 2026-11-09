#include "task_manager/task_action_client.hpp"
#include "task_manager/log_helper.hpp"
#include "task_manager/vision_pose_utils.hpp"
#include "task_manager/waist_turn_utils.hpp"
#include "task_manager/current_tcp_height.hpp"
#include "task_manager/ik_lift_recovery.hpp"
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
#include "com_interfaces/leg_joint_goal.hpp"
#include "com_interfaces/leg_body_motion_goal.hpp"
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

  geometry_msgs::msg::PoseStamped offset_target;
  geometry_msgs::msg::PoseStamped left_offset_target, right_offset_target;
  const bool dual_offsets = param.left_vision_pose_offset.enabled || param.right_vision_pose_offset.enabled;
  if (dual_offsets) {
    std::string error;
    if (param.use_config_pose || param.vision_pose_offset.enabled ||
        (param.arm_id != MovePose::Goal::ARM_BOTH && param.arm_id != MovePose::Goal::WHOLE_BODY_BOTH) ||
        !offset_vision_pose(target_pose, param.left_vision_pose_offset, param.frame_id, left_offset_target, error) ||
        !offset_vision_pose(target_pose, param.right_vision_pose_offset, param.frame_id, right_offset_target, error)) {
      LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Invalid dual vision offsets: " << error);
      return ActionCallResult::FAIL_PARAM_INVALID;
    }
  }
  if (!param.vision_result_key.empty()) {
    std::string error;
    if (param.use_config_pose || !valid_vision_pose(target_pose, param.frame_id, error)) {
      LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Invalid named vision target: " << error);
      return ActionCallResult::FAIL_PARAM_INVALID;
    }
  }
  if (param.vision_pose_offset.enabled) {
    std::string error;
    if (param.vision_pose_offset.keep_current_tcp_height) {
      com_interfaces::msg::MotorStatus::ConstSharedPtr tcp;
      sensor_msgs::msg::JointState::ConstSharedPtr joints;
      std::chrono::steady_clock::time_point tcp_received, joints_received;
      {
        std::lock_guard<std::mutex> lock(tcp_state_mutex_);
        tcp = tcp_state_;
        tcp_received = tcp_state_received_;
      }
      {
        std::lock_guard<std::mutex> lock(waist_state_mutex_);
        joints = waist_state_;
        joints_received = waist_state_received_;
      }
      const auto now = node_->now();
      const auto steady_now = std::chrono::steady_clock::now();
      const auto fresh = [&](const std_msgs::msg::Header& header,
                             std::chrono::steady_clock::time_point received) {
        const rclcpp::Time stamp(header.stamp, now.get_clock_type());
        const auto age_ns = (now - stamp).nanoseconds();
        return stamp.nanoseconds() > 0 && age_ns >= 0 && age_ns <= 250000000LL &&
          steady_now - received <= std::chrono::milliseconds(250);
      };
      if (!tcp || !joints || !fresh(tcp->header, tcp_received) ||
          !fresh(joints->header, joints_received)) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
          "current_tcp_height needs TCP and lower-body feedback no older than 250 ms");
        return ActionCallResult::FAIL_GENERAL;
      }
      std::vector<std::string> names;
      std::vector<double> positions;
      double local_height = 0.0;
      if (!expand_waist_turn(*joints, 0.0, false, names, positions, error) ||
          !resolve_current_tcp_height(target_pose, param, *tcp, offset_target, local_height, error)) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "current_tcp_height: " << error);
        return ActionCallResult::FAIL_GENERAL;
      }
      LOG_INFO(node_->get_logger(), task_log::Tag::ARM,
        "current_tcp_height [" << state_key << "]: base_z=" << offset_target.pose.position.z <<
        ", vision_z=" << local_height << ", arm_id=" << static_cast<int>(param.arm_id));
    } else if (!offset_vision_pose(target_pose, param, offset_target, error)) {
      LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, error);
      return ActionCallResult::FAIL_PARAM_INVALID;
    }
  }

  auto goal = MovePose::Goal();
  goal.header.stamp = node_->now();
  goal.header.frame_id = param.frame_id;

  goal.task_id = task_id;

  // arm_id 直接使用 uint8，与 com_interfaces 保持一致
  goal.arm_id = param.arm_id;

  goal.enable_collision_check = param.enable_collision_check;

  //   com_interfaces/action/MovePose.action 定义：
  //   ARM_LEFT/ARM_RIGHT/HEAD/LEG → 使用 target_pose + bound_size
  //   ARM_BOTH → 使用 left_target_pose/left_bound_size + right_target_pose/right_bound_size + synchronization_mode
  if (param.arm_id == MovePose::Goal::ARM_BOTH || param.arm_id == MovePose::Goal::WHOLE_BODY_BOTH)
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
    else if (dual_offsets)
    {
      goal.left_target_pose = left_offset_target;
      goal.right_target_pose = right_offset_target;
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
      goal.target_pose = param.vision_pose_offset.enabled ? offset_target : target_pose;
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
  for (const auto& values : param.via_poses) {
    geometry_msgs::msg::PoseStamped via;
    via.header = goal.target_pose.header;
    via.pose.position.x = values.at(0); via.pose.position.y = values.at(1); via.pose.position.z = values.at(2);
    via.pose.orientation.x = values.at(3); via.pose.orientation.y = values.at(4);
    via.pose.orientation.z = values.at(5); via.pose.orientation.w = values.at(6);
    goal.via_poses.push_back(via);
  }
  goal.use_force_control = param.use_force_control;

  goal.speed = param.speed;
  goal.acceleration_scale = param.acceleration_scale;
  goal.jerk_scale = param.jerk_scale;
  if (!param.hand_action_type.empty())
  {
    goal.hand_action_type = param.hand_action_type;
  }
  goal.env_point_cloud = cloud;
  goal.point_cloud_frame_id = frame_id;
  goal.timeout_ms = param.timeout_ms;
  goal.enable_move_line = param.enable_move_line;
  goal.whole_body_waist_only = param.whole_body_waist_only;
  goal.precision_arrival = param.precision_arrival;
  goal.keep_tcp_level = param.keep_tcp_level;
  goal.tcp_level_tolerance_rad = param.tcp_level_tolerance_rad;
  goal.keep_grasp_rigid = param.keep_grasp_rigid;
  goal.grasp_position_tolerance_m = param.grasp_position_tolerance_m;
  goal.grasp_orientation_tolerance_rad = param.grasp_orientation_tolerance_rad;

  // ========== 参数打印（实际发送的 Goal） ==========
  {
    std::ostringstream oss;
    oss << "MovePose Goal: task_id=" << goal.task_id
        << "\n arm_id=" << static_cast<int>(goal.arm_id)
        << "\n collision_check=" << static_cast<unsigned>(goal.enable_collision_check)
        << "\n force_control=" << (goal.use_force_control ? "true" : "false")
        << "\n enable_move_line=" << (goal.enable_move_line ? "true" : "false")
        << "\n whole_body_waist_only=" << (goal.whole_body_waist_only ? "true" : "false")
        << "\n precision_arrival=" << (goal.precision_arrival ? "true" : "false")
        << "\n speed=" << goal.speed
        << "\n timeout_ms=" << goal.timeout_ms
        << "\n cloud_frame=" << goal.point_cloud_frame_id
        << "\n cloud_size=" << goal.env_point_cloud.data.size()
        << "\n Target.head.frame_id=" << goal.target_pose.header.frame_id
        << "\n";
    if (goal.arm_id == MovePose::Goal::ARM_BOTH || goal.arm_id == MovePose::Goal::WHOLE_BODY_BOTH)
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
  const auto generation = begin_goal_request(state_key);

  send_goal_options.goal_response_callback =
    [this, state_key, generation](GoalHandleMovePose::SharedPtr goal_handle)
    {
      bool stale = false;
      {
        std::lock_guard<std::mutex> context_lock(ctx_mutex_);
        stale = action_contexts_[state_key].goal_generation != generation;
        if (!stale && goal_handle) {
          std::lock_guard<std::mutex> active_lock(active_goal_mutex_);
          active_arm_goal_[state_key] = goal_handle;
        }
      }
      if (stale) {
        if (goal_handle) {
          try { arm_client_->async_cancel_goal(goal_handle); }
          catch (const std::exception& error) {
            LOG_WARN(node_->get_logger(), task_log::Tag::ARM, "Stale goal cancel: " << error.what());
          }
        }
        return;
      }
      auto goal = goal_handle.get();
      if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Goal was rejected by server [" << state_key << "]");
        finish_action_timing(state_key, "GOAL_REJECTED");
        complete_action_context(state_key, ActionRunState::FAILURE, ActionCallResult::FAIL_GOAL_REJECTED, generation);
      } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Goal accepted by server [" << state_key << "], waiting for result...");
      }
    };

  send_goal_options.feedback_callback =
    [this, state_key, generation](GoalHandleMovePose::SharedPtr handle,
        const std::shared_ptr<const MovePose::Feedback> feedback) {
      if (!is_current_goal_request(state_key, generation)) return;
      if (feedback && (feedback->dual_task_status == MovePose::Feedback::STATUS_EXECUTING ||
          feedback->dual_task_status == MovePose::Feedback::STATUS_WAITING_FOR_COMPLETION ||
          feedback->dual_task_status == MovePose::Feedback::STATUS_STOPPING)) {
        std::lock_guard<std::mutex> lock(ctx_mutex_);
        if (action_contexts_[state_key].goal_generation != generation) return;
        action_contexts_[state_key].move_pose_execution_observed = true;
      }
      arm_feedback_callback(handle, feedback);
    };

  send_goal_options.result_callback =
    [this, state_key, generation](const GoalHandleMovePose::WrappedResult & result)
    {
      if (!is_current_goal_request(state_key, generation)) return;
      {
        std::lock_guard<std::mutex> lock(ctx_mutex_);
        auto& ctx = action_contexts_[state_key];
        if (ctx.goal_generation != generation) return;
        ctx.result_error_message = result.result ? result.result->error_message : "Empty MovePose result";
        ctx.pre_motion_ik_failure = result.result && is_pre_motion_whole_body_ik_failure(
            ctx.result_error_message, result.code == rclcpp_action::ResultCode::ABORTED,
            result.result->success, ctx.move_pose_execution_observed,
            result.result->timing_breakdown_valid, result.result->motion_time);
      }
      {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_arm_goal_.erase(state_key);
      }

      const bool succeeded = result.result && result.result->success;
      finish_action_timing(
        state_key, succeeded ? "SUCCESS" : "FAILURE",
        result.result && result.result->timing_breakdown_valid,
        result.result ? result.result->ik_time : 0.0,
        result.result ? result.result->planning_time : 0.0,
        result.result ? result.result->motion_time : 0.0);
      if (!result.result)
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
          "MovePose returned an empty result [" << state_key << "]");
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GENERAL, generation);
        return;
      }
      if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM, "MovePose succeeded [" << state_key << "]");
      }
      else
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MovePose failed [" << state_key << "]: " << result.result->error_message);
      }

      // 释放该 action 占用的资源
      complete_action_context(state_key, succeeded ? ActionRunState::SUCCESS : ActionRunState::FAILURE,
          succeeded ? ActionCallResult::SUCCESS : ActionCallResult::FAIL_GENERAL, generation);
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

  const bool waist_only_goal = param.arm_id == MoveJoints::Goal::LEG &&
    param.joint_name == std::vector<std::string>{"body_joint"};
  std::string body_error;
  if (!com_interfaces::validateLegBodyMotionGoal(param.arm_id, param.leg_motion_mode,
      param.body_tilt_deg, param.body_height_m, param.joint_name, param.position,
      param.speed, param.timeout_ms, 0, param.acceleration_scale, param.jerk_scale,
      param.relative_position, body_error)) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Invalid leg body motion: " << body_error);
    return ActionCallResult::FAIL_PARAM_INVALID;
  }
  if (param.leg_motion_mode != MoveJoints::Goal::LEG_JOINT_TARGETS) {
    // Validated body modes deliberately carry no joint-space targets.
  }
  else if (param.relative_position || waist_only_goal) {
    if (param.arm_id != MoveJoints::Goal::LEG ||
        param.joint_name != std::vector<std::string>{"body_joint"} || param.position.size() != 1 ||
        !std::isfinite(param.position[0]) || !std::isfinite(param.speed) ||
        param.speed <= 0 || param.speed > 1 || param.timeout_ms <= 0) {
      LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Invalid waist-only MoveJ");
      return ActionCallResult::FAIL_PARAM_INVALID;
    }
  }
  else if (param.arm_id == MoveJoints::Goal::ARM_BOTH)
  {
    constexpr size_t kArmJointCount = 7U;
    if (param.left_joint_name.size() != kArmJointCount ||
        param.left_position.size() != kArmJointCount ||
        param.right_joint_name.size() != kArmJointCount ||
        param.right_position.size() != kArmJointCount)
    {
      LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
        "ARM_BOTH requires exactly 7 joint names and positions per arm");
      return ActionCallResult::FAIL_PARAM_INVALID;
    }
  }
  else
  {
    const bool leg_orientation_goal = param.arm_id == MoveJoints::Goal::LEG &&
      (param.joint_name.size() == 2 || param.position.size() == 2 ||
       param.joint_name.size() == 3 || param.position.size() == 3);
    if (leg_orientation_goal) {
      com_interfaces::LegJointSelection selection;
      std::string error;
      if (!com_interfaces::validateLegOrientationGoal(param.joint_name, param.position, selection, error) ||
          !std::isfinite(param.speed) || param.speed <= 0 || param.speed > 1 || param.timeout_ms <= 0) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Invalid body-orientation leg MoveJ: " << error);
        return ActionCallResult::FAIL_PARAM_INVALID;
      }
    }
    const size_t expected_joint_count = leg_orientation_goal ? param.joint_name.size() :
      (param.arm_id == MoveJoints::Goal::HEAD) ? 2U :
      (param.arm_id == MoveJoints::Goal::LEG) ? 4U : 7U;
    if (param.joint_name.size() != expected_joint_count ||
        param.position.size() != expected_joint_count)
    {
      LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
        "arm_id=" << static_cast<int>(param.arm_id) << " requires exactly "
        << expected_joint_count << " joint names and positions, got names="
        << param.joint_name.size() << ", positions=" << param.position.size());
      return ActionCallResult::FAIL_PARAM_INVALID;
    }
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
  goal.acceleration_scale = param.acceleration_scale;
  goal.jerk_scale = param.jerk_scale;
  goal.timeout_ms = param.timeout_ms;
  goal.leg_motion_mode = param.leg_motion_mode;
  goal.keep_grasp_rigid = param.keep_grasp_rigid;
  goal.grasp_position_tolerance_m = param.grasp_position_tolerance_m;
  goal.grasp_orientation_tolerance_rad = param.grasp_orientation_tolerance_rad;
  goal.body_tilt_deg = param.body_tilt_deg;
  goal.body_height_m = param.body_height_m;
  goal.precision_arrival = param.precision_arrival;
  if (param.leg_motion_mode != MoveJoints::Goal::LEG_JOINT_TARGETS) goal.header.frame_id = "base_link";

  // ========== 参数打印（实际发送的 Goal） ==========
  {
    std::ostringstream oss;
    oss << "MoveJoints Goal: task_id=" << goal.task_id
        << ", arm_id=" << static_cast<int>(goal.arm_id)
        << ", collision_check=" << static_cast<unsigned>(goal.enable_collision_check)
        << ", speed=" << goal.speed
        << ", timeout_ms=" << goal.timeout_ms
        << ", precision_arrival=" << (goal.precision_arrival ? "true" : "false")
        << ", leg_motion_mode=" << static_cast<int>(goal.leg_motion_mode)
        << ", body_tilt_deg=" << goal.body_tilt_deg
        << ", body_height_m=" << goal.body_height_m;
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

  if (param.relative_position || (waist_only_goal && !param.keep_grasp_rigid)) {
    std::lock_guard<std::mutex> lock(waist_state_mutex_);
    const double age = waist_state_ ?
      (node_->now().nanoseconds() - rclcpp::Time(waist_state_->header.stamp).nanoseconds()) * 1e-9 : 1e9;
    const double received_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - waist_state_received_).count();
    std::string error;
    if (!waist_state_ || rclcpp::Time(waist_state_->header.stamp).nanoseconds() <= 0 ||
        age < -0.05 || age > 0.25 || received_age > 0.25 ||
        !expand_waist_turn(*waist_state_, param.position[0], param.relative_position,
                           goal.joint_name, goal.position, error)) {
      LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
        "Waist-only MoveJ requires fresh, stopped /wbc/joint_states: " << error);
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
          "[GX12_DIAG] has_state=" << (waist_state_ ? 1 : 0)
          << ", stamp_age_ms=" << age * 1000.0
          << ", received_age_ms=" << received_age * 1000.0
          << ", error=" << error);

      return ActionCallResult::FAIL_PARAM_INVALID;
    }
    LOG_INFO(node_->get_logger(), task_log::Tag::ARM,
      "J4 " << (param.relative_position ? "delta=" : "absolute=")
      << param.position[0] << ", target=" << goal.position[3]
      << ", held legs=[" << goal.position[0] << "," << goal.position[1] << "," << goal.position[2] << "]");
    if (param.keep_grasp_rigid) {
      // The server freezes legs from its own fresh measured start. Only resolve
      // a relative waist angle here; do not turn it into three leg commands.
      const double waist_target=goal.position[3];
      goal.joint_name={"body_joint"};goal.position={waist_target};
    }
  }

  auto send_goal_options = rclcpp_action::Client<MoveJoints>::SendGoalOptions();
  const auto generation = begin_goal_request(state_key);

  send_goal_options.goal_response_callback =
    [this, state_key, generation](GoalHandleMoveJoints::SharedPtr goal_handle)
    {
      bool stale = false;
      {
        std::lock_guard<std::mutex> context_lock(ctx_mutex_);
        stale = action_contexts_[state_key].goal_generation != generation;
        if (!stale && goal_handle) {
          std::lock_guard<std::mutex> active_lock(active_goal_mutex_);
          active_move_joints_goal_[state_key] = goal_handle;
        }
      }
      if (stale) {
        if (goal_handle) {
          try { move_joints_client_->async_cancel_goal(goal_handle); }
          catch (const std::exception& error) {
            LOG_WARN(node_->get_logger(), task_log::Tag::ARM, "Stale goal cancel: " << error.what());
          }
        }
        return;
      }
      auto goal = goal_handle.get();
      if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MoveJoints goal rejected by server [" << state_key << "]");
        finish_action_timing(state_key, "GOAL_REJECTED");
        complete_action_context(state_key, ActionRunState::FAILURE, ActionCallResult::FAIL_GOAL_REJECTED, generation);
      } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "MoveJoints goal accepted by server [" << state_key << "], waiting for result...");
      }
    };

  send_goal_options.feedback_callback =
    std::bind(&TaskActionClient::move_joints_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback =
    [this, state_key, generation](const GoalHandleMoveJoints::WrappedResult & result)
    {
      if (!is_current_goal_request(state_key, generation)) return;
      {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_move_joints_goal_.erase(state_key);
      }

      const bool succeeded = result.result && result.result->success;
      finish_action_timing(
        state_key, succeeded ? "SUCCESS" : "FAILURE",
        result.result && result.result->timing_breakdown_valid,
        result.result ? result.result->ik_time : 0.0,
        result.result ? result.result->planning_time : 0.0,
        result.result ? result.result->motion_time : 0.0);
      if (!result.result)
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
          "MoveJoints returned an empty result [" << state_key << "]");
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GENERAL, generation);
        return;
      }
      if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM, "MoveJoints succeeded [" << state_key << "]");
      }
      else
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MoveJoints failed [" << state_key << "]: " << result.result->error_message);
      }

      // 只释放当前 Goal 的资源；不能影响仍在运行的并行兄弟动作。
      complete_action_context(state_key, succeeded ? ActionRunState::SUCCESS : ActionRunState::FAILURE,
          succeeded ? ActionCallResult::SUCCESS : ActionCallResult::FAIL_GENERAL, generation);
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
