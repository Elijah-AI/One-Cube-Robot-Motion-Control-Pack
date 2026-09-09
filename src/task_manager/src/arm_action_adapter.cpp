// Copyright (c) 2026 Robot Control System
// All rights reserved.

#include "task_manager/arm_action_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <string>
#include <vector>

namespace task_act
{
namespace
{

GoalBuildResult failure(const std::string& message)
{
  return {false, message};
}

GoalBuildResult parse_arm_id(const std::string& text, std::uint8_t& arm_id)
{
  try
  {
    std::size_t parsed = 0;
    const int value = std::stoi(text, &parsed);
    if (parsed != text.size() || value < 0 || value > 2)
    {
      return failure("arm_id must be 0 (left), 1 (right), or 2 (both)");
    }
    arm_id = static_cast<std::uint8_t>(value);
    return {true, {}};
  }
  catch (const std::exception&)
  {
    return failure("arm_id must be an integer string: 0, 1, or 2");
  }
}

bool finite_pose(const std::vector<float>& values)
{
  return values.size() == 7U &&
    std::all_of(values.begin(), values.end(), [](float value) {
      return std::isfinite(value);
    }) &&
    std::hypot(
      std::hypot(values[3], values[4]),
      std::hypot(values[5], values[6])) > 1.0e-6F;
}

void assign_pose(
  const std::vector<float>& values, const std::string& frame_id,
  geometry_msgs::msg::PoseStamped& pose)
{
  pose.header.frame_id = frame_id;
  pose.pose.position.x = values[0];
  pose.pose.position.y = values[1];
  pose.pose.position.z = values[2];
  pose.pose.orientation.x = values[3];
  pose.pose.orientation.y = values[4];
  pose.pose.orientation.z = values[5];
  pose.pose.orientation.w = values[6];
}

bool valid_speed(float speed)
{
  return std::isfinite(speed) && speed > 0.0F && speed <= 1.0F;
}

bool empty_cloud(const sensor_msgs::msg::PointCloud2& cloud)
{
  return cloud.data.empty() && cloud.width == 0U && cloud.height == 0U;
}

}  // namespace

GoalBuildResult build_move_pose_goal(
  const std::string& task_id,
  const task_cfg::MovePoseParam& param,
  const geometry_msgs::msg::PoseStamped& runtime_target_pose,
  const sensor_msgs::msg::PointCloud2& cloud,
  const std::string& point_cloud_frame_id,
  com_interfaces::action::MovePose::Goal& goal)
{
  if (task_id.empty())
  {
    return failure("task_id is empty");
  }
  if (!valid_speed(param.speed))
  {
    return failure("speed must be finite and in (0, 1]");
  }

  std::uint8_t arm_id = 0;
  const auto arm_result = parse_arm_id(param.arm_id, arm_id);
  if (!arm_result)
  {
    return arm_result;
  }

  goal = com_interfaces::action::MovePose::Goal{};
  goal.task_id = task_id;
  goal.arm_id = arm_id;
  goal.enable_collision_check = param.enable_collision_check;
  goal.use_force_control = param.use_force_control;
  goal.speed = param.speed;
  goal.hand_action_type = param.hand_action_type;
  goal.timeout_ms = param.timeout_ms > 0 ? param.timeout_ms : 5000;
  goal.env_point_cloud = cloud;
  goal.point_cloud_frame_id = point_cloud_frame_id;

  if (arm_id != com_interfaces::action::MovePose::Goal::ARM_BOTH)
  {
    goal.target_pose = runtime_target_pose;
    if (param.use_config_pose)
    {
      if (!finite_pose(param.config_pose))
      {
        return failure(
          "target_pose must contain seven finite values with a non-zero quaternion");
      }
      const std::string frame_id = param.frame_id.empty() ?
        runtime_target_pose.header.frame_id : param.frame_id;
      if (frame_id.empty())
      {
        return failure("single-arm target frame_id is empty");
      }
      assign_pose(param.config_pose, frame_id, goal.target_pose);
    }
    if (goal.target_pose.header.frame_id.empty())
    {
      return failure("single-arm target frame_id is empty");
    }
    goal.bound_size = param.bound_size;
    return {true, {}};
  }

  if (!finite_pose(param.left_target_pose) ||
    !finite_pose(param.right_target_pose))
  {
    return failure(
      "ARM_BOTH requires left_target_pose and right_target_pose with seven "
      "finite values and non-zero quaternions");
  }
  if (param.frame_id.empty())
  {
    return failure("ARM_BOTH frame_id is empty");
  }
  if (param.synchronization_mode !=
    com_interfaces::action::MovePose::Goal::SYNC_START_AND_FINISH)
  {
    return failure("ARM_BOTH currently requires synchronization_mode 0");
  }
  if (param.use_force_control || !param.hand_action_type.empty())
  {
    return failure("ARM_BOTH force/tool actions are not supported");
  }
  if (!empty_cloud(cloud))
  {
    return failure("ARM_BOTH environment point clouds are not supported");
  }
  const auto valid_zero_bound = [](const std::vector<float>& bound) {
    return bound.size() == 3U &&
      std::all_of(bound.begin(), bound.end(), [](float value) {
        return std::isfinite(value) && value == 0.0F;
      });
  };
  if (!valid_zero_bound(param.left_bound_size) ||
    !valid_zero_bound(param.right_bound_size))
  {
    return failure(
      "ARM_BOTH left_bound_size/right_bound_size must each be [0, 0, 0]");
  }

  assign_pose(param.left_target_pose, param.frame_id, goal.left_target_pose);
  assign_pose(param.right_target_pose, param.frame_id, goal.right_target_pose);
  std::copy(
    param.left_bound_size.begin(), param.left_bound_size.end(),
    goal.left_bound_size.begin());
  std::copy(
    param.right_bound_size.begin(), param.right_bound_size.end(),
    goal.right_bound_size.begin());
  goal.synchronization_mode = param.synchronization_mode;
  return {true, {}};
}

GoalBuildResult build_move_joints_goal(
  const std::string& task_id,
  const task_cfg::MoveJointsParam& param,
  com_interfaces::action::MoveJoints::Goal& goal)
{
  if (task_id.empty())
  {
    return failure("task_id is empty");
  }
  if (!valid_speed(param.speed))
  {
    return failure("speed must be finite and in (0, 1]");
  }

  std::uint8_t arm_id = 0;
  const auto arm_result = parse_arm_id(param.arm_id, arm_id);
  if (!arm_result)
  {
    return arm_result;
  }

  goal = com_interfaces::action::MoveJoints::Goal{};
  goal.task_id = task_id;
  goal.arm_id = arm_id;
  goal.enable_collision_check = param.enable_collision_check;
  goal.speed = param.speed;
  goal.timeout_ms = param.timeout_ms > 0 ? param.timeout_ms : 5000;

  if (arm_id != com_interfaces::action::MoveJoints::Goal::ARM_BOTH)
  {
    if (param.joint_name.size() != 7U || param.position.size() != 7U)
    {
      return failure(
        "single-arm MoveJoints requires exactly seven joint names and positions");
    }
    goal.joint_name = param.joint_name;
    goal.position = param.position;
    return {true, {}};
  }

  if (param.synchronization_mode !=
    com_interfaces::action::MoveJoints::Goal::SYNC_START_AND_FINISH)
  {
    return failure("ARM_BOTH currently requires synchronization_mode 0");
  }
  if (param.left_joint_name.size() != 7U ||
    param.left_position.size() != 7U ||
    param.right_joint_name.size() != 7U ||
    param.right_position.size() != 7U)
  {
    return failure(
      "ARM_BOTH MoveJoints requires exactly seven names and positions per arm");
  }

  std::copy(
    param.left_joint_name.begin(), param.left_joint_name.end(),
    goal.left_joint_name.begin());
  std::copy(
    param.left_position.begin(), param.left_position.end(),
    goal.left_position.begin());
  std::copy(
    param.right_joint_name.begin(), param.right_joint_name.end(),
    goal.right_joint_name.begin());
  std::copy(
    param.right_position.begin(), param.right_position.end(),
    goal.right_position.begin());
  goal.synchronization_mode = param.synchronization_mode;
  return {true, {}};
}

}  // namespace task_act
