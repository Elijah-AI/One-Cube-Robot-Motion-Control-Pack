// Copyright (c) 2026 Robot Control System
// All rights reserved.

#ifndef TASK_MANAGER__ARM_ACTION_ADAPTER_HPP_
#define TASK_MANAGER__ARM_ACTION_ADAPTER_HPP_

#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "com_interfaces/action/move_joints.hpp"
#include "com_interfaces/action/move_pose.hpp"
#include "task_manager/task_config_parser.hpp"

namespace task_act
{

struct GoalBuildResult
{
  bool success{false};
  std::string error_message;

  explicit operator bool() const noexcept { return success; }
};

/// Build one merged MovePose goal. Legacy single-arm fields remain supported;
/// ARM_BOTH uses the explicit left/right fields from the merged action.
GoalBuildResult build_move_pose_goal(
  const std::string& task_id,
  const task_cfg::MovePoseParam& param,
  const geometry_msgs::msg::PoseStamped& runtime_target_pose,
  const sensor_msgs::msg::PointCloud2& cloud,
  const std::string& point_cloud_frame_id,
  com_interfaces::action::MovePose::Goal& goal);

/// Build one merged MoveJoints goal. Legacy single-arm fields remain supported;
/// ARM_BOTH uses the explicit seven-joint targets for each arm.
GoalBuildResult build_move_joints_goal(
  const std::string& task_id,
  const task_cfg::MoveJointsParam& param,
  com_interfaces::action::MoveJoints::Goal& goal);

}  // namespace task_act

#endif  // TASK_MANAGER__ARM_ACTION_ADAPTER_HPP_
