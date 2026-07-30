// Copyright (c) 2026 Robot Control System
// All rights reserved.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "task_manager/arm_action_adapter.hpp"
#include "task_manager/task_action_client.hpp"

namespace
{

using MoveJoints = com_interfaces::action::MoveJoints;
using MovePose = com_interfaces::action::MovePose;

const std::vector<std::string> kLeftNames{
  "Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L",
  "Joint5_L", "Joint6_L", "Joint7_L"};
const std::vector<std::string> kRightNames{
  "Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R",
  "Joint5_R", "Joint6_R", "Joint7_R"};

TEST(ArmActionAdapterTest, PreservesLegacySingleArmPoseFields)
{
  task_cfg::MovePoseParam param;
  param.arm_id = "0";
  param.enable_collision_check = true;
  param.use_config_pose = true;
  param.frame_id = "base_link";
  param.config_pose = {0.2F, 1.0F, 2.0F, 0.0F, 0.0F, 0.0F, 1.0F};
  param.bound_size = {0.1F, 0.2F, 0.3F};
  param.use_force_control = false;
  param.speed = 0.2F;
  param.timeout_ms = 6000;

  geometry_msgs::msg::PoseStamped runtime_pose;
  runtime_pose.header.frame_id = "camera";
  sensor_msgs::msg::PointCloud2 cloud;
  MovePose::Goal goal;

  const auto result = task_act::build_move_pose_goal(
    "single_pose", param, runtime_pose, cloud, "", goal);

  ASSERT_TRUE(result.success) << result.error_message;
  EXPECT_EQ(goal.arm_id, MovePose::Goal::ARM_LEFT);
  EXPECT_EQ(goal.target_pose.header.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(goal.target_pose.pose.position.x, 0.2F);
  EXPECT_EQ(goal.bound_size.size(), 3U);
  EXPECT_TRUE(goal.left_target_pose.header.frame_id.empty());
}

TEST(ArmActionAdapterTest, MapsDualPoseIntoMergedExplicitFields)
{
  task_cfg::MovePoseParam param;
  param.arm_id = "2";
  param.frame_id = "base_link";
  param.left_target_pose = {0.21F, 1.08F, 2.06F, -0.64F, 0.0F, 0.0F, 0.76F};
  param.right_target_pose = {0.21F, -1.08F, 2.06F, 0.64F, 0.0F, 0.0F, 0.76F};
  param.left_bound_size = {0.0F, 0.0F, 0.0F};
  param.right_bound_size = {0.0F, 0.0F, 0.0F};
  param.synchronization_mode = 0;
  param.enable_collision_check = false;
  param.use_force_control = false;
  param.speed = 0.03F;
  param.timeout_ms = 10000;

  geometry_msgs::msg::PoseStamped runtime_pose;
  sensor_msgs::msg::PointCloud2 cloud;
  MovePose::Goal goal;

  const auto result = task_act::build_move_pose_goal(
    "dual_pose", param, runtime_pose, cloud, "", goal);

  ASSERT_TRUE(result.success) << result.error_message;
  EXPECT_EQ(goal.arm_id, MovePose::Goal::ARM_BOTH);
  EXPECT_TRUE(goal.target_pose.header.frame_id.empty());
  EXPECT_EQ(goal.left_target_pose.header.frame_id, "base_link");
  EXPECT_EQ(goal.right_target_pose.header.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(goal.left_target_pose.pose.position.y, 1.08F);
  EXPECT_DOUBLE_EQ(goal.right_target_pose.pose.position.y, -1.08F);
  EXPECT_EQ(goal.synchronization_mode, MovePose::Goal::SYNC_START_AND_FINISH);
}

TEST(ArmActionAdapterTest, RejectsUnsupportedDualPoseToolStage)
{
  task_cfg::MovePoseParam param;
  param.arm_id = "2";
  param.frame_id = "base_link";
  param.left_target_pose = {0.2F, 1.0F, 2.0F, 0.0F, 0.0F, 0.0F, 1.0F};
  param.right_target_pose = {0.2F, -1.0F, 2.0F, 0.0F, 0.0F, 0.0F, 1.0F};
  param.hand_action_type = {1U};
  param.speed = 0.1F;

  geometry_msgs::msg::PoseStamped runtime_pose;
  sensor_msgs::msg::PointCloud2 cloud;
  MovePose::Goal goal;
  const auto result = task_act::build_move_pose_goal(
    "invalid_dual_pose", param, runtime_pose, cloud, "", goal);

  EXPECT_FALSE(result.success);
}

TEST(ArmActionAdapterTest, PreservesLegacySingleArmJointFields)
{
  task_cfg::MoveJointsParam param;
  param.arm_id = "1";
  param.joint_name = kRightNames;
  param.position = std::vector<double>(7U, 0.1);
  param.speed = 0.2F;
  param.timeout_ms = 5000;

  MoveJoints::Goal goal;
  const auto result = task_act::build_move_joints_goal(
    "single_joints", param, goal);

  ASSERT_TRUE(result.success) << result.error_message;
  EXPECT_EQ(goal.arm_id, MoveJoints::Goal::ARM_RIGHT);
  EXPECT_EQ(goal.joint_name, kRightNames);
  EXPECT_TRUE(goal.left_joint_name.front().empty());
}

TEST(ArmActionAdapterTest, MapsDualJointsIntoMergedExplicitFields)
{
  task_cfg::MoveJointsParam param;
  param.arm_id = "2";
  param.left_joint_name = kLeftNames;
  param.left_position = std::vector<double>(7U, 0.1);
  param.right_joint_name = kRightNames;
  param.right_position = std::vector<double>(7U, -0.1);
  param.synchronization_mode = 0;
  param.speed = 0.2F;
  param.timeout_ms = 5000;

  MoveJoints::Goal goal;
  const auto result = task_act::build_move_joints_goal(
    "dual_joints", param, goal);

  ASSERT_TRUE(result.success) << result.error_message;
  EXPECT_EQ(goal.arm_id, MoveJoints::Goal::ARM_BOTH);
  EXPECT_TRUE(goal.joint_name.empty());
  EXPECT_EQ(goal.left_joint_name.front(), "Joint1_L");
  EXPECT_EQ(goal.right_joint_name.back(), "Joint7_R");
  EXPECT_DOUBLE_EQ(goal.left_position.front(), 0.1);
  EXPECT_DOUBLE_EQ(goal.right_position.front(), -0.1);
  EXPECT_EQ(goal.synchronization_mode, MoveJoints::Goal::SYNC_START_AND_FINISH);
}

TEST(ArmActionAdapterTest, RejectsIncompleteDualJointGoal)
{
  task_cfg::MoveJointsParam param;
  param.arm_id = "2";
  param.left_joint_name = kLeftNames;
  param.left_position = std::vector<double>(7U, 0.0);
  param.right_joint_name = kRightNames;
  param.right_position = std::vector<double>(6U, 0.0);
  param.speed = 0.1F;

  MoveJoints::Goal goal;
  const auto result = task_act::build_move_joints_goal(
    "invalid_dual_joints", param, goal);

  EXPECT_FALSE(result.success);
}

TEST(TaskActionClientResourceTest, CompletingOneArmKeepsOtherBranchLeased)
{
  std::map<std::string, task_act::ActionContext> contexts;
  std::mutex contexts_mutex;
  contexts["left"].state = task_act::ActionRunState::RUNNING;
  contexts["left"].acquired_resources = {task_act::ResourceType::LEFT_ARM};
  contexts["right"].state = task_act::ActionRunState::RUNNING;
  contexts["right"].acquired_resources = {task_act::ResourceType::RIGHT_ARM};

  const auto released = task_act::detail::complete_context(
    contexts, contexts_mutex, "left", task_act::ActionRunState::SUCCESS,
    task_act::ActionCallResult::SUCCESS);

  ASSERT_EQ(released.size(), 1U);
  EXPECT_EQ(released.front(), task_act::ResourceType::LEFT_ARM);
  EXPECT_EQ(contexts.at("left").state, task_act::ActionRunState::SUCCESS);
  EXPECT_TRUE(contexts.at("left").acquired_resources.empty());
  EXPECT_EQ(contexts.at("right").state, task_act::ActionRunState::RUNNING);
  ASSERT_EQ(contexts.at("right").acquired_resources.size(), 1U);
  EXPECT_EQ(
    contexts.at("right").acquired_resources.front(),
    task_act::ResourceType::RIGHT_ARM);
}

}  // namespace
