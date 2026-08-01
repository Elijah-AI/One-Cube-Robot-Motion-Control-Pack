// Copyright (c) 2026 Robot Control System
// All rights reserved.

#include <gtest/gtest.h>

#include "task_manager/task_config_parser.hpp"

#ifndef TASK_MANAGER_TEST_CONFIG_PATH
#error "TASK_MANAGER_TEST_CONFIG_PATH must point to the parser fixture"
#endif

TEST(TaskConfigParserTest, LoadsMergedDualArmFieldsFromYaml)
{
  task_cfg::TaskConfigParser parser;
  task_cfg::GlobalConfig config;

  ASSERT_TRUE(parser.load_config(TASK_MANAGER_TEST_CONFIG_PATH, config));
  ASSERT_EQ(config.task_dict.size(), 2U);

  const auto& joint_task = config.task_dict.at("dual_joint_task");
  ASSERT_EQ(joint_task.action_seq.size(), 1U);
  ASSERT_EQ(joint_task.action_seq.front().size(), 1U);
  const auto& joints = joint_task.action_seq.front().front().move_joints_param;
  EXPECT_EQ(joints.arm_id, "2");
  EXPECT_EQ(joints.left_joint_name.size(), 7U);
  EXPECT_EQ(joints.right_joint_name.size(), 7U);
  EXPECT_EQ(joints.left_position.at(6), 0.6);
  EXPECT_EQ(joints.right_position.at(6), -0.6);
  EXPECT_EQ(joints.synchronization_mode, 0U);

  const auto& pose_task = config.task_dict.at("dual_pose_task");
  ASSERT_EQ(pose_task.action_seq.size(), 1U);
  ASSERT_EQ(pose_task.action_seq.front().size(), 1U);
  const auto& pose = pose_task.action_seq.front().front().arm_param;
  EXPECT_EQ(pose.arm_id, "2");
  EXPECT_EQ(pose.frame_id, "base_link");
  EXPECT_EQ(pose.left_target_pose.size(), 7U);
  EXPECT_EQ(pose.right_target_pose.size(), 7U);
  EXPECT_FLOAT_EQ(pose.left_target_pose.at(1), 1.08F);
  EXPECT_FLOAT_EQ(pose.right_target_pose.at(1), -1.08F);
  EXPECT_EQ(pose.left_bound_size.size(), 3U);
  EXPECT_EQ(pose.right_bound_size.size(), 3U);
  EXPECT_EQ(pose.synchronization_mode, 0U);
}
