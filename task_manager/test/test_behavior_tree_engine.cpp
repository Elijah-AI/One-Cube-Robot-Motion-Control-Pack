// Copyright (c) 2026 Robot Control System
// All rights reserved.

#include <gtest/gtest.h>

#include <cstddef>
#include <map>
#include <mutex>
#include <string>

#include "task_manager/behavior_tree_engine.hpp"
#include "task_manager/task_action_client.hpp"

namespace
{

TEST(ParallelNodeTest, WaitsForAlreadyRunningSiblingBeforeFailure)
{
  auto logger = rclcpp::get_logger("parallel_node_test");
  ParallelNode parallel("parallel", logger);

  int running_ticks = 0;
  parallel.addChild(std::make_shared<ActionNode>(
    "running_then_success",
    [&running_ticks]() {
      ++running_ticks;
      return running_ticks == 1 ? BTStatus::RUNNING : BTStatus::SUCCESS;
    },
    logger));
  parallel.addChild(std::make_shared<ActionNode>(
    "failure", []() { return BTStatus::FAILURE; }, logger));

  EXPECT_EQ(parallel.tick(), BTStatus::RUNNING);
  EXPECT_EQ(running_ticks, 1);
  EXPECT_EQ(parallel.tick(), BTStatus::FAILURE);
  EXPECT_EQ(running_ticks, 2);
}

TEST(ParallelNodeTest, DoesNotStartIdleSiblingAfterFailure)
{
  auto logger = rclcpp::get_logger("parallel_node_test");
  ParallelNode parallel("parallel", logger);

  int starts = 0;
  parallel.addChild(std::make_shared<ActionNode>(
    "failure", []() { return BTStatus::FAILURE; }, logger));
  parallel.addChild(std::make_shared<ActionNode>(
    "must_not_start",
    [&starts]() {
      ++starts;
      return BTStatus::RUNNING;
    },
    logger));

  EXPECT_EQ(parallel.tick(), BTStatus::FAILURE);
  EXPECT_EQ(starts, 0);
}

TEST(ParallelNodeTest, SucceedsOnlyAfterEveryBranchSucceeds)
{
  auto logger = rclcpp::get_logger("parallel_node_test");
  ParallelNode parallel("parallel", logger);

  int first_ticks = 0;
  int second_ticks = 0;
  parallel.addChild(std::make_shared<ActionNode>(
    "first",
    [&first_ticks]() {
      ++first_ticks;
      return first_ticks == 1 ? BTStatus::RUNNING : BTStatus::SUCCESS;
    },
    logger));
  parallel.addChild(std::make_shared<ActionNode>(
    "second",
    [&second_ticks]() {
      ++second_ticks;
      return second_ticks < 3 ? BTStatus::RUNNING : BTStatus::SUCCESS;
    },
    logger));

  EXPECT_EQ(parallel.tick(), BTStatus::RUNNING);
  EXPECT_EQ(parallel.tick(), BTStatus::RUNNING);
  EXPECT_EQ(parallel.tick(), BTStatus::SUCCESS);
  EXPECT_EQ(first_ticks, 2);
  EXPECT_EQ(second_ticks, 3);
}

TEST(GripperGoalTest, BuildsAllDynamicGoalArrays)
{
  task_cfg::GripperParam param{};
  param.gripper_id = 0U;
  param.position = {120U, 0U};
  param.speed = {30U, 30U};
  param.max_torque = {40U, 40U};
  param.com_port = {2U, 2U};
  param.block = true;
  param.timeout_ms = 10000;

  task_act::CtrlGripper::Goal goal;
  std::string error_message;
  const auto result = task_act::detail::build_gripper_goal(
    "manual_left_open", param, goal, error_message);

  ASSERT_EQ(result, task_act::ActionCallResult::SUCCESS) << error_message;
  EXPECT_EQ(goal.gripper_id, 0U);
  EXPECT_EQ(goal.position, param.position);
  EXPECT_EQ(goal.speed, param.speed);
  EXPECT_EQ(goal.max_torque, param.max_torque);
  EXPECT_EQ(goal.com_port, param.com_port);
  EXPECT_TRUE(goal.block_until_complete);
  EXPECT_EQ(goal.timeout_ms, 10000);
}

TEST(GripperGoalTest, RejectsAnyIncompleteDynamicArray)
{
  task_cfg::GripperParam valid_param{};
  valid_param.gripper_id = 0U;
  valid_param.position = {120U, 0U};
  valid_param.speed = {30U, 30U};
  valid_param.max_torque = {40U, 40U};
  valid_param.com_port = {2U, 2U};
  valid_param.block = true;
  valid_param.timeout_ms = 10000;

  for (std::size_t incomplete_field = 0U; incomplete_field < 4U; ++incomplete_field)
  {
    auto param = valid_param;
    if (incomplete_field == 0U) param.position.pop_back();
    if (incomplete_field == 1U) param.speed.pop_back();
    if (incomplete_field == 2U) param.max_torque.pop_back();
    if (incomplete_field == 3U) param.com_port.pop_back();

    task_act::CtrlGripper::Goal goal;
    std::string error_message;
    EXPECT_EQ(
      task_act::detail::build_gripper_goal(
        "left_open", param, goal, error_message),
      task_act::ActionCallResult::FAIL_PARAM_INVALID);
    EXPECT_FALSE(error_message.empty());
  }
}

TEST(ActionContextTest, CompletingOneBranchKeepsSiblingResources)
{
  std::map<std::string, task_act::ActionContext> contexts;
  std::mutex contexts_mutex;
  contexts["left"].state = task_act::ActionRunState::RUNNING;
  contexts["left"].acquired_resources = {task_act::ResourceType::LEFT_GRIPPER};
  contexts["waist"].state = task_act::ActionRunState::RUNNING;
  contexts["waist"].acquired_resources = {task_act::ResourceType::LOWER_BODY};

  const auto released = task_act::detail::complete_context(
    contexts, contexts_mutex, "left", task_act::ActionRunState::SUCCESS,
    task_act::ActionCallResult::SUCCESS);

  ASSERT_EQ(released.size(), 1U);
  EXPECT_EQ(released.front(), task_act::ResourceType::LEFT_GRIPPER);
  EXPECT_TRUE(contexts.at("left").acquired_resources.empty());
  EXPECT_EQ(contexts.at("waist").state, task_act::ActionRunState::RUNNING);
  ASSERT_EQ(contexts.at("waist").acquired_resources.size(), 1U);
  EXPECT_EQ(
    contexts.at("waist").acquired_resources.front(),
    task_act::ResourceType::LOWER_BODY);
}

}  // namespace
