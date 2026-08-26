#include <gtest/gtest.h>

#include <cstdlib>

#include "task_manager/task_config_parser.hpp"

#ifndef TASK_MANAGER_LOCAL_PARALLEL_CONFIG_PATH
#error "TASK_MANAGER_LOCAL_PARALLEL_CONFIG_PATH is not defined"
#endif

TEST(TaskConfigParserTest, LoadsLocalParallelGroupAndMoveLine)
{
  task_cfg::TaskConfigParser parser;
  task_cfg::GlobalConfig config;

  ASSERT_TRUE(parser.load_config(TASK_MANAGER_LOCAL_PARALLEL_CONFIG_PATH, config));
  const auto& task = config.task_dict.at("local_parallel_task");
  ASSERT_EQ(task.action_seq.size(), 1U);
  ASSERT_EQ(task.action_seq.front().size(), 2U);

  const auto& group = task.action_seq.front().at(0);
  EXPECT_TRUE(group.is_parallel_group);
  EXPECT_EQ(group.action_type, "parallel_actions");
  ASSERT_EQ(group.parallel_actions.size(), 2U);
  EXPECT_EQ(group.parallel_actions.at(0).move_joints_param.arm_id, 4U);
  EXPECT_EQ(group.parallel_actions.at(0).move_joints_param.position.size(), 4U);
  EXPECT_EQ(group.parallel_actions.at(1).move_joints_param.arm_id, 0U);
  EXPECT_EQ(group.parallel_actions.at(1).move_joints_param.position.size(), 7U);

  const auto& move_pose = task.action_seq.front().at(1);
  EXPECT_FALSE(move_pose.is_parallel_group);
  EXPECT_TRUE(move_pose.arm_param.enable_move_line);
}

TEST(TaskConfigParserTest, LoadsExternalFourPositionConfigWhenProvided)
{
  const char* config_path = std::getenv("TASK_MANAGER_REAL_CONFIG_PATH");
  if (config_path == nullptr)
  {
    GTEST_SKIP() << "TASK_MANAGER_REAL_CONFIG_PATH is not set";
  }

  task_cfg::TaskConfigParser parser;
  task_cfg::GlobalConfig config;
  ASSERT_TRUE(parser.load_config(config_path, config));

  const auto& steps = config.task_dict.at("move_box").action_seq.front();
  size_t parallel_group_count = 0U;
  for (const auto& step : steps)
  {
    if (!step.is_parallel_group)
    {
      continue;
    }
    ++parallel_group_count;
    ASSERT_EQ(step.parallel_actions.size(), 2U);
    EXPECT_EQ(step.parallel_actions.at(0).move_joints_param.arm_id, 4U);
    EXPECT_EQ(step.parallel_actions.at(1).move_joints_param.arm_id, 0U);
  }
  EXPECT_EQ(parallel_group_count, 3U);
}
