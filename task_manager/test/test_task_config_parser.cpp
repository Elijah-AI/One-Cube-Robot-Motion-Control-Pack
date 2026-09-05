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

TEST(TaskConfigParserTest, PreservesExternalTaskActionsAndParallelGroups)
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
  const auto source_steps = YAML::LoadFile(config_path)["tasks"]["move_box"]["action_sequence1"];
  ASSERT_EQ(steps.size(), source_steps.size());
  const auto check_action = [](const task_cfg::AtomicAction& action, const YAML::Node& source) {
    EXPECT_EQ(action.action_type, source["action_type"].as<std::string>());
    const auto params = source["params"];
    if (action.action_type == "move_joints") {
      EXPECT_EQ(action.move_joints_param.arm_id, params["arm_id"].as<unsigned>());
      EXPECT_EQ(action.move_joints_param.position, params["position"].as<std::vector<double>>());
    } else if (action.action_type == "move_pose") {
      EXPECT_EQ(action.arm_param.arm_id, params["arm_id"].as<unsigned>());
      EXPECT_EQ(action.arm_param.config_pose, params["target_pose"].as<std::vector<float>>());
      EXPECT_EQ(action.arm_param.enable_move_line, params["enable_move_line"].as<bool>(false));
    } else if (action.action_type == "gripper_ctrl") {
      EXPECT_EQ(action.gripper_param.gripper_id, params["gripper_id"].as<unsigned>());
      EXPECT_EQ(action.gripper_param.position, params["position"].as<std::vector<uint8_t>>());
    }
  };
  size_t parallel_group_count = 0U;
  for (size_t index = 0; index < steps.size(); ++index)
  {
    const auto& step = steps[index];
    const auto source_group = source_steps[index]["parallel_actions"];
    EXPECT_EQ(step.is_parallel_group, source_group.IsDefined());
    if (source_group.IsDefined()) {
      ++parallel_group_count;
      ASSERT_EQ(step.parallel_actions.size(), source_group.size());
      for (size_t child = 0; child < source_group.size(); ++child) {
        check_action(step.parallel_actions[child], source_group[child]);
      }
    } else {
      check_action(step, source_steps[index]);
    }
  }
  EXPECT_GT(parallel_group_count, 0U);
}
