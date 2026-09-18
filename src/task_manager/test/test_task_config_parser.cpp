// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Description:
// 任务配置解析器单元测试。
// 验证"主级 + 次级"配置分级加载（load_all）：
//   - 主配置解析运行环境参数（node_id / modbus_slave）；
//   - tasks.* 通过 config_file 外链加载次级任务动作文件；
//   - boot_task 通过 config_file 外链加载启动任务（自检 / 复位）。

#include <gtest/gtest.h>
#include <string>
#include "task_manager/task_config_parser.hpp"

#ifndef SOURCE_DIR
#define SOURCE_DIR "."
#endif

using namespace task_cfg;

// 主配置文件路径（编译期注入源码目录）
static std::string main_config_path()
{
  return std::string(SOURCE_DIR) + "/config/task_config.yaml";
}

TEST(TaskConfigParserTest, LoadAllMainAndSecondary)
{
  GlobalConfig cfg;
  TaskConfigParser parser;
  ASSERT_TRUE(parser.load_all(main_config_path(), cfg));

  // ---- 运行环境参数 ----
  EXPECT_EQ(cfg.node_id, 1);
  EXPECT_EQ(cfg.modbus_slave_id, 1);
  EXPECT_EQ(cfg.modbus_port, 12000);
  EXPECT_EQ(cfg.modbus_listen_ip, "0.0.0.0");

  // ---- 合并任务及新增插槽任务（主配置外链 → 次级动作文件）----
  ASSERT_EQ(cfg.task_dict.size(), 11u);
  EXPECT_TRUE(cfg.task_dict.count("move_box") == 1);
  EXPECT_TRUE(cfg.task_dict.count("go_pick_pose") == 1);
  EXPECT_TRUE(cfg.task_dict.count("go_idle_pose") == 1);
  EXPECT_TRUE(cfg.task_dict.count("auto_calibrate") == 1);

  // ---- 寄存器映射 ----
  EXPECT_EQ(cfg.task_dict.at("move_box").trigger_reg, 40001u);
  EXPECT_EQ(cfg.task_dict.at("move_box").status_reg, 50001u);
  EXPECT_EQ(cfg.task_dict.at("go_pick_pose").trigger_reg, 40005u);
  EXPECT_EQ(cfg.task_dict.at("go_pick_pose").status_reg, 50005u);
  EXPECT_EQ(cfg.task_dict.at("go_idle_pose").trigger_reg, 40003u);
  EXPECT_EQ(cfg.task_dict.at("go_idle_pose").status_reg, 50003u);
  EXPECT_EQ(cfg.task_dict.at("auto_calibrate").trigger_reg, 40004u);
  EXPECT_EQ(cfg.task_dict.at("auto_calibrate").status_reg, 50004u);

  // ---- 级联加载：每个任务都通过 config_file 加载出动作序列 ----
  const auto& mb = cfg.task_dict.at("move_box");
  EXPECT_FALSE(mb.config_file.empty());
  EXPECT_EQ(mb.action_seq.size(), 1u);              // move_box: 1 个动作序列
  ASSERT_FALSE(mb.action_seq[0].empty());
  EXPECT_EQ(mb.action_seq[0].size(), 31u);           // HHworkspace actions and local parallel groups
  EXPECT_TRUE(mb.action_seq[0][0].is_parallel_group);
  EXPECT_EQ(cfg.task_dict.at("control_suck_example").trigger_reg, 40002u);
  EXPECT_EQ(cfg.task_dict.at("control_suck_example").status_reg, 50002u);

  const auto& gp = cfg.task_dict.at("go_pick_pose");
  EXPECT_FALSE(gp.config_file.empty());
  EXPECT_EQ(gp.action_seq.size(), 2u);              // go_pick_pose: 2 个动作序列
  ASSERT_FALSE(gp.action_seq[0].empty());
  ASSERT_FALSE(gp.action_seq[1].empty());
  EXPECT_EQ(gp.action_seq[0][0].action_type, "move_joints");

  const auto& gi = cfg.task_dict.at("go_idle_pose");
  EXPECT_FALSE(gi.config_file.empty());
  EXPECT_EQ(gi.action_seq.size(), 2u);

  // ---- auto_calibrate：自动标定（单动作 auto_calibrate + 示例位姿池）----
  const auto& ac = cfg.task_dict.at("auto_calibrate");
  EXPECT_FALSE(ac.config_file.empty());
  EXPECT_EQ(ac.action_seq.size(), 1u);
  ASSERT_FALSE(ac.action_seq[0].empty());
  EXPECT_EQ(ac.action_seq[0][0].action_type, "auto_calibrate");
  EXPECT_EQ(ac.action_seq[0][0].auto_calibrate_param.arm_id, 1);  // 默认右臂
  EXPECT_EQ(ac.action_seq[0][0].auto_calibrate_param.mode, "TO_HAND");
  EXPECT_EQ(ac.action_seq[0][0].auto_calibrate_param.sample_num, 12);
  EXPECT_GE(ac.action_seq[0][0].auto_calibrate_param.pose_pool.size(), 12u);
  EXPECT_EQ(ac.action_seq[0][0].auto_calibrate_param.pose_pool[0].size(), 7u);

  // ---- boot_task 外链：自检 / 复位 ----
  EXPECT_EQ(cfg.boot_tasks.self_check_seq.size(), 2u);
  EXPECT_EQ(cfg.boot_tasks.reset_seq.size(), 2u);
  ASSERT_FALSE(cfg.boot_tasks.self_check_seq.empty());
  EXPECT_EQ(cfg.boot_tasks.self_check_seq[0].action_type, "vision_check_self");
  EXPECT_EQ(cfg.boot_tasks.self_check_seq[1].action_type, "control_check_self");
  ASSERT_FALSE(cfg.boot_tasks.reset_seq.empty());
  EXPECT_EQ(cfg.boot_tasks.reset_seq[0].action_type, "vision_reset");
  EXPECT_EQ(cfg.boot_tasks.reset_seq[1].action_type, "control_reset");
}

#include <fstream>
#include <unistd.h>

namespace {
bool load_mode(const std::string& mode, task_cfg::GlobalConfig& cfg,
               const std::string& legacy_cloud = "") {
  char path[] = "/tmp/ylf_collision_parser_XXXXXX";
  const int fd = mkstemp(path);
  if (fd < 0) return false;
  close(fd);
  {
    std::ofstream out(path);
    out << "action_sequencenum: 1\naction_sequence1:\n";
    for (const auto& type : {"move_joints", "move_pose", "auto_calibrate"}) {
      out << "  - action_type: " << type << "\n    params:\n"
          << "      arm_id: 0\n      joint_name: [Joint1_L]\n      position: [0.0]\n"
          << "      enable_collision_check: " << mode << "\n"
          << "      target_pose: [0.2, 0.4, 0.8, 0, 0, 0, 1]\n"
          << "      bound_size: [0, 0, 0]\n      use_config_pose: true\n"
          << "      use_force_control: false\n      speed: 0.2\n      timeout_ms: 15000\n";
      if (!legacy_cloud.empty()) out << "      enable_point_cloud_check: " << legacy_cloud << "\n";
    }
  }
  TaskConfigParser parser;
  const bool ok = parser.load_task_config(path, cfg, "mode_test");
  std::remove(path);
  return ok;
}
}

TEST(TaskConfigParserTest, CollisionModesSurviveAllMotionParameters) {
  for (int mode = 0; mode <= 2; ++mode) {
    GlobalConfig cfg;
    ASSERT_TRUE(load_mode(std::to_string(mode), cfg, "true"));
    const auto& seq = cfg.task_dict.at("mode_test").action_seq.at(0);
    ASSERT_EQ(seq.size(), 3u);
    EXPECT_EQ(seq[0].move_joints_param.enable_collision_check, mode);
    EXPECT_EQ(seq[1].arm_param.enable_collision_check, mode);
    EXPECT_EQ(seq[2].auto_calibrate_param.enable_collision_check, mode);
  }
}

TEST(TaskConfigParserTest, LegacyBooleansAndInvalidModes) {
  for (const auto& entry : std::vector<std::pair<std::string, int>>{{"false",0},{"true",1}}) {
    GlobalConfig cfg;
    ASSERT_TRUE(load_mode(entry.first, cfg));
    EXPECT_EQ(cfg.task_dict.at("mode_test").action_seq[0][1].arm_param.enable_collision_check, entry.second);
  }
  GlobalConfig old_cloud;
  ASSERT_TRUE(load_mode("true", old_cloud, "true"));
  EXPECT_EQ(old_cloud.task_dict.at("mode_test").action_seq[0][1].arm_param.enable_collision_check, 2);
  for (const auto* invalid : {"-1", "3", "256", "1.5", "null", "[]", "abc"}) {
    GlobalConfig cfg;
    EXPECT_FALSE(load_mode(invalid, cfg)) << invalid;
  }
}

TEST(TaskConfigParserTest, KeepsLocalParallelGroupsAndCartesianLine) {
  GlobalConfig cfg;
  TaskConfigParser parser;
  ASSERT_TRUE(parser.load_all(std::string(SOURCE_DIR)+"/test/local_parallel_task_config.yaml", cfg));
  const auto& seq = cfg.task_dict.at("local_parallel_task").action_seq.at(0);
  ASSERT_EQ(seq.size(), 2u);
  ASSERT_TRUE(seq[0].is_parallel_group);
  ASSERT_EQ(seq[0].parallel_actions.size(), 2u);
  EXPECT_EQ(seq[0].parallel_actions[0].move_joints_param.arm_id, 4);
  EXPECT_EQ(seq[0].parallel_actions[1].move_joints_param.arm_id, 0);
  EXPECT_TRUE(seq[1].arm_param.enable_move_line);
}
