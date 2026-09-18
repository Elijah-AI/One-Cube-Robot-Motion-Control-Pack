#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <limits>
#include <unistd.h>
#include "task_manager/vision_pose_utils.hpp"

namespace
{
task_cfg::MovePoseParam slot_param(int arm, double height)
{
  task_cfg::MovePoseParam param{};
  param.arm_id = arm;
  param.frame_id = "base_link";
  param.vision_pose_offset.enabled = true;
  param.vision_pose_offset.translation = {0.204, arm == 1 ? 0.017 : 0.0665, height};
  param.vision_pose_offset.rotation = {0.707107, arm == 1 ? 0.707107 : -0.707107, 0, 0};
  return param;
}

geometry_msgs::msg::PoseStamped corner_pose()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = -0.2;
  pose.pose.position.z = 0.5;
  pose.pose.orientation.w = 1;
  return pose;
}

bool load_yaml(const YAML::Node& root, task_cfg::GlobalConfig& cfg)
{
  char path[] = "/tmp/ylf_slot_parser_XXXXXX";
  const int fd = mkstemp(path);
  if (fd < 0) {return false;}
  close(fd);
  {std::ofstream out(path); out << root;}
  task_cfg::TaskConfigParser parser;
  const bool ok = parser.load_task_config(path, cfg, "slot_test");
  std::remove(path);
  return ok;
}
}  // namespace

TEST(VisionPoseOffset, UsesBaseAxesAndPostMultipliesLocalRotation)
{
  auto corner = corner_pose();
  corner.pose.orientation.z = std::sqrt(0.5);
  corner.pose.orientation.w = std::sqrt(0.5);  // corner rotated +90 degrees about base Z
  std::string error;
  geometry_msgs::msg::PoseStamped target;
  ASSERT_TRUE(task_act::offset_vision_pose(corner, slot_param(1, 0.25), target, error)) << error;
  EXPECT_NEAR(target.pose.position.x, 0.504, 1e-12);  // translation must NOT rotate
  EXPECT_NEAR(target.pose.position.y, -0.183, 1e-12);
  EXPECT_NEAR(target.pose.position.z, 0.75, 1e-12);
  EXPECT_NEAR(target.pose.orientation.x, 0, 1e-12);
  EXPECT_NEAR(target.pose.orientation.y, 1, 1e-12);
  EXPECT_NEAR(target.pose.orientation.z, 0, 1e-12);
  EXPECT_NEAR(target.pose.orientation.w, 0, 1e-12);
  ASSERT_TRUE(task_act::offset_vision_pose(corner, slot_param(0, 0.10), target, error)) << error;
  EXPECT_NEAR(target.pose.position.y, -0.1335, 1e-12);
  EXPECT_NEAR(target.pose.orientation.x, 1, 1e-12);
  EXPECT_NEAR(target.pose.orientation.y, 0, 1e-12);
}

TEST(VisionPoseOffset, NormalizesRoundedQuaternionAndDoesNotAccumulate)
{
  auto corner = corner_pose();
  corner.pose.orientation.w = 2;
  std::string error;
  geometry_msgs::msg::PoseStamped target;
  for (double height : {0.25, 0.10, 0.25}) {
    ASSERT_TRUE(task_act::offset_vision_pose(corner, slot_param(1, height), target, error));
    EXPECT_NEAR(target.pose.position.z, 0.5 + height, 1e-12);
    EXPECT_NEAR(target.pose.position.x, 0.504, 1e-12);
    EXPECT_NEAR(target.pose.orientation.x, std::sqrt(0.5), 1e-12);
    EXPECT_NEAR(target.pose.orientation.y, std::sqrt(0.5), 1e-12);
  }
  EXPECT_DOUBLE_EQ(corner.pose.position.x, 0.3);
  EXPECT_DOUBLE_EQ(corner.pose.position.z, 0.5);
  EXPECT_DOUBLE_EQ(corner.pose.orientation.w, 2);
}

TEST(VisionPoseOffset, RejectsWrongFrameAndInvalidPose)
{
  std::string error;
  geometry_msgs::msg::PoseStamped target;
  auto corner = corner_pose();
  corner.header.frame_id = "gripper_link";
  EXPECT_FALSE(task_act::offset_vision_pose(corner, slot_param(1, 0.25), target, error));
  corner = corner_pose();
  corner.pose.orientation.w = 0;
  EXPECT_FALSE(task_act::offset_vision_pose(corner, slot_param(1, 0.25), target, error));
  corner = corner_pose();
  corner.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(task_act::offset_vision_pose(corner, slot_param(1, 0.25), target, error));
}

TEST(VisionPoseOffset, ChecksActualTargetFrameAndSingleCornerBeforeMotion)
{
  task_cfg::VisionActionParam param{};
  param.expected_frame_id = "base_link";
  param.require_expected_frame = true;
  param.require_single_target = true;
  com_interfaces::action::DetectionPoseCloud::Result result;
  result.success = true;
  result.header.frame_id = "base_link";
  std::string error;
  EXPECT_FALSE(task_act::validate_vision_result(result, param, error));
  result.detected_targets.resize(1);
  result.detected_targets[0].pose = corner_pose();
  EXPECT_TRUE(task_act::validate_vision_result(result, param, error));
  result.detected_targets[0].pose.header.frame_id = "gripper_link";
  EXPECT_FALSE(task_act::validate_vision_result(result, param, error));
  result.detected_targets[0].pose = corner_pose();
  result.detected_targets.resize(2, result.detected_targets.front());
  EXPECT_FALSE(task_act::validate_vision_result(result, param, error));
  // Existing tasks keep their optional frame/count semantics.
  param.require_expected_frame = false;
  param.require_single_target = false;
  EXPECT_TRUE(task_act::validate_vision_result(result, param, error));
}

TEST(SlotTask, PreservesMeasuredPregraspAndSerialRightThenLeftSequence)
{
  task_cfg::GlobalConfig cfg;
  task_cfg::TaskConfigParser parser;
  ASSERT_TRUE(parser.load_all(std::string(SOURCE_DIR) + "/config/task_config.yaml", cfg));
  const auto& task = cfg.task_dict.at("slot_box_right_1_left_3");
  EXPECT_EQ(task.trigger_reg, 40009u);
  EXPECT_EQ(task.status_reg, 50009u);
  ASSERT_EQ(task.action_seq.size(), 1u);
  const auto& seq = task.action_seq[0];
  ASSERT_EQ(seq.size(), 12u);
  EXPECT_EQ(seq[0].action_type, "vision_detect");
  EXPECT_TRUE(seq[0].vision_param.require_single_target);
  EXPECT_TRUE(seq[0].vision_param.require_expected_frame);
  EXPECT_EQ(seq[1].move_joints_param.arm_id, 2);
  const auto& baseline = cfg.task_dict.at("move_box").action_seq[0][0].parallel_actions;
  EXPECT_EQ(seq[1].move_joints_param.left_position, baseline[1].move_joints_param.position);
  EXPECT_EQ(seq[1].move_joints_param.right_position, baseline[2].move_joints_param.position);
  for (size_t start : {2u, 7u}) {
    const int arm = start == 2 ? 1 : 0;
    for (size_t index : {start, start + 1, start + 3}) {
      EXPECT_FALSE(seq[index].is_parallel_group);
      EXPECT_EQ(seq[index].action_type, "move_pose");
      const auto& p = seq[index].arm_param;
      EXPECT_EQ(p.arm_id, arm);
      EXPECT_FALSE(p.enable_move_line);
      EXPECT_FALSE(p.use_config_pose);
      EXPECT_EQ(p.enable_collision_check, 1);
      EXPECT_TRUE(p.vision_pose_offset.enabled);
      EXPECT_EQ(p.vision_pose_offset.translation,
        slot_param(arm, index == start+1 ? 0.10 : 0.25).vision_pose_offset.translation);
      EXPECT_EQ(p.vision_pose_offset.rotation, slot_param(arm, 0.25).vision_pose_offset.rotation);
    }
    EXPECT_EQ(seq[start+2].action_type, "gripper_ctrl");
    EXPECT_EQ(seq[start+2].gripper_param.gripper_id, arm);
    EXPECT_EQ(seq[start+2].gripper_param.position.at(arm), 0);
    EXPECT_TRUE(seq[start+2].gripper_param.block);
    EXPECT_EQ(seq[start+4].action_type, "move_joints");
    EXPECT_EQ(seq[start+4].move_joints_param.arm_id, arm);
    EXPECT_EQ(seq[start+4].move_joints_param.position,
      arm == 1 ? seq[1].move_joints_param.right_position : seq[1].move_joints_param.left_position);
  }
}

TEST(SlotTask, RejectsAmbiguousOrMalformedOffsetConfiguration)
{
  const auto source = YAML::LoadFile(std::string(SOURCE_DIR) +
    "/config/tasks/slot_box_right_1_left_3.yaml");
  for (const auto* field : {"use_config_pose", "arm_id", "frame_id", "bound_size",
                           "translation", "rotation", "zero_rotation", "nan_translation"}) {
    YAML::Node root = YAML::Clone(source);
    auto p = root["action_sequence1"][2]["params"];
    const std::string key(field);
    if (key == "use_config_pose") {p[key] = true;}
    else if (key == "arm_id") {p[key] = 2;}
    else if (key == "frame_id") {p[key] = "";}
    else if (key == "bound_size") {p[key] = YAML::Load("[0.1, 0, 0]");}
    else if (key == "zero_rotation") {p["vision_pose_offset"]["rotation"] = YAML::Load("[0,0,0,0]");}
    else if (key == "nan_translation") {p["vision_pose_offset"]["translation"] = YAML::Load("[.nan,0,0]");}
    else {p["vision_pose_offset"][key] = YAML::Load("[0,0]");}
    task_cfg::GlobalConfig cfg;
    EXPECT_FALSE(load_yaml(root, cfg)) << field;
  }
}
