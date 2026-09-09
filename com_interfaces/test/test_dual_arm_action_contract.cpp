#include <gtest/gtest.h>

#include <cstddef>
#include <tuple>
#include <utility>

#include "com_interfaces/action/move_joints.hpp"
#include "com_interfaces/action/move_pose.hpp"

namespace
{

using MoveJoints = com_interfaces::action::MoveJoints;
using MovePose = com_interfaces::action::MovePose;

using LeftJointNames = decltype(
  std::declval<MoveJoints::Goal>().left_joint_name);
using RightJointNames = decltype(
  std::declval<MoveJoints::Goal>().right_joint_name);
using LeftJointPositions = decltype(
  std::declval<MoveJoints::Goal>().left_position);
using RightJointPositions = decltype(
  std::declval<MoveJoints::Goal>().right_position);
using LeftBoundSize = decltype(
  std::declval<MovePose::Goal>().left_bound_size);
using RightBoundSize = decltype(
  std::declval<MovePose::Goal>().right_bound_size);

static_assert(std::tuple_size<LeftJointNames>::value == 7U);
static_assert(std::tuple_size<RightJointNames>::value == 7U);
static_assert(std::tuple_size<LeftJointPositions>::value == 7U);
static_assert(std::tuple_size<RightJointPositions>::value == 7U);
static_assert(std::tuple_size<LeftBoundSize>::value == 3U);
static_assert(std::tuple_size<RightBoundSize>::value == 3U);

TEST(MergedArmActionContractTest, ArmBothAndSafeSynchronizationModeAreStable)
{
  const MoveJoints::Goal joint_goal;
  const MovePose::Goal pose_goal;

  EXPECT_EQ(MoveJoints::Goal::ARM_BOTH, 2U);
  EXPECT_EQ(MovePose::Goal::ARM_BOTH, 2U);
  EXPECT_EQ(MoveJoints::Goal::SYNC_START_AND_FINISH, 0U);
  EXPECT_EQ(MovePose::Goal::SYNC_START_AND_FINISH, 0U);
  EXPECT_EQ(
    joint_goal.synchronization_mode,
    MoveJoints::Goal::SYNC_START_AND_FINISH);
  EXPECT_EQ(
    pose_goal.synchronization_mode,
    MovePose::Goal::SYNC_START_AND_FINISH);
}

TEST(MergedArmActionContractTest, FixedSizeJointTargetsStartEmptyAndZeroed)
{
  const MoveJoints::Goal goal;

  for (std::size_t index = 0; index < goal.left_joint_name.size(); ++index) {
    EXPECT_TRUE(goal.left_joint_name[index].empty());
    EXPECT_TRUE(goal.right_joint_name[index].empty());
    EXPECT_DOUBLE_EQ(goal.left_position[index], 0.0);
    EXPECT_DOUBLE_EQ(goal.right_position[index], 0.0);
  }
}

TEST(MergedArmActionContractTest, FeedbackTracksBothArmsSeparately)
{
  MoveJoints::Feedback feedback;
  feedback.dual_task_status = MoveJoints::Feedback::STATUS_EXECUTING;
  feedback.progress_rate = 50.0F;
  feedback.left_progress_rate = 60.0F;
  feedback.right_progress_rate = 40.0F;

  EXPECT_EQ(
    feedback.dual_task_status,
    MoveJoints::Feedback::STATUS_EXECUTING);
  EXPECT_FLOAT_EQ(feedback.progress_rate, 50.0F);
  EXPECT_FLOAT_EQ(feedback.left_progress_rate, 60.0F);
  EXPECT_FLOAT_EQ(feedback.right_progress_rate, 40.0F);
}

TEST(MergedArmActionContractTest, ResultsExposePerArmErrorsAndTimeSkew)
{
  MovePose::Result result;
  result.left_final_distance_error_m = 0.001;
  result.right_final_distance_error_m = 0.002;
  result.completion_time_skew_sec = 0.004;

  EXPECT_DOUBLE_EQ(result.left_final_distance_error_m, 0.001);
  EXPECT_DOUBLE_EQ(result.right_final_distance_error_m, 0.002);
  EXPECT_DOUBLE_EQ(result.completion_time_skew_sec, 0.004);
}

TEST(MergedArmActionContractTest, ResultsExposeValidatedPhaseTimings)
{
  MoveJoints::Result joint_result;
  joint_result.timing_breakdown_valid = true;
  joint_result.ik_time = 0.0;
  joint_result.planning_time = 0.012;
  joint_result.motion_time = 1.25;

  MovePose::Result pose_result;
  pose_result.timing_breakdown_valid = true;
  pose_result.ik_time = 0.034;
  pose_result.planning_time = 0.056;
  pose_result.motion_time = 1.75;

  EXPECT_TRUE(joint_result.timing_breakdown_valid);
  EXPECT_DOUBLE_EQ(joint_result.ik_time, 0.0);
  EXPECT_DOUBLE_EQ(joint_result.planning_time, 0.012);
  EXPECT_DOUBLE_EQ(joint_result.motion_time, 1.25);
  EXPECT_TRUE(pose_result.timing_breakdown_valid);
  EXPECT_DOUBLE_EQ(pose_result.ik_time, 0.034);
  EXPECT_DOUBLE_EQ(pose_result.planning_time, 0.056);
  EXPECT_DOUBLE_EQ(pose_result.motion_time, 1.75);
}

TEST(MergedArmActionContractTest, DualPoseRequiresTwoExplicitTargets)
{
  MovePose::Goal goal;
  goal.left_target_pose.header.frame_id = "left_sensor";
  goal.right_target_pose.header.frame_id = "right_sensor";

  EXPECT_NE(
    goal.left_target_pose.header.frame_id,
    goal.right_target_pose.header.frame_id);
  for (std::size_t index = 0; index < goal.left_bound_size.size(); ++index) {
    EXPECT_FLOAT_EQ(goal.left_bound_size[index], 0.0F);
    EXPECT_FLOAT_EQ(goal.right_bound_size[index], 0.0F);
  }
}

}  // namespace
