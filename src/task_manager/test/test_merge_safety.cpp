#include <gtest/gtest.h>
#include "task_manager/safety_task_types.hpp"
#include "com_interfaces/action/move_pose.hpp"
#include "com_interfaces/action/move_joints.hpp"
#include <type_traits>

TEST(MergeSafety, NormalHeartbeatCannotClearOtherStopSource) {
  EXPECT_EQ(task_mgr::effective_safety_status(0, 2), 2);
  EXPECT_EQ(task_mgr::effective_safety_status(2, 0), 2);
  EXPECT_EQ(task_mgr::effective_safety_status(1, 0), 1);
  EXPECT_EQ(task_mgr::effective_safety_status(0, 1), 1);
  EXPECT_EQ(task_mgr::effective_safety_status(1, 2), 2);
  EXPECT_EQ(task_mgr::effective_safety_status(0, 0), 0);
  EXPECT_EQ(task_mgr::effective_safety_status(-1, 0), 2);
}
TEST(MergeActionContract, CollisionModeTwoIsNotTruncatedToBool) {
  com_interfaces::action::MovePose::Goal pose;
  com_interfaces::action::MoveJoints::Goal joints;
  static_assert(std::is_same_v<decltype(pose.enable_collision_check), uint8_t>);
  static_assert(std::is_same_v<decltype(joints.enable_collision_check), uint8_t>);
  EXPECT_EQ(pose.enable_collision_check, 0);
  pose.enable_collision_check = decltype(pose)::COLLISION_SELF_AND_ENVIRONMENT;
  joints.enable_collision_check = decltype(joints)::COLLISION_SELF_AND_ENVIRONMENT;
  EXPECT_EQ(pose.enable_collision_check, 2);
  EXPECT_EQ(joints.enable_collision_check, 2);
}
