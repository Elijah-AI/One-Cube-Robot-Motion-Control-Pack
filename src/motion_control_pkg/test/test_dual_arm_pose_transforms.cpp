#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

#include "motion_control_pkg/kinematics/transform_utils.hpp"

namespace
{

geometry_msgs::msg::TransformStamped makeTransform(
  const std::string & parent, const std::string & child, double x)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.translation.x = x;
  transform.transform.rotation.w = 1.0;
  return transform;
}

geometry_msgs::msg::PoseStamped makePose(
  const std::string & frame, double x)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.orientation.w = 2.0;  // 验证入口同时执行四元数归一化。
  return pose;
}

TEST(DualArmPoseTransformTest, TransformsLeftAndRightTargetsIndependently)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setUsingDedicatedThread(true);
  ASSERT_TRUE(buffer.setTransform(
      makeTransform("base_link", "left_camera", 1.0), "test", true));
  ASSERT_TRUE(buffer.setTransform(
      makeTransform("base_link", "right_camera", -1.0), "test", true));
  motion_control_pkg::kinematics::PoseFrameTransformer transformer(
    buffer, "base_link", 0.01);

  const auto left = transformer.transformToPlanningFrame(
    makePose("left_camera", 0.2));
  const auto right = transformer.transformToPlanningFrame(
    makePose("right_camera", 0.3));

  ASSERT_TRUE(left.success) << left.error_message;
  ASSERT_TRUE(right.success) << right.error_message;
  EXPECT_EQ(left.pose.header.frame_id, "base_link");
  EXPECT_EQ(right.pose.header.frame_id, "base_link");
  EXPECT_NEAR(left.pose.pose.position.x, 1.2, 1.0e-12);
  EXPECT_NEAR(right.pose.pose.position.x, -0.7, 1.0e-12);
  EXPECT_NEAR(left.pose.pose.orientation.w, 1.0, 1.0e-12);
  EXPECT_NEAR(right.pose.pose.orientation.w, 1.0, 1.0e-12);
}

TEST(DualArmPoseTransformTest, MissingTransformFailsClosed)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setUsingDedicatedThread(true);
  motion_control_pkg::kinematics::PoseFrameTransformer transformer(
    buffer, "base_link", 0.001);

  const auto result = transformer.transformToPlanningFrame(
    makePose("missing_frame", 0.0));

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error_message.find("TF transform"), std::string::npos);
}

TEST(DualArmPoseTransformTest, TcpOffsetDirectionRoundTrips)
{
  geometry_msgs::msg::Pose tcp_pose;
  tcp_pose.position.x = 1.0;
  tcp_pose.orientation.w = 1.0;
  Eigen::Isometry3d end_to_tcp = Eigen::Isometry3d::Identity();
  end_to_tcp.translation().x() = 0.2;

  const auto planning_to_end =
    motion_control_pkg::kinematics::targetTcpToModelEndPose(
    tcp_pose, end_to_tcp);
  const auto reconstructed =
    motion_control_pkg::kinematics::composeToolTcpPose(
    planning_to_end.matrix(), end_to_tcp);

  EXPECT_NEAR(planning_to_end.translation().x(), 0.8, 1.0e-12);
  EXPECT_NEAR(reconstructed.position.x, 1.0, 1.0e-12);
}

}  // namespace
