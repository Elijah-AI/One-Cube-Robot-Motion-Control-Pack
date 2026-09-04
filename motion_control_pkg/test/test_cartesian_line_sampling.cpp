#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "motion_control_pkg/motion/cartesian_line_sampling.hpp"
#include "motion_control_pkg/motion/toppra_path_preparation.hpp"

namespace
{

Eigen::Isometry3d makePose(
  const Eigen::Vector3d & translation,
  const Eigen::AngleAxisd & rotation = Eigen::AngleAxisd::Identity())
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = translation;
  pose.linear() = rotation.toRotationMatrix();
  return pose;
}

TEST(CartesianLineSamplingTest, UsesTranslationAndRotationLimits)
{
  const auto start = makePose(Eigen::Vector3d::Zero());
  const auto translation_goal = makePose(Eigen::Vector3d(0.0, 0.0, 0.25));
  const auto translation_result =
    motion_control_pkg::motion::computeCartesianLineSegmentCount(
      start, translation_goal, 0.005, 0.02);
  ASSERT_TRUE(translation_result.valid);
  EXPECT_EQ(translation_result.segments, 50U);

  const auto rotation_goal = makePose(
    Eigen::Vector3d(0.0, 0.0, 0.01),
    Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitZ()));
  const auto rotation_result =
    motion_control_pkg::motion::computeCartesianLineSegmentCount(
      start, rotation_goal, 0.005, 0.02);
  ASSERT_TRUE(rotation_result.valid);
  EXPECT_EQ(rotation_result.segments, 5U);
}

TEST(CartesianLineSamplingTest, RejectsInvalidLimitsAndExcessiveSegments)
{
  const auto start = makePose(Eigen::Vector3d::Zero());
  const auto goal = makePose(Eigen::Vector3d(0.0, 0.0, 0.25));
  EXPECT_FALSE(
    motion_control_pkg::motion::computeCartesianLineSegmentCount(
      start, goal, 0.0, 0.02).valid);
  EXPECT_FALSE(
    motion_control_pkg::motion::computeCartesianLineSegmentCount(
      start, goal, 0.001, 0.02, 10U).valid);
}

TEST(CartesianLineSamplingTest, MeasuresLineAndOrientationDeviation)
{
  const auto start = makePose(Eigen::Vector3d::Zero());
  const auto goal = makePose(
    Eigen::Vector3d(0.0, 0.0, 0.25),
    Eigen::AngleAxisd(0.20, Eigen::Vector3d::UnitZ()));
  const auto on_line = makePose(
    Eigen::Vector3d(0.0, 0.0, 0.125),
    Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitZ()));
  const auto on_line_error =
    motion_control_pkg::motion::evaluateCartesianLineSample(start, goal, on_line);
  ASSERT_TRUE(on_line_error.valid);
  EXPECT_NEAR(on_line_error.progress, 0.5, 1.0e-12);
  EXPECT_NEAR(on_line_error.position_error_m, 0.0, 1.0e-12);
  EXPECT_NEAR(on_line_error.orientation_error_rad, 0.0, 1.0e-12);

  const auto off_line = makePose(
    Eigen::Vector3d(0.002, 0.0, 0.125),
    Eigen::AngleAxisd(0.13, Eigen::Vector3d::UnitZ()));
  const auto off_line_error =
    motion_control_pkg::motion::evaluateCartesianLineSample(start, goal, off_line);
  ASSERT_TRUE(off_line_error.valid);
  EXPECT_NEAR(off_line_error.position_error_m, 0.002, 1.0e-12);
  EXPECT_NEAR(off_line_error.orientation_error_rad, 0.03, 1.0e-12);
}

TEST(CartesianLineSamplingTest, IncludesEndpointOvershootInPositionError)
{
  const auto start = makePose(Eigen::Vector3d::Zero());
  const auto goal = makePose(Eigen::Vector3d(0.0, 0.0, 0.25));
  const auto overshoot = makePose(Eigen::Vector3d(0.0, 0.0, 0.26));
  const auto error =
    motion_control_pkg::motion::evaluateCartesianLineSample(start, goal, overshoot);
  ASSERT_TRUE(error.valid);
  EXPECT_NEAR(error.progress, 1.04, 1.0e-12);
  EXPECT_NEAR(error.position_error_m, 0.01, 1.0e-12);
}

TEST(ToppraPathPreparationTest, RemovesAdjacentDuplicatesAndPreservesEndpoint)
{
  const std::vector<std::vector<double>> input{
    {0.0, 0.0},
    {0.1, 0.2},
    {0.1 + 1.0e-10, 0.2},
    {0.3, 0.4},
    {0.3 + 1.0e-10, 0.4}};
  const auto prepared =
    motion_control_pkg::motion::prepareToppraWaypointPath(
      input, 2U,
      motion_control_pkg::motion::kToppraDuplicateWaypointToleranceRad,
      true);
  ASSERT_TRUE(prepared.valid) << prepared.error_message;
  ASSERT_EQ(prepared.waypoints.size(), 3U);
  EXPECT_EQ(prepared.removed_waypoints, 2U);
  EXPECT_DOUBLE_EQ(prepared.waypoints.back()[0], input.back()[0]);
  EXPECT_DOUBLE_EQ(prepared.breakpoints.front(), 0.0);
  EXPECT_DOUBLE_EQ(prepared.breakpoints[1], 0.5);
  EXPECT_DOUBLE_EQ(prepared.breakpoints.back(), 1.0);
}

TEST(ToppraPathPreparationTest, ReversePathHasStrictUniformBreakpoints)
{
  const std::vector<std::vector<double>> upward{
    {0.0, -0.2}, {0.01, -0.19}, {0.03, -0.15}, {0.08, -0.1}};
  auto downward = upward;
  std::reverse(downward.begin(), downward.end());
  const auto up = motion_control_pkg::motion::prepareToppraWaypointPath(
    upward, 2U, 1.0e-7, true);
  const auto down = motion_control_pkg::motion::prepareToppraWaypointPath(
    downward, 2U, 1.0e-7, true);
  ASSERT_TRUE(up.valid) << up.error_message;
  ASSERT_TRUE(down.valid) << down.error_message;
  ASSERT_EQ(up.breakpoints.size(), down.breakpoints.size());
  for (std::size_t index = 1U; index < up.breakpoints.size(); ++index) {
    EXPECT_GT(up.breakpoints[index], up.breakpoints[index - 1U]);
    EXPECT_DOUBLE_EQ(up.breakpoints[index], down.breakpoints[index]);
  }
}

TEST(ToppraPathPreparationTest, RejectsNonFiniteAndCollapsedPaths)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    motion_control_pkg::motion::prepareToppraWaypointPath(
      {{0.0, 0.0}, {nan, 0.1}}, 2U).valid);
  EXPECT_FALSE(
    motion_control_pkg::motion::prepareToppraWaypointPath(
      {{0.0, 0.0}, {1.0e-10, 0.0}}, 2U, 1.0e-7, true).valid);
}

}  // namespace
