#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/spline.hpp>
#include <toppra/solver/seidel.hpp>

#include "motion_control_pkg/motion/toppra_path_preparation.hpp"

namespace
{

struct SolverResult
{
  bool valid{false};
  double elapsed_seconds{0.0};
  double duration_seconds{0.0};
  double maximum_segment_duration_seconds{0.0};
  double endpoint_path_derivative_norm{0.0};
  double maximum_waypoint_error{0.0};
  std::size_t sample_count{0U};
  std::size_t retained_waypoints{0U};
  std::size_t removed_waypoints{0U};
};

std::vector<std::vector<double>> makeMoveLineLikePath()
{
  constexpr std::size_t kWaypointCount = 51U;
  constexpr std::size_t kDof = 7U;
  std::vector<std::vector<double>> path;
  path.reserve(kWaypointCount);
  for (std::size_t index = 0U; index < kWaypointCount; ++index) {
    const double alpha = static_cast<double>(index) /
      static_cast<double>(kWaypointCount - 1U);
    const double curve = std::sin(alpha * 3.14159265358979323846);
    path.push_back({
      0.12 * alpha,
      -0.18 * alpha + 0.004 * curve,
      0.10 * alpha - 0.003 * curve,
      -0.15 * alpha,
      0.08 * alpha + 0.002 * curve,
      -0.06 * alpha,
      0.05 * alpha});
  }
  // Reproduce the numerical pathology class seen in sequential IK: one
  // adjacent waypoint carries no meaningful joint-space progress.
  for (std::size_t joint = 0U; joint < kDof; ++joint) {
    path[25U][joint] = path[24U][joint] + 1.0e-10;
  }
  return path;
}

SolverResult solveWithStableSeidel(
  const std::vector<std::vector<double>> & input)
{
  constexpr std::size_t kDof = 7U;
  SolverResult result;
  const auto prepared =
    motion_control_pkg::motion::prepareToppraWaypointPath(
      input, kDof,
      motion_control_pkg::motion::kToppraDuplicateWaypointToleranceRad,
      true);
  if (!prepared.valid) {
    return result;
  }
  result.retained_waypoints = prepared.waypoints.size();
  result.removed_waypoints = prepared.removed_waypoints;

  toppra::Vectors positions;
  positions.reserve(prepared.waypoints.size());
  for (const auto & waypoint : prepared.waypoints) {
    toppra::Vector point(static_cast<Eigen::Index>(kDof));
    for (std::size_t joint = 0U; joint < kDof; ++joint) {
      point(static_cast<Eigen::Index>(joint)) = waypoint[joint];
    }
    positions.push_back(std::move(point));
  }

  toppra::Vector breakpoints(
    static_cast<Eigen::Index>(prepared.breakpoints.size()));
  for (std::size_t index = 0U; index < prepared.breakpoints.size(); ++index) {
    breakpoints(static_cast<Eigen::Index>(index)) = prepared.breakpoints[index];
  }
  const std::array<toppra::BoundaryCond, 2> boundary_conditions{
    toppra::BoundaryCond(
      motion_control_pkg::motion::kToppraStableCartesianBoundaryCondition),
    toppra::BoundaryCond(
      motion_control_pkg::motion::kToppraStableCartesianBoundaryCondition)};
  auto path = std::make_shared<toppra::PiecewisePolyPath>(
    toppra::PiecewisePolyPath::CubicSpline(
      positions, breakpoints, boundary_conditions));
  result.endpoint_path_derivative_norm =
    path->eval_single(breakpoints(breakpoints.size() - 1), 1).norm();
  for (Eigen::Index index = 0; index < breakpoints.size(); ++index) {
    result.maximum_waypoint_error = std::max(
      result.maximum_waypoint_error,
      (path->eval_single(breakpoints(index), 0) - positions[index]).norm());
  }

  toppra::Vector velocity_lower =
    Eigen::VectorXd::Constant(static_cast<Eigen::Index>(kDof), -0.8);
  toppra::Vector velocity_upper =
    Eigen::VectorXd::Constant(static_cast<Eigen::Index>(kDof), 0.8);
  toppra::Vector acceleration_lower =
    Eigen::VectorXd::Constant(static_cast<Eigen::Index>(kDof), -1.5);
  toppra::Vector acceleration_upper =
    Eigen::VectorXd::Constant(static_cast<Eigen::Index>(kDof), 1.5);
  toppra::LinearConstraintPtrs constraints;
  constraints.emplace_back(
    std::make_shared<toppra::constraint::LinearJointVelocity>(
      velocity_lower, velocity_upper));
  constraints.emplace_back(
    std::make_shared<toppra::constraint::LinearJointAcceleration>(
      acceleration_lower, acceleration_upper));
  for (auto & constraint : constraints) {
    constraint->discretizationType(toppra::DiscretizationType::Interpolation);
  }

  toppra::algorithm::TOPPRA optimizer(constraints, path);
  optimizer.solver(std::make_shared<toppra::solver::Seidel>());
  const auto started_at = std::chrono::steady_clock::now();
  const auto return_code = optimizer.computePathParametrization();
  result.elapsed_seconds = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started_at).count();
  if (return_code != toppra::ReturnCode::OK) {
    return result;
  }

  const auto & data = optimizer.getParameterizationData();
  toppra::parametrizer::Spline parameterization(
    path, data.gridpoints, data.parametrization);
  if (!parameterization.validate()) {
    return result;
  }
  const toppra::Vector & times = parameterization.getTimes();
  if (times.size() == 0) {
    return result;
  }
  result.duration_seconds = times(times.size() - 1);
  if (!std::isfinite(result.duration_seconds) ||
    result.duration_seconds <= 0.0)
  {
    return result;
  }
  for (Eigen::Index index = 1; index < times.size(); ++index) {
    result.maximum_segment_duration_seconds = std::max(
      result.maximum_segment_duration_seconds,
      times(index) - times(index - 1));
  }
  result.sample_count = static_cast<std::size_t>(
    std::ceil(result.duration_seconds / 0.004));

  toppra::Vector sample_times(3);
  sample_times << 0.0, result.duration_seconds * 0.5, result.duration_seconds;
  const auto samples = parameterization.eval(sample_times, 0);
  if (samples.size() != 3U ||
    !samples.front().allFinite() || !samples.back().allFinite())
  {
    return result;
  }
  result.valid = true;
  return result;
}

TEST(ToppraStableSolverTest, SolvesForwardMoveLineLikePathWithoutStalling)
{
  const auto result = solveWithStableSeidel(makeMoveLineLikePath());
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.retained_waypoints, 50U);
  EXPECT_EQ(result.removed_waypoints, 1U);
  EXPECT_LT(result.elapsed_seconds, 5.0);
  EXPECT_LT(result.duration_seconds, 5.0);
  EXPECT_LT(result.maximum_segment_duration_seconds, 1.0);
  EXPECT_GT(result.endpoint_path_derivative_norm, 1.0e-6);
  EXPECT_LT(result.maximum_waypoint_error, 1.0e-10);
  EXPECT_LT(result.sample_count, 2000U);
  RecordProperty(
    "solve_elapsed_us",
    std::to_string(static_cast<long long>(result.elapsed_seconds * 1.0e6)));
}

TEST(ToppraStableSolverTest, SolvesReverseMoveLineLikePathWithoutStalling)
{
  auto path = makeMoveLineLikePath();
  std::reverse(path.begin(), path.end());
  const auto result = solveWithStableSeidel(path);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.retained_waypoints, 50U);
  EXPECT_EQ(result.removed_waypoints, 1U);
  EXPECT_LT(result.elapsed_seconds, 5.0);
  EXPECT_LT(result.duration_seconds, 5.0);
  EXPECT_LT(result.maximum_segment_duration_seconds, 1.0);
  EXPECT_GT(result.endpoint_path_derivative_norm, 1.0e-6);
  EXPECT_LT(result.maximum_waypoint_error, 1.0e-10);
  EXPECT_LT(result.sample_count, 2000U);
  RecordProperty(
    "solve_elapsed_us",
    std::to_string(static_cast<long long>(result.elapsed_seconds * 1.0e6)));
}

}  // namespace
