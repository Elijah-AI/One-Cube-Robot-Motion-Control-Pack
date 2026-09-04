#ifndef MOTION_CONTROL_PKG_MOTION_TOPPRA_PATH_PREPARATION_HPP_
#define MOTION_CONTROL_PKG_MOTION_TOPPRA_PATH_PREPARATION_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace motion_control_pkg::motion
{

inline constexpr double kToppraDuplicateWaypointToleranceRad = 1.0e-7;
inline constexpr char kToppraStableCartesianBoundaryCondition[] = "natural";

struct PreparedToppraPath
{
  bool valid{false};
  std::vector<std::vector<double>> waypoints;
  std::vector<double> breakpoints;
  std::size_t removed_waypoints{0U};
  double minimum_joint_step_rad{std::numeric_limits<double>::infinity()};
  double maximum_joint_step_rad{0.0};
  std::string error_message;
};

inline PreparedToppraPath prepareToppraWaypointPath(
  const std::vector<std::vector<double>> & input,
  std::size_t dof,
  double duplicate_tolerance_rad = kToppraDuplicateWaypointToleranceRad,
  bool uniform_breakpoints = false)
{
  PreparedToppraPath result;
  if (input.size() < 2U || dof == 0U ||
    !std::isfinite(duplicate_tolerance_rad) || duplicate_tolerance_rad < 0.0)
  {
    result.error_message = "TOPPRA path dimensions or duplicate tolerance are invalid";
    return result;
  }

  const auto validate_waypoint = [dof](const std::vector<double> & waypoint) {
      return waypoint.size() == dof && std::all_of(
        waypoint.begin(), waypoint.end(),
        [](double value) {return std::isfinite(value);});
    };
  if (!validate_waypoint(input.front())) {
    result.error_message = "TOPPRA path contains an invalid first waypoint";
    return result;
  }

  result.waypoints.reserve(input.size());
  result.waypoints.push_back(input.front());
  for (std::size_t waypoint_index = 1U;
    waypoint_index < input.size(); ++waypoint_index)
  {
    const auto & candidate = input[waypoint_index];
    if (!validate_waypoint(candidate)) {
      result.error_message =
        "TOPPRA path contains a non-finite or dimension-mismatched waypoint";
      result.waypoints.clear();
      return result;
    }

    double squared_distance = 0.0;
    for (std::size_t joint = 0U; joint < dof; ++joint) {
      const double delta = candidate[joint] - result.waypoints.back()[joint];
      squared_distance += delta * delta;
    }
    const double distance = std::sqrt(squared_distance);
    if (!std::isfinite(distance)) {
      result.error_message = "TOPPRA path joint distance is not finite";
      result.waypoints.clear();
      return result;
    }
    if (distance <= duplicate_tolerance_rad) {
      ++result.removed_waypoints;
      // Keep the requested endpoint exact even when it is numerically
      // indistinguishable from the preceding waypoint.
      if (waypoint_index + 1U == input.size()) {
        result.waypoints.back() = candidate;
      }
      continue;
    }
    result.minimum_joint_step_rad =
      std::min(result.minimum_joint_step_rad, distance);
    result.maximum_joint_step_rad =
      std::max(result.maximum_joint_step_rad, distance);
    result.waypoints.push_back(candidate);
  }

  if (result.waypoints.size() < 2U) {
    result.error_message = "TOPPRA path has fewer than two distinct waypoints";
    result.waypoints.clear();
    return result;
  }

  result.breakpoints.resize(result.waypoints.size(), 0.0);
  if (uniform_breakpoints) {
    const double denominator =
      static_cast<double>(result.waypoints.size() - 1U);
    for (std::size_t index = 1U; index < result.breakpoints.size(); ++index) {
      result.breakpoints[index] = static_cast<double>(index) / denominator;
    }
  } else {
    for (std::size_t index = 1U; index < result.waypoints.size(); ++index) {
      double squared_distance = 0.0;
      for (std::size_t joint = 0U; joint < dof; ++joint) {
        const double delta =
          result.waypoints[index][joint] - result.waypoints[index - 1U][joint];
        squared_distance += delta * delta;
      }
      result.breakpoints[index] =
        result.breakpoints[index - 1U] + std::sqrt(squared_distance);
    }
  }

  for (std::size_t index = 1U; index < result.breakpoints.size(); ++index) {
    if (!std::isfinite(result.breakpoints[index]) ||
      result.breakpoints[index] <= result.breakpoints[index - 1U])
    {
      result.error_message =
        "TOPPRA path breakpoints are not finite and strictly increasing";
      result.waypoints.clear();
      result.breakpoints.clear();
      return result;
    }
  }

  result.valid = true;
  return result;
}

}  // namespace motion_control_pkg::motion

#endif  // MOTION_CONTROL_PKG_MOTION_TOPPRA_PATH_PREPARATION_HPP_
