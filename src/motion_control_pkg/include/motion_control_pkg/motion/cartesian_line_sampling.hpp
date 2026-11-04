#ifndef MOTION_CONTROL_PKG_MOTION_CARTESIAN_LINE_SAMPLING_HPP_
#define MOTION_CONTROL_PKG_MOTION_CARTESIAN_LINE_SAMPLING_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include <Eigen/Geometry>

namespace motion_control_pkg::motion
{

inline constexpr double kCartesianIkTranslationStepMeters = 0.005;
inline constexpr double kCartesianIkRotationStepRadians = 0.02;
inline constexpr double kCartesianLinePositionToleranceMeters = 0.0015;
inline constexpr double kCartesianLineOrientationToleranceRadians = 0.02;
inline constexpr std::size_t kMaxCartesianIkSegments = 4096U;

// Cartesian MoveL means a straight translation plus shortest-arc rotation.
// SE3 exponential interpolation describes a screw and is not generally straight.
inline Eigen::Isometry3d interpolateCartesianLine(
  const Eigen::Isometry3d& start, const Eigen::Isometry3d& goal, double progress)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = start.translation() + progress*(goal.translation()-start.translation());
  result.linear() = Eigen::Quaterniond(start.rotation()).normalized().slerp(
    progress, Eigen::Quaterniond(goal.rotation()).normalized()).normalized().toRotationMatrix();
  return result;
}

struct CartesianLineSegmentCount
{
  bool valid{false};
  std::size_t segments{0U};
  double translation_distance_m{0.0};
  double rotation_distance_rad{0.0};
  std::string error_message;
};

inline double quaternionAngularDistance(
  const Eigen::Quaterniond & lhs,
  const Eigen::Quaterniond & rhs)
{
  Eigen::Quaterniond lhs_normalized = lhs.normalized();
  Eigen::Quaterniond rhs_normalized = rhs.normalized();
  const double dot = std::clamp(
    std::abs(lhs_normalized.dot(rhs_normalized)), 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

inline CartesianLineSegmentCount computeCartesianLineSegmentCount(
  const Eigen::Isometry3d & start,
  const Eigen::Isometry3d & goal,
  double max_translation_step_m,
  double max_rotation_step_rad,
  std::size_t max_segments = kMaxCartesianIkSegments)
{
  CartesianLineSegmentCount result;
  if (!start.matrix().allFinite() || !goal.matrix().allFinite()) {
    result.error_message = "Cartesian line start or goal contains non-finite values";
    return result;
  }
  if (!std::isfinite(max_translation_step_m) || max_translation_step_m <= 0.0 ||
    !std::isfinite(max_rotation_step_rad) || max_rotation_step_rad <= 0.0 ||
    max_segments == 0U)
  {
    result.error_message = "Cartesian line sampling limits must be finite and positive";
    return result;
  }

  result.translation_distance_m =
    (goal.translation() - start.translation()).norm();
  result.rotation_distance_rad = quaternionAngularDistance(
    Eigen::Quaterniond(start.rotation()), Eigen::Quaterniond(goal.rotation()));
  if (!std::isfinite(result.translation_distance_m) ||
    !std::isfinite(result.rotation_distance_rad))
  {
    result.error_message = "Cartesian line distance is not finite";
    return result;
  }

  constexpr double kCeilTolerance = 1.0e-12;
  const double translation_segments = std::ceil(
    result.translation_distance_m / max_translation_step_m - kCeilTolerance);
  const double rotation_segments = std::ceil(
    result.rotation_distance_rad / max_rotation_step_rad - kCeilTolerance);
  const double required_segments = std::max(
    1.0, std::max(translation_segments, rotation_segments));
  if (required_segments > static_cast<double>(max_segments)) {
    result.error_message = "Cartesian line requires too many IK segments";
    return result;
  }

  result.valid = true;
  result.segments = static_cast<std::size_t>(required_segments);
  return result;
}

struct CartesianLineSampleError
{
  bool valid{false};
  double progress{0.0};
  double position_error_m{std::numeric_limits<double>::infinity()};
  double orientation_error_rad{std::numeric_limits<double>::infinity()};
  std::string error_message;
};

inline CartesianLineSampleError evaluateCartesianLineSample(
  const Eigen::Isometry3d & start,
  const Eigen::Isometry3d & goal,
  const Eigen::Isometry3d & sample)
{
  CartesianLineSampleError result;
  if (!start.matrix().allFinite() || !goal.matrix().allFinite() ||
    !sample.matrix().allFinite())
  {
    result.error_message = "Cartesian line validation pose contains non-finite values";
    return result;
  }

  const Eigen::Vector3d delta = goal.translation() - start.translation();
  const double squared_distance = delta.squaredNorm();
  if (!std::isfinite(squared_distance)) {
    result.error_message = "Cartesian line distance is not finite";
    return result;
  }
  if (squared_distance > 1.0e-12) {
    result.progress = (sample.translation() - start.translation()).dot(delta) / squared_distance;
  } else {
    // A pure rotation still has a signed path coordinate. Projection on the
    // shortest rotation axis detects backwards motion and wrong-axis rotations.
    const Eigen::AngleAxisd total(start.rotation().transpose() * goal.rotation());
    const Eigen::AngleAxisd actual(start.rotation().transpose() * sample.rotation());
    result.progress = total.angle() > 1.0e-10 ?
        actual.angle() * actual.axis().dot(total.axis()) / total.angle() : 0.0;
  }
  const double clamped_progress = std::clamp(result.progress, 0.0, 1.0);
  const Eigen::Vector3d expected_translation =
    start.translation() + clamped_progress * delta;
  result.position_error_m =
    (sample.translation() - expected_translation).norm();

  Eigen::Quaterniond start_orientation(start.rotation());
  Eigen::Quaterniond goal_orientation(goal.rotation());
  Eigen::Quaterniond sample_orientation(sample.rotation());
  start_orientation.normalize();
  goal_orientation.normalize();
  sample_orientation.normalize();
  const Eigen::Quaterniond expected_orientation =
    start_orientation.slerp(clamped_progress, goal_orientation).normalized();
  result.orientation_error_rad = quaternionAngularDistance(
    expected_orientation, sample_orientation);

  result.valid = std::isfinite(result.progress) &&
    std::isfinite(result.position_error_m) &&
    std::isfinite(result.orientation_error_rad);
  if (!result.valid) {
    result.error_message = "Cartesian line validation produced a non-finite error";
  }
  return result;
}

}  // namespace motion_control_pkg::motion

#endif  // MOTION_CONTROL_PKG_MOTION_CARTESIAN_LINE_SAMPLING_HPP_
