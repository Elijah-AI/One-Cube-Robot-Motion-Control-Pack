#pragma once

#include "motion_control_pkg/motion/leg_body_orientation.hpp"

namespace motion_control_pkg::motion {
enum class LegBodyMotionMode : unsigned char { Tilt = 1, RelativeHeight = 2, AbsoluteHeight = 3 };

struct LegBodyMotionTrajectory {
  LegOrientationTrajectory joints;
  Eigen::Isometry3d start_body{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d target_body{Eigen::Isometry3d::Identity()};
  bool vertical{false};
  double max_position_error_m{0};
};

// URDF FK defines the body frame, joint signs and limits. Height IK follows the
// starting branch, fixes the waist, and verifies Cartesian invariants at samples
// AND at interpolated midpoints before returning any trajectory for dispatch.
class LegBodyMotionPlanner {
public:
  static constexpr double kPositionTolerance = 1e-5;
  static constexpr double kOrientationTolerance = LegBodyOrientationPlanner::kOrientationTolerance;
  explicit LegBodyMotionPlanner(const pinocchio::Model& model);
  ~LegBodyMotionPlanner();
  LegBodyMotionTrajectory plan(const LegJoints& start, LegBodyMotionMode mode,
      double tilt_deg, double height_m, double speed, double period, double max_duration);
  Eigen::Isometry3d pose(const LegJoints& q);
  double tiltDegrees(const LegJoints& q);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace motion_control_pkg::motion
