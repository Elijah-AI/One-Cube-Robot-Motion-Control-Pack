#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <pinocchio/multibody/fwd.hpp>

namespace motion_control_pkg::motion {

using LegJoints = std::array<double, 4>;

struct LegOrientationTrajectory {
  LegJoints target{};
  std::size_t compensated_joint{0};
  Eigen::Matrix3d reference_orientation{Eigen::Matrix3d::Identity()};
  std::vector<std::vector<double>> positions;
  std::vector<std::vector<double>> velocities;
  std::vector<std::vector<double>> accelerations;
  double duration{0.0};
  double max_orientation_error{0.0};
};

// Uses the loaded URDF model and a single scalar Ruckig trajectory for all
// four joints. Each instance owns its FK data; no shared robot state is mutated.
class LegBodyOrientationPlanner {
public:
  static constexpr double kOrientationTolerance = 1e-4;  // rad, includes URDF rpy rounding
  explicit LegBodyOrientationPlanner(const pinocchio::Model& model);
  ~LegBodyOrientationPlanner();
  LegBodyOrientationPlanner(const LegBodyOrientationPlanner&) = delete;
  LegBodyOrientationPlanner& operator=(const LegBodyOrientationPlanner&) = delete;

  LegOrientationTrajectory plan(const LegJoints& start,
      const std::vector<std::string>& names, const std::vector<double>& targets,
      double speed, double sample_period, double max_duration);
  Eigen::Matrix3d orientation(const LegJoints& joints);
  Eigen::Isometry3d pose(const LegJoints& joints);
  double orientationError(const LegJoints& joints, const Eigen::Matrix3d& reference);
  void validatePosition(const LegJoints& joints) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace motion_control_pkg::motion
