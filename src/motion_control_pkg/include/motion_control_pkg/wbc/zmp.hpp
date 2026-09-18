#pragma once
#include "motion_control_pkg/wbc/model.hpp"

namespace motion_control_pkg::wbc {
struct ZmpPrediction {
  bool valid{false};
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d required_force{Eigen::Vector3d::Zero()};
  // Moment about (0,0,support.plane_z), expressed in the root frame.
  Eigen::Vector3d required_moment{Eigen::Vector3d::Zero()};
  double margin{};
  std::string error;
};

// Horizontal support, stationary root, gravity and ground contact only. This
// inverse-dynamics requirement neither measures contact nor proves friction
// feasibility, actuator feasibility, or stability of a moving floating base.
ZmpPrediction predictZmp(const CentroidalDynamics& dynamics, const SupportResult& support,
                        double gravity = 9.81, double min_normal_force = 1.0);
}  // namespace motion_control_pkg::wbc
