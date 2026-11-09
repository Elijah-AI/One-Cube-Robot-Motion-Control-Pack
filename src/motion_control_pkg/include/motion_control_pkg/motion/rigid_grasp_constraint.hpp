#pragma once
#include <Eigen/Geometry>
#include <pinocchio/multibody/model.hpp>
#include <memory>
#include <string>
#include <vector>

namespace motion_control_pkg::motion {
struct RigidGraspOptions {
  bool enabled{false};
  double position_tolerance_m{.001};
  double orientation_tolerance_rad{.005};
};
// Both calibrated suction TCPs face down and share one rigid, level object.
// The initial left_T_right is the reference; common translation/yaw are free.
class RigidGraspConstraint {
public:
  using Q=std::vector<double>;
  using Path=std::vector<Q>;
  RigidGraspConstraint(const pinocchio::Model&, Q reference,
      const Eigen::Isometry3d& left_tool, const Eigen::Isometry3d& right_tool,
      RigidGraspOptions options);
  ~RigidGraspConstraint();
  bool initialize(std::string& error);
  bool check(const Q&, std::string& error);
  bool checkTargets(const Eigen::Isometry3d& left, const Eigen::Isometry3d& right,
                    std::string& error) const;
  bool projectPath(Path&, const Path& pinned, std::string& error);
  // Prescribed leg path: preserve every non-arm coordinate, solve both arms
  // including the final grasp pose, and leave the input unchanged on failure.
  bool compensateLegPath(Path&, std::string& error);
  bool validatePath(const Path&, std::string& error);
  bool checkMeasured(const std::vector<std::string>&, const Q&, std::string& error);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
