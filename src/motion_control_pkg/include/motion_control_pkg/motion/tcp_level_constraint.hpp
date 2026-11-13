#pragma once

#include <Eigen/Geometry>
#include <pinocchio/multibody/model.hpp>
#include <memory>
#include <string>
#include <vector>

namespace motion_control_pkg::motion {

// Per-action Pinocchio workspace: no mutable global IK state or shared Data.
// Only the selected arms may be projected; every other joint is preserved.
class TcpLevelConstraint {
public:
  using Configuration = std::vector<double>;
  using Path = std::vector<Configuration>;
  TcpLevelConstraint(const pinocchio::Model& model, Configuration reference,
                     bool left, bool right, const Eigen::Isometry3d& left_tool,
                     const Eigen::Isometry3d& right_tool, double tolerance);
  ~TcpLevelConstraint();
  TcpLevelConstraint(const TcpLevelConstraint&) = delete;
  bool initialize(std::string& error);
  bool check(const Configuration& q, std::string& error);
  bool checkTarget(bool left, const Eigen::Isometry3d& root_to_tcp, std::string& error) const;
  // Checks samples AND the piecewise-linear joint interpolation between them.
  bool validatePath(const Path& path, std::string& error);
  // Leaves endpoints and explicitly pinned via configurations unchanged.
  // Transactional: a failed projection never publishes a partial path.
  bool projectPath(Path& path, const Path& pinned, std::string& error);
  // Algorithm-level comparison/initialization helper; transactional, arms only.
  bool projectConfiguration(Configuration& q, std::string& error);
  bool checkMeasured(const std::vector<std::string>& names,
                     const Configuration& positions, std::string& error);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace motion_control_pkg::motion
