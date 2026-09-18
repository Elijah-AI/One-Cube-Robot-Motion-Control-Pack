#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace motion_control_pkg::wbc {

// SI units throughout. All properties, including locked joints, come from URDF.
struct JointSpec {
  std::string name;
  double lower{}, upper{}, velocity{}, effort{};
  bool locked{};
};

struct JointSnapshot {
  std::vector<std::string> names;
  std::vector<double> positions, velocities;
  std::vector<std::string> missing, stale;
  std::vector<std::string> assumed, estimated_velocity;
  double oldest_stamp{}, newest_stamp{};
  bool position_valid{false}, velocity_valid{false}, synthetic{false};
};

struct StatePolicy {
  std::map<std::string, double> assumed_positions;
  std::vector<std::string> differentiate_joints;
  double filter_time_constant{0.03}, min_dt{0.001}, max_dt{0.15};
};

// Aggregates partial packets in one clock domain. No interpolation or implicit
// zero fill for an unobserved moving joint. An update is atomic on validation.
class WholeBodyState {
public:
  explicit WholeBodyState(std::vector<JointSpec> joints, StatePolicy policy = {});
  bool update(const std::vector<std::string>& names,
              const std::vector<double>& positions,
              const std::vector<double>& velocities,
              double stamp, std::string& error);
  JointSnapshot snapshot(double now, double max_age, double max_skew) const;
  // Live readers must sample the clock while holding the state lock. Otherwise
  // an update between clock sampling and copying can look spuriously future.
  // The callback must be non-blocking and must not call back into this object.
  JointSnapshot snapshotNow(const std::function<double()>& clock, double max_age, double max_skew) const;
  void clear();
private:
  JointSnapshot snapshotLocked(double now, double max_age, double max_skew) const;
  struct Sample { double position{}, velocity{}, stamp{}; bool velocity_valid{}, estimated{}; };
  StatePolicy policy_;
  std::vector<JointSpec> joints_;
  std::map<std::string, JointSpec> specs_;
  std::map<std::string, Sample> samples_;
  mutable std::mutex mutex_;
};

struct CentroidalResult {
  double mass{};
  Eigen::Vector3d com{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  // Columns correspond exactly to joint_names (root is held stationary).
  Eigen::MatrixXd com_jacobian;
  std::vector<std::string> joint_names;
  std::map<std::string, Eigen::Isometry3d> link_poses;
  bool velocity_valid{false}, synthetic{false};
};

// Model prediction in the stationary URDF root frame; not a force measurement.
struct CentroidalDynamics {
  double mass{};
  Eigen::Vector3d com, velocity, acceleration;
  Eigen::Vector3d angular_momentum, angular_momentum_rate;
};

// A separate floating-root model includes root-link inertia, without changing
// the existing IK model or its indices. Stage 1 holds root pose/velocity fixed.
class CentroidalModel {
public:
  explicit CentroidalModel(const std::string& urdf_path);
  ~CentroidalModel();
  const std::vector<JointSpec>& joints() const;
  const std::string& rootFrame() const;
  double mass() const;
  JointSnapshot neutralSnapshot() const;  // explicitly synthetic, offline only
  CentroidalResult evaluate(const JointSnapshot& state) const;
  CentroidalDynamics dynamics(const JointSnapshot& state,
                             const std::vector<double>& accelerations) const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct SupportResult {
  bool geometry_valid{false};
  // Geometry alone does NOT establish actual contact, friction or stability.
  bool contact_observed{false};
  double plane_z{}, signed_margin{};
  std::vector<Eigen::Vector2d> polygon;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;  // inward margin already applied, A rows are unit normals
  std::map<std::string, Eigen::Vector3d> contact_candidates;
  std::string error;
};

struct SteeringBounds {
  double com_xy{}, support_xy{};
};

// Stage 1: stationary root, horizontal ground, URDF collision geometry only.
// Support link names identify semantic roles; no mass, radius or wheelbase
// parameters override the URDF. Meshes are loaded once, never in evaluate().
class SupportModel {
public:
  SupportModel(const std::string& urdf_path,
               const std::vector<std::string>& support_links,
               const std::vector<std::string>& package_dirs = {});
  SupportResult evaluate(const CentroidalResult& model, double safety_margin,
                         double coplanarity_tolerance = 0.002) const;
  SteeringBounds steeringBounds(const CentroidalModel& model,
                                const std::vector<std::string>& unknown) const;
private:
  std::string urdf_path_;
  std::map<std::string, std::vector<Eigen::Vector3d>> vertices_;
};

}  // namespace motion_control_pkg::wbc
