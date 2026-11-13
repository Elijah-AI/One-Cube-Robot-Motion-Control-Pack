#include "motion_control_pkg/wbc/model.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <urdf_parser/urdf_parser.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace motion_control_pkg::wbc {
struct CentroidalModel::Impl {
  pinocchio::Model model;
  std::vector<JointSpec> joints;
  std::vector<int> qi, vi;
  std::vector<std::string> links;
  std::string root;
  double total_mass{};
};

CentroidalModel::CentroidalModel(const std::string& path) : CentroidalModel(path, {}) {}

CentroidalModel::CentroidalModel(const std::string& path,
    const PositionLimitOverrides& position_limits) : impl_(std::make_unique<Impl>()) {
  const auto urdf = urdf::parseURDFFile(path);
  if (!urdf || !urdf->getRoot()) throw std::invalid_argument("cannot parse URDF: " + path);
  impl_->root = urdf->getRoot()->name;
  double xml_mass = 0.0;
  for (const auto& entry : urdf->links_) {
    impl_->links.push_back(entry.first);
    const auto& in = entry.second->inertial;
    if (!in) continue;  // massless frames are permitted
    const Eigen::Vector3d com(in->origin.position.x, in->origin.position.y, in->origin.position.z);
    Eigen::Matrix3d I;
    I << in->ixx, in->ixy, in->ixz, in->ixy, in->iyy, in->iyz, in->ixz, in->iyz, in->izz;
    if (!std::isfinite(in->mass) || in->mass < 0 || !com.allFinite() || !I.allFinite())
      throw std::invalid_argument("invalid URDF inertia: " + entry.first);
    const Eigen::Vector3d eig = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(I).eigenvalues();
    if ((in->mass > 0.0 && eig.minCoeff() <= 0.0) ||
        eig[2] > eig[0] + eig[1] + 1e-9 || (in->mass == 0.0 && I.norm() > 1e-12))
      throw std::invalid_argument("nonphysical URDF inertia: " + entry.first);
    xml_mass += in->mass;
  }
  // In a fixed-root model root-link inertia resides in universe (joint 0),
  // which standard Pinocchio COM accumulation excludes. Preserve it explicitly
  // as a floating joint, held stationary in all stage-1 evaluations.
  pinocchio::urdf::buildModel(path, pinocchio::JointModelFreeFlyer(), impl_->model);
  impl_->total_mass = pinocchio::computeTotalMass(impl_->model);
  if (!(xml_mass > 0.0) || std::abs(impl_->total_mass - xml_mass) > 1e-8 * xml_mass)
    throw std::invalid_argument("Pinocchio/URDF whole-robot mass mismatch");
  for (pinocchio::JointIndex id = 2; id < impl_->model.joints.size(); ++id) {
    const auto& pj = impl_->model.joints[id];
    const auto& name = impl_->model.names[id];
    const auto uj = urdf->getJoint(name);
    // Do not silently assume scalar layout for continuous/mimic/multidof joints.
    if (!uj || !uj->limits || uj->mimic || pj.nq() != 1 || pj.nv() != 1)
      throw std::invalid_argument("stage 1 requires bounded independent scalar joint: " + name);
    const auto& l = *uj->limits;
    if (!std::isfinite(l.lower) || !std::isfinite(l.upper) || l.lower > l.upper ||
        !std::isfinite(l.velocity) || l.velocity < 0 || !std::isfinite(l.effort) || l.effort < 0)
      throw std::invalid_argument("invalid URDF limits: " + name);
    const auto override = position_limits.find(name);
    const double lower = override == position_limits.end() ? l.lower : override->second.first;
    const double upper = override == position_limits.end() ? l.upper : override->second.second;
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper)
      throw std::invalid_argument("invalid runtime position limits: " + name);
    impl_->model.lowerPositionLimit[pj.idx_q()] = lower;
    impl_->model.upperPositionLimit[pj.idx_q()] = upper;
    impl_->joints.push_back({name, lower, upper, l.velocity, l.effort, lower == upper});
    impl_->qi.push_back(pj.idx_q());
    impl_->vi.push_back(pj.idx_v());
  }
  for (const auto& entry : position_limits)
    if (std::none_of(impl_->joints.begin(), impl_->joints.end(),
        [&](const auto& joint) { return joint.name == entry.first; }))
      throw std::invalid_argument("unknown position-limit joint: " + entry.first);
}
CentroidalModel::~CentroidalModel() = default;
const std::vector<JointSpec>& CentroidalModel::joints() const { return impl_->joints; }
const std::string& CentroidalModel::rootFrame() const { return impl_->root; }
double CentroidalModel::mass() const { return impl_->total_mass; }

JointSnapshot CentroidalModel::neutralSnapshot() const {
  JointSnapshot out;
  out.position_valid = out.velocity_valid = out.synthetic = true;
  for (const auto& j : joints()) {
    out.names.push_back(j.name);
    out.positions.push_back(std::clamp(0.0, j.lower, j.upper));
    out.velocities.push_back(0.0);
  }
  return out;
}

CentroidalResult CentroidalModel::evaluate(const JointSnapshot& state) const {
  if (!state.position_valid || state.names.size() != joints().size() ||
      state.positions.size() != state.names.size() ||
      (state.velocity_valid && state.velocities.size() != state.names.size()))
    throw std::invalid_argument("COM requires a complete valid whole-body snapshot");
  std::map<std::string, std::size_t> indices;
  for (std::size_t i = 0; i < state.names.size(); ++i)
    if (!indices.emplace(state.names[i], i).second)
      throw std::invalid_argument("duplicate snapshot joint");
  Eigen::VectorXd q = pinocchio::neutral(impl_->model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(impl_->model.nv);
  for (std::size_t j = 0; j < joints().size(); ++j) {
    const auto& spec = joints()[j];
    auto it = indices.find(spec.name);
    if (it == indices.end()) throw std::invalid_argument("missing snapshot joint: " + spec.name);
    const auto i = it->second;
    if (!std::isfinite(state.positions[i]) || state.positions[i] < spec.lower - 1e-8 ||
        state.positions[i] > spec.upper + 1e-8)
      throw std::invalid_argument("invalid snapshot position: " + spec.name);
    q[impl_->qi[j]] = state.positions[i];
    if (state.velocity_valid) {
      if (!std::isfinite(state.velocities[i]) || (spec.locked && std::abs(state.velocities[i]) > 1e-8))
        throw std::invalid_argument("invalid snapshot velocity: " + spec.name);
      v[impl_->vi[j]] = state.velocities[i];
    }
  }
  // Each caller owns its Data workspace: no shared IK lock or solver mutation.
  pinocchio::Data data(impl_->model);
  pinocchio::centerOfMass(impl_->model, data, q, v);
  CentroidalResult out;
  out.mass = mass();
  out.com = data.com[0];
  out.velocity = data.vcom[0];
  out.velocity_valid = state.velocity_valid;
  out.synthetic = state.synthetic;
  pinocchio::jacobianCenterOfMass(impl_->model, data, q);
  out.com_jacobian.resize(3, joints().size());
  for (std::size_t j = 0; j < joints().size(); ++j) {
    out.joint_names.push_back(joints()[j].name);
    out.com_jacobian.col(j) = data.Jcom.col(impl_->vi[j]);
  }
  pinocchio::updateFramePlacements(impl_->model, data);
  for (const auto& link : impl_->links) {
    const auto& T = data.oMf[impl_->model.getFrameId(link)];
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = T.rotation();
    pose.translation() = T.translation();
    out.link_poses.emplace(link, pose);
  }
  return out;
}
CentroidalDynamics CentroidalModel::dynamics(const JointSnapshot& state,
                                           const std::vector<double>& accelerations) const {
  // Reuse complete name/position validation. Stage-2 assumed steering angles
  // and assumed zero rates are NOT a certified dynamic uncertainty bound.
  if (!state.velocity_valid || !state.assumed.empty() || accelerations.size() != state.names.size())
    throw std::invalid_argument("dynamics requires complete explicit q/v/a; assumed joints are unsupported");
  evaluate(state);
  std::map<std::string, std::size_t> indices;
  for (std::size_t i = 0; i < state.names.size(); ++i) indices.emplace(state.names[i], i);
  Eigen::VectorXd q = pinocchio::neutral(impl_->model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(impl_->model.nv), a = v;
  for (std::size_t j = 0; j < joints().size(); ++j) {
    const auto i = indices.at(joints()[j].name);
    if (!std::isfinite(accelerations[i]) || (joints()[j].locked && std::abs(accelerations[i]) > 1e-8))
      throw std::invalid_argument("invalid acceleration: " + joints()[j].name);
    q[impl_->qi[j]] = state.positions[i];
    v[impl_->vi[j]] = state.velocities[i];
    a[impl_->vi[j]] = accelerations[i];
  }
  pinocchio::Data data(impl_->model);
  pinocchio::computeCentroidalMomentumTimeVariation(impl_->model, data, q, v, a);
  const Eigen::Vector3d h = data.hg.angular(), dh = data.dhg.angular();
  pinocchio::centerOfMass(impl_->model, data, q, v, a);
  CentroidalDynamics result{mass(), data.com[0], data.vcom[0], data.acom[0], h, dh};
  if (!result.com.allFinite() || !result.acceleration.allFinite() || !dh.allFinite())
    throw std::invalid_argument("non-finite centroidal dynamics");
  return result;
}
}  // namespace motion_control_pkg::wbc
