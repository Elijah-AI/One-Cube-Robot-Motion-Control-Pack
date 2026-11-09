#include "motion_control_pkg/motion/leg_body_orientation.hpp"
#include "com_interfaces/leg_joint_goal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <ruckig/ruckig.hpp>

namespace motion_control_pkg::motion {
namespace {
const std::array<std::string, 4> kNames{{"leg_joint1", "leg_joint2", "leg_joint3", "body_joint"}};
void require(bool condition, const std::string& error) {
  if (!condition) throw std::invalid_argument(error);
}
}

struct LegBodyOrientationPlanner::Impl {
  const pinocchio::Model& model;
  pinocchio::Data data;
  Eigen::VectorXd q;
  std::array<int, 4> q_index{}, v_index{};
  pinocchio::FrameIndex body_frame{}, base_frame{};

  explicit Impl(const pinocchio::Model& input) : model(input), data(input), q(pinocchio::neutral(input)) {
    require(model.existFrame("base_link") && model.existFrame("body_link"),
        "Body-orientation MoveJ requires base_link and body_link in the loaded URDF");
    base_frame = model.getFrameId("base_link");
    body_frame = model.getFrameId("body_link");
    pinocchio::JointIndex parent = model.frames[base_frame].parentJoint;
    for (std::size_t i = 0; i < kNames.size(); ++i) {
      require(model.existJointName(kNames[i]), "Missing URDF joint: " + kNames[i]);
      const auto id = model.getJointId(kNames[i]);
      require(model.joints[id].nq() == 1 && model.joints[id].nv() == 1 && model.parents[id] == parent,
          "Body-orientation MoveJ requires the serial four-joint leg chain");
      q_index[i] = model.joints[id].idx_q();
      v_index[i] = model.joints[id].idx_v();
      parent = id;
      require(std::isfinite(model.lowerPositionLimit[q_index[i]]) &&
          std::isfinite(model.upperPositionLimit[q_index[i]]) &&
          model.lowerPositionLimit[q_index[i]] < model.upperPositionLimit[q_index[i]] &&
          std::isfinite(model.velocityLimit[v_index[i]]) && model.velocityLimit[v_index[i]] > 0,
          "Invalid URDF limits for " + kNames[i]);
    }
    require(model.frames[body_frame].parentJoint == parent,
        "body_link must be attached to body_joint");
  }

  Eigen::Matrix3d rotation(const LegJoints& joints) {
    for (std::size_t i = 0; i < joints.size(); ++i) q[q_index[i]] = joints[i];
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    return data.oMf[base_frame].rotation().transpose() * data.oMf[body_frame].rotation();
  }

  Eigen::Isometry3d pose(const LegJoints& joints) {
    rotation(joints);
    const auto relative = data.oMf[base_frame].inverse() * data.oMf[body_frame];
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = relative.rotation();
    result.translation() = relative.translation();
    return result;
  }
};

LegBodyOrientationPlanner::LegBodyOrientationPlanner(const pinocchio::Model& model)
    : impl_(std::make_unique<Impl>(model)) {}
LegBodyOrientationPlanner::~LegBodyOrientationPlanner() = default;

void LegBodyOrientationPlanner::validatePosition(const LegJoints& joints) const {
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const auto index = impl_->q_index[i];
    require(std::isfinite(joints[i]) && joints[i] >= impl_->model.lowerPositionLimit[index] &&
        joints[i] <= impl_->model.upperPositionLimit[index],
        kNames[i] + " is non-finite or outside its URDF position limits");
  }
}

Eigen::Matrix3d LegBodyOrientationPlanner::orientation(const LegJoints& joints) {
  validatePosition(joints);
  return impl_->rotation(joints);
}

Eigen::Isometry3d LegBodyOrientationPlanner::pose(const LegJoints& joints) {
  validatePosition(joints);
  return impl_->pose(joints);
}

double LegBodyOrientationPlanner::orientationError(const LegJoints& joints, const Eigen::Matrix3d& reference) {
  require(reference.allFinite(), "Invalid reference body orientation");
  return Eigen::AngleAxisd(reference.transpose() * orientation(joints)).angle();
}

LegOrientationTrajectory LegBodyOrientationPlanner::plan(const LegJoints& start,
    const std::vector<std::string>& names, const std::vector<double>& targets,
    double speed, double sample_period, double max_duration) {
  com_interfaces::LegJointSelection selection;
  std::string error;
  require(com_interfaces::validateLegOrientationGoal(names, targets, selection, error), error);
  require(std::isfinite(speed) && speed > 0 && speed <= 1, "speed must be finite and in (0, 1]");
  require(std::isfinite(sample_period) && sample_period > 0 &&
      std::isfinite(max_duration) && max_duration > 0, "Invalid planning period or timeout");
  validatePosition(start);
  LegOrientationTrajectory output;
  output.reference_orientation = orientation(start);
  output.compensated_joint = selection.compensated;
  output.target = start;

  // Determine the actual signed pitch axes from FK, including URDF joint origins
  // and axis signs. A different robot with nonparallel axes is rejected.
  std::array<Eigen::Vector3d, 3> axes;
  std::array<double, 3> signs{};
  constexpr double probe_angle = 1e-5;
  for (std::size_t i = 0; i < 3; ++i) {
    auto probe = start;
    probe[i] += probe_angle;
    const Eigen::AngleAxisd delta(impl_->rotation(probe) * output.reference_orientation.transpose());
    axes[i] = delta.axis() * (delta.angle() / probe_angle);
    require(std::abs(axes[i].norm() - 1.0) < 1e-5, "Leg joints must be revolute for body-orientation MoveJ");
    axes[i].normalize();
    require(axes[0].cross(axes[i]).norm() < 1e-4,
        "The URDF leg pitch axes are not parallel; two-joint body-orientation MoveJ is unsupported");
    signs[i] = axes[0].dot(axes[i]) >= 0 ? 1.0 : -1.0;
  }
  double pitch_change = 0;
  for (std::size_t i = 0; i < selection.commanded.size(); ++i) {
    const auto joint = selection.commanded[i];
    const double target = targets[selection.target_indices[i]];
    output.target[joint] = target;
    pitch_change += signs[joint] * (target - start[joint]);
  }
  output.target[selection.compensated] -= pitch_change / signs[selection.compensated];
  if (selection.waist_commanded) output.target[3] = selection.waist_target;
  validatePosition(output.target);  // Never clamp a compensation that violates limits.

  LegJoints delta{};
  double scalar_v = std::numeric_limits<double>::infinity();
  double scalar_a = scalar_v, scalar_j = scalar_v, largest_delta = 0;
  for (std::size_t i = 0; i < delta.size(); ++i) {
    delta[i] = output.target[i] - start[i];
    const double distance = std::abs(delta[i]);
    largest_delta = std::max(largest_delta, distance);
    if (distance <= 1e-12) continue;
    const double velocity = std::min(1.5, impl_->model.velocityLimit[impl_->v_index[i]]) * speed;
    scalar_v = std::min(scalar_v, velocity / distance);
    scalar_a = std::min(scalar_a, 5.0 * speed / distance);
    scalar_j = std::min(scalar_j, 10.0 * speed / distance);
  }
  if (largest_delta <= 1e-12) {
    output.positions.emplace_back(start.begin(), start.end());
    output.velocities.emplace_back(4, 0.0);
    output.accelerations.emplace_back(4, 0.0);
    return output;
  }

  ruckig::InputParameter<1> input;
  input.current_position = {0.0}; input.current_velocity = {0.0}; input.current_acceleration = {0.0};
  input.target_position = {1.0}; input.target_velocity = {0.0}; input.target_acceleration = {0.0};
  input.max_velocity = {scalar_v}; input.max_acceleration = {scalar_a}; input.max_jerk = {scalar_j};
  ruckig::Ruckig<1> generator;
  ruckig::Trajectory<1> trajectory;
  require(generator.calculate(input, trajectory) >= 0, "Ruckig failed to plan body-orientation MoveJ");
  const double duration = trajectory.get_duration();
  require(std::isfinite(duration) && duration > 0 && duration < max_duration,
      "Body-orientation trajectory exceeds timeout_ms");
  const double steps = std::ceil(duration / sample_period);
  require(steps <= 250000 && steps * sample_period < max_duration,
      "Body-orientation trajectory exceeds timeout or sample budget");
  output.duration = steps * sample_period;
  for (std::size_t n = 0; n <= static_cast<std::size_t>(steps); ++n) {
    std::array<double, 1> s{}, v{}, a{};
    trajectory.at_time(std::min(n * sample_period, duration), s, v, a);
    require(std::isfinite(s[0]) && s[0] >= -1e-12 && s[0] <= 1.0 + 1e-12,
        "Invalid progress in body-orientation trajectory");
    const double progress = std::clamp(s[0], 0.0, 1.0);
    LegJoints sample{}, velocity{}, acceleration{};
    for (std::size_t j = 0; j < 4; ++j) {
      sample[j] = n == static_cast<std::size_t>(steps) ? output.target[j] : start[j] + delta[j] * progress;
      velocity[j] = delta[j] * v[0];
      acceleration[j] = delta[j] * a[0];
    }
    // Remove only the explicitly commanded waist turn from this check. The
    // actual four-axis sample (including waist) is still limit/collision checked.
    validatePosition(sample);
    auto compensated_sample = sample;
    if (selection.waist_commanded) compensated_sample[3] = start[3];
    const double deviation = orientationError(compensated_sample, output.reference_orientation);
    require(std::isfinite(deviation) && deviation <= kOrientationTolerance,
        "URDF FK rejected body orientation along the compensated trajectory");
    output.max_orientation_error = std::max(output.max_orientation_error, deviation);
    output.positions.emplace_back(sample.begin(), sample.end());
    output.velocities.emplace_back(velocity.begin(), velocity.end());
    output.accelerations.emplace_back(acceleration.begin(), acceleration.end());
  }
  return output;
}

}  // namespace motion_control_pkg::motion
