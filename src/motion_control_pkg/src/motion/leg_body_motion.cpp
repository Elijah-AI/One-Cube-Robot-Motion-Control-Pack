#include "motion_control_pkg/motion/leg_body_motion.hpp"
#include "com_interfaces/leg_body_motion_goal.hpp"

#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <ruckig/ruckig.hpp>
#include <pinocchio/multibody/model.hpp>

namespace motion_control_pkg::motion {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr std::array<const char*, 4> kNames{{"leg_joint1", "leg_joint2", "leg_joint3", "body_joint"}};
void require(bool condition, const std::string& error) {
  if (!condition) throw std::invalid_argument(error);
}
double pitch(const Eigen::Isometry3d& pose) {
  return std::atan2(pose.linear()(0, 2), pose.linear()(2, 2));
}
double rotationError(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle();
}
}

struct LegBodyMotionPlanner::Impl {
  LegBodyOrientationPlanner geometry;
  LegJoints lower{}, upper{}, velocity_limit{};
  explicit Impl(const pinocchio::Model& model) : geometry(model) {
    for (std::size_t i = 0; i < 4; ++i) {
      const auto& joint = model.joints[model.getJointId(kNames[i])];
      lower[i] = model.lowerPositionLimit[joint.idx_q()];
      upper[i] = model.upperPositionLimit[joint.idx_q()];
      velocity_limit[i] = std::min(1.5, model.velocityLimit[joint.idx_v()]);
    }
  }
  bool withinLimits(const LegJoints& q) const {
    for (std::size_t i = 0; i < 4; ++i)
      if (!std::isfinite(q[i]) || q[i] < lower[i] || q[i] > upper[i]) return false;
    return true;
  }
  Eigen::Vector3d residual(const LegJoints& q, const Eigen::Isometry3d& target) {
    const auto current = geometry.pose(q);
    return {target.translation().x() - current.translation().x(),
            target.translation().z() - current.translation().z(),
            std::remainder(pitch(target) - pitch(current), 2 * kPi)};
  }
  LegJoints solveHeight(LegJoints seed, const Eigen::Isometry3d& target) {
    // Solve the three independent planar coordinates using the actual URDF FK.
    // The complete 6D pose is checked afterwards, including URDF rpy rounding.
    for (unsigned iteration = 0; iteration < 60; ++iteration) {
      const auto error = residual(seed, target);
      if (error.norm() < 1e-12) {
        const auto actual = geometry.pose(seed);
        require((actual.translation() - target.translation()).norm() <= kPositionTolerance &&
            rotationError(actual, target) <= kOrientationTolerance,
            "Loaded URDF cannot preserve the complete body pose on a vertical path");
        return seed;
      }
      Eigen::Matrix3d jacobian;
      for (std::size_t j = 0; j < 3; ++j) {
        auto probe = seed;
        const double step = seed[j] + 1e-6 <= upper[j] ? 1e-6 : -1e-6;
        probe[j] += step;
        require(withinLimits(probe), "Insufficient joint range for height IK");
        jacobian.col(j) = (error - residual(probe, target)) / step;
      }
      Eigen::FullPivLU<Eigen::Matrix3d> decomposition(jacobian);
      require(decomposition.rank() == 3 && decomposition.rcond() > 1e-6,
          "Vertical body motion is singular at the current leg configuration");
      Eigen::Vector3d delta = decomposition.solve(error);
      require(delta.allFinite(), "Non-finite vertical body IK step");
      const double largest = delta.cwiseAbs().maxCoeff();
      if (largest > 0.15) delta *= 0.15 / largest;
      bool accepted = false;
      for (unsigned backtrack = 0; backtrack < 12; ++backtrack) {
        auto candidate = seed;
        for (std::size_t j = 0; j < 3; ++j) candidate[j] += delta[j];
        if (withinLimits(candidate) && residual(candidate, target).norm() < error.norm()) {
          seed = candidate;
          accepted = true;
          break;
        }
        delta *= 0.5;
      }
      require(accepted, "Vertical body target is unreachable on the current branch or violates URDF limits");
    }
    throw std::invalid_argument("Vertical body IK did not converge");
  }
  LegJoints solveTilt(const LegJoints& start, double degrees) {
    const double wanted = degrees * kPi / 180;
    auto probe = start;
    const double step = start[2] + 1e-5 <= upper[2] ? 1e-5 : -1e-5;
    probe[2] += step;
    require(withinLimits(probe), "Insufficient J3 range for body tilt");
    const auto initial = geometry.pose(start);
    const double slope = std::remainder(pitch(geometry.pose(probe)) - pitch(initial), 2 * kPi) / step;
    require(std::abs(slope) > 0.99 && std::abs(slope) < 1.01,
        "Body tilt requires a J3 pitch axis aligned with base_link Y");
    auto candidate = start;
    candidate[2] += std::remainder(wanted - pitch(initial), 2 * kPi) / slope;
    LegJoints best{};
    double best_distance = std::numeric_limits<double>::infinity();
    for (int turns = -1; turns <= 1; ++turns) {
      auto q = candidate;
      q[2] += turns * 2 * kPi;
      if (!withinLimits(q)) continue;
      // Correct the small nonlinear residue introduced by rounded URDF origins.
      for (unsigned n = 0; n < 5; ++n)
        q[2] += std::remainder(wanted - pitch(geometry.pose(q)), 2 * kPi) / slope;
      if (!withinLimits(q)) continue;
      const Eigen::Vector3d z = geometry.pose(q).linear().col(2);
      require((z - Eigen::Vector3d(std::sin(wanted), 0, std::cos(wanted))).norm() <= kOrientationTolerance,
          "J3 alone cannot produce the requested inclination in this URDF");
      if (std::abs(q[2] - start[2]) < best_distance) {
        best = q;
        best_distance = std::abs(q[2] - start[2]);
      }
    }
    require(std::isfinite(best_distance), "Requested body tilt violates the J3 URDF limits while J1/J2 are held");
    return best;
  }
};

LegBodyMotionPlanner::LegBodyMotionPlanner(const pinocchio::Model& model)
    : impl_(std::make_unique<Impl>(model)) {}
LegBodyMotionPlanner::~LegBodyMotionPlanner() = default;
Eigen::Isometry3d LegBodyMotionPlanner::pose(const LegJoints& q) { return impl_->geometry.pose(q); }
double LegBodyMotionPlanner::tiltDegrees(const LegJoints& q) { return pitch(pose(q)) * 180 / kPi; }

LegBodyMotionTrajectory LegBodyMotionPlanner::plan(const LegJoints& start, LegBodyMotionMode mode,
    double tilt_deg, double height_m, double speed, double period, double max_duration) {
  std::string error;
  require(com_interfaces::validateLegBodyMotionGoal(4, static_cast<uint8_t>(mode), tilt_deg, height_m,
      {}, {}, speed, 1, 0, 0, 0, false, error) && static_cast<uint8_t>(mode) != 0, error);
  require(std::isfinite(period) && period > 0 && std::isfinite(max_duration) && max_duration > 0,
      "Invalid body motion period or duration budget");
  impl_->geometry.validatePosition(start);
  LegBodyMotionTrajectory result;
  result.start_body = pose(start);
  result.target_body = result.start_body;
  result.vertical = mode != LegBodyMotionMode::Tilt;
  result.joints.reference_orientation = result.start_body.linear();
  if (result.vertical) {
    result.target_body.translation().z() = mode == LegBodyMotionMode::RelativeHeight ?
        result.start_body.translation().z() + height_m : height_m;
    require(result.target_body.translation().allFinite(), "Body height overflow");
  } else {
    result.joints.target = impl_->solveTilt(start, tilt_deg);
    result.target_body = pose(result.joints.target);
  }
  const double delta_z = result.target_body.translation().z() - result.start_body.translation().z();
  if (result.vertical && std::abs(delta_z) < 1e-12) result.joints.target = start;
  if ((result.vertical && std::abs(delta_z) < 1e-12) ||
      (!result.vertical && std::abs(result.joints.target[2] - start[2]) < 1e-12)) {
    result.joints.positions.emplace_back(start.begin(), start.end());
    result.joints.velocities.emplace_back(4, 0);
    result.joints.accelerations.emplace_back(4, 0);
    return result;
  }
  const auto targetAt = [&](double progress) {
    auto target = result.start_body;
    target.translation().z() += delta_z * progress;
    return target;
  };
  const auto sampleAt = [&](double progress, const LegJoints& seed) {
    if (result.vertical) return impl_->solveHeight(seed, targetAt(progress));
    auto q = start;
    q[2] += (result.joints.target[2] - start[2]) * progress;
    return q;
  };
  // Estimate the largest joint derivative along the selected continuous IK
  // branch, then validate the complete sampled dynamics and retime if needed.
  LegJoints derivative{}, previous = start;
  for (unsigned n = 1; n <= 100; ++n) {
    const auto q = sampleAt(n / 100.0, previous);
    for (std::size_t j = 0; j < 4; ++j)
      derivative[j] = std::max(derivative[j], 100 * std::abs(q[j] - previous[j]));
    previous = q;
  }
  result.joints.target = previous;
  double scalar_v = std::numeric_limits<double>::infinity(), scalar_a = scalar_v, scalar_j = scalar_v;
  for (std::size_t j = 0; j < 4; ++j) {
    if (derivative[j] < 1e-12) continue;
    scalar_v = std::min(scalar_v, impl_->velocity_limit[j] * speed / derivative[j]);
    scalar_a = std::min(scalar_a, 5 * speed / derivative[j]);
    scalar_j = std::min(scalar_j, 10 * speed / derivative[j]);
  }
  double slowdown = result.vertical ? 2.0 : 1.0;
  for (unsigned attempt = 0; attempt < 12; ++attempt) {
    ruckig::InputParameter<1> input;
    input.current_position = {0}; input.current_velocity = {0}; input.current_acceleration = {0};
    input.target_position = {1}; input.target_velocity = {0}; input.target_acceleration = {0};
    input.max_velocity = {scalar_v / slowdown};
    input.max_acceleration = {scalar_a / (slowdown * slowdown)};
    input.max_jerk = {scalar_j / (slowdown * slowdown * slowdown)};
    ruckig::Ruckig<1> generator;
    ruckig::Trajectory<1> profile;
    require(generator.calculate(input, profile) >= 0, "Ruckig body motion planning failed");
    const double steps = std::ceil(profile.get_duration() / period);
    require(std::isfinite(steps) && steps > 0 && steps <= 250000 && steps * period < max_duration,
        "Body motion exceeds timeout or trajectory sample budget");
    result.joints.positions.clear(); result.joints.velocities.clear(); result.joints.accelerations.clear();
    previous = start;
    for (std::size_t n = 0; n <= static_cast<std::size_t>(steps); ++n) {
      std::array<double, 1> s{}, v{}, a{};
      profile.at_time(std::min(n * period, profile.get_duration()), s, v, a);
      require(std::isfinite(s[0]) && s[0] >= -1e-12 && s[0] <= 1 + 1e-12, "Invalid body motion progress");
      auto q = n == 0 ? start : n == static_cast<std::size_t>(steps) ? result.joints.target :
          sampleAt(std::clamp(s[0], 0.0, 1.0), previous);
      impl_->geometry.validatePosition(q);
      require(q[3] == start[3], "Body motion changed the waist joint");
      if (!result.vertical) require(q[0] == start[0] && q[1] == start[1], "TILT changed J1/J2");
      if (result.vertical) {
        const auto actual = pose(q);
        const double position_error = (actual.translation() - targetAt(std::clamp(s[0], 0.0, 1.0)).translation()).norm();
        require(position_error <= kPositionTolerance && rotationError(actual, result.start_body) <= kOrientationTolerance,
            "Vertical body path violates its Cartesian pose constraints");
        result.max_position_error_m = std::max(result.max_position_error_m, position_error);
        if (n) {
          LegJoints midpoint{};
          for (std::size_t j = 0; j < 4; ++j) midpoint[j] = (previous[j] + q[j]) * 0.5;
          const auto middle = pose(midpoint);
          const double lateral = (middle.translation().head<2>() - result.start_body.translation().head<2>()).norm();
          require(lateral <= kPositionTolerance && rotationError(middle, result.start_body) <= kOrientationTolerance,
              "Vertical body path violates Cartesian constraints between samples");
          result.max_position_error_m = std::max(result.max_position_error_m, lateral);
        }
      }
      result.joints.positions.emplace_back(q.begin(), q.end());
      previous = q;
    }
    double ratio = 1;
    std::vector<double> old_velocity(4, 0), old_acceleration(4, 0);
    for (std::size_t n = 0; n <= result.joints.positions.size(); ++n) {
      std::vector<double> velocity(4, 0), acceleration(4, 0);
      for (std::size_t j = 0; j < 4; ++j) {
        if (n && n < result.joints.positions.size())
          velocity[j] = (result.joints.positions[n][j] - result.joints.positions[n - 1][j]) / period;
        acceleration[j] = (velocity[j] - old_velocity[j]) / period;
        const double jerk = (acceleration[j] - old_acceleration[j]) / period;
        ratio = std::max(ratio, std::abs(velocity[j]) / (impl_->velocity_limit[j] * speed));
        ratio = std::max(ratio, std::sqrt(std::abs(acceleration[j]) / (5 * speed)));
        ratio = std::max(ratio, std::cbrt(std::abs(jerk) / (10 * speed)));
      }
      if (n < result.joints.positions.size()) {
        result.joints.velocities.push_back(velocity);
        result.joints.accelerations.push_back(acceleration);
      }
      old_velocity = std::move(velocity);
      old_acceleration = std::move(acceleration);
    }
    require(std::isfinite(ratio), "Non-finite body trajectory dynamics");
    if (ratio <= 1 + 1e-6) {
      result.joints.duration = steps * period;
      // Boundary derivatives are exact by construction of the Ruckig profile.
      result.joints.velocities.front().assign(4, 0); result.joints.velocities.back().assign(4, 0);
      result.joints.accelerations.front().assign(4, 0); result.joints.accelerations.back().assign(4, 0);
      return result;
    }
    slowdown *= ratio * 1.1;
  }
  throw std::invalid_argument("Cannot satisfy body motion joint velocity/acceleration/jerk limits");
}
}  // namespace motion_control_pkg::motion
