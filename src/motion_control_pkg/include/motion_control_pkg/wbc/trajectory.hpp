#pragma once
#include "motion_control_pkg/wbc/static_balance.hpp"
#include <functional>
#include <limits>

namespace motion_control_pkg::wbc {
struct ReferenceSample {
  std::vector<double> positions, velocities, accelerations;
};

// Natural C2 cubic interpolant through equally spaced position knots. This is
// a NEW reference, not a claim about derivatives of a position-only command.
class CubicReference {
public:
  CubicReference(std::vector<std::vector<double>> positions, double period);
  ReferenceSample at(double time) const;
  std::vector<double> knots() const;
  double duration() const { return (q_.size()-1)*dt_; }
private:
  std::vector<std::vector<double>> q_, second_;
  double dt_;
};

struct ReferenceOptions {
  double period{.004};
  double maximum_duration{60.0};
  double maximum_zmp_scale{10.0};
  // In addition to command ticks, validate a midpoint each tick. This is a
  // sampled certificate, NOT a continuous collision/ZMP proof.
  std::size_t maximum_samples{50000};
};
struct ReferenceResult {
  bool valid{false};
  std::string error;
  double duration{}, zmp_time_scale{1};
  double minimum_zmp_margin{std::numeric_limits<double>::infinity()};
  std::size_t checked_samples{};
  std::vector<ReferenceSample> samples;
  std::function<ReferenceSample(double)> evaluate;
};

// All vectors use model().joints() order, including URDF-locked joints. Frozen
// head, steering and unused arm positions must be supplied explicitly. The
// caller must establish measured/resting start state and root/contact policy.
ReferenceResult prepareReference(const StaticBalance& balance,
  const JointSnapshot& start, const std::vector<std::vector<double>>& positions,
  const std::vector<double>& velocity, const std::vector<double>& acceleration,
  const std::vector<double>& jerk, const ReferenceOptions& options = {},
  const std::function<bool(const std::vector<double>&, std::string&)>& geometry_check = {});
}  // namespace motion_control_pkg::wbc
