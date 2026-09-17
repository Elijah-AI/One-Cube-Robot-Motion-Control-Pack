#ifndef MOTION_CONTROL_PKG_MOTION_TRAJECTORY_DERIVATIVE_LIMITS_HPP_
#define MOTION_CONTROL_PKG_MOTION_TRAJECTORY_DERIVATIVE_LIMITS_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace motion_control_pkg::motion
{
// Uniform command samples, extended by a stationary hold at both ends.
// Divided differences are averages of continuous derivatives, not estimates
// of their extrema. This is an additional check on what is actually sent;
// the continuous planner must enforce its own derivative bounds as well.
inline bool validateCommandDerivatives(
  const std::vector<std::vector<double>>& positions, double dt,
  const std::vector<double>& velocity, const std::vector<double>& acceleration,
  const std::vector<double>& jerk, std::string& error)
{
  error.clear();
  if (positions.empty() || velocity.empty() || !std::isfinite(dt) || dt <= 0.0 ||
    acceleration.size() != velocity.size() || jerk.size() != velocity.size()) {
    error = "invalid command derivative dimensions or period";
    return false;
  }
  for (std::size_t j = 0; j < velocity.size(); ++j) {
    if (!std::isfinite(velocity[j]) || velocity[j] <= 0 ||
      !std::isfinite(acceleration[j]) || acceleration[j] <= 0 ||
      !std::isfinite(jerk[j]) || jerk[j] <= 0) {
      error = "invalid command derivative limit";
      return false;
    }
  }
  for (const auto& q : positions) {
    if (q.size() != velocity.size() ||
      !std::all_of(q.begin(), q.end(), [](double x) {return std::isfinite(x);})) {
      error = "invalid command position";
      return false;
    }
  }
  std::vector<double> last_q = positions.front();
  std::vector<double> last_v(velocity.size(), 0.0), last_a(velocity.size(), 0.0);
  for (std::size_t i = 0; i < positions.size() + 3; ++i) {
    const auto& q = positions[std::min(i, positions.size() - 1)];
    for (std::size_t j = 0; j < velocity.size(); ++j) {
      const double v = (q[j] - last_q[j]) / dt;
      const double a = (v - last_v[j]) / dt;
      const double r = (a - last_a[j]) / dt;
      if (std::abs(v) > velocity[j] * (1.0 + 1e-6) + 1e-8 ||
        std::abs(a) > acceleration[j] * (1.0 + 1e-6) + 1e-7 ||
        std::abs(r) > jerk[j] * (1.0 + 1e-6) + 1e-5) {
        error = "command v/a/jerk bound exceeded at sample " + std::to_string(i) +
          " joint " + std::to_string(j + 1) + " v=" + std::to_string(v) +
          " a=" + std::to_string(a) + " jerk=" + std::to_string(r);
        return false;
      }
      last_q[j] = q[j]; last_v[j] = v; last_a[j] = a;
    }
  }
  return true;
}
}  // namespace motion_control_pkg::motion
#endif
