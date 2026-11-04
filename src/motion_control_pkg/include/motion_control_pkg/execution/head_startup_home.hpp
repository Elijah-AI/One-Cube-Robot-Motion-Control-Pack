#pragma once
#include "motion_control_pkg/execution/motion_safety.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_control_pkg::execution {
// The composition root supplies the user-selected -0.7 rad pitch lower bound.
// Other bounds are the loaded URDF limits; no other extra travel is granted.
struct HeadStartupHomePolicy {
  bool enabled{false};
  std::vector<double> lower, upper;
  // Command speed; measured speed remains bounded by the loaded URDF.
  double velocity{.1}, acceleration{.05}, jerk{.1};
  double feedback_wait{30}, timeout{180};
  void validate(const std::vector<SafetyJoint>& normal) const {
    if (normal.size()!=2 || lower.size()!=2 || upper.size()!=2)
      throw std::invalid_argument("head startup home requires yaw,pitch bounds");
    for (std::size_t j=0;j<2;++j)
      if (!std::isfinite(lower[j]) || !std::isfinite(upper[j]) ||
          !std::isfinite(normal[j].lower) || !std::isfinite(normal[j].upper) ||
          !std::isfinite(normal[j].velocity) || normal[j].velocity<=0 ||
          lower[j]>normal[j].lower || upper[j]<normal[j].upper ||
          normal[j].lower>=0 || normal[j].upper<=0)
        throw std::invalid_argument("head recovery bounds must include URDF range and zero");
    if (lower[0]!=normal[0].lower || upper[0]!=normal[0].upper || upper[1]!=normal[1].upper)
      throw std::invalid_argument("startup recovery may extend only the negative pitch bound");
    if (lower[1] < -.7)
      throw std::invalid_argument("head pitch startup recovery lower bound must be >= -0.7 rad");
    if (!std::isfinite(velocity) || velocity<=0 || velocity>.1 ||
        std::any_of(normal.begin(),normal.end(),[&](const auto& joint){return velocity>joint.velocity;}) ||
        !std::isfinite(acceleration) || acceleration<=0 || acceleration>.1 ||
        !std::isfinite(jerk) || jerk<=0 || jerk>.5 ||
        !std::isfinite(feedback_wait) || feedback_wait<.2 || feedback_wait>30 ||
        !std::isfinite(timeout) || timeout<=feedback_wait || timeout>600)
      throw std::invalid_argument("invalid head startup home timing or motion limits");
  }
};

// Fixed target zero. Each accepted setpoint must stay between the initial
// measured position and zero and never move farther from zero. Float transport
// uses a small numerical tolerance, not a motion-range extension.
inline bool headHomeCommandInward(const std::vector<double>& initial,
    const std::vector<double>& previous, const std::vector<double>& next) {
  if (initial.size()!=2 || previous.size()!=2 || next.size()!=2) return false;
  for (std::size_t j=0;j<2;++j)
    if (!std::isfinite(next[j]) || !std::isfinite(initial[j]) || !std::isfinite(previous[j]) ||
        next[j]<std::min(0.,initial[j])-1e-6 || next[j]>std::max(0.,initial[j])+1e-6 ||
        std::abs(next[j])>std::abs(previous[j])+1e-6) return false;
  return true;
}
}  // namespace motion_control_pkg::execution
