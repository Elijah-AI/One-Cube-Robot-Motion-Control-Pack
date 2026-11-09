#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

namespace motion_control_pkg::execution {
// Deliberately independent of arm tolerances and planning-start tolerances.
struct JointCompletionOptions {
  double position_tolerance{0.01};
  double velocity_tolerance{0.01};
  double stable_seconds{0.1};
  double stall_seconds{0.0};  // zero preserves the existing deadline-only policy
  double progress_rad{0.0002};
  void validate() const {
    if (!std::isfinite(position_tolerance) || position_tolerance <= 0 || position_tolerance > .1 ||
        !std::isfinite(velocity_tolerance) || velocity_tolerance <= 0 || velocity_tolerance > .1 ||
        !std::isfinite(stable_seconds) || stable_seconds < .1 || stable_seconds > 5 ||
        !std::isfinite(stall_seconds) || stall_seconds < 0 || stall_seconds > 30 ||
        !std::isfinite(progress_rad) || progress_rad <= 0 || progress_rad >= position_tolerance)
      throw std::invalid_argument("invalid joint completion policy");
  }
};

struct PrecisionPolicy {
  JointCompletionOptions completion{.002, .01, .2, 1.5, .0002};
  double completion_timeout{5.0};
  bool approach_enabled{true};
  double approach_distance{.03};
  double speed_scale{.2};
  double velocity_cap{.03}, acceleration_cap{.2}, jerk_cap{.5};
  void validate() const {
    completion.validate();
    if (!std::isfinite(completion_timeout) || completion_timeout < completion.stable_seconds ||
        completion_timeout > 30 ||
        !std::isfinite(approach_distance) || approach_distance <= completion.position_tolerance || approach_distance > .1 ||
        !std::isfinite(speed_scale) || speed_scale <= 0 || speed_scale > 1 ||
        !std::isfinite(velocity_cap) || velocity_cap <= 0 || velocity_cap > .1 ||
        !std::isfinite(acceleration_cap) || acceleration_cap <= 0 || acceleration_cap > 1 ||
        !std::isfinite(jerk_cap) || jerk_cap <= 0 || jerk_cap > 5)
      throw std::invalid_argument("invalid EtherCAT precision approach policy");
  }
};

// Select per Goal; never change the configured chain/startup policy in place.
// Ordinary completion still requires fresh measured frames and stopped joints.
inline PrecisionPolicy actionPrecisionPolicy(const PrecisionPolicy& configured,
    bool precision_arrival) {
  if (precision_arrival) return configured;
  PrecisionPolicy selected;
  selected.completion = JointCompletionOptions{};
  selected.completion.position_tolerance = 0.02;
  selected.completion_timeout = configured.completion_timeout;
  selected.approach_enabled = false;
  return selected;
}
}  // namespace motion_control_pkg::execution
