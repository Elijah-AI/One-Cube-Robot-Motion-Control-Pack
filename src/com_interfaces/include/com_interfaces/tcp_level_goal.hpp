#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace com_interfaces {
// TCP XY plane horizontal in base_link. Preserve the initial up/down branch.
// A tolerance of zero is NOT an unspecified value.
inline bool validateTcpLevelGoal(bool enabled, double tolerance, std::uint8_t arm_id,
                                 bool pose_goal, std::uint8_t control_mode,
                                 std::string& error) {
  if (!std::isfinite(tolerance) || tolerance < 1e-4 || tolerance > 0.1) {
    error = "tcp_level_tolerance_rad must be finite and in [0.0001, 0.1]";
    return false;
  }
  if (enabled && !(pose_goal && arm_id >= 5 && arm_id <= 7)) {
    error = "keep_tcp_level is only supported for WBC MovePose (arm_id 5/6/7)";
    return false;
  }
  if (enabled && control_mode != 0) {
    error = "keep_tcp_level requires position control; impedance may deflect the TCP";
    return false;
  }
  error.clear();
  return true;
}
}  // namespace com_interfaces
