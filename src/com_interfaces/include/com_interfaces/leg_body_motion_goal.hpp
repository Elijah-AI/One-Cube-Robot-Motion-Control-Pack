#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace com_interfaces {
inline bool validateLegBodyMotionGoal(uint8_t arm_id, uint8_t mode,
    double tilt_deg, double height_m, const std::vector<std::string>& names,
    const std::vector<double>& positions, double speed, int64_t timeout_ms,
    uint8_t control_mode, double acceleration_scale, double jerk_scale,
    bool relative_position, std::string& error) {
  error.clear();
  if (mode > 3 || !std::isfinite(tilt_deg) || !std::isfinite(height_m))
    error = "Invalid leg_motion_mode or non-finite body target";
  else if (mode == 0) {
    if (tilt_deg != 0 || height_m != 0)
      error = "Body targets require an explicit leg_motion_mode";
  } else if (arm_id != 4 || control_mode != 0 || !names.empty() ||
      !positions.empty() || relative_position || acceleration_scale != 0 ||
      jerk_scale != 0 || !std::isfinite(speed) || speed <= 0 || speed > 1 || timeout_ms <= 0)
    error = "Body motion requires LEG, position control, empty joint targets, speed (0,1], positive timeout and no relative_position or extra scales";
  else if (mode == 1 && (std::abs(tilt_deg) > 180 || height_m != 0))
    error = "TILT requires body_tilt_deg in [-180,180] and no height target";
  else if (mode != 1 && tilt_deg != 0)
    error = "Height motion preserves the starting inclination; omit body_tilt_deg";
  return error.empty();
}
}  // namespace com_interfaces
