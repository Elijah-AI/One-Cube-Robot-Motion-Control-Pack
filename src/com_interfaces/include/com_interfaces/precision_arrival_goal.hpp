#pragma once

#include <cstdint>
#include <string>

namespace com_interfaces {
inline bool validatePrecisionArrivalGoal(bool enabled, std::uint8_t arm_id,
    bool pose_goal, std::uint8_t control_mode, std::string& error) {
  const bool supported = pose_goal ? arm_id >= 4 && arm_id <= 7 :
      arm_id == 3 || arm_id == 4;
  if (enabled && !supported) {
    error = "precision_arrival supports HEAD/LEG MoveJoints and LEG/WBC MovePose only";
    return false;
  }
  if (enabled && control_mode != 0) {
    error = "precision_arrival requires position control";
    return false;
  }
  error.clear();
  return true;
}
}  // namespace com_interfaces
