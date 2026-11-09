#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace com_interfaces {

struct LegJointSelection {
  std::array<std::size_t, 2> commanded{};
  std::array<std::size_t, 2> target_indices{};
  std::size_t compensated{0};
  bool waist_commanded{false};
  double waist_target{0.0};
};

// Two leg joints preserve body orientation; adding body_joint coordinates a
// waist turn on the same trajectory while preserving the leg pitch compensation.
// The existing four-joint LEG goal retains its original semantics.
inline bool validateLegOrientationGoal(
    const std::vector<std::string>& names, const std::vector<double>& positions,
    LegJointSelection& selection, std::string& error) {
  selection = {};
  if ((names.size() != 2 && names.size() != 3) || positions.size() != names.size()) {
    error = "Compensated MoveJ requires two leg joints, optionally plus body_joint";
    return false;
  }
  const std::array<std::string, 3> supported{{"leg_joint1", "leg_joint2", "leg_joint3"}};
  std::array<bool, 3> used{};
  std::size_t leg_count = 0;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i] == "body_joint" && names.size() == 3 &&
        !selection.waist_commanded && std::isfinite(positions[i])) {
      selection.waist_commanded = true;
      selection.waist_target = positions[i];
      continue;
    }
    std::size_t index = 0;
    while (index < supported.size() && supported[index] != names[i]) ++index;
    if (index == supported.size() || used[index] || leg_count >= 2 || !std::isfinite(positions[i])) {
      error = "Select two distinct leg joints and optionally one body_joint, with finite targets";
      return false;
    }
    used[index] = true;
    selection.commanded[leg_count] = index;
    selection.target_indices[leg_count++] = i;
  }
  if (leg_count != 2 || (names.size() == 3 && !selection.waist_commanded)) {
    error = "Three-target compensated MoveJ requires exactly two leg joints and body_joint";
    return false;
  }
  for (std::size_t i = 0; i < used.size(); ++i)
    if (!used[i]) selection.compensated = i;
  error.clear();
  return true;
}

}  // namespace com_interfaces
