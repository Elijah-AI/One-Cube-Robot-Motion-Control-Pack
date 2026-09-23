#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace com_interfaces {

struct LegJointSelection {
  std::array<std::size_t, 2> commanded{};
  std::size_t compensated{0};
};

// LEG goals with two joints select body-orientation-preserving MoveJ.
// The existing four-joint LEG goal retains its original semantics.
inline bool validateLegOrientationGoal(
    const std::vector<std::string>& names, const std::vector<double>& positions,
    LegJointSelection& selection, std::string& error) {
  if (names.size() != 2 || positions.size() != 2) {
    error = "Body-orientation MoveJ requires exactly two leg joint names and positions";
    return false;
  }
  const std::array<std::string, 3> supported{{"leg_joint1", "leg_joint2", "leg_joint3"}};
  std::array<bool, 3> used{};
  for (std::size_t i = 0; i < 2; ++i) {
    std::size_t index = 0;
    while (index < supported.size() && supported[index] != names[i]) ++index;
    if (index == supported.size() || used[index] || !std::isfinite(positions[i])) {
      error = "Select two distinct leg_joint1/leg_joint2/leg_joint3 joints with finite targets";
      return false;
    }
    used[index] = true;
    selection.commanded[i] = index;
  }
  for (std::size_t i = 0; i < used.size(); ++i)
    if (!used[i]) selection.compensated = i;
  error.clear();
  return true;
}

}  // namespace com_interfaces
