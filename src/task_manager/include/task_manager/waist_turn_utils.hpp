#ifndef TASK_MANAGER__WAIST_TURN_UTILS_HPP_
#define TASK_MANAGER__WAIST_TURN_UTILS_HPP_

#include <array>
#include <cmath>
#include <string>
#include <vector>
#include "sensor_msgs/msg/joint_state.hpp"

namespace task_act
{
// Expand a waist-only task command into the existing four-axis LEG Action.
// Require a complete, stopped snapshot; the caller additionally enforces freshness.
inline bool expand_waist_turn(
  const sensor_msgs::msg::JointState& state, double target, bool relative,
  std::vector<std::string>& names, std::vector<double>& positions, std::string& error)
{
  const std::array<std::string, 4> required{"leg_joint1", "leg_joint2", "leg_joint3", "body_joint"};
  if (!std::isfinite(target) || state.position.size() != state.name.size() ||
      state.velocity.size() != state.name.size()) {
    error = "waist turn needs finite target and complete joint positions/velocities"; return false;
  }
  std::vector<double> result;
  for (const auto& name : required) {
    size_t count = 0;
    for (size_t i = 0; i < state.name.size(); ++i) {
      if (state.name[i] != name) {continue;}
      ++count;
      if (!std::isfinite(state.position[i]) || !std::isfinite(state.velocity[i]) ||
          std::abs(state.velocity[i]) > 0.01) {
        error = "waist turn requires finite, stopped lower-body feedback"; return false;
      }
      result.push_back(state.position[i]);
    }
    if (count != 1) {error = "missing or duplicate lower-body feedback: " + name; return false;}
  }
  result[3] = relative ? result[3] + target : target;
  if (!std::isfinite(result[3])) {error = "invalid waist target"; return false;}
  names.assign(required.begin(), required.end());
  positions = std::move(result);
  return true;
}

inline bool expand_relative_waist_turn(
  const sensor_msgs::msg::JointState& state, double delta,
  std::vector<std::string>& names, std::vector<double>& positions, std::string& error)
{
  return expand_waist_turn(state, delta, true, names, positions, error);
}
}  // namespace task_act
#endif
