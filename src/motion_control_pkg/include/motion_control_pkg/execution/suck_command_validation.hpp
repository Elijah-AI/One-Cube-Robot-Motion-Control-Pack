#ifndef MOTION_CONTROL_PKG_EXECUTION_SUCK_COMMAND_VALIDATION_HPP_
#define MOTION_CONTROL_PKG_EXECUTION_SUCK_COMMAND_VALIDATION_HPP_

#include <cstddef>
#include <string>

#include "com_interfaces/action/control_suck.hpp"

namespace motion_control_pkg::execution
{

struct SuckCommandValidation
{
  bool valid{false};
  std::string error_message;

  explicit operator bool() const noexcept {return valid;}
};

inline SuckCommandValidation validateControlSuckGoal(
  const com_interfaces::action::ControlSuck::Goal & goal)
{
  using Goal = com_interfaces::action::ControlSuck::Goal;

  if (goal.task_id.empty()) {
    return {false, "ControlSuck task_id must not be empty"};
  }
  if (goal.suck_id != Goal::SUCK_LEFT &&
      goal.suck_id != Goal::SUCK_RIGHT &&
      goal.suck_id != Goal::SUCK_BOTH)
  {
    return {false, "ControlSuck suck_id must be LEFT, RIGHT, or BOTH"};
  }
  constexpr std::size_t kRequiredElements = 2U;
  if (goal.active.size() < kRequiredElements) {
    return {false, "ControlSuck active must contain [left, right] values"};
  }
  if (goal.timeout_ms <= 0) {
    return {false, "ControlSuck timeout_ms must be positive"};
  }

  return {true, {}};
}

}  // namespace motion_control_pkg::execution

#endif  // MOTION_CONTROL_PKG_EXECUTION_SUCK_COMMAND_VALIDATION_HPP_
