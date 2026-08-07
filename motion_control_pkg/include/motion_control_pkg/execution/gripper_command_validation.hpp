#ifndef MOTION_CONTROL_PKG_EXECUTION_GRIPPER_COMMAND_VALIDATION_HPP_
#define MOTION_CONTROL_PKG_EXECUTION_GRIPPER_COMMAND_VALIDATION_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "com_interfaces/action/control_gripper.hpp"

namespace motion_control_pkg::execution
{

struct GripperCommandValidation
{
  bool valid{false};
  std::string error_message;

  explicit operator bool() const noexcept {return valid;}
};

inline bool isSupportedGripperSerialChannel(std::uint8_t channel) noexcept
{
  // Marvin tool channels: 2=COM1, 3=COM2. Channel 1 is CAN/CAN-FD and
  // must not be used for the Jodell Modbus-RTU transport.
  return channel == 2U || channel == 3U;
}

inline GripperCommandValidation validateControlGripperGoal(
  const com_interfaces::action::ControlGripper::Goal & goal)
{
  using Goal = com_interfaces::action::ControlGripper::Goal;

  if (goal.task_id.empty()) {
    return {false, "ControlGripper task_id must not be empty"};
  }
  if (goal.gripper_id != Goal::GRIPPER_LEFT &&
      goal.gripper_id != Goal::GRIPPER_RIGHT &&
      goal.gripper_id != Goal::GRIPPER_BOTH)
  {
    return {false, "ControlGripper gripper_id must be LEFT, RIGHT, or BOTH"};
  }
  constexpr std::size_t kRequiredElements = 2U;
  if (goal.position.size() < kRequiredElements ||
      goal.speed.size() < kRequiredElements ||
      goal.max_torque.size() < kRequiredElements ||
      goal.com_port.size() < kRequiredElements)
  {
    return {
      false,
      "ControlGripper position/speed/max_torque/com_port arrays must each "
      "contain at least two elements"};
  }
  if (!goal.block_until_complete) {
    return {
      false,
      "ControlGripper currently requires block_until_complete=true so the "
      "Action result reflects the physical gripper state"};
  }
  if (goal.timeout_ms <= 0) {
    return {false, "ControlGripper timeout_ms must be positive"};
  }

  const bool needs_left =
    goal.gripper_id == Goal::GRIPPER_LEFT ||
    goal.gripper_id == Goal::GRIPPER_BOTH;
  const bool needs_right =
    goal.gripper_id == Goal::GRIPPER_RIGHT ||
    goal.gripper_id == Goal::GRIPPER_BOTH;
  if (needs_left && !isSupportedGripperSerialChannel(goal.com_port[0])) {
    return {false, "ControlGripper left com_port must be 2 (COM1) or 3 (COM2)"};
  }
  if (needs_right && !isSupportedGripperSerialChannel(goal.com_port[1])) {
    return {false, "ControlGripper right com_port must be 2 (COM1) or 3 (COM2)"};
  }

  return {true, {}};
}

}  // namespace motion_control_pkg::execution

#endif  // MOTION_CONTROL_PKG_EXECUTION_GRIPPER_COMMAND_VALIDATION_HPP_
