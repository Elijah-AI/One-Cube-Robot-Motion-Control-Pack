#ifndef MOTION_CONTROL_PKG_EXECUTION_JOINT_COMMAND_HPP_
#define MOTION_CONTROL_PKG_EXECUTION_JOINT_COMMAND_HPP_

#include <array>

#include "motion_control_pkg/sdk/dual_arm_device_interface.hpp"

namespace motion_control_pkg::execution
{

using JointCommand =
  std::array<double, motion_control_pkg::sdk::kArmDof>;

}  // namespace motion_control_pkg::execution

#endif  // MOTION_CONTROL_PKG_EXECUTION_JOINT_COMMAND_HPP_
