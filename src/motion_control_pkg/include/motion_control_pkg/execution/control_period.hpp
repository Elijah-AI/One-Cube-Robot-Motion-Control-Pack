#ifndef MOTION_CONTROL_PKG_EXECUTION_CONTROL_PERIOD_HPP_
#define MOTION_CONTROL_PKG_EXECUTION_CONTROL_PERIOD_HPP_

#include <chrono>

#include "motion_control_pkg/ethercat_bus_manager.hpp"

namespace motion_control_pkg::execution
{

// 直接复用整机 EthercatBusManager 已定义的权威周期，不维护第二份数值。
// 这里只进行强类型转换；双臂通信仍走 MarvinSDK，不接入 EtherCAT 电机链。
inline constexpr std::chrono::microseconds kDualArmControlPeriod{
  TRAJ_CYCLE_US};
inline constexpr double kDualArmControlPeriodSeconds =
  static_cast<double>(TRAJ_CYCLE_US) / 1'000'000.0;

static_assert(
  TRAJ_CYCLE_US == 4000,
  "Stage four requires the framework control period to remain 4 ms");
static_assert(
  static_cast<int>(1'000'000.0 / TRAJ_FREQ) == TRAJ_CYCLE_US,
  "TRAJ_FREQ and TRAJ_CYCLE_US must describe the same framework period");

}  // namespace motion_control_pkg::execution

#endif  // MOTION_CONTROL_PKG_EXECUTION_CONTROL_PERIOD_HPP_
