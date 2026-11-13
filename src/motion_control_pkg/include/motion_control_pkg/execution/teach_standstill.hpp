#pragma once

#include <chrono>

namespace motion_control_pkg::execution {
// Teaching transitions only. Recovery, drive disable, fault reset and ordinary
// motion admission keep their independent, stricter standstill requirements.
inline constexpr auto kTeachStandstillTimeout = std::chrono::milliseconds(5000);
inline constexpr double kTeachStandstillVelocity = 0.03;      // rad/s
inline constexpr double kTeachStandstillDisplacement = 0.05;  // rad from window origin
}  // namespace motion_control_pkg::execution
