#pragma once
#include "ds485_driver/gripper_controller.h"
#include <string>

namespace ds485_driver {
// Same units and left/right layout as config/ds485_driver.yaml.
struct DriverConfig {
  std::array<GripperController::Config, 2> sides;
  int poll_ms = 20;
  double state_rate_hz = 10.0;
  int64_t default_timeout_ms = 10000;
};

// Pure C++ file loading; no ROS parameter services or hardware access.
// On failure, error contains the cause and output is left unchanged.
bool LoadGripperConfig(const std::string &path, DriverConfig &output, std::string &error);
} // namespace ds485_driver
