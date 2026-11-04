#pragma once
#include "motion_control_pkg/loong_driver_sdk.h"

namespace motion_control_pkg::sdk {
// Position feedback remains usable while drives are disabled. Acquisition
// timestamps, sequence advancement and finite values are checked by readers.
inline bool ethercatFeedbackValid(unsigned short status, unsigned short error) {
  return status != 0U && status != 0xffffU && error == 0U && (status & 0x0008U) == 0U;
}
inline bool ethercatFeedbackValid(const DriverSDK::motorActualStruct& motor) {
  return ethercatFeedbackValid(motor.statusWord, motor.errorCode);
}
// Match EthercatBusManager's existing enabled-state contract, including voltage
// enabled. A communication-valid status alone does not imply an enabled drive.
inline bool ethercatFeedbackOperational(const DriverSDK::motorActualStruct& motor) {
  return motor.errorCode == 0U && (motor.statusWord & 0x007fU) == 0x0037U;
}
}
