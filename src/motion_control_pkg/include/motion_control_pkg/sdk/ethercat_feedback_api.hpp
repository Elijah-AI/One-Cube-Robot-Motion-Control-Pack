#pragma once
#include <cstddef>
#include <cstdint>

// Optional C ABI supplied by the accompanying DriverSDK patch. The producer
// stamps only successfully received COMPLETE PDO frames, never application polls.
// Linux CLOCK_MONOTONIC nanoseconds, joint radians/rad/s and SDK effort units.
struct OnecubeEthercatFeedbackV1 {
  double position{}, velocity{}, effort{};
  std::uint64_t sequence{};
  std::int64_t acquired_ns{};
  std::uint16_t status{0xffff}, error{};
  std::int8_t mode{};
  std::uint8_t reserved[3]{};
};
static_assert(sizeof(OnecubeEthercatFeedbackV1)==48,"EtherCAT feedback ABI layout changed");
using OnecubeReadEthercatFeedbackV1 = int(*)(OnecubeEthercatFeedbackV1*,std::size_t,std::size_t);
