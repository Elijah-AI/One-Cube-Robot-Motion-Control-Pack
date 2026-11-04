#pragma once
#include <cstddef>
#include <cstdint>

// Separate versioned C ABI; the existing per-motor feedback V1 is unchanged.
// CLOCK_MONOTONIC times are produced by the PDO thread, never by the reader.
struct OnecubeEthercatBusStateV1 {
  std::uint64_t sequence{};
  std::int64_t acquired_ns{}, ready_since_ns{};
  std::uint32_t expected_slaves{}, responding_slaves{};
  std::uint32_t domain_count{}, complete_domains{};
  std::uint32_t al_states{}, link_up{};
};
static_assert(sizeof(OnecubeEthercatBusStateV1)==48, "EtherCAT bus state ABI changed");
using OnecubeReadEthercatBusStateV1 = int(*)(unsigned int, OnecubeEthercatBusStateV1*, std::size_t);

inline bool onecubeEthercatBusOperational(const OnecubeEthercatBusStateV1& s) {
  return s.link_up && s.al_states==0x08 && s.expected_slaves>0 &&
      s.responding_slaves==s.expected_slaves && s.domain_count>0 &&
      s.complete_domains==s.domain_count;
}
