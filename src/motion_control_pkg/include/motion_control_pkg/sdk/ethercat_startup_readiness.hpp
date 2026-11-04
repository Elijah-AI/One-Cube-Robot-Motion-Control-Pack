#pragma once
#include "motion_control_pkg/sdk/ethercat_bus_state_api.hpp"
#include "motion_control_pkg/sdk/ethercat_feedback_api.hpp"
#include <array>
#include <cmath>
#include <vector>

namespace motion_control_pkg::sdk {
constexpr std::int64_t kEthercatStartupStableNs = 2000000000LL;
constexpr std::int64_t kEthercatStartupMaxAgeNs = 20000000LL;
inline bool ethercatStartupReady(const OnecubeEthercatBusStateV1& bus,
    const std::vector<OnecubeEthercatFeedbackV1>& frames, std::int64_t now_ns) {
  if (!onecubeEthercatBusOperational(bus) || bus.expected_slaves!=6 || !bus.sequence ||
      bus.acquired_ns<=0 || bus.acquired_ns>now_ns ||
      now_ns-bus.acquired_ns>kEthercatStartupMaxAgeNs || bus.ready_since_ns<=0 ||
      bus.acquired_ns-bus.ready_since_ns<kEthercatStartupStableNs) return false;
  // The six EtherCAT axes in this robot; CAN/inactive indices are excluded.
  for (int id : {29,28,27,26,31,30}) {
    if (static_cast<std::size_t>(id)>=frames.size()) return false;
    const auto& f=frames[id];
    if (!f.sequence || f.acquired_ns<=0 || f.acquired_ns>now_ns ||
        now_ns-f.acquired_ns>kEthercatStartupMaxAgeNs ||
        f.status==0 || f.status==0xffff || !std::isfinite(f.position)) return false;
  }
  // Communication readiness is independent of CiA402 enable/fault state.
  // Existing fault-clear, enable and motion-safety checks still apply later.
  return true;
}
}
