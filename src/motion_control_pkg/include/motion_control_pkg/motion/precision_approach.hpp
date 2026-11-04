#pragma once
#include "motion_control_pkg/execution/precision_policy.hpp"
#include <algorithm>
#include <limits>
#include <vector>

namespace motion_control_pkg::motion {
inline constexpr std::size_t kNoApproachGate = std::numeric_limits<std::size_t>::max();
struct PrecisionApproach {
  std::vector<std::vector<double>> positions, velocities, accelerations;
  std::size_t gate{kNoApproachGate};
};

// Two local state-to-state Ruckig plans (not the cloud waypoint API). Both
// stop at the shared approach point. Execution waits there for measured rest,
// then sends the slow suffix exactly once. Never reset commands to feedback or
// alter the requested target to compensate a static servo error.
// The caller must revalidate the combined path for collisions/COM/constraints.
template<class Plan>
bool planPrecisionApproach(const std::vector<double>& start, const std::vector<double>& target,
    const execution::PrecisionPolicy& policy, Plan plan, PrecisionApproach& result,
    std::string& error) {
  policy.validate();
  result = {};
  if (start.empty() || start.size() != target.size()) { error = "precision approach dimensions"; return false; }
  double distance = 0;
  for (std::size_t j=0;j<start.size();++j) {
    if (!std::isfinite(start[j]) || !std::isfinite(target[j])) { error = "non-finite precision target"; return false; }
    distance = std::max(distance, std::abs(target[j]-start[j]));
  }
  auto segment = [&](const auto& from, const auto& to, bool slow, PrecisionApproach& out) {
    if (!plan(from,to,slow,out.positions,out.velocities,out.accelerations,error)) return false;
    if (out.positions.empty() || out.positions.front()!=from || out.positions.back()!=to ||
        out.velocities.size()!=out.positions.size() || out.accelerations.size()!=out.positions.size()) {
      error="precision planner did not preserve endpoints or sample dimensions"; return false;
    }
    for (const auto* samples : {&out.positions,&out.velocities,&out.accelerations})
      for (const auto& q:*samples)
        if (q.size()!=from.size() || !std::all_of(q.begin(),q.end(),[](double x){return std::isfinite(x);})) {
          error="invalid precision samples";return false;
        }
    for (const auto* samples : {&out.velocities,&out.accelerations})
      for (std::size_t j=0;j<from.size();++j)
        if (std::abs(samples->front()[j])>1e-9 || std::abs(samples->back()[j])>1e-9) {
          error="precision segment must start and end at rest";return false;
        }
    return true;
  };
  PrecisionApproach built;
  if (!policy.approach_enabled || distance <= policy.approach_distance) {
    if (!segment(start,target,policy.approach_enabled,built)) return false;
  } else {
    std::vector<double> approach(start.size());
    const double fraction=policy.approach_distance/distance;
    for (std::size_t j=0;j<start.size();++j) approach[j]=target[j]-(target[j]-start[j])*fraction;
    if (!segment(start,approach,false,built)) return false;
    PrecisionApproach tail;
    if (!segment(approach,target,true,tail)) return false;
    built.gate=built.positions.size()-1;
    // Keep one copy of the shared resting position; the next sample is 4 ms later.
    built.positions.insert(built.positions.end(),tail.positions.begin()+1,tail.positions.end());
    built.velocities.insert(built.velocities.end(),tail.velocities.begin()+1,tail.velocities.end());
    built.accelerations.insert(built.accelerations.end(),tail.accelerations.begin()+1,tail.accelerations.end());
  }
  result=std::move(built);error.clear();return true;
}
}  // namespace motion_control_pkg::motion
