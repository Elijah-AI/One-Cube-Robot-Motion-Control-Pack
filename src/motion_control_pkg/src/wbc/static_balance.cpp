#include "motion_control_pkg/wbc/static_balance.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_control_pkg::wbc {
StaticBalance::StaticBalance(const std::string& path, StaticBalanceOptions options,
                             const std::vector<std::string>& dirs)
  : options_(std::move(options)), model_(path), support_(path, options_.support_links, dirs) {
  if (!std::isfinite(options_.com_weight) || options_.com_weight < 0)
    throw std::invalid_argument("invalid COM weight");
  if (!std::isfinite(options_.inset) || options_.inset < 0)
    throw std::invalid_argument("invalid COM support inset");
  bounds_ = support_.steeringBounds(model_, options_.unknown_steering);
  region_ = support_.evaluate(model_.evaluate(model_.neutralSnapshot()),
    effectiveInset());
  if (!region_.geometry_valid) throw std::invalid_argument("invalid configured support: " + region_.error);
}
StatePolicy StaticBalance::statePolicy() const {
  StatePolicy policy;
  for (const auto& name : options_.unknown_steering) {
    const auto it = std::find_if(model_.joints().begin(), model_.joints().end(),
      [&](const auto& j) { return j.name == name; });
    policy.assumed_positions.emplace(name, std::clamp(0.0, it->lower, it->upper));
  }
  return policy;
}
BalanceEvaluation StaticBalance::evaluate(JointSnapshot state) const {
  const auto policy = statePolicy();
  // Always evaluate the nominal steering pose used to construct the robust
  // region. Unknown steering angles are never optimizer decision variables.
  for (std::size_t i = 0; i < state.names.size(); ++i) {
    auto assumed = policy.assumed_positions.find(state.names[i]);
    if (assumed != policy.assumed_positions.end()) {
      if (i < state.positions.size()) state.positions[i] = assumed->second;
      if (i < state.velocities.size()) state.velocities[i] = 0;
    }
  }
  auto result = model_.evaluate(state);
  for (std::size_t i = 0; i < result.joint_names.size(); ++i)
    if (policy.assumed_positions.count(result.joint_names[i])) result.com_jacobian.col(i).setZero();
  return {result.com, result.com_jacobian,
          (region_.b - region_.A * result.com.head<2>()).minCoeff()};
}
}  // namespace motion_control_pkg::wbc
