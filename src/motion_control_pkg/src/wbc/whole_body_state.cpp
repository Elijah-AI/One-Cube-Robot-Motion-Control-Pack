#include "motion_control_pkg/wbc/model.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace motion_control_pkg::wbc {
WholeBodyState::WholeBodyState(std::vector<JointSpec> joints, StatePolicy policy)
  : policy_(std::move(policy)), joints_(std::move(joints)) {
  for (const auto& j : joints_) {
    if (j.name.empty() || !specs_.emplace(j.name, j).second ||
        !std::isfinite(j.lower) || !std::isfinite(j.upper) || j.lower > j.upper ||
        j.locked != (j.lower == j.upper)) {
      throw std::invalid_argument("invalid joint specification: " + j.name);
    }
  }
  if (!std::isfinite(policy_.filter_time_constant) || policy_.filter_time_constant < 0 ||
      !std::isfinite(policy_.min_dt) || policy_.min_dt <= 0 ||
      !std::isfinite(policy_.max_dt) || policy_.max_dt < policy_.min_dt)
    throw std::invalid_argument("invalid differentiation policy");
  for (const auto& p : policy_.assumed_positions) {
    auto j = specs_.find(p.first);
    if (j == specs_.end() || !std::isfinite(p.second) || p.second < j->second.lower || p.second > j->second.upper)
      throw std::invalid_argument("invalid assumed joint: " + p.first);
  }
  for (const auto& name : policy_.differentiate_joints)
    if (!specs_.count(name) || policy_.assumed_positions.count(name))
      throw std::invalid_argument("invalid differentiated joint: " + name);
}

bool WholeBodyState::update(const std::vector<std::string>& names,
                           const std::vector<double>& positions,
                           const std::vector<double>& velocities,
                           double stamp, std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  error.clear();
  if (names.empty() || names.size() != positions.size() ||
      (!velocities.empty() && velocities.size() != names.size()) ||
      !std::isfinite(stamp) || stamp <= 0.0) {
    error = "invalid packet size or acquisition timestamp";
    return false;
  }
  std::set<std::string> seen;
  for (std::size_t i = 0; i < names.size(); ++i) {
    auto spec = specs_.find(names[i]);
    if (spec == specs_.end() || !seen.insert(names[i]).second) {
      error = "unknown or duplicate joint: " + names[i];
      return false;
    }
    const auto& j = spec->second;
    if (!std::isfinite(positions[i]) || positions[i] < j.lower - 1e-8 ||
        positions[i] > j.upper + 1e-8 ||
        (!velocities.empty() && (!std::isfinite(velocities[i]) ||
          (j.locked && std::abs(velocities[i]) > 1e-8)))) {
      error = "non-finite state or URDF position/locked-joint violation: " + names[i];
      return false;
    }
    auto old = samples_.find(names[i]);
    if (old != samples_.end() && stamp <= old->second.stamp) {
      error = "non-increasing acquisition timestamp: " + names[i];
      return false;
    }
  }
  for (std::size_t i = 0; i < names.size(); ++i) {
    Sample next{positions[i], velocities.empty() ? 0.0 : velocities[i], stamp, !velocities.empty(), false};
    if (std::find(policy_.differentiate_joints.begin(), policy_.differentiate_joints.end(), names[i]) !=
        policy_.differentiate_joints.end()) {
      next.velocity_valid = false;
      auto old = samples_.find(names[i]);
      if (old != samples_.end()) {
        const double dt = stamp - old->second.stamp;
        const double raw = (positions[i] - old->second.position) / dt;
        // Bounded revolute joints are NOT wrapped through +/-pi. A jump or a
        // long gap resets estimation; it is never silently clamped into validity.
        if (dt >= policy_.min_dt && dt <= policy_.max_dt && std::isfinite(raw) &&
            std::abs(raw) <= specs_.at(names[i]).velocity + 1e-8) {
          const double alpha = dt / (policy_.filter_time_constant + dt);
          next.velocity = old->second.velocity_valid ?
            old->second.velocity + alpha * (raw - old->second.velocity) : raw;
          next.velocity_valid = next.estimated = true;
        }
      }
    }
    samples_[names[i]] = next;
  }
  return true;
}

JointSnapshot WholeBodyState::snapshot(double now, double max_age, double max_skew) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshotLocked(now, max_age, max_skew);
}

JointSnapshot WholeBodyState::snapshotNow(const std::function<double()>& clock, double max_age, double max_skew) const {
  if (!clock) throw std::invalid_argument("missing live state clock");
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshotLocked(clock(), max_age, max_skew);
}

JointSnapshot WholeBodyState::snapshotLocked(double now, double max_age, double max_skew) const {
  if (!std::isfinite(now) || now < 0.0 || !std::isfinite(max_age) || max_age <= 0.0 ||
      !std::isfinite(max_skew) || max_skew < 0.0) {
    throw std::invalid_argument("invalid state freshness policy");
  }
  JointSnapshot out;
  out.velocity_valid = true;
  out.oldest_stamp = std::numeric_limits<double>::infinity();
  for (const auto& j : joints_) {
    out.names.push_back(j.name);
    const auto assumed = policy_.assumed_positions.find(j.name);
    if (assumed != policy_.assumed_positions.end()) {
      out.positions.push_back(assumed->second);
      out.velocities.push_back(0.0);
      out.assumed.push_back(j.name);
      continue;
    }
    if (j.locked) {
      out.positions.push_back(j.lower);
      out.velocities.push_back(0.0);
      continue;
    }
    auto it = samples_.find(j.name);
    if (it == samples_.end()) {
      out.missing.push_back(j.name);
      out.positions.push_back(std::numeric_limits<double>::quiet_NaN());
      out.velocities.push_back(std::numeric_limits<double>::quiet_NaN());
      out.velocity_valid = false;
      continue;
    }
    const auto& s = it->second;
    out.positions.push_back(s.position);
    out.velocities.push_back(s.velocity_valid ? s.velocity :
                              std::numeric_limits<double>::quiet_NaN());
    out.oldest_stamp = std::min(out.oldest_stamp, s.stamp);
    out.newest_stamp = std::max(out.newest_stamp, s.stamp);
    if (now - s.stamp > max_age || s.stamp > now) out.stale.push_back(j.name);
    out.velocity_valid = out.velocity_valid && s.velocity_valid;
    if (s.estimated && s.velocity_valid) out.estimated_velocity.push_back(j.name);
  }
  if (!std::isfinite(out.oldest_stamp)) out.oldest_stamp = out.newest_stamp = now;
  if (out.newest_stamp - out.oldest_stamp > max_skew) out.stale.push_back("timestamp_skew");
  out.position_valid = out.missing.empty() && out.stale.empty();
  out.velocity_valid = out.velocity_valid && out.position_valid;
  return out;
}

void WholeBodyState::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
}
}  // namespace motion_control_pkg::wbc
