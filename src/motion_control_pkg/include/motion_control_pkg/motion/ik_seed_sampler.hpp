#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace motion_control_pkg::motion::ik_seeds {

using Configuration = std::vector<double>;
constexpr double kPi = 3.14159265358979323846;
constexpr double radians(double degrees) { return degrees * kPi / 180.; }

enum class Stratum { Near, Medium, Far };
inline const char* name(Stratum value) {
  switch (value) {
    case Stratum::Near: return "near";
    case Stratum::Medium: return "medium";
    case Stratum::Far: return "far";
  }
  return "invalid";
}

struct ArmCoupling { std::size_t joint6, joint7; };

struct Request {
  Configuration reference, lower, upper;
  std::vector<std::size_t> active;
  std::vector<ArmCoupling> arms;
  // Only nearby seeds are appropriate for a Cartesian continuation request.
  bool continuation = false;
};

struct Options {
  std::uint32_t random_seed = 12345;
  double boundary_margin_rad = .015;
  double minimum_normalized_rms = .01;
  double continuation_radius_rad = radians(2.);
  double continuation_minimum_normalized_rms = .0005;
  std::size_t attempts_per_stratum = 512;
};

struct Seed {
  Configuration q;
  Stratum stratum = Stratum::Near;
  double distance_from_reference = 0.;
};

struct Statistics {
  std::size_t generated = 0, rejected_coupling = 0, rejected_stratum = 0;
  std::size_t rejected_distance = 0, rejected_external = 0;
  std::array<std::size_t, 3> accepted{};
  std::array<bool, 3> exhausted{};
  bool stopped = false;
};

struct Result {
  std::vector<Seed> seeds;
  Statistics statistics;
  bool complete = false;
};

inline bool coupledArmValid(const Configuration& q, ArmCoupling arm) {
  const double j6 = q.at(arm.joint6), j7 = q.at(arm.joint7);
  if (!std::isfinite(j6) || !std::isfinite(j7)) return false;
  if (std::abs(j6) <= radians(20.)) return true;
  const double limit = radians(109.5) - 1.025 * std::abs(j6);
  return limit >= 0. && std::abs(j7) <= limit + radians(1e-9);
}

inline double normalizedDistance(const Request& request,
                                 const Configuration& a, const Configuration& b) {
  double sum = 0.;
  for (auto j : request.active) {
    const double d = (a.at(j) - b.at(j)) / (request.upper.at(j) - request.lower.at(j));
    sum += d*d;
  }
  return std::sqrt(sum / request.active.size());
}

inline void validate(const Request& request, const Options& options) {
  const auto size = request.reference.size();
  if (!size || request.lower.size() != size || request.upper.size() != size || request.active.empty())
    throw std::invalid_argument("IK seed sampler: inconsistent configuration dimensions");
  std::vector<bool> seen(size, false);
  for (std::size_t j = 0; j < size; ++j) {
    if (!std::isfinite(request.reference[j]) || !std::isfinite(request.lower[j]) ||
        !std::isfinite(request.upper[j]) || request.lower[j] > request.upper[j] ||
        request.reference[j] < request.lower[j] - 1e-9 || request.reference[j] > request.upper[j] + 1e-9)
      throw std::invalid_argument("IK seed sampler: invalid reference or joint limits");
  }
  for (auto j : request.active) {
    if (j >= size || seen[j] || request.upper[j] - request.lower[j] <= 1e-12)
      throw std::invalid_argument("IK seed sampler: invalid or repeated active joint");
    seen[j] = true;
  }
  for (auto arm : request.arms) {
    if (arm.joint6 >= size || arm.joint7 >= size || arm.joint6 == arm.joint7)
      throw std::invalid_argument("IK seed sampler: invalid arm coupling indices");
    // An invalid physical starting configuration must be handled by the caller;
    // it must not be silently repaired into a different measured start pose.
    if (!coupledArmValid(request.reference, arm))
      throw std::invalid_argument("IK seed sampler: reference violates J6/J7 coupling");
  }
  if (!std::isfinite(options.boundary_margin_rad) || options.boundary_margin_rad < 0 ||
      !std::isfinite(options.minimum_normalized_rms) || options.minimum_normalized_rms <= 0 ||
      !std::isfinite(options.continuation_radius_rad) || options.continuation_radius_rad <= 0 ||
      !std::isfinite(options.continuation_minimum_normalized_rms) || options.continuation_minimum_normalized_rms <= 0 ||
      options.attempts_per_stratum == 0)
    throw std::invalid_argument("IK seed sampler: invalid sampling options");
}

// Additional seeds only: the failed reference configuration is never repeated.
// Each 4 accepted seeds request near/near/medium/far. Exhausted strata may be
// skipped, but limits, coupling and minimum distance are never relaxed.
// The first N seeds are independent of the requested final count.
inline Result generate(const Request& request, std::size_t count,
                       const Options& options = {},
                       const std::function<bool(const Configuration&)>& acceptable = {},
                       const std::function<bool()>& stop = {},
                       const std::function<void(std::size_t)>& on_accept = {}) {
  validate(request, options);
  if (count > 4096) throw std::invalid_argument("IK seed sampler: excessive seed request");
  Result result;
  result.seeds.reserve(count);
  std::mt19937 random(options.random_seed);
  auto unit = [&]() { return (static_cast<double>(random()) + .5) / 4294967296.; };
  std::vector<bool> active(request.reference.size(), false);
  for (auto j : request.active) active[j] = true;
  constexpr std::array<Stratum, 4> schedule{Stratum::Near, Stratum::Near, Stratum::Medium, Stratum::Far};
  const double minimum = request.continuation ? options.continuation_minimum_normalized_rms
                                               : options.minimum_normalized_rms;
  if (request.continuation) result.statistics.exhausted = {false, true, true};
  while (result.seeds.size() < count) {
    if (stop && stop()) { result.statistics.stopped = true; break; }
    const auto desired = request.continuation ? Stratum::Near : schedule[result.seeds.size() % schedule.size()];
    bool accepted = false;
    for (std::size_t offset = 0; offset < 3 && !accepted; ++offset) {
      const auto layer = (static_cast<std::size_t>(desired) + offset) % 3;
      if (result.statistics.exhausted[layer]) continue;
      const auto stratum = static_cast<Stratum>(layer);
      for (std::size_t attempt = 0; attempt < options.attempts_per_stratum; ++attempt) {
        if (stop && stop()) { result.statistics.stopped = true; return result; }
        ++result.statistics.generated;
        Configuration q = request.reference;
        Configuration lo = request.lower, hi = request.upper;
        for (auto j : request.active) {
          const auto span = request.upper[j] - request.lower[j];
          const auto margin = std::min(options.boundary_margin_rad, .01 * span);
          lo[j] += margin; hi[j] -= margin;
          if (request.continuation || stratum != Stratum::Far) {
            const auto radius = request.continuation ? options.continuation_radius_rad :
              stratum == Stratum::Near ? std::min(.06 * span, radians(15.)) :
                                        std::min(.18 * span, radians(45.));
            lo[j] = std::max(lo[j], request.reference[j] - radius);
            hi[j] = std::min(hi[j], request.reference[j] + radius);
          }
          if (lo[j] > hi[j]) { q.clear(); break; }
          q[j] = lo[j] + (hi[j] - lo[j]) * unit();
        }
        if (q.empty()) { ++result.statistics.rejected_coupling; continue; }
        bool coupling_ok = true;
        for (auto arm : request.arms) {
          if (active[arm.joint7] && std::abs(q[arm.joint6]) > radians(20.)) {
            const auto limit = radians(109.5) - 1.025 * std::abs(q[arm.joint6]);
            const auto margin = std::min(options.boundary_margin_rad, .01 * (request.upper[arm.joint7] - request.lower[arm.joint7]));
            const auto lower = std::max(lo[arm.joint7], -limit + margin);
            const auto upper = std::min(hi[arm.joint7], limit - margin);
            if (lower > upper) { coupling_ok = false; break; }
            q[arm.joint7] = lower + (upper - lower) * unit();
          }
          coupling_ok = coupling_ok && coupledArmValid(q, arm);
        }
        if (!coupling_ok) { ++result.statistics.rejected_coupling; continue; }
        const auto distance = normalizedDistance(request, q, request.reference);
        const bool in_stratum = request.continuation ||
          (stratum == Stratum::Near ? distance <= .06 :
           stratum == Stratum::Medium ? distance > .06 && distance <= .18 : distance > .18);
        if (!in_stratum) { ++result.statistics.rejected_stratum; continue; }
        bool separated = distance >= minimum;
        for (const auto& previous : result.seeds) {
          if (!separated) break;
          separated = normalizedDistance(request, q, previous.q) >= minimum;
        }
        if (!separated) { ++result.statistics.rejected_distance; continue; }
        if (acceptable && !acceptable(q)) { ++result.statistics.rejected_external; continue; }
        ++result.statistics.accepted[layer];
        result.seeds.push_back({std::move(q), stratum, distance});
        if (on_accept) on_accept(result.seeds.size());
        accepted = true;
        break;
      }
      if (!accepted) result.statistics.exhausted[layer] = true;
    }
    if (!accepted) break;
  }
  result.complete = result.seeds.size() == count;
  return result;
}

}  // namespace motion_control_pkg::motion::ik_seeds
