#include "motion_control_pkg/motion/ik_seed_sampler.hpp"
#include <iostream>

namespace seeds = motion_control_pkg::motion::ik_seeds;
static void check(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

int main() try {
  seeds::Request request;
  request.reference = {.8, -.7, .53, -.95, -.37, -.16, -.34, 1.25};
  request.lower = {-2.967, -2.094, -2.967, -2.531, -2.967, -seeds::radians(60), -seeds::radians(90), -2.};
  request.upper = {2.967, 1.047, 2.967, 1.047, 2.967, seeds::radians(60), seeds::radians(90), 2.};
  request.active = {0, 1, 2, 3, 4, 5, 6};
  request.arms = {{5, 6}};
  const auto all = seeds::generate(request, 96);
  check(all.complete && all.seeds.size() == 96, "full arm sample count");
  check(all.statistics.accepted == std::array<std::size_t, 3>{48, 24, 24}, "stratum proportions");
  for (std::size_t i = 0; i < all.seeds.size(); ++i) {
    const auto& q = all.seeds[i].q;
    check(q[7] == request.reference[7], "inactive joint must remain bitwise unchanged");
    check(seeds::coupledArmValid(q, request.arms[0]), "coupling must be valid");
    for (auto j : request.active)
      check(q[j] > request.lower[j] && q[j] < request.upper[j], "interior sampling without clamping");
    check(seeds::normalizedDistance(request, q, request.reference) >= .01, "reference must not be repeated");
    for (std::size_t k = 0; k < i; ++k)
      check(seeds::normalizedDistance(request, q, all.seeds[k].q) >= .01, "minimum pairwise separation");
  }
  for (auto count : {24U, 48U}) {
    const auto prefix = seeds::generate(request, count);
    for (std::size_t i = 0; i < count; ++i)
      check(prefix.seeds[i].q == all.seeds[i].q, "24/48/96 prefix must be stable");
  }
  auto line_request = request; line_request.continuation = true;
  const auto line = seeds::generate(line_request, 96);
  check(line.complete, "continuation sample count");
  for (const auto& seed : line.seeds) {
    check(seed.stratum == seeds::Stratum::Near, "continuation forbids distant strata");
    for (auto j : request.active)
      check(std::abs(seed.q[j] - request.reference[j]) <= seeds::radians(2.), "continuation radius");
  }
  std::size_t calls = 0;
  const auto filtered = seeds::generate(request, 24, {}, [&](const auto& q) { ++calls; return q[0] > request.reference[0]; });
  check(filtered.complete && filtered.statistics.rejected_external > 0, "external feasibility filtering");
  for (const auto& seed : filtered.seeds) check(seed.q[0] > request.reference[0], "external predicate preserved");
  check(calls == filtered.seeds.size() + filtered.statistics.rejected_external, "filter accounting");
  seeds::Options limited; limited.attempts_per_stratum = 8;
  const auto none = seeds::generate(request, 96, limited, [](const auto&) { return false; });
  check(!none.complete && none.seeds.empty() && none.statistics.generated == 24, "bounded exhaustion without invalid fill");
  std::size_t checks = 0;
  const auto cancelled = seeds::generate(request, 96, {}, {}, [&]() { return ++checks >= 10; });
  check(cancelled.statistics.stopped && !cancelled.complete && cancelled.seeds.size() < 96, "generation cancellation");
  auto invalid = request; invalid.reference[5] = seeds::radians(60.); invalid.reference[6] = seeds::radians(90.);
  bool rejected = false;
  try { seeds::generate(invalid, 1); } catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, "invalid physical reference must not be silently repaired");
  invalid = request; invalid.active.push_back(0); rejected = false;
  try { seeds::generate(invalid, 1); } catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, "duplicate active index rejection");
  seeds::Request head{{0., 0.}, {-2.92, -.62}, {2.92, .41}, {0, 1}, {}, false};
  const auto head_seeds = seeds::generate(head, 96);
  check(head_seeds.complete, "low dimensional head population");
  for (std::size_t i = 0; i < head_seeds.seeds.size(); ++i)
    for (std::size_t j = 0; j < i; ++j)
      check(seeds::normalizedDistance(head, head_seeds.seeds[i].q, head_seeds.seeds[j].q) >= .01, "head deduplication");
  std::cout << "PASS: bounds, coupling, frozen joints, strata, prefix, separation, continuation, filtering, exhaustion, cancellation, invalid inputs\n";
  return 0;
} catch (const std::exception& e) {
  std::cerr << "FAIL: " << e.what() << '\n';
  return 1;
}
