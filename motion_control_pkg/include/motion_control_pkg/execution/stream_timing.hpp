#ifndef MOTION_CONTROL_PKG_EXECUTION_STREAM_TIMING_HPP_
#define MOTION_CONTROL_PKG_EXECUTION_STREAM_TIMING_HPP_
#include <algorithm>
#include <cstdint>

namespace motion_control_pkg::execution
{
// A late wakeup or slow SDK transaction must not compress subsequent samples
// into a catch-up burst. Preserve the order and a full period between starts.
inline std::int64_t nextStreamTick(std::int64_t scheduled, std::int64_t started,
  std::int64_t completed, std::int64_t period)
{
  const auto next = std::max(scheduled + period, started + period);
  return completed > next ? completed + period : next;
}

struct StreamTiming
{
  std::int64_t previous{0}, maximum_gap{0}, maximum_lateness{0}, gap_sum{0};
  std::size_t samples{0}, late_samples{0};
  void observe(std::int64_t scheduled, std::int64_t started, std::int64_t period)
  {
    if (samples > 0) {
      const auto gap = started - previous;
      gap_sum += gap;
      maximum_gap = std::max(maximum_gap, gap);
    }
    maximum_lateness = std::max(maximum_lateness, started-scheduled);
    if (started-scheduled > period/4) {++late_samples;}
    previous = started;
    ++samples;
  }
};
}  // namespace motion_control_pkg::execution
#endif
