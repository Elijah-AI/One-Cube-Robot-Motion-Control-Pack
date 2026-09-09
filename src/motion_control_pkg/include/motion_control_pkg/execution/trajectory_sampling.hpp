#ifndef MOTION_CONTROL_PKG_EXECUTION_TRAJECTORY_SAMPLING_HPP_
#define MOTION_CONTROL_PKG_EXECUTION_TRAJECTORY_SAMPLING_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

namespace motion_control_pkg::execution
{

/**
 * @brief 生成包含起点和精确终点、且不重复终点的采样时间。
 *
 * 常规采样间隔等于 period；当总时长不是 period 的整数倍时，最后一个
 * 间隔会缩短，以确保最终目标点被显式包含一次。
 */
inline std::vector<double> makeInclusiveSampleTimes(
  double total_time, double period)
{
  if (!std::isfinite(total_time) || !std::isfinite(period) ||
    total_time < 0.0 || period <= 0.0)
  {
    return {};
  }

  std::vector<double> times{0.0};
  if (total_time == 0.0) {
    return times;
  }

  for (double time = period; time < total_time; time += period) {
    times.push_back(time);
  }

  const double tolerance =
    std::max(1.0, std::abs(total_time)) * 1e-12;
  if (std::abs(times.back() - total_time) <= tolerance) {
    times.back() = total_time;
  } else {
    times.push_back(total_time);
  }
  return times;
}

}  // namespace motion_control_pkg::execution

#endif  // MOTION_CONTROL_PKG_EXECUTION_TRAJECTORY_SAMPLING_HPP_
