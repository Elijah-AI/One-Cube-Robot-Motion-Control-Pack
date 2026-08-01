#include "motion_control_pkg/execution/synchronized_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace motion_control_pkg::execution
{
namespace
{

bool isFinite(const std::vector<JointCommand> & points)
{
  return std::all_of(points.begin(), points.end(), [](const auto & point) {
      return std::all_of(point.begin(), point.end(), [](double value) {
          return std::isfinite(value);
        });
    });
}

std::vector<JointCommand> resample(
  const std::vector<JointCommand> & input, std::size_t output_count)
{
  if (input.size() == output_count) {
    return input;
  }
  std::vector<JointCommand> output(output_count);
  if (output_count == 1U) {
    output.front() = input.front();
    return output;
  }
  if (input.size() == 1U) {
    std::fill(output.begin(), output.end(), input.front());
    return output;
  }

  const double input_span = static_cast<double>(input.size() - 1U);
  const double output_span = static_cast<double>(output_count - 1U);
  for (std::size_t sample = 0; sample < output_count; ++sample) {
    const double source = static_cast<double>(sample) * input_span / output_span;
    const auto lower = static_cast<std::size_t>(std::floor(source));
    const auto upper = std::min(lower + 1U, input.size() - 1U);
    const double fraction = source - static_cast<double>(lower);
    for (std::size_t joint = 0; joint < output[sample].size(); ++joint) {
      output[sample][joint] =
        input[lower][joint] +
        fraction * (input[upper][joint] - input[lower][joint]);
    }
  }
  output.front() = input.front();
  output.back() = input.back();
  return output;
}

}  // namespace

SynchronizedTrajectoryPair synchronizeTrajectoryPair(
  const std::vector<JointCommand> & left,
  const std::vector<JointCommand> & right)
{
  if (left.empty() || right.empty()) {
    return {false, "both trajectories must contain at least one sample", {}, {}};
  }
  if (!isFinite(left) || !isFinite(right)) {
    return {false, "trajectory contains a non-finite joint sample", {}, {}};
  }
  const std::size_t output_count = std::max(left.size(), right.size());
  return {
    true, {}, resample(left, output_count), resample(right, output_count)};
}

}  // namespace motion_control_pkg::execution
