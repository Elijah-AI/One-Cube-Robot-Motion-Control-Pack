#include "motion_control_pkg/execution/dual_arm_command_coordinator.hpp"

#include <cmath>
#include <utility>

namespace motion_control_pkg::execution
{

bool TrajectoryTicket::requestCancel()
{
  if (!state_) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (isTerminal(state_->state)) {
      return false;
    }
  }
  state_->cancel_requested.store(true);
  if (state_->coupled_cancel_requested) {
    state_->coupled_cancel_requested->store(true);
  }
  return true;
}

TrajectoryExecutionSnapshot TrajectoryTicket::snapshot() const
{
  if (!state_) {
    return {
      TrajectoryExecutionState::kFailed, 0, 0,
      "invalid trajectory ticket"};
  }

  std::lock_guard<std::mutex> lock(state_->mutex);
  return {
    state_->state, state_->samples_sent, state_->sample_count,
    state_->error_message};
}

bool TrajectoryTicket::waitFor(
  std::chrono::steady_clock::duration timeout) const
{
  if (!state_) {
    return true;
  }
  std::unique_lock<std::mutex> lock(state_->mutex);
  return state_->condition.wait_for(
    lock, timeout, [this]() {return isTerminal(state_->state);});
}

DualArmCommandCoordinator::DualArmCommandCoordinator(
  motion_control_pkg::sdk::SdkCommandGate & command_gate,
  std::chrono::steady_clock::duration period)
: command_gate_(command_gate), period_(period)
{
}

DualArmCommandCoordinator::~DualArmCommandCoordinator()
{
  stop();
}

bool DualArmCommandCoordinator::start()
{
  if (stopped_.load() || period_ <= std::chrono::steady_clock::duration::zero()) {
    return false;
  }

  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    return false;
  }
  try {
    worker_ = std::thread(&DualArmCommandCoordinator::run, this);
  } catch (...) {
    running_.store(false);
    return false;
  }
  return true;
}

void DualArmCommandCoordinator::stop()
{
  stopped_.store(true);
  running_.store(false);
  wait_condition_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  cancelActiveTrajectories("dual arm command coordinator stopped");
}

TrajectoryEnqueueResult DualArmCommandCoordinator::enqueueTrajectory(
  motion_control_pkg::sdk::ArmChannel channel,
  std::vector<JointCommand> points)
{
  if (stopped_.load()) {
    return {false, "dual arm command coordinator is stopped", {}};
  }
  if (!motion_control_pkg::sdk::isArmEnabled(
      command_gate_.configuration(), channel))
  {
    return {
      false,
      channel == motion_control_pkg::sdk::ArmChannel::kLeft ?
      "left arm disabled by configuration" :
      "right arm disabled by configuration",
      {}};
  }
  if (points.empty()) {
    return {false, "trajectory must contain at least one sample", {}};
  }
  for (const auto & point : points) {
    for (const double value : point) {
      if (!std::isfinite(value)) {
        return {false, "trajectory contains a non-finite joint sample", {}};
      }
    }
  }

  auto state = std::make_shared<detail::SharedTrajectoryState>();
  state->sample_count = points.size();
  const auto index = static_cast<std::size_t>(channel);
  {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    auto & slot = slots_[index];
    if (slot.active) {
      return {
        false,
        channel == motion_control_pkg::sdk::ArmChannel::kLeft ?
        "left arm trajectory is already active" :
        "right arm trajectory is already active",
        {}};
    }
    slot.points = std::move(points);
    slot.next_index = 0;
    slot.state = state;
    slot.active = true;
  }

  return {true, {}, TrajectoryTicket(state)};
}

SynchronizedTrajectoryEnqueueResult
DualArmCommandCoordinator::enqueueSynchronizedPair(
  std::vector<JointCommand> left_points,
  std::vector<JointCommand> right_points)
{
  if (stopped_.load()) {
    return {false, "dual arm command coordinator is stopped", {}, {}};
  }
  if (command_gate_.configuration() !=
    motion_control_pkg::sdk::ActiveArmConfiguration::kDualArm)
  {
    return {
      false, "synchronized trajectory requires dual_arm configuration", {}, {}};
  }
  if (left_points.empty() || right_points.empty()) {
    return {
      false, "synchronized trajectories must both contain samples", {}, {}};
  }
  if (left_points.size() != right_points.size()) {
    return {
      false, "synchronized trajectories must have equal sample counts", {}, {}};
  }
  const auto all_finite = [](const std::vector<JointCommand> & points) {
      for (const auto & point : points) {
        for (const double value : point) {
          if (!std::isfinite(value)) {
            return false;
          }
        }
      }
      return true;
    };
  if (!all_finite(left_points) || !all_finite(right_points)) {
    return {
      false, "synchronized trajectory contains a non-finite joint sample", {}, {}};
  }

  auto left_state = std::make_shared<detail::SharedTrajectoryState>();
  auto right_state = std::make_shared<detail::SharedTrajectoryState>();
  auto coupled_cancel = std::make_shared<std::atomic<bool>>(false);
  left_state->coupled_cancel_requested = coupled_cancel;
  right_state->coupled_cancel_requested = coupled_cancel;
  left_state->sample_count = left_points.size();
  right_state->sample_count = right_points.size();

  {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    const auto left_index = static_cast<std::size_t>(
      motion_control_pkg::sdk::ArmChannel::kLeft);
    const auto right_index = static_cast<std::size_t>(
      motion_control_pkg::sdk::ArmChannel::kRight);
    if (slots_[left_index].active || slots_[right_index].active) {
      return {
        false, "cannot atomically enqueue while either arm trajectory is active",
        {}, {}};
    }

    slots_[left_index] = {
      std::move(left_points), 0U, left_state, true};
    slots_[right_index] = {
      std::move(right_points), 0U, right_state, true};
  }

  return {
    true, {}, TrajectoryTicket(left_state), TrajectoryTicket(right_state)};
}

void DualArmCommandCoordinator::run()
{
  auto next_cycle = std::chrono::steady_clock::now();
  while (running_.load()) {
    processOneCycle();
    next_cycle += period_;

    const auto now = std::chrono::steady_clock::now();
    if (next_cycle <= now) {
      // 超时周期不追赶补发，避免在一次调度延迟后形成密集命令突发。
      next_cycle = now + period_;
    }

    std::unique_lock<std::mutex> lock(wait_mutex_);
    wait_condition_.wait_until(
      lock, next_cycle, [this]() {return !running_.load();});
  }
}

void DualArmCommandCoordinator::processOneCycle()
{
  std::lock_guard<std::mutex> lock(slots_mutex_);

  std::optional<JointCommand> left_command;
  std::optional<JointCommand> right_command;
  std::array<bool, motion_control_pkg::sdk::kArmCount> selected{false, false};

  for (std::size_t index = 0; index < slots_.size(); ++index) {
    auto & slot = slots_[index];
    if (!slot.active || !slot.state) {
      continue;
    }
    if (slot.state->cancel_requested.load() ||
      (slot.state->coupled_cancel_requested &&
      slot.state->coupled_cancel_requested->load()))
    {
      finishSlot(
        slot, TrajectoryExecutionState::kCanceled,
        "trajectory canceled before the next 4 ms sample");
      continue;
    }
    if (slot.next_index >= slot.points.size()) {
      finishSlot(slot, TrajectoryExecutionState::kSucceeded);
      continue;
    }

    selected[index] = true;
    if (index == static_cast<std::size_t>(
        motion_control_pkg::sdk::ArmChannel::kLeft))
    {
      left_command = slot.points[slot.next_index];
    } else {
      right_command = slot.points[slot.next_index];
    }
  }

  if (!left_command && !right_command) {
    return;
  }

  // 整个 A/B 命令批次由 SdkCommandGate 保持为一个不可拆分事务。
  const auto command_result =
    command_gate_.sendJointPositions(left_command, right_command);
  if (!command_result) {
    for (std::size_t index = 0; index < slots_.size(); ++index) {
      if (selected[index]) {
        finishSlot(
          slots_[index], TrajectoryExecutionState::kFailed,
          command_result.error_message);
      }
    }
    return;
  }

  for (std::size_t index = 0; index < slots_.size(); ++index) {
    if (!selected[index]) {
      continue;
    }
    auto & slot = slots_[index];
    ++slot.next_index;
    {
      std::lock_guard<std::mutex> state_lock(slot.state->mutex);
      slot.state->samples_sent = slot.next_index;
      slot.state->state = TrajectoryExecutionState::kRunning;
    }
    slot.state->condition.notify_all();
    if (slot.next_index == slot.points.size()) {
      finishSlot(slot, TrajectoryExecutionState::kSucceeded);
    }
  }
}

void DualArmCommandCoordinator::finishSlot(
  ArmSlot & slot, TrajectoryExecutionState state,
  const std::string & error_message)
{
  if (!slot.state) {
    slot.active = false;
    return;
  }
  {
    std::lock_guard<std::mutex> state_lock(slot.state->mutex);
    slot.state->state = state;
    slot.state->error_message = error_message;
  }
  slot.state->condition.notify_all();
  slot.active = false;
  slot.points.clear();
  slot.next_index = 0;
}

void DualArmCommandCoordinator::cancelActiveTrajectories(
  const std::string & reason)
{
  std::lock_guard<std::mutex> lock(slots_mutex_);
  for (auto & slot : slots_) {
    if (slot.active) {
      finishSlot(slot, TrajectoryExecutionState::kCanceled, reason);
    }
  }
}

}  // namespace motion_control_pkg::execution
