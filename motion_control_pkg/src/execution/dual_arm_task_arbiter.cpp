#include "motion_control_pkg/execution/dual_arm_task_arbiter.hpp"

#include <utility>

namespace motion_control_pkg::execution
{

namespace
{

constexpr std::size_t kLeftIndex = static_cast<std::size_t>(
  motion_control_pkg::sdk::ArmChannel::kLeft);
constexpr std::size_t kRightIndex = static_cast<std::size_t>(
  motion_control_pkg::sdk::ArmChannel::kRight);

}  // namespace

ArmTaskLease::ArmTaskLease(
  std::shared_ptr<ArmTaskArbitrationState> state,
  ArmTaskScope scope) noexcept
: state_(std::move(state)), scope_(scope)
{
}

ArmTaskLease::~ArmTaskLease()
{
  reset();
}

ArmTaskLease::ArmTaskLease(ArmTaskLease && other) noexcept
: state_(std::move(other.state_)), scope_(other.scope_)
{
}

ArmTaskLease & ArmTaskLease::operator=(ArmTaskLease && other) noexcept
{
  if (this != &other) {
    reset();
    state_ = std::move(other.state_);
    scope_ = other.scope_;
  }
  return *this;
}

void ArmTaskLease::reset() noexcept
{
  if (!state_) {
    return;
  }

  const auto claimed = DualArmTaskArbiter::maskFor(scope_);
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    for (std::size_t index = 0; index < claimed.size(); ++index) {
      if (claimed[index]) {
        state_->occupied[index] = false;
      }
    }
    if (scope_ == ArmTaskScope::kDual) {
      state_->dual_task_active = false;
    }
  }
  state_.reset();
}

DualArmTaskArbiter::DualArmTaskArbiter(
  motion_control_pkg::sdk::ActiveArmConfiguration configuration)
: state_(std::make_shared<ArmTaskArbitrationState>(configuration))
{
}

std::array<bool, motion_control_pkg::sdk::kArmCount>
DualArmTaskArbiter::maskFor(ArmTaskScope scope) noexcept
{
  switch (scope) {
    case ArmTaskScope::kLeft:
      return {true, false};
    case ArmTaskScope::kRight:
      return {false, true};
    case ArmTaskScope::kDual:
      return {true, true};
  }
  return {false, false};
}

ArmTaskAvailability DualArmTaskArbiter::checkAvailabilityLocked(
  const ArmTaskArbitrationState & state, ArmTaskScope scope)
{
  const auto requested = maskFor(scope);
  if (requested[kLeftIndex] && !motion_control_pkg::sdk::isArmEnabled(
      state.configuration, motion_control_pkg::sdk::ArmChannel::kLeft))
  {
    return {false, "left arm is disabled by active_arm_configuration"};
  }
  if (requested[kRightIndex] && !motion_control_pkg::sdk::isArmEnabled(
      state.configuration, motion_control_pkg::sdk::ArmChannel::kRight))
  {
    return {false, "right arm is disabled by active_arm_configuration"};
  }
  if (state.dual_task_active) {
    return {false, "a dual-arm task already owns both arms"};
  }
  if (requested[kLeftIndex] && state.occupied[kLeftIndex]) {
    return {false, "left arm is already occupied by another task"};
  }
  if (requested[kRightIndex] && state.occupied[kRightIndex]) {
    return {false, "right arm is already occupied by another task"};
  }
  return {true, {}};
}

ArmTaskAvailability DualArmTaskArbiter::canAcquire(ArmTaskScope scope) const
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  return checkAvailabilityLocked(*state_, scope);
}

ArmTaskAcquireResult DualArmTaskArbiter::tryAcquire(ArmTaskScope scope)
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  const auto availability = checkAvailabilityLocked(*state_, scope);
  if (!availability) {
    return {false, availability.error_message, {}};
  }

  const auto requested = maskFor(scope);
  for (std::size_t index = 0; index < requested.size(); ++index) {
    if (requested[index]) {
      state_->occupied[index] = true;
    }
  }
  state_->dual_task_active = scope == ArmTaskScope::kDual;
  return {true, {}, ArmTaskLease(state_, scope)};
}

ArmTaskArbitrationSnapshot DualArmTaskArbiter::snapshot() const
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  return {
    state_->occupied[kLeftIndex], state_->occupied[kRightIndex],
    state_->dual_task_active};
}

bool DualArmTaskTermination::tryFail(
  motion_control_pkg::sdk::ArmChannel failed_channel,
  const std::string & reason)
{
  const char * arm_name =
    failed_channel == motion_control_pkg::sdk::ArmChannel::kLeft ?
    "left" : "right";
  return tryStop(
    TerminalState::kAborted,
    std::string(arm_name) + " arm failure: " + reason);
}

bool DualArmTaskTermination::tryCancel(const std::string & reason)
{
  return tryStop(TerminalState::kCanceled, reason);
}

bool DualArmTaskTermination::trySucceed()
{
  return terminal_.tryFinish(TerminalState::kSucceeded);
}

std::string DualArmTaskTermination::errorMessage() const
{
  std::lock_guard<std::mutex> lock(error_mutex_);
  return error_message_;
}

bool DualArmTaskTermination::tryStop(
  TerminalState state, const std::string & reason)
{
  stop_requested_.store(true);
  std::lock_guard<std::mutex> lock(error_mutex_);
  if (!terminal_.tryFinish(state)) {
    return false;
  }
  error_message_ = reason;
  return true;
}

}  // namespace motion_control_pkg::execution
