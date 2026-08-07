#include "fake_dual_arm_device.hpp"

#include <algorithm>
#include <utility>

namespace motion_control_pkg::test
{

bool FakeDualArmDevice::checkCompatibility()
{
  FakeCall call;
  call.operation = FakeOperation::kCheckCompatibility;
  return recordCall(std::move(call));
}

bool FakeDualArmDevice::connect(const std::array<unsigned char, 4> & ip)
{
  FakeCall call;
  call.operation = FakeOperation::kConnect;
  call.ip = ip;
  return recordCall(std::move(call));
}

bool FakeDualArmDevice::release()
{
  FakeCall call;
  call.operation = FakeOperation::kRelease;
  return recordCall(std::move(call));
}

void FakeDualArmDevice::disableLogging()
{
  FakeCall call;
  call.operation = FakeOperation::kDisableLogging;
  (void)recordCall(std::move(call));
}

bool FakeDualArmDevice::readState(
  motion_control_pkg::sdk::DualArmDeviceState & state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  FakeCall call;
  call.operation = FakeOperation::kReadState;
  call.thread_id = std::this_thread::get_id();
  calls_.push_back(std::move(call));
  const bool failed = consumeFailureLocked(FakeOperation::kReadState);
  if (!failed) {
    state = state_;
  }
  call_condition_.notify_all();
  return !failed;
}

bool FakeDualArmDevice::beginCommandBatch()
{
  FakeCall call;
  call.operation = FakeOperation::kBeginCommandBatch;
  return recordCall(std::move(call));
}

bool FakeDualArmDevice::clearServoError(
  motion_control_pkg::sdk::ArmChannel channel)
{
  FakeCall call;
  call.operation = FakeOperation::kClearServoError;
  call.channel = channel;
  return recordCall(std::move(call));
}

bool FakeDualArmDevice::setPositionMode(
  motion_control_pkg::sdk::ArmChannel channel)
{
  FakeCall call;
  call.operation = FakeOperation::kSetPositionMode;
  call.channel = channel;
  return recordCall(std::move(call));
}

bool FakeDualArmDevice::setTorqueMode(
  motion_control_pkg::sdk::ArmChannel channel)
{
  FakeCall call;
  call.operation = FakeOperation::kSetTorqueMode;
  call.channel = channel;
  return recordCall(std::move(call));
}

bool FakeDualArmDevice::setJointLimits(
  motion_control_pkg::sdk::ArmChannel channel,
  int velocity_ratio, int acceleration_ratio)
{
  FakeCall call;
  call.operation = FakeOperation::kSetJointLimits;
  call.channel = channel;
  call.velocity_ratio = velocity_ratio;
  call.acceleration_ratio = acceleration_ratio;
  return recordCall(std::move(call));
}

bool FakeDualArmDevice::setTool(
  motion_control_pkg::sdk::ArmChannel channel,
  const motion_control_pkg::sdk::ToolParameters & parameters)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & arm = state_.arms[static_cast<std::size_t>(channel)];
    arm.tool_kinematics = parameters.kinematics;
    arm.tool_dynamics = parameters.dynamics;
  }
  FakeCall call;
  call.operation = FakeOperation::kSetTool;
  call.channel = channel;
  return recordCall(std::move(call));
}

bool FakeDualArmDevice::setJointImpedanceGains(
  motion_control_pkg::sdk::ArmChannel channel,
  const std::array<double, motion_control_pkg::sdk::kArmDof> & stiffness,
  const std::array<double, motion_control_pkg::sdk::kArmDof> & damping)
{
  (void)stiffness;
  (void)damping;
  FakeCall call;
  call.operation = FakeOperation::kSetJointImpedanceGains;
  call.channel = channel;
  return recordCall(std::move(call));
}

bool FakeDualArmDevice::setCartesianImpedanceGains(
  motion_control_pkg::sdk::ArmChannel channel,
  const std::array<double, motion_control_pkg::sdk::kArmDof> & stiffness,
  const std::array<double, motion_control_pkg::sdk::kArmDof> & damping)
{
  (void)stiffness;
  (void)damping;
  FakeCall call;
  call.operation = FakeOperation::kSetCartesianImpedanceGains;
  call.channel = channel;
  return recordCall(std::move(call));
}

bool FakeDualArmDevice::setImpedanceType(
  motion_control_pkg::sdk::ArmChannel channel,
  motion_control_pkg::sdk::ArmControlMode mode)
{
  (void)mode;
  FakeCall call;
  call.operation = FakeOperation::kSetImpedanceType;
  call.channel = channel;
  return recordCall(std::move(call));
}

bool FakeDualArmDevice::setJointPosition(
  motion_control_pkg::sdk::ArmChannel channel,
  const std::array<double, motion_control_pkg::sdk::kArmDof> & joints)
{
  FakeCall call;
  call.operation = FakeOperation::kSetJointPosition;
  call.channel = channel;
  call.joints = joints;
  return recordCall(std::move(call));
}

bool FakeDualArmDevice::commitCommandBatch()
{
  FakeCall call;
  call.operation = FakeOperation::kCommitCommandBatch;
  return recordCall(std::move(call));
}

void FakeDualArmDevice::failNext(FakeOperation operation, std::size_t count)
{
  std::lock_guard<std::mutex> lock(mutex_);
  failures_remaining_[operation] += count;
}

void FakeDualArmDevice::setState(
  const motion_control_pkg::sdk::DualArmDeviceState & state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = state;
}

void FakeDualArmDevice::clearCalls()
{
  std::lock_guard<std::mutex> lock(mutex_);
  calls_.clear();
}

std::vector<FakeCall> FakeDualArmDevice::calls() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return calls_;
}

std::size_t FakeDualArmDevice::callCount(FakeOperation operation) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return callCountLocked(operation);
}

std::size_t FakeDualArmDevice::callCount(
  FakeOperation operation,
  motion_control_pkg::sdk::ArmChannel channel) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<std::size_t>(std::count_if(
      calls_.begin(), calls_.end(),
      [operation, channel](const FakeCall & call) {
        return call.operation == operation && call.channel == channel;
      }));
}

bool FakeDualArmDevice::waitForCallCount(
  FakeOperation operation, std::size_t count,
  std::chrono::steady_clock::duration timeout) const
{
  std::unique_lock<std::mutex> lock(mutex_);
  return call_condition_.wait_for(
    lock, timeout,
    [this, operation, count]() {
      return callCountLocked(operation) >= count;
    });
}

bool FakeDualArmDevice::recordCall(FakeCall call)
{
  std::lock_guard<std::mutex> lock(mutex_);
  call.thread_id = std::this_thread::get_id();
  const auto operation = call.operation;
  calls_.push_back(std::move(call));
  const bool failed = consumeFailureLocked(operation);
  call_condition_.notify_all();
  return !failed;
}

bool FakeDualArmDevice::consumeFailureLocked(FakeOperation operation)
{
  const auto failure = failures_remaining_.find(operation);
  if (failure == failures_remaining_.end() || failure->second == 0) {
    return false;
  }
  --failure->second;
  return true;
}

std::size_t FakeDualArmDevice::callCountLocked(FakeOperation operation) const
{
  return static_cast<std::size_t>(std::count_if(
      calls_.begin(), calls_.end(),
      [operation](const FakeCall & call) {
        return call.operation == operation;
      }));
}

}  // namespace motion_control_pkg::test
