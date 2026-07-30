#include "motion_control_pkg/sdk/sdk_command_gate.hpp"

#include <thread>

namespace motion_control_pkg::sdk
{

SdkCommandGate::SdkCommandGate(
  DualArmDeviceInterface & device,
  ActiveArmConfiguration configuration,
  SdkCommandGateOptions options)
: device_(device), configuration_(configuration), options_(options)
{
}

SdkCommandResult SdkCommandGate::connect(
  const std::array<unsigned char, 4> & ip)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!device_.checkCompatibility()) {
    return fail("type_compatibility");
  }
  if (!device_.connect(ip)) {
    return fail("connect");
  }
  return SdkCommandResult::ok();
}

SdkCommandResult SdkCommandGate::release()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!device_.release()) {
    return fail("release");
  }
  return SdkCommandResult::ok();
}

void SdkCommandGate::disableLogging()
{
  std::lock_guard<std::mutex> lock(mutex_);
  device_.disableLogging();
}

SdkCommandResult SdkCommandGate::readState(DualArmDeviceState & state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!device_.readState(state)) {
    return fail("read_state");
  }
  return SdkCommandResult::ok();
}

SdkCommandResult SdkCommandGate::initializePositionMode(
  int velocity_ratio, int acceleration_ratio)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!beginCommandBatchWithRetry()) {
    return fail("startup_clear_error_begin");
  }
  for (const auto channel : {ArmChannel::kLeft, ArmChannel::kRight}) {
    if (isArmEnabled(configuration_, channel) &&
      !device_.clearServoError(channel))
    {
      return fail(
        channel == ArmChannel::kLeft ?
        "clear_left_servo_error" : "clear_right_servo_error");
    }
  }
  if (!device_.commitCommandBatch()) {
    return fail("startup_clear_error_commit");
  }

  // 厂商建议清错命令发送后等待，测试环境可将该时间配置为零。
  if (options_.clear_error_settle_time.count() > 0) {
    std::this_thread::sleep_for(options_.clear_error_settle_time);
  }

  if (!beginCommandBatchWithRetry()) {
    return fail("position_mode_begin");
  }
  for (const auto channel : {ArmChannel::kLeft, ArmChannel::kRight}) {
    if (!isArmEnabled(configuration_, channel)) {
      continue;
    }
    if (!device_.setPositionMode(channel)) {
      return fail(
        channel == ArmChannel::kLeft ?
        "set_left_position_mode" : "set_right_position_mode");
    }
    if (!device_.setJointLimits(channel, velocity_ratio, acceleration_ratio)) {
      return fail(
        channel == ArmChannel::kLeft ?
        "set_left_joint_limits" : "set_right_joint_limits");
    }
  }
  if (!device_.commitCommandBatch()) {
    return fail("position_mode_commit");
  }
  return SdkCommandResult::ok();
}

SdkCommandResult SdkCommandGate::sendJointPosition(
  ArmChannel channel, const std::array<double, kArmDof> & joints)
{
  if (channel == ArmChannel::kLeft) {
    return sendJointPositions(joints, std::nullopt);
  }
  return sendJointPositions(std::nullopt, joints);
}

SdkCommandResult SdkCommandGate::sendJointPositions(
  const std::optional<std::array<double, kArmDof>> & left_joints,
  const std::optional<std::array<double, kArmDof>> & right_joints)
{
  if (!left_joints && !right_joints) {
    return fail("empty_joint_command");
  }
  if (left_joints &&
    !isArmEnabled(configuration_, ArmChannel::kLeft))
  {
    return fail("left_arm_disabled_by_configuration");
  }
  if (right_joints &&
    !isArmEnabled(configuration_, ArmChannel::kRight))
  {
    return fail("right_arm_disabled_by_configuration");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!beginCommandBatchWithRetry()) {
    return fail("joint_command_begin");
  }
  if (left_joints &&
    !device_.setJointPosition(ArmChannel::kLeft, *left_joints))
  {
    return fail("set_left_joint_position");
  }
  if (right_joints &&
    !device_.setJointPosition(ArmChannel::kRight, *right_joints))
  {
    return fail("set_right_joint_position");
  }
  if (!device_.commitCommandBatch()) {
    return fail("joint_command_commit");
  }
  return SdkCommandResult::ok();
}

bool SdkCommandGate::beginCommandBatchWithRetry()
{
  const auto deadline = std::chrono::steady_clock::now() + options_.clear_timeout;
  do {
    if (device_.beginCommandBatch()) {
      return true;
    }
    if (options_.clear_retry_interval.count() > 0) {
      std::this_thread::sleep_for(options_.clear_retry_interval);
    } else {
      std::this_thread::yield();
    }
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

SdkCommandResult SdkCommandGate::fail(const std::string & operation) const
{
  return SdkCommandResult::failure("MarvinSDK operation failed: " + operation);
}

}  // namespace motion_control_pkg::sdk
