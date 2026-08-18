#include "motion_control_pkg/sdk/sdk_command_gate.hpp"

#include <cmath>
#include <cstdio>
#include <thread>

namespace motion_control_pkg::sdk
{

namespace
{

template<std::size_t Size>
bool arraysNear(
  const std::array<double, Size> & expected,
  const std::array<double, Size> & actual)
{
  constexpr double kReadbackTolerance = 1.0e-5;
  for (std::size_t index = 0; index < Size; ++index) {
    if (std::abs(expected[index] - actual[index]) > kReadbackTolerance) {
      return false;
    }
  }
  return true;
}

bool toolMatchesFeedback(
  const ToolParameters & expected, const ArmFeedback & actual)
{
  return arraysNear(expected.kinematics, actual.tool_kinematics) &&
         arraysNear(expected.dynamics, actual.tool_dynamics);
}

}  // namespace

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
  }

  // 厂商建议清错命令发送后等待，测试环境可将该时间配置为零。
  if (options_.clear_error_settle_time.count() > 0) {
    std::this_thread::sleep_for(options_.clear_error_settle_time);
  }

  std::vector<ArmControlModeCommand> commands;
  for (const auto channel : {ArmChannel::kLeft, ArmChannel::kRight}) {
    if (isArmEnabled(configuration_, channel)) {
      ArmControlModeCommand command;
      command.channel = channel;
      command.mode = ArmControlMode::kPosition;
      command.impedance.velocity_ratio = velocity_ratio;
      command.impedance.acceleration_ratio = acceleration_ratio;
      commands.push_back(command);
    }
  }
  return configureControlModes(commands);
}

SdkCommandResult SdkCommandGate::configureControlModes(
  const std::vector<ArmControlModeCommand> & commands)
{
  if (commands.empty()) {
    return fail("empty_control_mode_command");
  }

  std::array<bool, kArmCount> seen{false, false};
  bool has_impedance = false;
  for (const auto & command : commands) {
    const auto channel_index = static_cast<std::size_t>(command.channel);
    if (channel_index >= seen.size() || seen[channel_index]) {
      return fail("duplicate_or_invalid_control_mode_channel");
    }
    seen[channel_index] = true;
    if (!isArmEnabled(configuration_, command.channel)) {
      return fail("control_mode_arm_disabled_by_configuration");
    }
    if (command.impedance.velocity_ratio < 1 ||
      command.impedance.velocity_ratio > 100 ||
      command.impedance.acceleration_ratio < 1 ||
      command.impedance.acceleration_ratio > 100)
    {
      return fail("invalid_control_mode_joint_limits");
    }
    if (command.mode != ArmControlMode::kPosition &&
      command.mode != ArmControlMode::kJointImpedance &&
      command.mode != ArmControlMode::kCartesianImpedance)
    {
      return fail("invalid_control_mode");
    }
    if (command.mode == ArmControlMode::kPosition) {
      continue;
    }
    has_impedance = true;
    if (!isValidToolParameters(command.tool)) {
      return fail("invalid_tool_parameters");
    }
    for (std::size_t index = 0; index < kArmDof; ++index) {
      if (!std::isfinite(command.impedance.stiffness[index]) ||
        command.impedance.stiffness[index] < 0.0 ||
        !std::isfinite(command.impedance.damping[index]) ||
        command.impedance.damping[index] < 0.0 ||
        command.impedance.damping[index] > 1.0)
      {
        return fail("invalid_impedance_parameters");
      }
    }
  }

  std::lock_guard<std::mutex> transition_lock(mode_transition_mutex_);

  if (!has_impedance) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!beginCommandBatchWithRetry()) {
      return fail("position_mode_begin");
    }
    for (const auto & command : commands) {
      if (!device_.setPositionMode(command.channel)) {
        return fail(
          command.channel == ArmChannel::kLeft ?
          "set_left_position_mode" : "set_right_position_mode");
      }
      if (!device_.setJointLimits(
          command.channel, command.impedance.velocity_ratio,
          command.impedance.acceleration_ratio))
      {
        return fail(
          command.channel == ArmChannel::kLeft ?
          "set_left_joint_limits" : "set_right_joint_limits");
      }
    }
    if (!device_.commitCommandBatch()) {
      return fail("position_mode_commit");
    }
    return SdkCommandResult::ok();
  }

  // 先写入工具参数并回读确认，再设置 K/D 和扭矩/阻抗模式。
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!beginCommandBatchWithRetry()) {
      return fail("tool_parameters_begin");
    }
    for (const auto & command : commands) {
      if (command.mode != ArmControlMode::kPosition &&
        !device_.setTool(command.channel, command.tool))
      {
        return fail(
          command.channel == ArmChannel::kLeft ?
          "set_left_tool_parameters" : "set_right_tool_parameters");
      }
    }
    if (!device_.commitCommandBatch()) {
      return fail("tool_parameters_commit");
    }
  }

  if (options_.tool_parameter_settle_time.count() > 0) {
    std::this_thread::sleep_for(options_.tool_parameter_settle_time);
  }

  bool tool_parameters_verified = false;
  const std::size_t readback_attempts =
    options_.tool_readback_attempts == 0 ? 1 : options_.tool_readback_attempts;
  for (std::size_t attempt = 0; attempt < readback_attempts; ++attempt) {
    DualArmDeviceState state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!device_.readState(state)) {
        return fail("tool_parameters_readback");
      }
    }
    tool_parameters_verified = true;
    for (const auto & command : commands) {
      if (command.mode == ArmControlMode::kPosition) {
        continue;
      }
      const auto index = static_cast<std::size_t>(command.channel);
      if (!toolMatchesFeedback(command.tool, state.arms[index])) {
        tool_parameters_verified = false;
        break;
      }
    }
    if (tool_parameters_verified) {
      break;
    }
    if (attempt + 1 < readback_attempts &&
      options_.tool_readback_retry_interval.count() > 0)
    {
      std::this_thread::sleep_for(options_.tool_readback_retry_interval);
    }
  }
  if (!tool_parameters_verified) {
    return fail("tool_parameters_readback_mismatch");
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!beginCommandBatchWithRetry()) {
      return fail("impedance_parameters_begin");
    }
    for (const auto & command : commands) {
      if (!device_.setJointLimits(
          command.channel, command.impedance.velocity_ratio,
          command.impedance.acceleration_ratio))
      {
        return fail(
          command.channel == ArmChannel::kLeft ?
          "set_left_impedance_joint_limits" :
          "set_right_impedance_joint_limits");
      }
      if (command.mode == ArmControlMode::kJointImpedance &&
        !device_.setJointImpedanceGains(
          command.channel, command.impedance.stiffness,
          command.impedance.damping))
      {
        return fail(
          command.channel == ArmChannel::kLeft ?
          "set_left_joint_impedance_gains" :
          "set_right_joint_impedance_gains");
      }
      if (command.mode == ArmControlMode::kCartesianImpedance &&
        !device_.setCartesianImpedanceGains(
          command.channel, command.impedance.stiffness,
          command.impedance.damping))
      {
        return fail(
          command.channel == ArmChannel::kLeft ?
          "set_left_cartesian_impedance_gains" :
          "set_right_cartesian_impedance_gains");
      }
    }
    if (!device_.commitCommandBatch()) {
      return fail("impedance_parameters_commit");
    }
  }

  if (options_.impedance_parameter_settle_time.count() > 0) {
    std::this_thread::sleep_for(options_.impedance_parameter_settle_time);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!beginCommandBatchWithRetry()) {
      return fail("impedance_mode_begin");
    }
    for (const auto & command : commands) {
      if (command.mode == ArmControlMode::kPosition) {
        if (!device_.setPositionMode(command.channel)) {
          return fail(
            command.channel == ArmChannel::kLeft ?
            "set_left_position_mode" : "set_right_position_mode");
        }
        continue;
      }
      if (!device_.setTorqueMode(command.channel)) {
        return fail(
          command.channel == ArmChannel::kLeft ?
          "set_left_torque_mode" : "set_right_torque_mode");
      }
      if (!device_.setImpedanceType(command.channel, command.mode)) {
        return fail(
          command.channel == ArmChannel::kLeft ?
          "set_left_impedance_type" : "set_right_impedance_type");
      }
    }
    if (!device_.commitCommandBatch()) {
      return fail("impedance_mode_commit");
    }
  }

  if (options_.impedance_mode_settle_time.count() > 0) {
    std::this_thread::sleep_for(options_.impedance_mode_settle_time);
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
  const auto batch_start_time = std::chrono::steady_clock::now();
  const auto batch_start_timestamp_ms = std::chrono::duration_cast<
    std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  if (!beginCommandBatchWithRetry()) {
    const auto failure_time = std::chrono::steady_clock::now();
    const auto failure_timestamp_ms = std::chrono::duration_cast<
      std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto begin_elapsed_us = std::chrono::duration_cast<
      std::chrono::microseconds>(failure_time - batch_start_time).count();
    const auto since_last_commit_us = has_last_joint_command_commit_time_ ?
      std::chrono::duration_cast<std::chrono::microseconds>(
      failure_time - last_joint_command_commit_time_).count() : -1;
    if (has_last_joint_command_commit_time_) {
      std::fprintf(
        stderr,
        "[SdkCommandGate] begin_timestamp_ms=%lld failure_timestamp_ms=%lld "
        "joint batch begin=false begin_elapsed_us=%lld "
        "since_last_commit_us=%lld\n",
        static_cast<long long>(batch_start_timestamp_ms),
        static_cast<long long>(failure_timestamp_ms),
        static_cast<long long>(begin_elapsed_us),
        static_cast<long long>(since_last_commit_us));
    } else {
      std::fprintf(
        stderr,
        "[SdkCommandGate] begin_timestamp_ms=%lld failure_timestamp_ms=%lld "
        "joint batch begin=false begin_elapsed_us=%lld "
        "since_last_commit_us=N/A\n",
        static_cast<long long>(batch_start_timestamp_ms),
        static_cast<long long>(failure_timestamp_ms),
        static_cast<long long>(begin_elapsed_us));
    }
    return fail(
      "joint_command_begin; begin_elapsed_us=" +
      std::to_string(begin_elapsed_us) + "; since_last_commit_us=" +
      (has_last_joint_command_commit_time_ ?
      std::to_string(since_last_commit_us) : "N/A"));
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
  const bool commit_succeeded = device_.commitCommandBatch();
  const auto batch_end_time = std::chrono::steady_clock::now();
  // const auto batch_elapsed_us = std::chrono::duration_cast<
  //   std::chrono::microseconds>(batch_end_time - batch_start_time).count();
  // const auto batch_end_timestamp_ms = std::chrono::duration_cast<
  //   std::chrono::milliseconds>(
  //   std::chrono::system_clock::now().time_since_epoch()).count();
  // std::fprintf(
  //   stderr,
  //   "[SdkCommandGate] begin_timestamp_ms=%lld commit_timestamp_ms=%lld "
  //   "joint batch begin-to-commit elapsed_us=%lld commit=%s\n",
  //   static_cast<long long>(batch_start_timestamp_ms),
  //   static_cast<long long>(batch_end_timestamp_ms),
  //   static_cast<long long>(batch_elapsed_us),
  //   commit_succeeded ? "true" : "false");
  if (!commit_succeeded) {
    return fail("joint_command_commit");
  }
  last_joint_command_commit_time_ = batch_end_time;
  has_last_joint_command_commit_time_ = true;
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
