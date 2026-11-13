#include "motion_control_pkg/execution/shutdown_request.hpp"
#include "motion_control_pkg/execution/motion_safety.hpp"
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
    if (!std::isfinite(actual[index]) || std::abs(expected[index] - actual[index]) > kReadbackTolerance) {
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

// Fence each SDK transaction separately. Settling/readback waits must not hold
// the monitor mutex. A partly applied mode transaction requires explicit reset.
class ModeCommandFence {
public:
  explicit ModeCommandFence(unsigned channels,const std::function<bool()>& canceled={},
      std::chrono::milliseconds timeout=std::chrono::milliseconds(2000),bool teaching=false)
      : safety_(motion_control_pkg::execution::motionSafety()) {
    if (safety_) {
      if(teaching)permit_.emplace(safety_->teachingConfigurationWhenStationary(channels,error_,canceled,timeout));
      else if(canceled)permit_.emplace(safety_->configurationWhenStationary(channels,error_,canceled,timeout));
      else permit_.emplace(safety_->configuration(channels,error_));
    }
  }
  ~ModeCommandFence() {
    if (permit_ && *permit_ && !complete_)
      permit_->failed("arm control mode transaction incomplete");
  }
  explicit operator bool() const {return !permit_ || static_cast<bool>(*permit_);}
  const std::string& error() const {return error_;}
  void complete() {complete_=true;}
private:
  std::shared_ptr<motion_control_pkg::execution::MotionSafety> safety_;
  std::string error_;
  std::optional<motion_control_pkg::execution::MotionSafety::CommandPermit> permit_;
  bool complete_{false};
};

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
  if (execution::shutdownRequested()) return fail("shutdown: connect refused");
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
  if (released_) return SdkCommandResult::ok();
  if (!device_.release()) {
    return fail("release");
  }
  released_=true;
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
  if (released_) return fail("driver already released");
  if (!device_.readState(state)) {
    return fail("read_state");
  }
  const auto observed = std::chrono::steady_clock::now();
  for (std::size_t a=0;a<kArmCount;++a) {
    const int seq=state.arms[a].feedback_sequence, previous=last_feedback_sequence_[a];
    if (seq<0 || seq>1000000 || (seq<previous && !(previous>900000 && seq<100000)))
      return fail("feedback_sequence_invalid_or_reversed");
    if (seq!=previous) { last_feedback_sequence_[a]=seq; feedback_acquired_[a]=observed; }
    state.arms[a].acquired=feedback_acquired_[a];
  }
  return SdkCommandResult::ok();
}

SdkCommandResult SdkCommandGate::initializePositionMode(
  int velocity_ratio, int acceleration_ratio)
{
  // Runtime recovery must not silently clear a drive fault or reinitialize it.
  if (motion_control_pkg::execution::motionSafety())
    return fail("startup initialization refused after motion monitor installation");
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
    if (execution::shutdownRequested() || !device_.commitCommandBatch()) {
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
  const std::vector<ArmControlModeCommand> & commands,
  bool* command_issued,const std::function<bool()>& canceled)
{
  return configureControlModesImpl(commands,command_issued,canceled,false);
}

SdkCommandResult SdkCommandGate::configureControlModesImpl(
  const std::vector<ArmControlModeCommand> & commands,
  bool* command_issued,const std::function<bool()>& canceled,bool teaching)
{
  const auto timeout=teaching?options_.teach_transition_timeout:options_.mode_transition_timeout;
  if(command_issued)*command_issued=false;
  if(canceled && canceled())return fail("control mode change canceled");
  if(modeTransitionBlocksCaller())return fail("normal mode changes blocked by a teaching/recovery transition");
  if (teaching_mask_.load()) return fail("normal mode changes blocked by teaching mode");
  if (const auto safety=motion_control_pkg::execution::motionSafety(); safety && safety->latched())
    return fail("control mode change refused while motion fault is latched");

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
  if(modeTransitionBlocksCaller())return fail("normal mode changes blocked by a teaching/recovery transition");
  if (teaching_mask_.load()) return fail("normal mode changes blocked by teaching mode");
  const unsigned channels=(seen[0]?1U:0U)|(seen[1]?2U:0U);

  if (!has_impedance) {
    ModeCommandFence fence(channels,canceled,timeout,teaching);
    if (!fence) return SdkCommandResult::failure(fence.error());
    std::lock_guard<std::mutex> lock(mutex_);
    if (!beginCommandBatchWithRetry()) {
      return fail("position_mode_begin");
    }
    if(command_issued)*command_issued=true;
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
    if (execution::shutdownRequested() || (canceled && canceled()) || !device_.commitCommandBatch()) {
      return fail("position_mode_commit");
    }
    fence.complete();
    return SdkCommandResult::ok();
  }

  // 先写入工具参数并回读确认，再设置 K/D 和扭矩/阻抗模式。
  {
    ModeCommandFence fence(channels,canceled,timeout,teaching);
    if (!fence) return SdkCommandResult::failure(fence.error());
    std::lock_guard<std::mutex> lock(mutex_);
    if (!beginCommandBatchWithRetry()) {
      return fail("tool_parameters_begin");
    }
    if(command_issued)*command_issued=true;
    for (const auto & command : commands) {
      if (command.mode != ArmControlMode::kPosition &&
        !device_.setTool(command.channel, command.tool))
      {
        return fail(
          command.channel == ArmChannel::kLeft ?
          "set_left_tool_parameters" : "set_right_tool_parameters");
      }
    }
    if (execution::shutdownRequested() || (canceled && canceled()) || !device_.commitCommandBatch()) {
      return fail("tool_parameters_commit");
    }
    fence.complete();
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
      if (released_ || execution::shutdownRequested()) return fail("driver released or shutdown requested");
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
    ModeCommandFence fence(channels,canceled,timeout,teaching);
    if (!fence) return SdkCommandResult::failure(fence.error());
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
    if (execution::shutdownRequested() || (canceled && canceled()) || !device_.commitCommandBatch()) {
      return fail("impedance_parameters_commit");
    }
    fence.complete();
  }

  if (options_.impedance_parameter_settle_time.count() > 0) {
    std::this_thread::sleep_for(options_.impedance_parameter_settle_time);
  }

  {
    ModeCommandFence fence(channels,canceled,timeout,teaching);
    if (!fence) return SdkCommandResult::failure(fence.error());
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
    if (execution::shutdownRequested() || (canceled && canceled()) || !device_.commitCommandBatch()) {
      return fail("impedance_mode_commit");
    }
    fence.complete();
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
  if (teaching_mask_.load() || mode_feedback_mask_.load())
    return fail("trajectory commands blocked by teaching mode or mode transition");
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

  using namespace motion_control_pkg::execution;
  const auto safety = motionSafety();
  std::optional<MotionSafety::CommandPermit> permit;
  std::vector<double> left_rad, right_rad;
  constexpr double radians = 0.017453292519943295;
  if (left_joints) for (double q:*left_joints) left_rad.push_back(q*radians);
  if (right_joints) for (double q:*right_joints) right_rad.push_back(q*radians);
  if (safety) {
    std::string error;
    permit.emplace(safety->command((left_joints?kLeftSafety:0U)|(right_joints?kRightSafety:0U),error));
    if (!*permit) return SdkCommandResult::failure(error);
    if ((left_joints && !permit->validate(kLeftSafety,left_rad)) ||
        (right_joints && !permit->validate(kRightSafety,right_rad)))
      return SdkCommandResult::failure("joint command rejected by motion safety");
  }
  const auto fail_motion = [&](const std::string& operation) {
    if (permit) permit->failed(operation);
    return fail(operation);
  };
  std::lock_guard<std::mutex> lock(mutex_);
  const auto batch_start_time = std::chrono::steady_clock::now();
  if (teaching_mask_.load() || mode_feedback_mask_.load())
    return fail_motion("trajectory commands blocked by teaching mode or mode transition");
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
    return fail_motion(
      "joint_command_begin; begin_elapsed_us=" +
      std::to_string(begin_elapsed_us) + "; since_last_commit_us=" +
      (has_last_joint_command_commit_time_ ?
      std::to_string(since_last_commit_us) : "N/A"));
  }
  if (left_joints &&
    !device_.setJointPosition(ArmChannel::kLeft, *left_joints))
  {
    return fail_motion("set_left_joint_position");
  }
  if (right_joints &&
    !device_.setJointPosition(ArmChannel::kRight, *right_joints))
  {
    return fail_motion("set_right_joint_position");
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
    return fail_motion("joint_command_commit");
  }
  if (permit) {
    if ((left_joints && !permit->committed(kLeftSafety,left_rad)) ||
        (right_joints && !permit->committed(kRightSafety,right_rad)))
      return fail_motion("motion command history invalid");
  }
  last_joint_command_commit_time_ = batch_end_time;
  has_last_joint_command_commit_time_ = true;
  ++joint_command_batch_id_;
  // Do not flush stdout per trajectory point while holding the command fence.
  // Bounded counters/timings are exposed by the motion safety diagnostic topic.
  return SdkCommandResult::ok();
}

bool SdkCommandGate::beginCommandBatchWithRetry()
{
  const auto deadline = std::chrono::steady_clock::now() + options_.clear_timeout;
  do {
    if (released_ || execution::shutdownRequested()) return false;
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
