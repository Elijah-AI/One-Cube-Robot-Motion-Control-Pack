#ifndef MOTION_CONTROL_PKG_SDK_SDK_COMMAND_GATE_HPP_
#define MOTION_CONTROL_PKG_SDK_SDK_COMMAND_GATE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "motion_control_pkg/sdk/dual_arm_device_interface.hpp"
#include "motion_control_pkg/sdk/drive_control.hpp"
#include "motion_control_pkg/execution/teach_standstill.hpp"

namespace motion_control_pkg::sdk
{

struct SdkCommandResult
{
  bool success{false};
  std::string error_message;

  explicit operator bool() const noexcept {return success;}

  static SdkCommandResult ok() {return {true, {}};}
  static SdkCommandResult failure(std::string message)
  {
    return {false, std::move(message)};
  }
};

struct SdkCommandGateOptions
{
  std::chrono::microseconds clear_timeout{std::chrono::microseconds(20000)};
  std::chrono::microseconds clear_retry_interval{std::chrono::microseconds(200)};
  std::chrono::milliseconds clear_error_settle_time{std::chrono::milliseconds(200)};
  std::chrono::milliseconds tool_parameter_settle_time{
    std::chrono::milliseconds(20)};
  std::chrono::milliseconds tool_readback_retry_interval{
    std::chrono::milliseconds(20)};
  std::size_t tool_readback_attempts{3};
  std::chrono::milliseconds impedance_parameter_settle_time{
    std::chrono::milliseconds(200)};
  std::chrono::milliseconds impedance_mode_settle_time{
    std::chrono::milliseconds(1000)};
  std::chrono::milliseconds mode_transition_timeout{std::chrono::milliseconds(2000)};
  std::chrono::milliseconds teach_transition_timeout{execution::kTeachStandstillTimeout};
};

/**
 * @brief 串行化 MarvinSDK 的命令事务。
 *
 * OnClearSet 到 OnSetSend 之间是一个不可拆分事务。所有手臂通道共享同一把
 * 锁，避免左右臂线程相互清空对方尚未发送的缓存命令。
 */
class SdkCommandGate
{
public:
  SdkCommandGate(
    DualArmDeviceInterface & device,
    ActiveArmConfiguration configuration,
    SdkCommandGateOptions options = {});

  SdkCommandResult connect(const std::array<unsigned char, 4> & ip);
  SdkCommandResult release();
  SdkCommandResult disableForShutdown();
  bool readShutdownFeedback(std::vector<DriveAxis>& axes, std::string& detail);
  void disableLogging();
  SdkCommandResult readState(DualArmDeviceState & state);
  // Caller must fence all motion and reject clearing while resources are busy.
  DriveControlResult controlDrives(bool left, bool right, bool clear_fault, bool recovery_stop=false,
      const std::function<bool()>& cancel_clear={});
  DriveControlResult setTeachMode(const std::vector<ArmControlModeCommand>& commands,
      bool enable, const std::function<bool()>& canceled);
  DriveControlResult recoverPositionMode(
      const std::function<bool(const DualArmDeviceState&,std::string&)>& check,
      const std::function<bool()>& canceled,
      const std::function<bool(const DualArmDeviceState&,std::string&)>& settling_check = {});
  unsigned teachingMask() const { return teaching_mask_.load(); }
  // Only the expected vendor transition, selected arm, live transaction epoch
  // and bounded lifetime qualify. This is for safety sampling, never admission.
  bool expectedModeTransition(ArmChannel channel,const ArmFeedback& feedback) const;

  SdkCommandResult initializePositionMode(
    int velocity_ratio = 10, int acceleration_ratio = 10);

  SdkCommandResult configureControlModes(
    const std::vector<ArmControlModeCommand> & commands,
    bool* command_issued = nullptr, const std::function<bool()>& canceled = {});

  SdkCommandResult sendJointPosition(
    ArmChannel channel,
    const std::array<double, kArmDof> & joints);

  SdkCommandResult sendJointPositions(
    const std::optional<std::array<double, kArmDof>> & left_joints,
    const std::optional<std::array<double, kArmDof>> & right_joints);

  ActiveArmConfiguration configuration() const noexcept {return configuration_;}

private:
  SdkCommandResult configureControlModesImpl(
      const std::vector<ArmControlModeCommand>& commands, bool* command_issued,
      const std::function<bool()>& canceled, bool teaching);
  struct ModeFeedbackScope {
    ModeFeedbackScope(SdkCommandGate&,unsigned mask,int target,
        std::chrono::milliseconds lifetime,bool mode_lock_held=false);
    ~ModeFeedbackScope();
    explicit operator bool() const {return active;}
    ModeFeedbackScope(const ModeFeedbackScope&)=delete;
    ModeFeedbackScope& operator=(const ModeFeedbackScope&)=delete;
    SdkCommandGate& gate;
    bool active{false};
  };
  bool beginCommandBatchWithRetry();
  bool modeTransitionBlocksCaller() const;
  SdkCommandResult fail(const std::string & operation) const;

  DualArmDeviceInterface & device_;
  ActiveArmConfiguration configuration_;
  SdkCommandGateOptions options_;
  mutable std::mutex mutex_;
  mutable std::mutex mode_transition_mutex_;
  std::mutex drive_clear_mutex_;
  std::atomic<std::uint64_t> drive_epoch_{0};
  std::atomic<unsigned> teaching_mask_{0};
  std::atomic<unsigned> mode_feedback_mask_{0};
  std::atomic<int> mode_feedback_target_{-1};
  std::atomic<std::int64_t> mode_feedback_deadline_ns_{0};
  std::atomic<std::uint64_t> mode_feedback_epoch_{0};
  std::thread::id mode_feedback_owner_{}; // protected by mutex_
  std::chrono::steady_clock::time_point last_joint_command_commit_time_{};
  bool has_last_joint_command_commit_time_{false};
  std::size_t joint_command_batch_id_{0};
  std::array<int, kArmCount> last_feedback_sequence_{{-1,-1}};
  std::array<std::chrono::steady_clock::time_point, kArmCount> feedback_acquired_{};
  bool released_{false};
};

}  // namespace motion_control_pkg::sdk

#endif  // MOTION_CONTROL_PKG_SDK_SDK_COMMAND_GATE_HPP_
