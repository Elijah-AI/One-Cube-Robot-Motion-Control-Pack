#ifndef MOTION_CONTROL_PKG_SDK_SDK_COMMAND_GATE_HPP_
#define MOTION_CONTROL_PKG_SDK_SDK_COMMAND_GATE_HPP_

#include <array>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "motion_control_pkg/sdk/dual_arm_device_interface.hpp"

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
  std::chrono::microseconds clear_timeout{std::chrono::microseconds(8000)};
  std::chrono::microseconds clear_retry_interval{std::chrono::microseconds(200)};
  std::chrono::milliseconds clear_error_settle_time{std::chrono::milliseconds(200)};
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
  void disableLogging();
  SdkCommandResult readState(DualArmDeviceState & state);

  SdkCommandResult initializePositionMode(
    int velocity_ratio = 10, int acceleration_ratio = 10);

  SdkCommandResult sendJointPosition(
    ArmChannel channel,
    const std::array<double, kArmDof> & joints);

  SdkCommandResult sendJointPositions(
    const std::optional<std::array<double, kArmDof>> & left_joints,
    const std::optional<std::array<double, kArmDof>> & right_joints);

  ActiveArmConfiguration configuration() const noexcept {return configuration_;}

private:
  bool beginCommandBatchWithRetry();
  SdkCommandResult fail(const std::string & operation) const;

  DualArmDeviceInterface & device_;
  ActiveArmConfiguration configuration_;
  SdkCommandGateOptions options_;
  mutable std::mutex mutex_;
};

}  // namespace motion_control_pkg::sdk

#endif  // MOTION_CONTROL_PKG_SDK_SDK_COMMAND_GATE_HPP_
