#ifndef MOTION_CONTROL_PKG_TEST_FAKE_DUAL_ARM_DEVICE_HPP_
#define MOTION_CONTROL_PKG_TEST_FAKE_DUAL_ARM_DEVICE_HPP_

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "motion_control_pkg/sdk/dual_arm_device_interface.hpp"

namespace motion_control_pkg::test
{

enum class FakeOperation
{
  kCheckCompatibility,
  kConnect,
  kRelease,
  kDisableLogging,
  kReadState,
  kBeginCommandBatch,
  kClearServoError,
  kSetPositionMode,
  kSetJointLimits,
  kSetJointPosition,
  kCommitCommandBatch,
};

struct FakeCall
{
  FakeOperation operation{FakeOperation::kCheckCompatibility};
  std::optional<motion_control_pkg::sdk::ArmChannel> channel;
  std::array<double, motion_control_pkg::sdk::kArmDof> joints{};
  std::array<unsigned char, 4> ip{};
  int velocity_ratio{0};
  int acceleration_ratio{0};
  std::thread::id thread_id;
};

/**
 * @brief 可故障注入的线程安全双臂假设备。
 *
 * 该类型只实现厂商无关接口，不包含 MarvinSDK 头文件、符号或网络通信。
 * 所有调用都会被记录，供测试验证 OnClearSet -> A/B -> OnSetSend 的事务顺序。
 */
class FakeDualArmDevice final
  : public motion_control_pkg::sdk::DualArmDeviceInterface
{
public:
  bool checkCompatibility() override;
  bool connect(const std::array<unsigned char, 4> & ip) override;
  bool release() override;
  void disableLogging() override;
  bool readState(motion_control_pkg::sdk::DualArmDeviceState & state) override;

  bool beginCommandBatch() override;
  bool clearServoError(motion_control_pkg::sdk::ArmChannel channel) override;
  bool setPositionMode(motion_control_pkg::sdk::ArmChannel channel) override;
  bool setJointLimits(
    motion_control_pkg::sdk::ArmChannel channel,
    int velocity_ratio, int acceleration_ratio) override;
  bool setJointPosition(
    motion_control_pkg::sdk::ArmChannel channel,
    const std::array<double, motion_control_pkg::sdk::kArmDof> & joints) override;
  bool commitCommandBatch() override;

  void failNext(FakeOperation operation, std::size_t count = 1);
  void setState(const motion_control_pkg::sdk::DualArmDeviceState & state);
  void clearCalls();

  std::vector<FakeCall> calls() const;
  std::size_t callCount(FakeOperation operation) const;
  std::size_t callCount(
    FakeOperation operation,
    motion_control_pkg::sdk::ArmChannel channel) const;
  bool waitForCallCount(
    FakeOperation operation, std::size_t count,
    std::chrono::steady_clock::duration timeout) const;

private:
  bool recordCall(FakeCall call);
  bool consumeFailureLocked(FakeOperation operation);
  std::size_t callCountLocked(FakeOperation operation) const;

  mutable std::mutex mutex_;
  mutable std::condition_variable call_condition_;
  std::vector<FakeCall> calls_;
  std::map<FakeOperation, std::size_t> failures_remaining_;
  motion_control_pkg::sdk::DualArmDeviceState state_{};
};

}  // namespace motion_control_pkg::test

#endif  // MOTION_CONTROL_PKG_TEST_FAKE_DUAL_ARM_DEVICE_HPP_
