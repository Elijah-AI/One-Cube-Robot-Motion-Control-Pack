#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "fake_dual_arm_device.hpp"
#include "motion_control_pkg/sdk/sdk_command_gate.hpp"

namespace
{

using motion_control_pkg::sdk::ActiveArmConfiguration;
using motion_control_pkg::sdk::ArmChannel;
using motion_control_pkg::sdk::SdkCommandGate;
using motion_control_pkg::sdk::SdkCommandGateOptions;
using motion_control_pkg::test::FakeCall;
using motion_control_pkg::test::FakeDualArmDevice;
using motion_control_pkg::test::FakeOperation;

std::array<double, motion_control_pkg::sdk::kArmDof> makeJoints(double offset)
{
  std::array<double, motion_control_pkg::sdk::kArmDof> joints{};
  for (std::size_t index = 0; index < joints.size(); ++index) {
    joints[index] = offset + static_cast<double>(index);
  }
  return joints;
}

void expectDualJointBatch(
  const std::vector<FakeCall> & calls, std::size_t offset)
{
  ASSERT_GE(calls.size(), offset + 4U);
  EXPECT_EQ(calls[offset].operation, FakeOperation::kBeginCommandBatch);
  EXPECT_EQ(calls[offset + 1U].operation, FakeOperation::kSetJointPosition);
  EXPECT_EQ(calls[offset + 1U].channel, ArmChannel::kLeft);
  EXPECT_EQ(calls[offset + 2U].operation, FakeOperation::kSetJointPosition);
  EXPECT_EQ(calls[offset + 2U].channel, ArmChannel::kRight);
  EXPECT_EQ(calls[offset + 3U].operation, FakeOperation::kCommitCommandBatch);
}

TEST(SdkCommandGateTest, DualArmJointCommandIsOneOrderedSdkBatch)
{
  FakeDualArmDevice device;
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm);
  const auto left = makeJoints(10.0);
  const auto right = makeJoints(20.0);

  const auto result = gate.sendJointPositions(left, right);

  ASSERT_TRUE(result.success) << result.error_message;
  const auto calls = device.calls();
  ASSERT_EQ(calls.size(), 4U);
  expectDualJointBatch(calls, 0U);
  EXPECT_EQ(calls[1].joints, left);
  EXPECT_EQ(calls[2].joints, right);
}

TEST(SdkCommandGateTest, PositionModeInitializesBothArms)
{
  FakeDualArmDevice device;
  SdkCommandGateOptions options;
  options.clear_error_settle_time = std::chrono::milliseconds::zero();
  SdkCommandGate gate(
    device, ActiveArmConfiguration::kDualArm, options);

  const auto result = gate.initializePositionMode(7, 9);

  ASSERT_TRUE(result.success) << result.error_message;
  const auto calls = device.calls();
  ASSERT_EQ(calls.size(), 10U);
  EXPECT_EQ(calls[0].operation, FakeOperation::kBeginCommandBatch);
  EXPECT_EQ(calls[1].operation, FakeOperation::kClearServoError);
  EXPECT_EQ(calls[1].channel, ArmChannel::kLeft);
  EXPECT_EQ(calls[2].operation, FakeOperation::kClearServoError);
  EXPECT_EQ(calls[2].channel, ArmChannel::kRight);
  EXPECT_EQ(calls[3].operation, FakeOperation::kCommitCommandBatch);
  EXPECT_EQ(calls[4].operation, FakeOperation::kBeginCommandBatch);
  EXPECT_EQ(calls[5].operation, FakeOperation::kSetPositionMode);
  EXPECT_EQ(calls[5].channel, ArmChannel::kLeft);
  EXPECT_EQ(calls[6].operation, FakeOperation::kSetJointLimits);
  EXPECT_EQ(calls[6].channel, ArmChannel::kLeft);
  EXPECT_EQ(calls[6].velocity_ratio, 7);
  EXPECT_EQ(calls[6].acceleration_ratio, 9);
  EXPECT_EQ(calls[7].operation, FakeOperation::kSetPositionMode);
  EXPECT_EQ(calls[7].channel, ArmChannel::kRight);
  EXPECT_EQ(calls[8].operation, FakeOperation::kSetJointLimits);
  EXPECT_EQ(calls[8].channel, ArmChannel::kRight);
  EXPECT_EQ(calls[9].operation, FakeOperation::kCommitCommandBatch);
  EXPECT_EQ(
    device.callCount(FakeOperation::kSetPositionMode, ArmChannel::kRight),
    1U);
}

TEST(SdkCommandGateTest, SdkCommitFailureIsReturnedToCaller)
{
  FakeDualArmDevice device;
  device.failNext(FakeOperation::kCommitCommandBatch);
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm);

  const auto result = gate.sendJointPositions(
    makeJoints(1.0), makeJoints(2.0));

  EXPECT_FALSE(result.success);
  EXPECT_NE(
    result.error_message.find("joint_command_commit"),
    std::string::npos);
  ASSERT_EQ(device.calls().size(), 4U);
}

TEST(SdkCommandGateTest, JointWriteFailureStopsBatchBeforeCommit)
{
  FakeDualArmDevice device;
  device.failNext(FakeOperation::kSetJointPosition);
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm);

  const auto result = gate.sendJointPosition(
    ArmChannel::kLeft, makeJoints(1.0));

  EXPECT_FALSE(result.success);
  EXPECT_NE(
    result.error_message.find("set_left_joint_position"),
    std::string::npos);
  EXPECT_EQ(device.callCount(FakeOperation::kBeginCommandBatch), 1U);
  EXPECT_EQ(device.callCount(FakeOperation::kSetJointPosition), 1U);
  EXPECT_EQ(device.callCount(FakeOperation::kCommitCommandBatch), 0U);
}

TEST(SdkCommandGateTest, BeginBatchRetryHonorsTimeout)
{
  FakeDualArmDevice device;
  device.failNext(FakeOperation::kBeginCommandBatch);
  SdkCommandGateOptions options;
  options.clear_timeout = std::chrono::microseconds::zero();
  options.clear_retry_interval = std::chrono::microseconds::zero();
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm, options);

  const auto result = gate.sendJointPosition(
    ArmChannel::kLeft, makeJoints(1.0));

  EXPECT_FALSE(result.success);
  EXPECT_NE(
    result.error_message.find("joint_command_begin"),
    std::string::npos);
  EXPECT_EQ(device.callCount(FakeOperation::kBeginCommandBatch), 1U);
  EXPECT_EQ(device.callCount(FakeOperation::kSetJointPosition), 0U);
  EXPECT_EQ(device.callCount(FakeOperation::kCommitCommandBatch), 0U);
}

TEST(SdkCommandGateTest, ConcurrentCallersCannotInterleaveSdkBatches)
{
  constexpr std::size_t kThreadCount = 8;
  FakeDualArmDevice device;
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm);
  std::atomic<std::size_t> ready{0};
  std::atomic<bool> start{false};
  std::array<int, kThreadCount> success{};
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);

  for (std::size_t index = 0; index < kThreadCount; ++index) {
    threads.emplace_back([&, index]() {
      ready.fetch_add(1);
      while (!start.load()) {
        std::this_thread::yield();
      }
      const auto result = gate.sendJointPositions(
        makeJoints(static_cast<double>(index)),
        makeJoints(100.0 + static_cast<double>(index)));
      success[index] = result.success ? 1 : 0;
    });
  }
  while (ready.load() != kThreadCount) {
    std::this_thread::yield();
  }
  start.store(true);
  for (auto & thread : threads) {
    thread.join();
  }

  for (const int result : success) {
    EXPECT_EQ(result, 1);
  }
  const auto calls = device.calls();
  ASSERT_EQ(calls.size(), kThreadCount * 4U);
  for (std::size_t offset = 0; offset < calls.size(); offset += 4U) {
    expectDualJointBatch(calls, offset);
    EXPECT_EQ(calls[offset].thread_id, calls[offset + 1U].thread_id);
    EXPECT_EQ(calls[offset].thread_id, calls[offset + 2U].thread_id);
    EXPECT_EQ(calls[offset].thread_id, calls[offset + 3U].thread_id);
  }
}

}  // namespace
