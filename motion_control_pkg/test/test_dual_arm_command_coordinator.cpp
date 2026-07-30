#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>

#include "fake_dual_arm_device.hpp"
#include "motion_control_pkg/execution/dual_arm_command_coordinator.hpp"
#include "motion_control_pkg/execution/synchronized_trajectory.hpp"

namespace
{

using motion_control_pkg::execution::DualArmCommandCoordinator;
using motion_control_pkg::execution::JointCommand;
using motion_control_pkg::execution::TrajectoryEnqueueResult;
using motion_control_pkg::execution::TrajectoryExecutionState;
using motion_control_pkg::sdk::ActiveArmConfiguration;
using motion_control_pkg::sdk::ArmChannel;
using motion_control_pkg::sdk::SdkCommandGate;
using motion_control_pkg::test::FakeCall;
using motion_control_pkg::test::FakeDualArmDevice;
using motion_control_pkg::test::FakeOperation;

std::vector<JointCommand> makeTrajectory(
  std::size_t sample_count, double offset)
{
  std::vector<JointCommand> trajectory(sample_count);
  for (std::size_t sample = 0; sample < sample_count; ++sample) {
    for (std::size_t joint = 0; joint < trajectory[sample].size(); ++joint) {
      trajectory[sample][joint] =
        offset + static_cast<double>(sample * 10U + joint);
    }
  }
  return trajectory;
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

TEST(DualArmCommandCoordinatorTest, UsesFourMillisecondAtomicDualArmCycles)
{
  constexpr std::size_t kSamples = 3;
  FakeDualArmDevice device;
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm);
  DualArmCommandCoordinator coordinator(gate);
  EXPECT_EQ(
    coordinator.period(),
    motion_control_pkg::execution::kDualArmControlPeriod);

  auto left = coordinator.enqueueTrajectory(
    ArmChannel::kLeft, makeTrajectory(kSamples, 10.0));
  auto right = coordinator.enqueueTrajectory(
    ArmChannel::kRight, makeTrajectory(kSamples, 20.0));
  ASSERT_TRUE(left.success) << left.error_message;
  ASSERT_TRUE(right.success) << right.error_message;
  ASSERT_TRUE(coordinator.start());

  ASSERT_TRUE(left.ticket.waitFor(std::chrono::milliseconds(500)));
  ASSERT_TRUE(right.ticket.waitFor(std::chrono::milliseconds(500)));
  EXPECT_EQ(left.ticket.snapshot().state, TrajectoryExecutionState::kSucceeded);
  EXPECT_EQ(right.ticket.snapshot().state, TrajectoryExecutionState::kSucceeded);
  coordinator.stop();

  const auto calls = device.calls();
  ASSERT_EQ(calls.size(), kSamples * 4U);
  for (std::size_t offset = 0; offset < calls.size(); offset += 4U) {
    expectDualJointBatch(calls, offset);
  }
}

TEST(DualArmCommandCoordinatorTest, AtomicPairStartsAndFinishesOnSameCycles)
{
  constexpr std::size_t kSamples = 4U;
  FakeDualArmDevice device;
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm);
  DualArmCommandCoordinator coordinator(gate, std::chrono::milliseconds(1));

  auto pair = coordinator.enqueueSynchronizedPair(
    makeTrajectory(kSamples, 10.0), makeTrajectory(kSamples, 20.0));
  ASSERT_TRUE(pair.success) << pair.error_message;
  ASSERT_TRUE(coordinator.start());
  ASSERT_TRUE(pair.left_ticket.waitFor(std::chrono::milliseconds(500)));
  ASSERT_TRUE(pair.right_ticket.waitFor(std::chrono::milliseconds(500)));
  coordinator.stop();

  const auto left = pair.left_ticket.snapshot();
  const auto right = pair.right_ticket.snapshot();
  EXPECT_EQ(left.state, TrajectoryExecutionState::kSucceeded);
  EXPECT_EQ(right.state, TrajectoryExecutionState::kSucceeded);
  EXPECT_EQ(left.samples_sent, kSamples);
  EXPECT_EQ(left.samples_sent, right.samples_sent);
  const auto calls = device.calls();
  ASSERT_EQ(calls.size(), kSamples * 4U);
  for (std::size_t offset = 0; offset < calls.size(); offset += 4U) {
    expectDualJointBatch(calls, offset);
  }
}

TEST(DualArmCommandCoordinatorTest, AtomicPairHasNoPartialReservationOnFailure)
{
  FakeDualArmDevice device;
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm);
  DualArmCommandCoordinator coordinator(gate);
  auto left = coordinator.enqueueTrajectory(
    ArmChannel::kLeft, makeTrajectory(2U, 10.0));
  ASSERT_TRUE(left.success);

  const auto pair = coordinator.enqueueSynchronizedPair(
    makeTrajectory(2U, 20.0), makeTrajectory(2U, 30.0));
  EXPECT_FALSE(pair.success);
  const auto right = coordinator.enqueueTrajectory(
    ArmChannel::kRight, makeTrajectory(2U, 40.0));
  EXPECT_TRUE(right.success) << right.error_message;
  coordinator.stop();
}

TEST(DualArmCommandCoordinatorTest, CancelingEitherAtomicTicketCancelsBoth)
{
  FakeDualArmDevice device;
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm);
  DualArmCommandCoordinator coordinator(gate, std::chrono::milliseconds(2));
  auto pair = coordinator.enqueueSynchronizedPair(
    makeTrajectory(100U, 10.0), makeTrajectory(100U, 20.0));
  ASSERT_TRUE(pair.success);
  ASSERT_TRUE(coordinator.start());
  ASSERT_TRUE(device.waitForCallCount(
      FakeOperation::kCommitCommandBatch, 2U, std::chrono::milliseconds(250)));

  ASSERT_TRUE(pair.left_ticket.requestCancel());
  ASSERT_TRUE(pair.left_ticket.waitFor(std::chrono::milliseconds(500)));
  ASSERT_TRUE(pair.right_ticket.waitFor(std::chrono::milliseconds(500)));
  coordinator.stop();

  EXPECT_EQ(
    pair.left_ticket.snapshot().state, TrajectoryExecutionState::kCanceled);
  EXPECT_EQ(
    pair.right_ticket.snapshot().state, TrajectoryExecutionState::kCanceled);
  EXPECT_EQ(
    device.callCount(FakeOperation::kSetJointPosition, ArmChannel::kLeft),
    device.callCount(FakeOperation::kSetJointPosition, ArmChannel::kRight));
}

TEST(SynchronizedTrajectoryTest, StretchesShorterSideAndPreservesEndpoints)
{
  const auto left = makeTrajectory(2U, 10.0);
  const auto right = makeTrajectory(5U, 20.0);
  const auto synchronized =
    motion_control_pkg::execution::synchronizeTrajectoryPair(left, right);

  ASSERT_TRUE(synchronized.success) << synchronized.error_message;
  ASSERT_EQ(synchronized.left.size(), right.size());
  EXPECT_EQ(synchronized.right, right);
  EXPECT_EQ(synchronized.left.front(), left.front());
  EXPECT_EQ(synchronized.left.back(), left.back());
  for (std::size_t joint = 0; joint < JointCommand{}.size(); ++joint) {
    EXPECT_DOUBLE_EQ(
      synchronized.left[2U][joint],
      (left.front()[joint] + left.back()[joint]) / 2.0);
  }
}

TEST(DualArmCommandCoordinatorTest, SdkBatchFailureFailsBothSelectedArms)
{
  FakeDualArmDevice device;
  device.failNext(FakeOperation::kCommitCommandBatch);
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm);
  DualArmCommandCoordinator coordinator(gate);
  auto left = coordinator.enqueueTrajectory(
    ArmChannel::kLeft, makeTrajectory(1U, 10.0));
  auto right = coordinator.enqueueTrajectory(
    ArmChannel::kRight, makeTrajectory(1U, 20.0));
  ASSERT_TRUE(left.success);
  ASSERT_TRUE(right.success);
  ASSERT_TRUE(coordinator.start());

  ASSERT_TRUE(left.ticket.waitFor(std::chrono::milliseconds(500)));
  ASSERT_TRUE(right.ticket.waitFor(std::chrono::milliseconds(500)));
  const auto left_state = left.ticket.snapshot();
  const auto right_state = right.ticket.snapshot();
  EXPECT_EQ(left_state.state, TrajectoryExecutionState::kFailed);
  EXPECT_EQ(right_state.state, TrajectoryExecutionState::kFailed);
  EXPECT_NE(left_state.error_message.find("joint_command_commit"), std::string::npos);
  EXPECT_EQ(left_state.error_message, right_state.error_message);
  coordinator.stop();
}

TEST(DualArmCommandCoordinatorTest, BothArmChannelsCanBeQueuedIndependently)
{
  FakeDualArmDevice device;
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm);
  DualArmCommandCoordinator coordinator(gate);

  const auto left = coordinator.enqueueTrajectory(
    ArmChannel::kLeft, makeTrajectory(1U, 10.0));
  const auto right = coordinator.enqueueTrajectory(
    ArmChannel::kRight, makeTrajectory(1U, 20.0));

  EXPECT_TRUE(left.success) << left.error_message;
  EXPECT_TRUE(right.success) << right.error_message;
  EXPECT_TRUE(device.calls().empty());
  coordinator.stop();
}

TEST(DualArmCommandCoordinatorTest, CancelStopsOneArmWithoutClearingOtherArm)
{
  constexpr std::size_t kLeftSamples = 100;
  constexpr std::size_t kRightSamples = 6;
  FakeDualArmDevice device;
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm);
  DualArmCommandCoordinator coordinator(gate, std::chrono::milliseconds(2));
  auto left = coordinator.enqueueTrajectory(
    ArmChannel::kLeft, makeTrajectory(kLeftSamples, 10.0));
  auto right = coordinator.enqueueTrajectory(
    ArmChannel::kRight, makeTrajectory(kRightSamples, 20.0));
  ASSERT_TRUE(left.success);
  ASSERT_TRUE(right.success);
  ASSERT_TRUE(coordinator.start());
  ASSERT_TRUE(device.waitForCallCount(
      FakeOperation::kCommitCommandBatch, 2U, std::chrono::milliseconds(250)));

  ASSERT_TRUE(left.ticket.requestCancel());
  ASSERT_TRUE(left.ticket.waitFor(std::chrono::milliseconds(500)));
  ASSERT_TRUE(right.ticket.waitFor(std::chrono::milliseconds(500)));
  EXPECT_EQ(left.ticket.snapshot().state, TrajectoryExecutionState::kCanceled);
  EXPECT_EQ(right.ticket.snapshot().state, TrajectoryExecutionState::kSucceeded);
  coordinator.stop();

  EXPECT_LT(
    device.callCount(FakeOperation::kSetJointPosition, ArmChannel::kLeft),
    kLeftSamples);
  EXPECT_EQ(
    device.callCount(FakeOperation::kSetJointPosition, ArmChannel::kRight),
    kRightSamples);
}

TEST(DualArmCommandCoordinatorTest, TicketWaitReportsTimeoutWithoutStoppingTrajectory)
{
  FakeDualArmDevice device;
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm);
  DualArmCommandCoordinator coordinator(gate, std::chrono::milliseconds(20));
  auto trajectory = coordinator.enqueueTrajectory(
    ArmChannel::kLeft, makeTrajectory(100U, 10.0));
  ASSERT_TRUE(trajectory.success);
  ASSERT_TRUE(coordinator.start());
  ASSERT_TRUE(device.waitForCallCount(
      FakeOperation::kCommitCommandBatch, 1U, std::chrono::milliseconds(250)));

  EXPECT_FALSE(trajectory.ticket.waitFor(std::chrono::milliseconds(1)));
  EXPECT_TRUE(trajectory.ticket.requestCancel());
  ASSERT_TRUE(trajectory.ticket.waitFor(std::chrono::milliseconds(250)));
  EXPECT_EQ(
    trajectory.ticket.snapshot().state,
    TrajectoryExecutionState::kCanceled);
  coordinator.stop();
}

TEST(DualArmCommandCoordinatorTest, ConcurrentSameArmEnqueueHasSingleWinner)
{
  FakeDualArmDevice device;
  SdkCommandGate gate(device, ActiveArmConfiguration::kDualArm);
  DualArmCommandCoordinator coordinator(gate);
  std::atomic<std::size_t> ready{0};
  std::atomic<bool> start{false};
  std::array<TrajectoryEnqueueResult, 2> results;
  std::array<std::thread, 2> threads{
    std::thread([&]() {
      ready.fetch_add(1);
      while (!start.load()) {
        std::this_thread::yield();
      }
      results[0] = coordinator.enqueueTrajectory(
        ArmChannel::kLeft, makeTrajectory(1U, 10.0));
    }),
    std::thread([&]() {
      ready.fetch_add(1);
      while (!start.load()) {
        std::this_thread::yield();
      }
      results[1] = coordinator.enqueueTrajectory(
        ArmChannel::kLeft, makeTrajectory(1U, 20.0));
    })};

  while (ready.load() != threads.size()) {
    std::this_thread::yield();
  }
  start.store(true);
  for (auto & thread : threads) {
    thread.join();
  }

  const int winner_count =
    (results[0].success ? 1 : 0) + (results[1].success ? 1 : 0);
  EXPECT_EQ(winner_count, 1);
  EXPECT_TRUE(
    results[0].success ||
    results[0].error_message.find("already active") != std::string::npos);
  EXPECT_TRUE(
    results[1].success ||
    results[1].error_message.find("already active") != std::string::npos);
  coordinator.stop();
}

}  // namespace
