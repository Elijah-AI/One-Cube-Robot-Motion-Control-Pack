#include <gtest/gtest.h>

#include <atomic>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "motion_control_pkg/execution/dual_arm_task_arbiter.hpp"
#include "motion_control_pkg/execution/execution_control.hpp"

namespace
{

using motion_control_pkg::execution::ArmTaskLease;
using motion_control_pkg::execution::ArmTaskScope;
using motion_control_pkg::execution::DualArmTaskArbiter;
using motion_control_pkg::sdk::ActiveArmConfiguration;

TEST(DualArmTaskArbiterTest, DualLeaseAtomicallyOwnsBothArms)
{
  DualArmTaskArbiter arbiter(ActiveArmConfiguration::kDualArm);

  auto dual = arbiter.tryAcquire(ArmTaskScope::kDual);

  ASSERT_TRUE(dual.success) << dual.error_message;
  const auto snapshot = arbiter.snapshot();
  EXPECT_TRUE(snapshot.left_occupied);
  EXPECT_TRUE(snapshot.right_occupied);
  EXPECT_TRUE(snapshot.dual_task_active);
  EXPECT_FALSE(arbiter.canAcquire(ArmTaskScope::kLeft));
  EXPECT_FALSE(arbiter.canAcquire(ArmTaskScope::kRight));
  EXPECT_FALSE(arbiter.canAcquire(ArmTaskScope::kDual));
}

TEST(DualArmTaskArbiterTest, EitherSingleArmLeaseRejectsDualTask)
{
  DualArmTaskArbiter arbiter(ActiveArmConfiguration::kDualArm);

  auto left = arbiter.tryAcquire(ArmTaskScope::kLeft);
  ASSERT_TRUE(left.success);
  EXPECT_FALSE(arbiter.canAcquire(ArmTaskScope::kDual));

  left.lease.reset();
  auto right = arbiter.tryAcquire(ArmTaskScope::kRight);
  ASSERT_TRUE(right.success);
  EXPECT_FALSE(arbiter.canAcquire(ArmTaskScope::kDual));
}

TEST(DualArmTaskArbiterTest, IndependentSingleArmTasksMayRunTogether)
{
  DualArmTaskArbiter arbiter(ActiveArmConfiguration::kDualArm);

  auto left = arbiter.tryAcquire(ArmTaskScope::kLeft);
  auto right = arbiter.tryAcquire(ArmTaskScope::kRight);

  ASSERT_TRUE(left.success) << left.error_message;
  ASSERT_TRUE(right.success) << right.error_message;
  const auto snapshot = arbiter.snapshot();
  EXPECT_TRUE(snapshot.left_occupied);
  EXPECT_TRUE(snapshot.right_occupied);
  EXPECT_FALSE(snapshot.dual_task_active);
}

TEST(DualArmTaskArbiterTest, LeaseMoveAndScopeExitReleaseExactlyOnce)
{
  DualArmTaskArbiter arbiter(ActiveArmConfiguration::kDualArm);

  {
    auto acquired = arbiter.tryAcquire(ArmTaskScope::kDual);
    ASSERT_TRUE(acquired.success);
    ArmTaskLease moved = std::move(acquired.lease);
    EXPECT_FALSE(acquired.lease.valid());
    EXPECT_TRUE(moved.valid());
  }

  const auto snapshot = arbiter.snapshot();
  EXPECT_FALSE(snapshot.left_occupied);
  EXPECT_FALSE(snapshot.right_occupied);
  EXPECT_FALSE(snapshot.dual_task_active);
  EXPECT_TRUE(arbiter.canAcquire(ArmTaskScope::kDual));
}

TEST(DualArmTaskArbiterTest, ConcurrentDualGoalsHaveOneWinner)
{
  DualArmTaskArbiter arbiter(ActiveArmConfiguration::kDualArm);
  constexpr int kThreadCount = 16;
  std::atomic<int> ready{0};
  std::atomic<bool> start{false};
  std::mutex leases_mutex;
  std::vector<ArmTaskLease> winning_leases;
  std::vector<std::thread> callers;

  for (int index = 0; index < kThreadCount; ++index) {
    callers.emplace_back([&]() {
      ready.fetch_add(1);
      while (!start.load()) {
        std::this_thread::yield();
      }
      auto acquired = arbiter.tryAcquire(ArmTaskScope::kDual);
      if (acquired.success) {
        std::lock_guard<std::mutex> lock(leases_mutex);
        winning_leases.push_back(std::move(acquired.lease));
      }
    });
  }
  while (ready.load() != kThreadCount) {
    std::this_thread::yield();
  }
  start.store(true);
  for (auto & caller : callers) {
    caller.join();
  }

  ASSERT_EQ(winning_leases.size(), 1U);
  EXPECT_TRUE(arbiter.snapshot().dual_task_active);
  winning_leases.clear();
  EXPECT_TRUE(arbiter.canAcquire(ArmTaskScope::kDual));
}

TEST(DualArmTaskArbiterTest, ConcurrentTerminalTransitionsHaveOneWinner)
{
  motion_control_pkg::execution::TerminalStateMachine terminal;
  std::atomic<int> winners{0};
  std::vector<std::thread> callers;

  for (int index = 0; index < 12; ++index) {
    callers.emplace_back([&, index]() {
      const auto state = index % 3 == 0 ?
        motion_control_pkg::execution::TerminalState::kSucceeded :
        (index % 3 == 1 ?
        motion_control_pkg::execution::TerminalState::kAborted :
        motion_control_pkg::execution::TerminalState::kCanceled);
      if (terminal.tryFinish(state)) {
        winners.fetch_add(1);
      }
    });
  }
  for (auto & caller : callers) {
    caller.join();
  }

  EXPECT_EQ(winners.load(), 1);
  EXPECT_NE(
    terminal.state(),
    motion_control_pkg::execution::TerminalState::kActive);
}

TEST(DualArmTaskArbiterTest, EitherArmFailureRequestsOneUnifiedStop)
{
  motion_control_pkg::execution::DualArmTaskTermination termination;

  EXPECT_TRUE(termination.tryFail(
    motion_control_pkg::sdk::ArmChannel::kRight, "SDK feedback lost"));
  EXPECT_TRUE(termination.stopRequested());
  EXPECT_EQ(
    termination.state(),
    motion_control_pkg::execution::TerminalState::kAborted);
  EXPECT_EQ(
    termination.errorMessage(),
    "right arm failure: SDK feedback lost");
  EXPECT_FALSE(termination.trySucceed());
  EXPECT_FALSE(termination.tryCancel("late cancel"));
  EXPECT_EQ(
    termination.errorMessage(),
    "right arm failure: SDK feedback lost");
}

TEST(DualArmTaskArbiterTest, ClientCancelWinsAgainstLateSideFailure)
{
  motion_control_pkg::execution::DualArmTaskTermination termination;

  EXPECT_TRUE(termination.tryCancel("dual-arm goal canceled by client"));
  EXPECT_FALSE(termination.tryFail(
    motion_control_pkg::sdk::ArmChannel::kLeft, "late planner failure"));
  EXPECT_TRUE(termination.stopRequested());
  EXPECT_EQ(
    termination.state(),
    motion_control_pkg::execution::TerminalState::kCanceled);
  EXPECT_EQ(
    termination.errorMessage(),
    "dual-arm goal canceled by client");
}

}  // namespace
