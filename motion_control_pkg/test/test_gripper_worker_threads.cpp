#include <gtest/gtest.h>

#include <chrono>
#include <future>

#include "motion_control_pkg/execution/gripper_worker_threads.hpp"

namespace
{

using motion_control_pkg::execution::GripperWorkerSlot;
using motion_control_pkg::execution::GripperWorkerThreads;
using namespace std::chrono_literals;

TEST(GripperWorkerThreadsTest, LeftAndRightWorkersStartWithoutWaitingForEachOther)
{
  GripperWorkerThreads workers;
  std::promise<void> release_left_promise;
  auto release_left = release_left_promise.get_future().share();
  std::promise<void> left_started_promise;
  auto left_started = left_started_promise.get_future();

  workers.start(GripperWorkerSlot::kLeft, [&]() {
    left_started_promise.set_value();
    release_left.wait();
  });
  ASSERT_EQ(left_started.wait_for(1s), std::future_status::ready);

  std::promise<void> right_started_promise;
  auto right_started = right_started_promise.get_future();
  auto right_start_call = std::async(std::launch::async, [&]() {
    workers.start(GripperWorkerSlot::kRight, [&]() {
      right_started_promise.set_value();
    });
  });

  const auto right_status = right_started.wait_for(1s);
  release_left_promise.set_value();
  ASSERT_EQ(right_start_call.wait_for(1s), std::future_status::ready);
  right_start_call.get();
  workers.joinAll();

  EXPECT_EQ(right_status, std::future_status::ready);
}

TEST(GripperWorkerThreadsTest, ReusingOneSideWaitsForItsPreviousWorker)
{
  GripperWorkerThreads workers;
  std::promise<void> release_first_promise;
  auto release_first = release_first_promise.get_future().share();
  std::promise<void> first_started_promise;
  auto first_started = first_started_promise.get_future();

  workers.start(GripperWorkerSlot::kLeft, [&]() {
    first_started_promise.set_value();
    release_first.wait();
  });
  ASSERT_EQ(first_started.wait_for(1s), std::future_status::ready);

  std::promise<void> second_started_promise;
  auto second_started = second_started_promise.get_future();
  auto second_start_call = std::async(std::launch::async, [&]() {
    workers.start(GripperWorkerSlot::kLeft, [&]() {
      second_started_promise.set_value();
    });
  });

  EXPECT_EQ(second_started.wait_for(100ms), std::future_status::timeout);
  release_first_promise.set_value();
  ASSERT_EQ(second_start_call.wait_for(1s), std::future_status::ready);
  second_start_call.get();
  EXPECT_EQ(second_started.wait_for(1s), std::future_status::ready);
  workers.joinAll();
}

}  // namespace
