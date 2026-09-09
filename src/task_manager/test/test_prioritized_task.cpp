// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Description:
// 优先级任务队列排序单元测试。
// 测试 PrioritizedTask 结构体的 operator< 和 std::priority_queue 的排序行为，
// 确保高优先级任务先出队、同优先级按 FIFO 顺序。

#include <gtest/gtest.h>
#include <queue>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "task_manager/safety_task_types.hpp"

using namespace task_mgr;

/// @brief 创建测试用的 PrioritizedTask
static PrioritizedTask make_task(
  const std::string& id,
  int32_t priority,
  const std::string& type = "emergency_stop",
  int64_t timeout_ms = 5000)
{
  PrioritizedTask t;
  t.task_id = id;
  t.task_type = type;
  t.target_node = "test_node";
  t.parameters_json = "{}";
  t.priority = priority;
  t.timeout_ms = timeout_ms;
  // arrival_time 用简单的递增模拟; 实际测试中通过 relative order 验证
  t.arrival_time = rclcpp::Time(0, 0, RCL_ROS_TIME);  // 由测试控制
  return t;
}

// ============================================================================
// operator< 基本行为测试
// ============================================================================

TEST(PrioritizedTaskTest, HigherPriorityOrderedFirstInQueue)
{
  // std::priority_queue 默认是大顶堆（最大元素在 top）
  // 验证：高优先级任务在 priority_queue 中排在低优先级任务前面
  std::priority_queue<PrioritizedTask> pq;
  PrioritizedTask low  = make_task("low",  5);
  PrioritizedTask high = make_task("high", 25);

  pq.push(low);
  pq.push(high);

  // 高优先级应该排在堆顶（先出队）
  EXPECT_EQ(pq.top().task_id, "high");
  pq.pop();
  EXPECT_EQ(pq.top().task_id, "low");
}

TEST(PrioritizedTaskTest, SamePriorityFifoOrder)
{
  // 同优先级按到达时间 FIFO：早到达的优先
  std::priority_queue<PrioritizedTask> pq;
  PrioritizedTask first  = make_task("first",  10);
  PrioritizedTask second = make_task("second", 10);

  first.arrival_time  = rclcpp::Time(1, 0, RCL_ROS_TIME);   // 1ns
  second.arrival_time = rclcpp::Time(2, 0, RCL_ROS_TIME);   // 2ns

  pq.push(first);
  pq.push(second);

  // 早到达的（first）应该排在堆顶（先出队）
  EXPECT_EQ(pq.top().task_id, "first");
  pq.pop();
  EXPECT_EQ(pq.top().task_id, "second");
}

TEST(PrioritizedTaskTest, PriorityDominatesArrivalTime)
{
  // 优先级差异应优先于到达时间差异
  std::priority_queue<PrioritizedTask> pq;
  PrioritizedTask early_low  = make_task("early_low",  5);
  PrioritizedTask late_high  = make_task("late_high", 25);

  early_low.arrival_time  = rclcpp::Time(1, 0, RCL_ROS_TIME);   // 先到达
  late_high.arrival_time  = rclcpp::Time(100, 0, RCL_ROS_TIME); // 后到达

  pq.push(early_low);
  pq.push(late_high);

  // 高优先级即使后到达也应排在堆顶（先出队）
  EXPECT_EQ(pq.top().task_id, "late_high");
  pq.pop();
  EXPECT_EQ(pq.top().task_id, "early_low");
}

// ============================================================================
// std::priority_queue 集成测试
// ============================================================================

TEST(PrioritizedTaskTest, PriorityQueueOrdersByPriorityDescending)
{
  std::priority_queue<PrioritizedTask> pq;

  pq.push(make_task("t5",  5));
  pq.push(make_task("t25", 25));
  pq.push(make_task("t10", 10));
  pq.push(make_task("t1",  1));
  pq.push(make_task("t20", 20));

  // 期望出队顺序：25 -> 20 -> 10 -> 5 -> 1
  EXPECT_EQ(pq.top().task_id, "t25"); pq.pop();
  EXPECT_EQ(pq.top().task_id, "t20"); pq.pop();
  EXPECT_EQ(pq.top().task_id, "t10"); pq.pop();
  EXPECT_EQ(pq.top().task_id, "t5");  pq.pop();
  EXPECT_EQ(pq.top().task_id, "t1");  pq.pop();
  EXPECT_TRUE(pq.empty());
}

TEST(PrioritizedTaskTest, PriorityQueueFifoForSamePriority)
{
  std::priority_queue<PrioritizedTask> pq;

  PrioritizedTask a = make_task("a", 15);
  PrioritizedTask b = make_task("b", 15);
  PrioritizedTask c = make_task("c", 15);

  a.arrival_time = rclcpp::Time(1, 0, RCL_ROS_TIME);
  b.arrival_time = rclcpp::Time(2, 0, RCL_ROS_TIME);
  c.arrival_time = rclcpp::Time(3, 0, RCL_ROS_TIME);

  // 入队顺序: a, b, c（全部相同优先级 15）
  pq.push(a); pq.push(b); pq.push(c);

  // 期望出队顺序: a -> b -> c（FIFO）
  EXPECT_EQ(pq.top().task_id, "a"); pq.pop();
  EXPECT_EQ(pq.top().task_id, "b"); pq.pop();
  EXPECT_EQ(pq.top().task_id, "c"); pq.pop();
}

TEST(PrioritizedTaskTest, PriorityQueueMixedPrioritiesAndArrivalTimes)
{
  std::priority_queue<PrioritizedTask> pq;

  // 场景：混合优先级和到达时间
  PrioritizedTask t1 = make_task("p20_early", 20);
  PrioritizedTask t2 = make_task("p10_late",  10);
  PrioritizedTask t3 = make_task("p20_late",  20);
  PrioritizedTask t4 = make_task("p10_early", 10);

  t1.arrival_time = rclcpp::Time(1, 0, RCL_ROS_TIME);
  t2.arrival_time = rclcpp::Time(4, 0, RCL_ROS_TIME);
  t3.arrival_time = rclcpp::Time(3, 0, RCL_ROS_TIME);
  t4.arrival_time = rclcpp::Time(2, 0, RCL_ROS_TIME);

  pq.push(t1); pq.push(t2); pq.push(t3); pq.push(t4);

  // 期望顺序：p20_early (prio20, t=1) -> p20_late (prio20, t=3)
  //          -> p10_early (prio10, t=2) -> p10_late (prio10, t=4)
  EXPECT_EQ(pq.top().task_id, "p20_early"); pq.pop();
  EXPECT_EQ(pq.top().task_id, "p20_late");  pq.pop();
  EXPECT_EQ(pq.top().task_id, "p10_early"); pq.pop();
  EXPECT_EQ(pq.top().task_id, "p10_late");  pq.pop();
  EXPECT_TRUE(pq.empty());
}

// ============================================================================
// 边界条件测试
// ============================================================================

TEST(PrioritizedTaskTest, PriorityBoundaryValues)
{
  // 确保优先级 1 和 25 都能正常工作
  std::priority_queue<PrioritizedTask> pq;

  pq.push(make_task("min", 1));
  pq.push(make_task("max", SAFETY_TASK_PRIORITY_MAX));  // 25

  EXPECT_EQ(pq.top().task_id, "max"); pq.pop();
  EXPECT_EQ(pq.top().task_id, "min"); pq.pop();
}

TEST(PrioritizedTaskTest, EmptyQueueBehavior)
{
  std::priority_queue<PrioritizedTask> pq;
  EXPECT_TRUE(pq.empty());
  EXPECT_EQ(pq.size(), 0u);
}

TEST(PrioritizedTaskTest, EmergencyThresholdCheck)
{
  // 验证紧急优先级阈值边界
  PrioritizedTask emergency = make_task("emerg", SAFETY_TASK_PRIORITY_EMERGENCY);  // 20
  PrioritizedTask normal    = make_task("normal", SAFETY_TASK_PRIORITY_EMERGENCY - 1);  // 19

  EXPECT_GE(emergency.priority, SAFETY_TASK_PRIORITY_EMERGENCY)
    << "priority 20 should be considered emergency";
  EXPECT_LT(normal.priority, SAFETY_TASK_PRIORITY_EMERGENCY)
    << "priority 19 should not be considered emergency";
}

// ============================================================================
// 线程安全测试（基本验证）
// ============================================================================

TEST(PrioritizedTaskTest, MutexLockGuardCompiles)
{
  // 验证 std::lock_guard<std::mutex> 能正常编译和使用
  std::mutex mtx;
  std::priority_queue<PrioritizedTask> pq;

  {
    std::lock_guard<std::mutex> lock(mtx);
    pq.push(make_task("thread_safe", 10));
  }

  {
    std::lock_guard<std::mutex> lock(mtx);
    EXPECT_FALSE(pq.empty());
    EXPECT_EQ(pq.top().task_id, "thread_safe");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
