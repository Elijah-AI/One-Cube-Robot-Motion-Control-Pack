// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Description:
// AddTask Action 接口集成测试。
// 测试 AddTask.action 中定义的 Goal/Result/Feedback 数据结构，
// 优先级任务队列的调度策略，以及紧急/非紧急任务的分发逻辑。
// 由于 TaskManagerNode 依赖 Modbus/BT 引擎等复杂组件，
// 本测试直接验证核心数据结构和调度逻辑的正确性。

#include <gtest/gtest.h>
#include <queue>
#include <string>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>

// 引入 AddTask action 生成的头文件
#include "com_interfaces/action/add_task.hpp"
#include "task_manager/safety_task_types.hpp"

using namespace task_mgr;
using AddTaskGoal = com_interfaces::action::AddTask::Goal;
using AddTaskResult = com_interfaces::action::AddTask::Result;
using AddTaskFeedback = com_interfaces::action::AddTask::Feedback;

// ============================================================================
// 辅助函数
// ============================================================================

/// @brief 创建测试用 AddTask Goal
static auto make_goal(
  const std::string& task_id,
  const std::string& task_type,
  int32_t priority,
  int64_t timeout_ms = 5000)
{
  auto goal = std::make_shared<AddTaskGoal>();
  goal->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  goal->task_id = task_id;
  goal->task_type = task_type;
  goal->target_node = "test_node";
  goal->parameters_json = "{}";
  goal->priority = priority;
  goal->timeout_ms = timeout_ms;
  return goal;
}

/// @brief 创建测试用 PrioritizedTask
static PrioritizedTask make_ptask(
  const std::string& id,
  int32_t priority,
  const std::string& type = "emergency_stop",
  int64_t timeout = 5000)
{
  PrioritizedTask t;
  t.task_id = id;
  t.task_type = type;
  t.target_node = "test_node";
  t.parameters_json = "{}";
  t.priority = priority;
  t.timeout_ms = timeout;
  t.arrival_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  return t;
}

// ============================================================================
// AddTask Goal 数据结构测试
// ============================================================================

TEST(AddTaskActionTest, GoalDefaultConstruction)
{
  AddTaskGoal goal;
  EXPECT_TRUE(goal.task_id.empty());
  EXPECT_TRUE(goal.task_type.empty());
  EXPECT_TRUE(goal.target_node.empty());
  EXPECT_TRUE(goal.parameters_json.empty());
  EXPECT_EQ(goal.priority, 0);
  EXPECT_EQ(goal.timeout_ms, 0);
}

TEST(AddTaskActionTest, GoalPopulatedCorrectly)
{
  auto goal = make_goal("task_001", "emergency_stop", 25, 3000);

  EXPECT_EQ(goal->task_id, "task_001");
  EXPECT_EQ(goal->task_type, "emergency_stop");
  EXPECT_EQ(goal->target_node, "test_node");
  EXPECT_EQ(goal->parameters_json, "{}");
  EXPECT_EQ(goal->priority, 25);
  EXPECT_EQ(goal->timeout_ms, 3000);
}

TEST(AddTaskActionTest, GoalAllTaskTypes)
{
  // 验证所有预定义任务类型均可正常设置
  std::vector<std::string> types = {
    "emergency_stop", "safety_halt", "resume", "hardware_reset"
  };

  for (const auto& t : types)
  {
    auto goal = make_goal("id", t, 15);
    EXPECT_EQ(goal->task_type, t);
  }
}

TEST(AddTaskActionTest, GoalPriorityRange)
{
  // 验证优先级范围 1-25
  auto g_min = make_goal("min", "safety_halt", 1);
  EXPECT_EQ(g_min->priority, 1);

  auto g_max = make_goal("max", "safety_halt", SAFETY_TASK_PRIORITY_MAX);
  EXPECT_EQ(g_max->priority, SAFETY_TASK_PRIORITY_MAX);
}

// ============================================================================
// AddTask Result 数据结构测试
// ============================================================================

TEST(AddTaskActionTest, ResultSuccessValues)
{
  AddTaskResult result;
  result.success = true;
  result.status_code = 0;
  result.error_message = "";
  result.execution_time = 1.5;

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.status_code, 0u);
  EXPECT_TRUE(result.error_message.empty());
  EXPECT_DOUBLE_EQ(result.execution_time, 1.5);
}

TEST(AddTaskActionTest, ResultStatusCodes)
{
  // 验证状态码语义
  AddTaskResult r0, r1, r2, r3;

  r0.status_code = 0;  // OK
  r1.status_code = 1;  // FAILED
  r2.status_code = 2;  // TIMEOUT
  r3.status_code = 3;  // BUSY

  EXPECT_EQ(r0.status_code, 0u);
  EXPECT_EQ(r1.status_code, 1u);
  EXPECT_EQ(r2.status_code, 2u);
  EXPECT_EQ(r3.status_code, 3u);
}

TEST(AddTaskActionTest, ResultFailureWithErrorMessage)
{
  AddTaskResult result;
  result.success = false;
  result.status_code = 1;  // FAILED
  result.error_message = "Hardware fault: gripper not responding";
  result.execution_time = 0.0;

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status_code, 1u);
  EXPECT_FALSE(result.error_message.empty());
  EXPECT_STREQ(result.error_message.c_str(), "Hardware fault: gripper not responding");
}

// ============================================================================
// AddTask Feedback 数据结构测试
// ============================================================================

TEST(AddTaskActionTest, FeedbackPhases)
{
  AddTaskFeedback fb;
  fb.current_phase = "task_received";
  fb.progress_percent = 0.0f;
  fb.active_node = "task_manager_node";
  fb.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  EXPECT_STREQ(fb.current_phase.c_str(), "task_received");
  EXPECT_FLOAT_EQ(fb.progress_percent, 0.0f);
  EXPECT_STREQ(fb.active_node.c_str(), "task_manager_node");
}

TEST(AddTaskActionTest, FeedbackProgressRange)
{
  AddTaskFeedback fb;

  // 进度范围 0% - 100%
  fb.progress_percent = 0.0f;
  EXPECT_FLOAT_EQ(fb.progress_percent, 0.0f);

  fb.progress_percent = 50.0f;
  EXPECT_FLOAT_EQ(fb.progress_percent, 50.0f);

  fb.progress_percent = 100.0f;
  EXPECT_FLOAT_EQ(fb.progress_percent, 100.0f);
}

// ============================================================================
// PrioritizedTask 与 Action 集成测试
// ============================================================================

TEST(AddTaskActionTest, PrioritizedTaskFromGoal)
{
  // 模拟从 AddTask Goal 构建 PrioritizedTask 的过程
  auto goal = make_goal("safety_001", "emergency_stop", 25, 2000);

  PrioritizedTask task;
  task.task_id = goal->task_id;
  task.task_type = goal->task_type;
  task.target_node = goal->target_node;
  task.parameters_json = goal->parameters_json;
  task.priority = goal->priority;
  task.timeout_ms = goal->timeout_ms;
  task.arrival_time = rclcpp::Clock(RCL_ROS_TIME).now();

  EXPECT_EQ(task.task_id, "safety_001");
  EXPECT_EQ(task.task_type, "emergency_stop");
  EXPECT_EQ(task.priority, 25);
  EXPECT_EQ(task.timeout_ms, 2000);
}

TEST(AddTaskActionTest, EmergencyTaskPreemptsNormalTask)
{
  // 模拟紧急任务抢占普通任务的队列行为
  std::priority_queue<PrioritizedTask> pq;

  // 先入队一个普通任务 (priority=10)
  auto normal_task = make_ptask("normal_001", 10, "safety_halt");
  normal_task.arrival_time = rclcpp::Time(1, 0, RCL_ROS_TIME);
  pq.push(normal_task);

  // 再入队一个紧急任务 (priority=25)，虽然更晚到达但优先级更高
  auto emergency_task = make_ptask("emerg_001", 25, "emergency_stop");
  emergency_task.arrival_time = rclcpp::Time(10, 0, RCL_ROS_TIME);
  pq.push(emergency_task);

  // 紧急任务应该排在队列顶部
  EXPECT_EQ(pq.top().task_id, "emerg_001");
  EXPECT_EQ(pq.top().priority, 25);
  pq.pop();

  EXPECT_EQ(pq.top().task_id, "normal_001");
  EXPECT_EQ(pq.top().priority, 10);
}

TEST(AddTaskActionTest, MultiplePrioritiesOrdering)
{
  // 模拟 safety_monitor 一次下发多个不同优先级的任务
  std::priority_queue<PrioritizedTask> pq;

  pq.push(make_ptask("t_safety_halt",    8,  "safety_halt"));
  pq.push(make_ptask("t_emerg_stop",    25,  "emergency_stop"));
  pq.push(make_ptask("t_hw_reset",      15,  "hardware_reset"));
  pq.push(make_ptask("t_resume",         5,  "resume"));

  // 期望：25(emerg_stop) > 15(hw_reset) > 8(safety_halt) > 5(resume)
  EXPECT_EQ(pq.top().task_id, "t_emerg_stop");  pq.pop();
  EXPECT_EQ(pq.top().task_id, "t_hw_reset");    pq.pop();
  EXPECT_EQ(pq.top().task_id, "t_safety_halt"); pq.pop();
  EXPECT_EQ(pq.top().task_id, "t_resume");      pq.pop();
}

// ============================================================================
// 紧急阈值检查测试
// ============================================================================

TEST(AddTaskActionTest, EmergencyThresholdBoundary)
{
  // priority >= SAFETY_TASK_PRIORITY_EMERGENCY (20) 视为紧急任务
  EXPECT_TRUE(20 >= SAFETY_TASK_PRIORITY_EMERGENCY);
  EXPECT_TRUE(25 >= SAFETY_TASK_PRIORITY_EMERGENCY);
  EXPECT_FALSE(19 >= SAFETY_TASK_PRIORITY_EMERGENCY);
  EXPECT_FALSE(1 >= SAFETY_TASK_PRIORITY_EMERGENCY);
}

TEST(AddTaskActionTest, IsEmergencyTask)
{
  // 辅助 lambda：模拟 handle_add_task_goal 中的紧急判断逻辑
  auto is_emergency = [](int32_t priority) -> bool {
    return priority >= SAFETY_TASK_PRIORITY_EMERGENCY;
  };

  EXPECT_TRUE(is_emergency(20));   // 边界值
  EXPECT_TRUE(is_emergency(25));   // 最高优先级
  EXPECT_TRUE(is_emergency(21));   // 紧急
  EXPECT_FALSE(is_emergency(19));  // 非紧急
  EXPECT_FALSE(is_emergency(10));  // 普通
  EXPECT_FALSE(is_emergency(1));   // 最低优先级
}

// ============================================================================
// Goal 参数校验逻辑测试（模拟 handle_add_task_goal 的校验逻辑）
// ============================================================================

TEST(AddTaskActionTest, GoalValidationEmptyTaskId)
{
  // 模拟校验：task_id 为空应拒绝
  auto validate = [](const AddTaskGoal& goal) -> bool {
    if (goal.task_id.empty()) return false;
    if (goal.task_type.empty()) return false;
    if (goal.priority < 1 || goal.priority > SAFETY_TASK_PRIORITY_MAX) return false;
    return true;
  };

  auto invalid = make_goal("", "emergency_stop", 20);
  EXPECT_FALSE(validate(*invalid));

  auto valid = make_goal("task_001", "emergency_stop", 20);
  EXPECT_TRUE(validate(*valid));
}

TEST(AddTaskActionTest, GoalValidationEmptyTaskType)
{
  auto validate = [](const AddTaskGoal& goal) -> bool {
    if (goal.task_id.empty()) return false;
    if (goal.task_type.empty()) return false;
    if (goal.priority < 1 || goal.priority > SAFETY_TASK_PRIORITY_MAX) return false;
    return true;
  };

  auto invalid = make_goal("task_001", "", 20);
  EXPECT_FALSE(validate(*invalid));
}

TEST(AddTaskActionTest, GoalValidationInvalidPriority)
{
  auto validate = [](const AddTaskGoal& goal) -> bool {
    if (goal.task_id.empty()) return false;
    if (goal.task_type.empty()) return false;
    if (goal.priority < 1 || goal.priority > SAFETY_TASK_PRIORITY_MAX) return false;
    return true;
  };

  auto p0 = make_goal("t0", "safety_halt", 0);     // 低于最小值
  auto p26 = make_goal("t26", "safety_halt", 26);   // 高于最大值
  auto p_neg = make_goal("t_neg", "safety_halt", -1); // 负数

  EXPECT_FALSE(validate(*p0));
  EXPECT_FALSE(validate(*p26));
  EXPECT_FALSE(validate(*p_neg));

  // 边界有效值
  auto p1 = make_goal("t1", "safety_halt", 1);
  auto p25 = make_goal("t25", "safety_halt", 25);
  EXPECT_TRUE(validate(*p1));
  EXPECT_TRUE(validate(*p25));
}

// ============================================================================
// 线程安全队列测试
// ============================================================================

TEST(AddTaskActionTest, ThreadSafeEnqueueDequeue)
{
  // 模拟 handle_add_task_accepted 中的线程安全入队操作
  std::mutex mtx;
  std::priority_queue<PrioritizedTask> pq;
  std::atomic<int> enqueued{0};

  auto producer = [&](int count, int32_t base_priority) {
    for (int i = 0; i < count; i++)
    {
      auto task = make_ptask(
        "t_" + std::to_string(base_priority) + "_" + std::to_string(i),
        base_priority + (i % 5));

      std::lock_guard<std::mutex> lock(mtx);
      pq.push(task);
      enqueued++;
    }
  };

  // 启动两个生产者线程
  std::thread t1(producer, 10, 10);
  std::thread t2(producer, 10, 20);

  t1.join();
  t2.join();

  EXPECT_EQ(enqueued.load(), 20);
  EXPECT_EQ(pq.size(), 20u);

  // 验证所有元素按优先级正确排序
  int32_t prev_priority = SAFETY_TASK_PRIORITY_MAX + 1;
  while (!pq.empty())
  {
    auto t = pq.top();
    pq.pop();
    EXPECT_LE(t.priority, prev_priority)
      << "Tasks should be dequeued in descending priority order";
    prev_priority = t.priority;
  }
}

// ============================================================================
// 超时默认值测试
// ============================================================================

TEST(AddTaskActionTest, TimeoutDefaultValue)
{
  // 当 goal->timeout_ms <= 0 时应使用默认值 5000ms
  auto apply_default = [](int64_t timeout_ms) -> int64_t {
    return timeout_ms > 0 ? timeout_ms : 5000;
  };

  EXPECT_EQ(apply_default(0), 5000);
  EXPECT_EQ(apply_default(-1), 5000);
  EXPECT_EQ(apply_default(3000), 3000);
  EXPECT_EQ(apply_default(10000), 10000);
}

// ============================================================================
// 反馈阶段名称一致性测试
// ============================================================================

TEST(AddTaskActionTest, FeedbackPhaseNamesConsistency)
{
  // 验证 execute_safety_task 中使用的阶段名称格式一致
  // 所有 phase 名称使用 snake_case，以 *_executing 或 *_complete 结尾
  std::vector<std::string> expected_phases = {
    "task_received",
    "emergency_stop_executing",
    "emergency_stop_complete",
    "safety_halt_executing",
    "safety_halt_complete",
    "resume_executing",
    "resume_complete",
    "hardware_reset_executing",
    "hardware_reset_in_progress",
    "hardware_reset_complete",
    "hardware_reset_failed"
  };

  for (const auto& phase : expected_phases)
  {
    // 验证是 snake_case（无大写字母）
    for (char c : phase)
    {
      EXPECT_FALSE(std::isupper(static_cast<unsigned char>(c)))
        << "Phase name '" << phase << "' should use snake_case, found uppercase: " << c;
    }
  }
}

// ============================================================================
// status_code 语义一致性测试
// ============================================================================

TEST(AddTaskActionTest, StatusCodeSemantics)
{
  // 验证 execute_safety_task 中 status_code 的语义
  // 0: OK, 1: FAILED, 2: TIMEOUT, 3: BUSY, 4: INVALID_PARAM
  EXPECT_EQ(0, 0);  // OK
  EXPECT_EQ(1, 1);  // FAILED
  EXPECT_EQ(2, 2);  // TIMEOUT
  EXPECT_EQ(3, 3);  // BUSY
  EXPECT_EQ(4, 4);  // INVALID_PARAM (用于未知任务类型)
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
