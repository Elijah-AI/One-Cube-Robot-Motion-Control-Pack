// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Description:
// 安全控制任务共享类型定义头文件。
// 包含优先级任务结构体、优先级常量、AddTask Action 类型别名等。
// 本文件保持轻量依赖，可被测试代码单独引用，
// 无需链接 Modbus / yaml-cpp / uuid 等重型库。

#ifndef TASK_MANAGER__SAFETY_TASK_TYPES_HPP_
#define TASK_MANAGER__SAFETY_TASK_TYPES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "com_interfaces/action/add_task.hpp"
#include <string>
#include <memory>
#include <cstdint>

namespace task_mgr
{

// ==================== AddTask Action 类型别名 ====================
using AddTaskAction   = com_interfaces::action::AddTask;
using GoalHandleAddTask = rclcpp_action::ServerGoalHandle<AddTaskAction>;

// ==================== 安全控制任务优先级常量 ====================
constexpr int32_t SAFETY_TASK_PRIORITY_MAX      = 25;  ///< 最高优先级（紧急停止）
constexpr int32_t SAFETY_TASK_PRIORITY_EMERGENCY = 20;  ///< 紧急任务优先级阈值

// A normal heartbeat from one safety interface cannot clear the other's stop.
inline int8_t effective_safety_status(int8_t device, int8_t legacy) {
  if (device < 0 || device > 2 || legacy < 0 || legacy > 2) return 2;
  return device > legacy ? device : legacy;
}

/// @brief 优先级任务队列项
/// 封装从 Action 接收的安全控制任务，按优先级排序
struct PrioritizedTask
{
  std::string task_id;           ///< 唯一任务标识符
  std::string task_type;         ///< 任务类型 (e.g., "emergency_stop")
  std::string target_node;       ///< 目标节点名称
  std::string parameters_json;   ///< JSON 格式的任务参数
  int32_t priority;              ///< 优先级 (1-25, 25最高)
  int64_t timeout_ms;            ///< 超时时间（毫秒）
  rclcpp::Time arrival_time;     ///< 任务到达时间戳
  std::shared_ptr<GoalHandleAddTask> goal_handle; ///< 关联的 Action goal handle

  /// @brief 优先级比较运算符（高优先级在前）
  bool operator<(const PrioritizedTask& other) const
  {
    // 高优先级值排前面，同优先级按到达时间 FIFO
    if (priority != other.priority)
      return priority < other.priority;  // std::priority_queue 默认大顶堆，所以用 <
    return arrival_time > other.arrival_time;  // 早到达的优先
  }
};

}  // namespace task_mgr

#endif  // TASK_MANAGER__SAFETY_TASK_TYPES_HPP_
