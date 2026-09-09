// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Author: gene.wang
// Date: 2026
//
// Description:
// 前置条件管理器（PreconditionManager / StateCache 组件）。
// 负责缓存来自 /Safety/device_safety_state 的设备健康数据（安全状态 / 机器人状态 / 电池电量），
// 并基于 TaskActionClient 本地资源锁评估动作前置条件（preconditions）。

#ifndef TASK_MANAGER__PRECONDITION_MANAGER_HPP_
#define TASK_MANAGER__PRECONDITION_MANAGER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "com_interfaces/msg/safety_state.hpp"
#include "task_manager/task_action_client.hpp"
#include "task_manager/task_config_parser.hpp"

namespace task_mgr
{

/// @brief 前置条件评估结果
struct PrecheckResult
{
  bool satisfied{false};        ///< 是否满足
  std::string fail_reason;      ///< 不满足原因（供日志）
};

/// @brief 前置条件管理器
/// 数据缓存（StateCache）：
///   - 设备安全状态 / 机器人状态 / 电池电量：来自 /Safety/device_safety_state（安全监控节点统一出口）
///   - 各部件忙闲：直接读取 TaskActionClient::is_resource_busy()（本地资源锁，task_manager 内部维护）
/// 数据缺失保护：缓存尚未收到数据时，相关条件一律视为"不满足"，避免在无数据时误放行动作。
class PreconditionManager
{
public:
  /// @brief 初始化（保存节点引用，供日志使用）
  void init(rclcpp::Node::SharedPtr node);

  /// @brief 设备安全状态订阅回调（由 TaskManagerNode 在 safety_state_callback 中转发）
  void safety_state_cb(const com_interfaces::msg::SafetyState::SharedPtr msg);

  /// @brief 评估一组前置条件（按 logical_op 组合：and / or）
  PrecheckResult evaluate(const task_cfg::PreconditionList& pre,
                          const task_act::TaskActionClient& act_client) const;

  /// @brief 评估单个前置条件
  bool evaluate_one(const task_cfg::PreconditionParam& p,
                    std::string& fail_reason,
                    const task_act::TaskActionClient& act_client) const;

  /// @brief 是否已收到设备安全状态数据（含机器人状态 / 电池电量）
  bool has_device_state() const { return has_state_.load(std::memory_order_acquire); }

private:
  /// @brief 数值比较辅助（>=, >, <=, <, ==, !=；未知运算符按 >= 处理）
  static bool compare_value(double value, const std::string& op, double threshold);

  /// @brief 将条件类型转为可读字符串
  static std::string pre_type_to_str(task_cfg::PreconditionType t);

  /// @brief 将部件空闲类条件的 type + part 解析为具体资源集合
  void resolve_idle_resources(const task_cfg::PreconditionParam& p,
                              std::vector<task_act::ResourceType>& resources) const;

  /// @brief 部件空闲条件评估（arm/head/waist/gripper/suck/dexterous_hand_idle）
  /// @param resources 该条件对应的资源集合（按 part 解析）
  /// @param expected_idle 期望空闲（true）还是占用（false）
  bool evaluate_idle(const std::vector<task_act::ResourceType>& resources,
                     bool expected_idle,
                     const task_act::TaskActionClient& act_client,
                     std::string& fail_reason) const;

  // ==================== 数据缓存（订阅回调更新） ====================
  std::atomic<bool> has_state_{false};              ///< 是否已收到设备状态数据
  std::atomic<int8_t> safety_status_{0};            ///< SafetyState.status（0=正常 / 1=受限 / 2=急停）
  std::atomic<int8_t> robot_state_{0};              ///< SafetyState.robot_state（SystemFsmState 口径）
  std::atomic<float> battery_percentage_{0.0f};     ///< SafetyState.battery_percentage
  std::atomic<float> battery_voltage_{0.0f};        ///< SafetyState.battery_voltage

  rclcpp::Logger logger_{rclcpp::get_logger("PreconditionManager")};
};

} // namespace task_mgr

#endif // TASK_MANAGER__PRECONDITION_MANAGER_HPP_
