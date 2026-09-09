// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Author: gene.wang
// Date: 2026
//
// Description:
// 前置条件管理器实现（PreconditionManager / StateCache 组件）。
// 缓存来自 /Safety/device_safety_state 的设备健康数据（安全状态 / 机器人状态 / 电池电量），
// 并基于 TaskActionClient 本地资源锁评估动作前置条件（preconditions）。
// 数据缺失保护：缓存尚未收到数据时，相关条件一律视为"不满足"。

#include "task_manager/precondition_manager.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "task_manager/log_helper.hpp"

namespace task_mgr
{

// ============================================================================
// 公开接口
// ============================================================================

void PreconditionManager::init(rclcpp::Node::SharedPtr node)
{
  if (node) {
    logger_ = node->get_logger();
  }
  has_state_.store(false, std::memory_order_release);
  safety_status_.store(0, std::memory_order_release);
  robot_state_.store(0, std::memory_order_release);
  battery_percentage_.store(0.0f, std::memory_order_release);
  battery_voltage_.store(0.0f, std::memory_order_release);
  LOG_INFO(logger_, task_log::Tag::SYSTEM, "PreconditionManager 初始化完成");
}

void PreconditionManager::safety_state_cb(const com_interfaces::msg::SafetyState::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  safety_status_.store(msg->status, std::memory_order_release);
  robot_state_.store(msg->robot_state, std::memory_order_release);
  battery_percentage_.store(msg->battery_percentage, std::memory_order_release);
  battery_voltage_.store(msg->battery_voltage, std::memory_order_release);
  has_state_.store(true, std::memory_order_release);

  LOG_DEBUG(logger_, task_log::Tag::SYSTEM,
            "设备状态已缓存: safety_status=" << static_cast<int>(msg->status)
            << " robot_state=" << static_cast<int>(msg->robot_state)
            << " battery=" << msg->battery_percentage << "% / " << msg->battery_voltage << "V");
}

PrecheckResult PreconditionManager::evaluate(
  const task_cfg::PreconditionList& pre,
  const task_act::TaskActionClient& act_client) const
{
  PrecheckResult result;
  result.satisfied = true;
  result.fail_reason.clear();

  // 未配置前置条件（或条件列表为空）→ 恒满足
  if (!pre.has || pre.conditions.empty()) {
    return result;
  }

  const bool is_or = (pre.logical_op == task_cfg::PreLogicalOp::OR);
  size_t enabled_count = 0;

  for (const auto& cond : pre.conditions) {
    if (!cond.enable) {
      continue;  // 禁用的条件恒通过
    }
    ++enabled_count;

    std::string cond_reason;
    const bool ok = evaluate_one(cond, cond_reason, act_client);

    if (is_or) {
      // OR：任一满足即整体满足（短路）
      if (ok) {
        result.satisfied = true;
        result.fail_reason.clear();
        return result;
      }
      // 记录最近一条失败原因，供全部失败时输出
      result.fail_reason = cond_reason;
    } else {
      // AND：任一不满足即整体不满足（短路）
      if (!ok) {
        result.satisfied = false;
        result.fail_reason = cond_reason;
        return result;
      }
    }
  }

  if (is_or) {
    // OR：没有启用条件满足 → 整体不满足（全部禁用时视为满足）
    result.satisfied = (enabled_count == 0);
    if (!result.satisfied) {
      LOG_WARN(logger_, task_log::Tag::ACTION,
               "前置条件(OR)全部不满足: " << result.fail_reason);
    }
  } else {
    // AND：全部启用条件满足 → 整体满足
    result.satisfied = true;
    result.fail_reason.clear();
  }

  return result;
}

bool PreconditionManager::evaluate_one(
  const task_cfg::PreconditionParam& p,
  std::string& fail_reason,
  const task_act::TaskActionClient& act_client) const
{
  fail_reason.clear();

  // 未启用 → 恒通过
  if (!p.enable) {
    return true;
  }

  switch (p.type) {
    case task_cfg::PreconditionType::BATTERY_LEVEL:
    {
      if (!has_state_.load(std::memory_order_acquire)) {
        fail_reason = "电池条件评估失败：尚未收到设备安全状态数据";
        return false;
      }
      const bool use_voltage = (p.field == "voltage");
      const double value = use_voltage
        ? static_cast<double>(battery_voltage_.load(std::memory_order_acquire))
        : static_cast<double>(battery_percentage_.load(std::memory_order_acquire));
      const bool ok = compare_value(value, p.operator_str, p.threshold);
      if (!ok) {
        std::ostringstream oss;
        oss << "电池条件不满足: field=" << (use_voltage ? "voltage" : "percentage")
            << " value=" << value
            << " op=" << p.operator_str << " threshold=" << p.threshold;
        fail_reason = oss.str();
      }
      return ok;
    }

    case task_cfg::PreconditionType::ROBOT_STATUS:
    case task_cfg::PreconditionType::SAFETY_STATUS:
    {
      if (!has_state_.load(std::memory_order_acquire)) {
        fail_reason = (p.type == task_cfg::PreconditionType::ROBOT_STATUS)
          ? "机器人状态条件评估失败：尚未收到设备安全状态数据"
          : "安全状态条件评估失败：尚未收到设备安全状态数据";
        return false;
      }
      const int32_t cur = (p.type == task_cfg::PreconditionType::ROBOT_STATUS)
        ? static_cast<int32_t>(robot_state_.load(std::memory_order_acquire))
        : static_cast<int32_t>(safety_status_.load(std::memory_order_acquire));
      const bool in_set =
        std::find(p.expected.begin(), p.expected.end(), cur) != p.expected.end();
      if (!in_set) {
        std::ostringstream oss;
        oss << pre_type_to_str(p.type) << " 条件不满足: 当前值=" << cur << " 期望∈{";
        for (size_t i = 0; i < p.expected.size(); ++i) {
          if (i) {
            oss << ",";
          }
          oss << p.expected[i];
        }
        oss << "}";
        fail_reason = oss.str();
      }
      return in_set;
    }

    case task_cfg::PreconditionType::ARM_IDLE:
    case task_cfg::PreconditionType::HEAD_IDLE:
    case task_cfg::PreconditionType::WAIST_IDLE:
    case task_cfg::PreconditionType::GRIPPER_IDLE:
    case task_cfg::PreconditionType::SUCK_IDLE:
    case task_cfg::PreconditionType::DEXTEROUS_HAND_IDLE:
    {
      std::vector<task_act::ResourceType> resources;
      resolve_idle_resources(p, resources);
      return evaluate_idle(resources, p.expected_bool, act_client, fail_reason);
    }

    default:
      fail_reason = "未知前置条件类型（type=" + pre_type_to_str(p.type) + "），按不满足处理";
      return false;
  }
}

// ============================================================================
// 私有辅助
// ============================================================================

bool PreconditionManager::compare_value(double value, const std::string& op, double threshold)
{
  constexpr double kEps = 1e-6;
  if (op == ">") {
    return value > threshold;
  }
  if (op == "<") {
    return value < threshold;
  }
  if (op == "<=") {
    return value <= threshold;
  }
  if (op == "==") {
    return std::fabs(value - threshold) <= kEps;
  }
  if (op == "!=") {
    return std::fabs(value - threshold) > kEps;
  }
  // 默认（含 ">=" 与未知运算符）按 >= 处理
  return value >= threshold;
}

std::string PreconditionManager::pre_type_to_str(task_cfg::PreconditionType t)
{
  switch (t) {
    case task_cfg::PreconditionType::BATTERY_LEVEL:       return "battery_level";
    case task_cfg::PreconditionType::ROBOT_STATUS:        return "robot_status";
    case task_cfg::PreconditionType::SAFETY_STATUS:       return "safety_status";
    case task_cfg::PreconditionType::ARM_IDLE:            return "arm_idle";
    case task_cfg::PreconditionType::HEAD_IDLE:           return "head_idle";
    case task_cfg::PreconditionType::WAIST_IDLE:          return "waist_idle";
    case task_cfg::PreconditionType::GRIPPER_IDLE:        return "gripper_idle";
    case task_cfg::PreconditionType::SUCK_IDLE:           return "suck_idle";
    case task_cfg::PreconditionType::DEXTEROUS_HAND_IDLE: return "dexterous_hand_idle";
    default:                                              return "unknown";
  }
}

void PreconditionManager::resolve_idle_resources(
  const task_cfg::PreconditionParam& p,
  std::vector<task_act::ResourceType>& resources) const
{
  resources.clear();
  const bool left = (p.part == task_cfg::PrePart::LEFT || p.part == task_cfg::PrePart::BOTH);
  const bool right = (p.part == task_cfg::PrePart::RIGHT || p.part == task_cfg::PrePart::BOTH);

  switch (p.type) {
    case task_cfg::PreconditionType::ARM_IDLE:
      if (left) {
        resources.push_back(task_act::ResourceType::LEFT_ARM);
      }
      if (right) {
        resources.push_back(task_act::ResourceType::RIGHT_ARM);
      }
      break;
    case task_cfg::PreconditionType::HEAD_IDLE:
      // 头无左右之分，忽略 part
      resources.push_back(task_act::ResourceType::HEAD);
      break;
    case task_cfg::PreconditionType::WAIST_IDLE:
      // 腰部统一并入 LEG（与资源锁一致，不单列 WAIST）
      resources.push_back(task_act::ResourceType::LEG);
      break;
    case task_cfg::PreconditionType::GRIPPER_IDLE:
      if (left) {
        resources.push_back(task_act::ResourceType::LEFT_GRIPPER);
      }
      if (right) {
        resources.push_back(task_act::ResourceType::RIGHT_GRIPPER);
      }
      break;
    case task_cfg::PreconditionType::SUCK_IDLE:
      if (left) {
        resources.push_back(task_act::ResourceType::LEFT_SUCK);
      }
      if (right) {
        resources.push_back(task_act::ResourceType::RIGHT_SUCK);
      }
      break;
    case task_cfg::PreconditionType::DEXTEROUS_HAND_IDLE:
      if (left) {
        resources.push_back(task_act::ResourceType::LEFT_HAND);
      }
      if (right) {
        resources.push_back(task_act::ResourceType::RIGHT_HAND);
      }
      break;
    default:
      break;
  }
}

bool PreconditionManager::evaluate_idle(
  const std::vector<task_act::ResourceType>& resources,
  bool expected_idle,
  const task_act::TaskActionClient& act_client,
  std::string& fail_reason) const
{
  if (resources.empty()) {
    fail_reason = "部件空闲条件评估失败：未解析到任何资源（part 配置无效）";
    return false;
  }

  std::vector<std::string> violated;
  for (const auto res : resources) {
    const bool busy = act_client.is_resource_busy(res);
    const bool ok = (expected_idle ? !busy : busy);
    if (!ok) {
      violated.push_back(task_act::resource_type_to_str(res));
    }
  }

  if (!violated.empty()) {
    std::ostringstream oss;
    oss << "部件空闲条件不满足: 期望=" << (expected_idle ? "空闲" : "占用") << " 违反资源=[";
    for (size_t i = 0; i < violated.size(); ++i) {
      if (i) {
        oss << ",";
      }
      oss << violated[i];
    }
    oss << "]";
    fail_reason = oss.str();
    return false;
  }
  return true;
}

} // namespace task_mgr
