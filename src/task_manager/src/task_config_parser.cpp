// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Author: gene.wang
// Date: 2026
//
// Description:
// 任务配置文件解析器实现。
// 负责加载 YAML 格式的任务配置文件，解析其中的节点配置、
// Modbus 参数、任务定义（含视觉检测、机械臂移动、夹爪控制等动作），
// 以及开机自检/复位任务序列。

#include "task_manager/task_config_parser.hpp"
#include "com_interfaces/tcp_level_goal.hpp"
#include "com_interfaces/rigid_grasp_goal.hpp"
#include "com_interfaces/action/move_pose.hpp"
#include <algorithm>
#include "task_manager/log_helper.hpp"
#include "com_interfaces/leg_body_motion_goal.hpp"
#include <rclcpp/rclcpp.hpp>

namespace task_cfg
{

// 配置解析器统一日志器名称
static const char* kConfigLogger = "task_config_parser";

// Match the installed motion controller's per-Goal precision contract.
static bool parse_precision_arrival(const YAML::Node& p, uint8_t arm_id, bool pose_goal)
{
  const bool enabled = p["precision_arrival"] ? p["precision_arrival"].as<bool>() : false;
  if (!enabled) return false;
  const bool supported = pose_goal ? arm_id >= 4 && arm_id <= 7 : arm_id == 3 || arm_id == 4;
  if (!supported) {
    throw std::invalid_argument(
      "precision_arrival supports HEAD/LEG MoveJoints and LEG/WBC MovePose only");
  }
  if (p["control_mode"] && p["control_mode"].as<uint8_t>() != 0) {
    throw std::invalid_argument("precision_arrival requires position control");
  }
  return true;
}

// Numeric modes are authoritative. Legacy booleans remain readable.
static uint8_t parse_collision_mode(const YAML::Node& p)
{
  const auto node = p["enable_collision_check"];
  if (!node) return 0;
  const auto value = node.as<std::string>();
  if (value == "false" || value == "False" || value == "FALSE") return 0;
  if (value == "true" || value == "True" || value == "TRUE") {
    return p["enable_point_cloud_check"] && p["enable_point_cloud_check"].as<bool>() ? 2 : 1;
  }
  if (value == "0" || value == "1" || value == "2")
    return static_cast<uint8_t>(value[0] - '0');
  throw std::invalid_argument("enable_collision_check must be 0, 1 or 2");
}

// ============================================================================
// 前置条件 / 结果分支 字符串映射工具
// ============================================================================

/// @brief 将条件类型字符串映射为枚举
static PreconditionType pre_type_from_str(const std::string& s)
{
  if (s == "battery_level")       { return PreconditionType::BATTERY_LEVEL; }
  if (s == "robot_status")        { return PreconditionType::ROBOT_STATUS; }
  if (s == "safety_status")       { return PreconditionType::SAFETY_STATUS; }
  if (s == "arm_idle")            { return PreconditionType::ARM_IDLE; }
  if (s == "head_idle")           { return PreconditionType::HEAD_IDLE; }
  if (s == "waist_idle")          { return PreconditionType::WAIST_IDLE; }
  if (s == "gripper_idle")        { return PreconditionType::GRIPPER_IDLE; }
  if (s == "suck_idle")           { return PreconditionType::SUCK_IDLE; }
  if (s == "dexterous_hand_idle") { return PreconditionType::DEXTEROUS_HAND_IDLE; }
  return PreconditionType::UNKNOWN;
}

/// @brief 将组合逻辑字符串映射为枚举（and / or，默认 and）
static PreLogicalOp pre_op_from_str(const std::string& s)
{
  if (s == "or") { return PreLogicalOp::OR; }
  return PreLogicalOp::AND;
}

/// @brief 将处理策略字符串映射为枚举（wait / fail_fast / skip，默认 wait）
static PrePolicy pre_policy_from_str(const std::string& s)
{
  if (s == "fail_fast") { return PrePolicy::FAIL_FAST; }
  if (s == "skip")      { return PrePolicy::SKIP; }
  return PrePolicy::WAIT;
}

/// @brief 将等待超时行为字符串映射为枚举（fail / pass / abort，默认 fail）
static PreTimeoutAction pre_timeout_from_str(const std::string& s)
{
  if (s == "pass")  { return PreTimeoutAction::PASS; }
  if (s == "abort") { return PreTimeoutAction::ABORT; }
  return PreTimeoutAction::FAIL;
}

/// @brief 将部件方位字符串映射为枚举（left / right / both，默认 both）
static PrePart pre_part_from_str(const std::string& s)
{
  if (s == "left")  { return PrePart::LEFT; }
  if (s == "right") { return PrePart::RIGHT; }
  return PrePart::BOTH;
}

/// @brief 将期望状态节点（数值或枚举名）解析为枚举值
/// robot_status 以安全监控节点 SystemFsmState 为准：
///   UNKNOWN=0 INIT=1 SELF_CHECK=2 RESET=3 RESET_DONE=4 IDLE=5 RUN=6 ABNORMAL=7 EMERGENCY=8
/// 支持 "IDLE" / 5 等数值或字符串写法。
/// @return 解析失败时返回 -1
static int32_t status_value_from_node(const YAML::Node& n)
{
  // 先尝试数值
  try
  {
    return n.as<int32_t>();
  }
  catch (...)
  {
  }
  // 再尝试枚举名
  try
  {
    std::string s = n.as<std::string>();
    if (s == "UNKNOWN")     { return 0; }
    if (s == "INIT")        { return 1; }
    if (s == "SELF_CHECK")  { return 2; }
    if (s == "RESET")       { return 3; }
    if (s == "RESET_DONE")  { return 4; }
    if (s == "IDLE")        { return 5; }
    if (s == "RUN" || s == "RUNNING") { return 6; }
    if (s == "ABNORMAL")    { return 7; }
    if (s == "EMERGENCY")   { return 8; }
  }
  catch (...)
  {
  }
  return -1;
}

/// @brief 加载并解析 YAML 配置文件（兼容单文件格式，等价于 load_all）
/// @param file_path 配置文件路径
/// @param out_cfg [输出] 解析后的全局配置
/// @return true 解析成功，false 解析失败
bool TaskConfigParser::load_config(const std::string& file_path, GlobalConfig& out_cfg)
{
  return load_all(file_path, out_cfg);
}

/// @brief 加载全部配置（主配置 + 级联加载外链的次级任务配置文件）
bool TaskConfigParser::load_all(const std::string& main_file_path, GlobalConfig& out_cfg)
{
  // 清空，避免重复加载时残留旧数据
  out_cfg = GlobalConfig{};

  if (!load_main_config(main_file_path, out_cfg))
  {
    return false;
  }

  // 级联加载外链的次级任务配置文件（config_file 已在 load_main_config 中解析为绝对/相对当前目录路径）
  for (auto& [name, item] : out_cfg.task_dict)
  {
    if (!item.config_file.empty() && item.action_seq.empty())
    {
      if (!load_task_config(item.config_file, out_cfg, name))
      {
        return false;
      }
    }
  }
  return true;
}

/// @brief 加载主配置文件（节点 ID、Modbus 参数、任务注册表、启动任务）
bool TaskConfigParser::load_main_config(const std::string& file_path, GlobalConfig& out_cfg)
{
  try{
    YAML::Node root = YAML::LoadFile(file_path);

    // 检查根节点是否为有效映射
    if(!root.IsMap())
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "YAML file empty or invalid map, path: " << file_path);
      return false;
    }

    // 解析节点 ID
    out_cfg.node_id = root["node_id"].as<int32_t>();
    LOG_INFO(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "node_id: " << out_cfg.node_id);

    // 解析 Modbus 从站配置
    auto mb_node = root["modbus_slave"];
    out_cfg.modbus_listen_ip = mb_node["listen_ip"].as<std::string>();
    out_cfg.modbus_port = mb_node["listen_port"].as<uint16_t>();
    out_cfg.modbus_slave_id = mb_node["slave_id"] ? mb_node["slave_id"].as<uint8_t>() : 1;
    LOG_INFO(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
      "modbus_slave: ip=" << out_cfg.modbus_listen_ip << ", port=" << out_cfg.modbus_port
      << ", slave_id=" << static_cast<int>(out_cfg.modbus_slave_id));

    // 主配置文件所在目录（用于解析相对 config_file 路径，见设计方案 9.10）
    std::string base_dir = ".";
    {
      size_t pos = file_path.find_last_of('/');
      if (pos != std::string::npos)
      {
        base_dir = file_path.substr(0, pos);
      }
    }

    // 解析任务列表
    auto tasks_node = root["tasks"];
    for(auto it = tasks_node.begin(); it != tasks_node.end(); ++it)
    {
      auto name = it->first.as<std::string>();
      LOG_INFO_FLOW(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "Parsing task: " << name);

      TaskMapItem item;
      item.trigger_reg = it->second["trigger_reg"].as<uint32_t>();
      item.status_reg = it->second["status_reg"].as<uint32_t>();

      // 外链格式：config_file 指向次级任务动作配置文件（相对主配置文件所在目录解析）
      auto cf = it->second["config_file"];
      if (cf.IsDefined() && !cf.IsNull())
      {
        item.config_file = cf.as<std::string>();
        if (!item.config_file.empty() && item.config_file[0] != '/')
        {
          item.config_file = base_dir + "/" + item.config_file;
        }
        LOG_INFO(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
          "Task [" << name << "] uses external config_file: " << item.config_file);
      }
      else
      {
        // 内联格式：直接解析动作序列
        if(!parse_task(it->second, item))
        {
          LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "Failed to parse inline task: " << name);
          return false;
        }
      }
      out_cfg.task_dict[name] = item;
    }

    // 解析启动任务（支持外链 config_file 或内联 self_check/reset，见设计方案 4.1.4/4.1.5）
    auto boot_node = root["boot_task"];
    auto boot_cf = boot_node["config_file"];
    if (boot_cf.IsDefined() && !boot_cf.IsNull())
    {
      std::string boot_file = boot_cf.as<std::string>();
      if (!boot_file.empty() && boot_file[0] != '/')
      {
        boot_file = base_dir + "/" + boot_file;
      }
      LOG_INFO(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "boot_task uses external config_file: " << boot_file);
      YAML::Node ext_boot = YAML::LoadFile(boot_file);
      if (!parse_boot(ext_boot, out_cfg.boot_tasks))
      {
        return false;
      }
    }
    else
    {
      if (!parse_boot(boot_node, out_cfg.boot_tasks))
      {
        return false;
      }
    }

    LOG_SUCCESS(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
      "Main config loaded successfully, tasks: " << out_cfg.task_dict.size());
    return true;

  }catch(std::exception& e){
    LOG_ERROR_CRITICAL(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "YAML parse error: " << e.what());
    return false;
  }
}

/// @brief 加载次级任务动作配置文件并合并到 out_cfg
bool TaskConfigParser::load_task_config(const std::string& file_path, GlobalConfig& out_cfg, const std::string& task_name)
{
  try{
    YAML::Node root = YAML::LoadFile(file_path);

    // 检查根节点是否为有效映射
    if(!root.IsMap())
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "Task config file empty or invalid map, path: " << file_path);
      return false;
    }

    // 查找（或防御性创建）目标任务条目
    auto it = out_cfg.task_dict.find(task_name);
    if (it == out_cfg.task_dict.end())
    {
      out_cfg.task_dict[task_name] = TaskMapItem{};
      it = out_cfg.task_dict.find(task_name);
    }

    if (parse_task(root, it->second))
    {
      LOG_INFO(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Task [" << task_name << "] actions loaded from external file: " << file_path);
      return true;
    }
    return false;

  }catch(std::exception& e){
    LOG_ERROR_CRITICAL(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "Task config YAML parse error: " << e.what());
    return false;
  }
}

/// @brief 解析单个动作节点
/// @param action_node YAML 动作节点
/// @param act [输出] 解析后的原子动作
/// @return true 解析成功
bool TaskConfigParser::parse_action(YAML::Node action_node, AtomicAction& act, int depth)
{
  act.action_type = action_node["action_type"].as<std::string>();

  // 解析执行前延迟时间（毫秒），可选参数，默认 0 表示不延迟
  act.delay_ms = action_node["delay_ms"] ? action_node["delay_ms"].as<int64_t>() : 0;
  if (act.delay_ms > 0)
  {
    LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
      "Action [" << act.action_type << "] will delay " << act.delay_ms << " ms before execution");
  }

  auto p = action_node["params"];

  // 根据动作类型分别解析对应参数
  // 根据动作类型分别解析对应参数（各动作类型的解析逻辑已拆分为独立函数，
  // 见本文件 parse_action 之后的 parse_*_params 实现）
  if (act.action_type == "vision_detect")
  {
    if (!parse_vision_detect_params(p, act))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse vision_detect params");
      return false;
    }
  }
  else if (act.action_type == "move_pose")
  {
    if (!parse_move_pose_params(p, act))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse move_pose params");
      return false;
    }
  }
  else if (act.action_type == "gripper_ctrl")
  {
    if (!parse_gripper_ctrl_params(p, act))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse gripper_ctrl params");
      return false;
    }
  }
  else if (act.action_type == "vision_check_self")
  {
    if (!parse_vision_check_self_params(p, act))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse vision_check_self params");
      return false;
    }
  }
  else if (act.action_type == "control_check_self")
  {
    if (!parse_control_check_self_params(p, act))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse control_check_self params");
      return false;
    }
  }
  else if (act.action_type == "control_reset")
  {
    if (!parse_control_reset_params(p, act))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse control_reset params");
      return false;
    }
  }
  else if (act.action_type == "vision_reset")
  {
    if (!parse_vision_reset_params(p, act))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse vision_reset params");
      return false;
    }
  }
  else if (act.action_type == "move_joints")
  {
    if (!parse_move_joints_params(p, act))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse move_joints params");
      return false;
    }
  }
  else if (act.action_type == "control_suck")
  {
    if (!parse_control_suck_params(p, act))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse control_suck params");
      return false;
    }
  }
  else if (act.action_type == "calibration_control")
  {
    if (!parse_calibration_control_params(p, act))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse calibration_control params");
      return false;
    }
  }
  else if (act.action_type == "auto_calibrate")
  {
    if (!parse_auto_calibrate_params(p, act))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse auto_calibrate params");
      return false;
    }
  }
  else if (act.action_type == "move_compound")
  {
    // 复合动作：内部可包含多个 move_pose / move_joints 子动作，并行执行
    if (!parse_compound_move_params(p, act))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse move_compound params");
      return false;
    }
  }

  // 解析动作反馈条件检查（可选，用于条件重试）
  // 示例：动作（如 vision_detect）执行成功后检查反馈字段（如 pose_confidence）是否满足阈值，
  //       若不满足则依次执行 retry_actions 中的调整动作，然后重新执行本动作。
  auto fc = action_node["feedback_check"];
  if (fc.IsDefined() && !fc.IsNull())
  {
    act.feedback_check.enable = fc["enable"] ? fc["enable"].as<bool>() : true;
    act.feedback_check.check_field = fc["check_field"] ? fc["check_field"].as<std::string>() : "pose_confidence";
    act.feedback_check.operator_str = fc["operator"] ? fc["operator"].as<std::string>() : ">=";
    act.feedback_check.threshold = fc["threshold"] ? fc["threshold"].as<double>() : 0.0;
    act.feedback_check.max_retries = fc["max_retries"] ? fc["max_retries"].as<int32_t>() : 1;
    act.feedback_check.on_exhaust = fc["on_exhaust"] ? fc["on_exhaust"].as<std::string>() : "fail";

    // 递归解析调整动作序列（条件不满足时按 when 条件选择执行）
    auto retry_node = fc["retry_actions"];
    if (retry_node.IsDefined() && !retry_node.IsNull() && retry_node.IsSequence())
    {
      for (auto ra : retry_node)
      {
        RetryActionParam retry_param;

        // 解析基于反馈参数的选择条件（when，可选；未配置则始终执行该调整动作）
        auto when_node = ra["when"];
        if (when_node.IsDefined() && !when_node.IsNull())
        {
          retry_param.when.enable = when_node["enable"] ? when_node["enable"].as<bool>() : true;
          retry_param.when.field = when_node["field"] ? when_node["field"].as<std::string>() : "pose_confidence";
          retry_param.when.operator_str = when_node["operator"] ? when_node["operator"].as<std::string>() : ">=";
          retry_param.when.threshold = when_node["threshold"] ? when_node["threshold"].as<double>() : 0.0;
          LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
            "Retry action when-condition: field=" << retry_param.when.field
            << ", operator=" << retry_param.when.operator_str
            << ", threshold=" << retry_param.when.threshold);
        }

        auto retry_act = std::make_shared<AtomicAction>();
        if (parse_action(ra, *retry_act))
        {
          retry_param.action = retry_act;
          act.feedback_check.retry_actions.push_back(retry_param);
        }
        else
        {
          LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
            "Failed to parse retry action for feedback_check of: " << act.action_type);
        }
      }
    }

    LOG_INFO(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
      "Action [" << act.action_type << "] feedback_check enabled, field=" << act.feedback_check.check_field
      << ", operator=" << act.feedback_check.operator_str
      << ", threshold=" << act.feedback_check.threshold
      << ", max_retries=" << act.feedback_check.max_retries
      << ", on_exhaust=" << act.feedback_check.on_exhaust
      << ", retry_actions=" << act.feedback_check.retry_actions.size());
  }

  // 解析前置条件（可选，见设计方案 4.2）
  auto pre_node = action_node["preconditions"];
  if (pre_node.IsDefined() && !pre_node.IsNull())
  {
    if (!parse_preconditions(pre_node, act.preconditions))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse preconditions of action: " << act.action_type);
      return false;
    }
  }

  // 解析结果分支（可选，见设计方案 4.3；支持递归嵌套，受深度上限保护）
  if (depth >= kMaxBranchDepth)
  {
    LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
      "Action [" << act.action_type << "] result branch nesting depth exceeds limit " << kMaxBranchDepth);
    return false;
  }

  auto os_node = action_node["on_success"];
  if (os_node.IsDefined() && !os_node.IsNull())
  {
    if (!parse_result_branch(os_node, act.on_success, depth))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse on_success branch of action: " << act.action_type);
      return false;
    }
  }

  auto of_node = action_node["on_failure"];
  if (of_node.IsDefined() && !of_node.IsNull())
  {
    if (!parse_result_branch(of_node, act.on_failure, depth))
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "Failed to parse on_failure branch of action: " << act.action_type);
      return false;
    }
  }
  if (auto recovery = action_node["ik_failure_recovery"]) {
    auto& config = act.ik_failure_recovery;
    config.enabled = recovery["enabled"] ? recovery["enabled"].as<bool>() : true;
    config.key = recovery["key"] ? recovery["key"].as<std::string>() : "";
    config.lift_m = recovery["lift_m"] ? recovery["lift_m"].as<double>() : 0.05;
    config.speed = recovery["speed"] ? recovery["speed"].as<float>() : 0.10f;
    config.timeout_ms = recovery["timeout_ms"] ? recovery["timeout_ms"].as<int64_t>() : 30000;
    config.collision_mode = recovery["enable_collision_check"] ? parse_collision_mode(recovery) :
        act.arm_param.enable_collision_check;
    if (config.enabled && (act.action_type != "move_pose" || act.arm_param.arm_id < 5 ||
        act.arm_param.arm_id > 7 || !act.arm_param.whole_body_waist_only || config.key.empty() ||
        !std::isfinite(config.lift_m) || config.lift_m <= 0 || !std::isfinite(config.speed) ||
        config.speed <= 0 || config.speed > 1 || config.timeout_ms <= 0 || act.feedback_check.enable))
      throw std::invalid_argument("IK leg recovery requires a waist-only whole-body MovePose, key, positive lift, valid speed/timeout and no feedback_check");
  }
  if (action_node["if_recovery_applied"]) {
    act.if_recovery_applied = action_node["if_recovery_applied"].as<std::string>();
    const auto& move = act.move_joints_param;
    const bool restore = move.leg_motion_mode == 2 &&
        std::isfinite(move.body_height_m) && move.body_height_m < 0;
    const bool recovery_turn = move.leg_motion_mode == 0 && !move.relative_position &&
        move.joint_name == std::vector<std::string>{"body_joint"} && move.position.size() == 1 &&
        std::isfinite(move.position[0]);
    if (act.if_recovery_applied.empty() || act.action_type != "move_joints" ||
        move.arm_id != 4 || (!restore && !recovery_turn) ||
        act.ik_failure_recovery.enabled || act.feedback_check.enable)
      throw std::invalid_argument("Conditional recovery action requires a key and a negative LEG vertical-relative restore or absolute waist turn");
  }
  return true;
}

// ============================================================================
// 各动作类型参数解析函数（由 parse_action 按 action_type 分发调用）
// 说明：参数结构字段缺失/类型不符时 .as<>() 会抛异常，
//       由上层 load_main_config / load_task_config 的 try/catch 统一处理。
// ============================================================================

/// @brief 解析视觉检测动作参数（vision_detect）
bool TaskConfigParser::parse_vision_detect_params(YAML::Node p, AtomicAction& act)
{
  // 视觉检测参数
  act.vision_param.target_obj_name = p["target_obj_name"].as<std::string>();
  act.vision_param.expected_frame_id = p["expected_frame_id"].as<std::string>();
  act.vision_param.priority = p["priority"].as<int32_t>();
  act.vision_param.type = p["type"] ? p["type"].as<int8_t>() : 0;
  act.vision_param.result_key = p["result_key"] ? p["result_key"].as<std::string>() : "";
  act.vision_param.require_single_target = p["require_single_target"] ?
    p["require_single_target"].as<bool>() : false;
  act.vision_param.require_expected_frame = p["require_expected_frame"] ?
    p["require_expected_frame"].as<bool>() : false;
  if (act.vision_param.require_expected_frame && act.vision_param.expected_frame_id.empty()) {
    throw std::invalid_argument("require_expected_frame needs a nonempty expected_frame_id");
  }
  if (p["canonical_y_direction"]) {
    auto& param = act.vision_param;
    param.canonical_y_direction = p["canonical_y_direction"].as<std::vector<double>>();
    param.canonical_y_min_alignment = p["canonical_y_min_alignment"] ?
      p["canonical_y_min_alignment"].as<double>() : 0.5;
    double norm2 = 0.0;
    for (const double v : param.canonical_y_direction) norm2 += v*v;
    if (param.canonical_y_direction.size() != 3U || !std::isfinite(norm2) ||
        norm2 < 1e-12 || !param.require_expected_frame || !param.require_single_target ||
        !std::isfinite(param.canonical_y_min_alignment) ||
        param.canonical_y_min_alignment <= 0.0 || param.canonical_y_min_alignment > 1.0) {
      throw std::invalid_argument(
        "canonical_y_direction needs a finite nonzero 3-vector, checked frame, single target "
        "and canonical_y_min_alignment in (0,1]");
    }
  } else if (p["canonical_y_min_alignment"]) {
    throw std::invalid_argument("canonical_y_min_alignment requires canonical_y_direction");
  }
  return true;
}

/// @brief 解析机械臂位姿移动动作参数（move_pose）
bool TaskConfigParser::parse_move_pose_params(YAML::Node p, AtomicAction& act)
{
  const auto scale = [&](const char* key) {
    const float value = p[key] ? p[key].as<float>() : 0.0f;
    if (!std::isfinite(value) || value < 0 || value > 1) {
      throw std::invalid_argument(std::string(key) + " must be in [0,1]");
    }
    return value;
  };

  // 机械臂位姿移动参数（对应 MovePose.action）
  act.arm_param.arm_id = p["arm_id"].as<uint8_t>();
  act.arm_param.precision_arrival = parse_precision_arrival(p, act.arm_param.arm_id, true);
  act.arm_param.keep_tcp_level = p["keep_tcp_level"] ? p["keep_tcp_level"].as<bool>() : false;
  act.arm_param.tcp_level_tolerance_rad = p["tcp_level_tolerance_rad"] ?
    p["tcp_level_tolerance_rad"].as<double>() : 0.02;
  std::string level_error;
  if (!com_interfaces::validateTcpLevelGoal(act.arm_param.keep_tcp_level,
      act.arm_param.tcp_level_tolerance_rad, act.arm_param.arm_id, true,
      p["control_mode"] ? p["control_mode"].as<uint8_t>() : 0, level_error))
    throw std::invalid_argument(level_error);
  act.arm_param.enable_collision_check = parse_collision_mode(p);
  act.arm_param.keep_grasp_rigid = p["keep_grasp_rigid"] ? p["keep_grasp_rigid"].as<bool>() : false;
  act.arm_param.grasp_position_tolerance_m = p["grasp_position_tolerance_m"] ? p["grasp_position_tolerance_m"].as<double>() : .001;
  act.arm_param.grasp_orientation_tolerance_rad = p["grasp_orientation_tolerance_rad"] ? p["grasp_orientation_tolerance_rad"].as<double>() : .005;
  struct RigidGoalView {
    bool keep_grasp_rigid, keep_tcp_level, use_force_control, enable_move_line;
    int arm_id, control_mode, synchronization_mode;
    double grasp_position_tolerance_m, grasp_orientation_tolerance_rad;
  } rigid_view{act.arm_param.keep_grasp_rigid, act.arm_param.keep_tcp_level,
    p["use_force_control"] ? p["use_force_control"].as<bool>() : false,
    p["enable_move_line"] ? p["enable_move_line"].as<bool>() : false, act.arm_param.arm_id,
    p["control_mode"] ? p["control_mode"].as<int>() : 0,
    p["synchronization_mode"] ? p["synchronization_mode"].as<int>() : 0,
    act.arm_param.grasp_position_tolerance_m, act.arm_param.grasp_orientation_tolerance_rad};
  if(!com_interfaces::validateRigidGraspGoal(rigid_view,level_error)) throw std::invalid_argument(level_error);
  act.arm_param.use_config_pose = p["use_config_pose"].as<bool>();
  act.arm_param.use_force_control = p["use_force_control"].as<bool>();
    act.arm_param.enable_move_line = p["enable_move_line"] ?
      p["enable_move_line"].as<bool>() : false;
    act.arm_param.whole_body_waist_only = p["whole_body_waist_only"] ?
      p["whole_body_waist_only"].as<bool>() : false;
    if (act.arm_param.whole_body_waist_only &&
        (act.arm_param.arm_id < 5 || act.arm_param.arm_id > 7 ||
         act.arm_param.enable_move_line)) {
      throw std::invalid_argument(
        "whole_body_waist_only requires whole-body MoveP (arm_id=5/6/7, enable_move_line=false)");
    }
    act.arm_param.acceleration_scale = scale("acceleration_scale");
    act.arm_param.jerk_scale = scale("jerk_scale");
    act.arm_param.via_poses.clear();
    if (p["via_poses"]) {
      act.arm_param.via_poses = p["via_poses"].as<std::vector<std::vector<double>>>();
      if ((act.arm_param.arm_id > 1 && !com_interfaces::isRigidGraspLeaderGoal(act.arm_param)) || act.arm_param.enable_move_line ||
          act.arm_param.via_poses.size() > 64 || !act.arm_param.use_config_pose) {
        throw std::invalid_argument("via_poses requires configured single-arm or rigid leader MoveP, at most 64 poses");
      }
      for (const auto& pose : act.arm_param.via_poses) {
        if (pose.size() != 7) {throw std::invalid_argument("via_poses needs seven values per pose");}
        for (double v : pose) {if (!std::isfinite(v)) {throw std::invalid_argument("non-finite via pose");}}
        double norm = 0;
        for (std::size_t i = 3; i < 7; ++i) {norm += pose[i]*pose[i];}
        if (norm < 1e-12) {throw std::invalid_argument("zero via quaternion");}
      }
    }
  act.arm_param.speed = p["speed"] ? p["speed"].as<float>() : 0.5f;
  act.arm_param.hand_action_type = p["hand_action_type"] ? p["hand_action_type"].as<std::vector<uint8_t>>() : std::vector<uint8_t>{};
  act.arm_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  act.arm_param.frame_id = p["frame_id"] ? p["frame_id"].as<std::string>() : "";
  act.arm_param.vision_result_key = p["vision_result_key"] ? p["vision_result_key"].as<std::string>() : "";
  if (!act.arm_param.vision_result_key.empty() &&
      (act.arm_param.use_config_pose || act.arm_param.frame_id.empty())) {
    throw std::invalid_argument("vision_result_key requires vision pose and explicit frame_id");
  }

  //   com_interfaces/action/MovePose.action 定义：
  //   ARM_LEFT/ARM_RIGHT/HEAD/LEG → 使用 target_pose + bound_size
  //   ARM_BOTH → 使用 left_target_pose/left_bound_size + right_target_pose/right_bound_size + synchronization_mode
  if (act.arm_param.arm_id == 2 || act.arm_param.arm_id == 7) // ARM_BOTH
  {
    act.arm_param.left_target_pose = p["left_target_pose"].as<std::vector<float>>();
    act.arm_param.right_target_pose = p["right_target_pose"].as<std::vector<float>>();
    act.arm_param.left_bound_size = p["left_bound_size"].as<std::vector<float>>();
    act.arm_param.right_bound_size = p["right_bound_size"].as<std::vector<float>>();
    act.arm_param.synchronization_mode = p["synchronization_mode"].as<uint8_t>();
  }
  else
  {
    act.arm_param.config_pose = p["target_pose"].as<std::vector<float>>();
    act.arm_param.bound_size = p["bound_size"].as<std::vector<float>>();
  }
  const auto parse_offset = [&](const char* key, VisionPoseOffset& offset,
                                const std::vector<float>& bounds) {
    if (!p[key]) {return;}
    offset.enabled = true;
    const auto translation = p[key]["translation"];
    if (!translation.IsSequence() || translation.size() != 3U) {
      throw std::invalid_argument("vision translation needs three values");
    }
    offset.keep_current_tcp_height = translation[2].IsScalar() &&
      translation[2].Scalar() == "current_tcp_height";
    offset.translation = {
      translation[0].as<double>(), translation[1].as<double>(),
      offset.keep_current_tcp_height ? 0.0 : translation[2].as<double>()};
    offset.minimum_z = p[key]["minimum_z"] ? p[key]["minimum_z"].as<double>() : 0.0;
    offset.rotation = p[key]["rotation"].as<std::vector<double>>();
    offset.force_vertical_down = p[key]["force_vertical_down"] ?
      p[key]["force_vertical_down"].as<bool>() : false;
    offset.translation_frame = p[key]["translation_frame"] ?
      p[key]["translation_frame"].as<std::string>() : "base";
    if (act.arm_param.use_config_pose || act.arm_param.frame_id.empty() || offset.translation.size() != 3 ||
        offset.rotation.size() != 4) {
      throw std::invalid_argument(
        "vision offsets require vision pose, explicit frame_id, translation[3], rotation[4]");
    }
    if (offset.translation_frame != "base" && offset.translation_frame != "vision") {
      throw std::invalid_argument("translation_frame must be base or vision");
    }
    if (offset.force_vertical_down && act.arm_param.frame_id != "base_link") {
      throw std::invalid_argument("force_vertical_down requires frame_id=base_link");
    }
    if (offset.keep_current_tcp_height &&
        (std::string(key) != "vision_pose_offset" || act.arm_param.arm_id > 1 ||
         act.arm_param.frame_id != "base_link" || offset.translation_frame != "vision" ||
         !std::isfinite(offset.minimum_z) || offset.minimum_z < 0.0)) {
      throw std::invalid_argument(
        "current_tcp_height requires a single-arm base_link vision-local target and minimum_z >= 0");
    }
    if (!offset.keep_current_tcp_height && p[key]["minimum_z"]) {
      throw std::invalid_argument("minimum_z requires translation[2]=current_tcp_height");
    }
    for (double v : offset.translation) {
      if (!std::isfinite(v)) {throw std::invalid_argument("non-finite vision translation");}
    }
    double norm = 0.0;
    for (double v : offset.rotation) {
      if (!std::isfinite(v)) {throw std::invalid_argument("non-finite vision rotation");}
      norm += v * v;
    }
    if (!std::isfinite(norm) || norm < 1e-12) {
      throw std::invalid_argument("invalid vision offset quaternion");
    }
    // bound_size 在运控中还会叠加平移，禁止重复偏移。
    for (float v : bounds) {
      if (v != 0.0f) {throw std::invalid_argument("vision_pose_offset needs zero bound_size");}
    }
  };
  auto& move = act.arm_param;
  parse_offset("vision_pose_offset", move.vision_pose_offset, move.bound_size);
  parse_offset("left_vision_pose_offset", move.left_vision_pose_offset, move.left_bound_size);
  parse_offset("right_vision_pose_offset", move.right_vision_pose_offset, move.right_bound_size);
  const bool dual = move.arm_id == 2 || move.arm_id == 7;
  if (move.vision_pose_offset.enabled &&
      (move.arm_id != 0 && move.arm_id != 1 && move.arm_id != 5 && move.arm_id != 6)) {
    throw std::invalid_argument("vision_pose_offset requires arm_id=0/1/5/6");
  }
  if ((move.left_vision_pose_offset.enabled || move.right_vision_pose_offset.enabled) &&
      (!dual || !move.left_vision_pose_offset.enabled || !move.right_vision_pose_offset.enabled ||
       move.vision_pose_offset.enabled)) {
    throw std::invalid_argument("dual vision offsets require arm_id=2/7 and both left/right offsets only");
  }
  return true;
}

/// @brief 解析夹爪控制动作参数（gripper_ctrl）
bool TaskConfigParser::parse_gripper_ctrl_params(YAML::Node p, AtomicAction& act)
{
  // 夹爪控制参数
  act.gripper_param.gripper_id = p["gripper_id"].as<uint8_t>();
  act.gripper_param.com_port = p["comport"].as<std::vector<uint8_t>>();
  act.gripper_param.position = p["position"].as<std::vector<uint8_t>>();
  act.gripper_param.speed = p["speed"].as<std::vector<uint8_t>>();
  act.gripper_param.max_torque = p["max_torque"].as<std::vector<uint8_t>>();
  act.gripper_param.block = p["block"].as<bool>();
  act.gripper_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  return true;
}

/// @brief 解析视觉自检动作参数（vision_check_self）
bool TaskConfigParser::parse_vision_check_self_params(YAML::Node p, AtomicAction& act)
{
  act.vision_check_self_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  return true;
}

/// @brief 解析控制自检动作参数（control_check_self）
bool TaskConfigParser::parse_control_check_self_params(YAML::Node p, AtomicAction& act)
{
  act.control_check_self_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  // HardwareCheck.action 定义 com_port 为 int32[]，使用 int32_t 解析
  if (p["comport"].IsDefined() && !p["comport"].IsNull())
  {
    act.control_check_self_param.com_port = p["comport"].as<std::vector<int32_t>>();
  }
  return true;
}

/// @brief 解析控制复位动作参数（control_reset）
bool TaskConfigParser::parse_control_reset_params(YAML::Node p, AtomicAction& act)
{
  act.control_reset_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  return true;
}

/// @brief 解析视觉复位动作参数（vision_reset）
bool TaskConfigParser::parse_vision_reset_params(YAML::Node p, AtomicAction& act)
{
  act.vision_reset_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  return true;
}

/// @brief 解析关节空间移动动作参数（move_joints）
bool TaskConfigParser::parse_move_joints_params(YAML::Node p, AtomicAction& act)
{
  const auto scale = [&](const char* key) {
    const float value = p[key] ? p[key].as<float>() : 0.0f;
    if (!std::isfinite(value) || value < 0 || value > 1) {
      throw std::invalid_argument(std::string(key) + " must be in [0,1]");
    }
    return value;
  };

  // 关节空间移动参数
  act.move_joints_param.arm_id = p["arm_id"].as<uint8_t>();
  act.move_joints_param.precision_arrival = parse_precision_arrival(p, act.move_joints_param.arm_id, false);
  act.move_joints_param.keep_grasp_rigid = p["keep_grasp_rigid"] ? p["keep_grasp_rigid"].as<bool>() : false;
  act.move_joints_param.grasp_position_tolerance_m = p["grasp_position_tolerance_m"] ? p["grasp_position_tolerance_m"].as<double>() : .002;
  act.move_joints_param.grasp_orientation_tolerance_rad = p["grasp_orientation_tolerance_rad"] ? p["grasp_orientation_tolerance_rad"].as<double>() : .005;
  act.move_joints_param.enable_collision_check = parse_collision_mode(p);
  act.move_joints_param.acceleration_scale = scale("acceleration_scale");
  act.move_joints_param.jerk_scale = scale("jerk_scale");
  act.move_joints_param.speed = p["speed"].as<float>();
  act.move_joints_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  act.move_joints_param.relative_position = p["relative_position"] ? p["relative_position"].as<bool>() : false;
  if (p["leg_motion_mode"]) {
    const auto mode = p["leg_motion_mode"].as<std::string>();
    if (mode == "joint_targets" || mode == "0") act.move_joints_param.leg_motion_mode = 0;
    else if (mode == "body_tilt" || mode == "1") act.move_joints_param.leg_motion_mode = 1;
    else if (mode == "vertical_relative" || mode == "2") act.move_joints_param.leg_motion_mode = 2;
    else if (mode == "vertical_absolute" || mode == "3") act.move_joints_param.leg_motion_mode = 3;
    else throw std::invalid_argument("Unknown leg_motion_mode");
  }
  act.move_joints_param.body_tilt_deg = p["body_tilt_deg"] ? p["body_tilt_deg"].as<double>() : 0;
  act.move_joints_param.body_height_m = p["body_height_m"] ? p["body_height_m"].as<double>() : 0;

  // com_interfaces/action/MoveJoints.action 定义：
  //   ARM_LEFT/ARM_RIGHT/HEAD/LEG → 使用 joint_name + position
  //   ARM_BOTH → 使用 left_joint_name/left_position + right_joint_name/right_position + synchronization_mode
  if (act.move_joints_param.arm_id == 2) // ARM_BOTH
  {
    act.move_joints_param.left_joint_name = p["left_joint_name"].as<std::vector<std::string>>();
    act.move_joints_param.left_position = p["left_position"].as<std::vector<double>>();
    act.move_joints_param.right_joint_name = p["right_joint_name"].as<std::vector<std::string>>();
    act.move_joints_param.right_position = p["right_position"].as<std::vector<double>>();
    act.move_joints_param.synchronization_mode = p["synchronization_mode"].as<uint8_t>();
  }
  else
  {
    if (act.move_joints_param.leg_motion_mode == 0 || p["joint_name"])
      act.move_joints_param.joint_name = p["joint_name"].as<std::vector<std::string>>();
    if (act.move_joints_param.leg_motion_mode == 0 || p["position"])
      act.move_joints_param.position = p["position"].as<std::vector<double>>();
  }
  const auto& move = act.move_joints_param;
  std::string body_error;
  if (!com_interfaces::validateRigidLegGraspRequest(move.keep_grasp_rigid,
      move.grasp_position_tolerance_m,move.grasp_orientation_tolerance_rad,move.arm_id,
      p["control_mode"] ? p["control_mode"].as<uint8_t>() : 0,
      p["synchronization_mode"] ? p["synchronization_mode"].as<uint8_t>() : 0,
      p["keep_tcp_level"] ? p["keep_tcp_level"].as<bool>() : false,body_error))
    throw std::invalid_argument(body_error);
  if (move.keep_grasp_rigid && (!std::isfinite(move.speed)||move.speed<=0||move.speed>1||move.timeout_ms<=0||
      move.acceleration_scale!=0||move.jerk_scale!=0||
      !com_interfaces::validateRigidLegTargets(move.joint_name,move.position,move.leg_motion_mode,body_error)))
    throw std::invalid_argument("Invalid rigid LEG MoveJ speed, timeout, extra scales or targets: "+body_error);
  if (!com_interfaces::validateLegBodyMotionGoal(move.arm_id, move.leg_motion_mode,
      move.body_tilt_deg, move.body_height_m, move.joint_name, move.position, move.speed,
      move.timeout_ms, p["control_mode"] ? p["control_mode"].as<uint8_t>() : 0,
      move.acceleration_scale, move.jerk_scale, move.relative_position, body_error))
    throw std::invalid_argument(body_error);
  const bool waist_only_goal = move.arm_id == 4 &&
    move.joint_name == std::vector<std::string>{"body_joint"};
  if ((move.relative_position || waist_only_goal) &&
      (move.arm_id != 4 || move.joint_name != std::vector<std::string>{"body_joint"} ||
       move.position.size() != 1 || !std::isfinite(move.position[0]) ||
       !std::isfinite(move.speed) || move.speed <= 0 || move.speed > 1 || move.timeout_ms <= 0)) {
    throw std::invalid_argument("waist-only/relative MoveJ requires LEG/body_joint with one finite target, valid speed and timeout");
  }
  return true;
}

/// @brief 解析吸盘控制动作参数（control_suck）
bool TaskConfigParser::parse_control_suck_params(YAML::Node p, AtomicAction& act)
{
  // 吸盘控制参数
  act.suck_param.suck_id = p["suck_id"].as<uint8_t>();
  act.suck_param.active = p["active"].as<std::vector<bool>>();
  act.suck_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  return true;
}

/// @brief 解析视觉标定控制动作参数（calibration_control）
bool TaskConfigParser::parse_calibration_control_params(YAML::Node p, AtomicAction& act)
{
  // 视觉标定控制参数（对应 com_interfaces/action/CalibrationControl.action）
  act.calibration_param.camera_id = p["camera_id"] ? p["camera_id"].as<uint8_t>() : 0;
  act.calibration_param.arm_id = p["arm_id"] ? p["arm_id"].as<uint8_t>() : 0;
  act.calibration_param.command = p["command"] ? p["command"].as<std::string>() : "";
  act.calibration_param.auto_execute = p["auto_execute"] ? p["auto_execute"].as<bool>() : false;
  act.calibration_param.mode = p["mode"] ? p["mode"].as<std::string>() : "TO_HAND";
  act.calibration_param.timeout_ms = p["timeout_ms"] ? p["timeout_ms"].as<int64_t>() : 60000;
  return true;
}

/// @brief 解析自动标定任务动作参数（auto_calibrate）
bool TaskConfigParser::parse_auto_calibrate_params(YAML::Node p, AtomicAction& act)
{
  // 自动标定任务参数（复合动作，由 task_action_client 后台编排执行）
  act.auto_calibrate_param.camera_id = p["camera_id"] ? p["camera_id"].as<uint8_t>() : 0;
  act.auto_calibrate_param.arm_id = p["arm_id"] ? p["arm_id"].as<uint8_t>() : 1;   // 默认右臂
  act.auto_calibrate_param.mode = p["mode"] ? p["mode"].as<std::string>() : "TO_HAND";
  act.auto_calibrate_param.sample_num = p["sample_num"] ? p["sample_num"].as<int32_t>() : 12;
  act.auto_calibrate_param.joint_name =
    p["joint_name"] ? p["joint_name"].as<std::vector<std::string>>() : std::vector<std::string>();
  // 自动标定流程下默认关闭自碰撞检测
  act.auto_calibrate_param.enable_collision_check =
    parse_collision_mode(p);
  act.auto_calibrate_param.speed = p["speed"] ? p["speed"].as<float>() : 0.2f;

  // —— 头部初始到位（可选）：自动标定开始时先将头部 move_joints 到指定关节角 ——
  act.auto_calibrate_param.head_move_enable =
    p["head_move_enable"] ? p["head_move_enable"].as<bool>() : false;
  act.auto_calibrate_param.head_joint_name =
    p["head_joint_name"] ? p["head_joint_name"].as<std::vector<std::string>>() : std::vector<std::string>();
  act.auto_calibrate_param.head_position =
    p["head_position"] ? p["head_position"].as<std::vector<float>>() : std::vector<float>();
  act.auto_calibrate_param.head_speed =
    p["head_speed"] ? p["head_speed"].as<float>() : 0.2f;
  act.auto_calibrate_param.head_move_timeout_ms =
    p["head_move_timeout_ms"] ? p["head_move_timeout_ms"].as<int64_t>() : 15000;

  // —— move_joints 到位后到触发采集前的稳定等待（可选，默认 0 不等待）——
  act.auto_calibrate_param.post_move_delay_ms =
    p["post_move_delay_ms"] ? p["post_move_delay_ms"].as<int64_t>() : 0;

  act.auto_calibrate_param.move_timeout_ms =
    p["move_timeout_ms"] ? p["move_timeout_ms"].as<int64_t>() : 15000;
  act.auto_calibrate_param.capture_timeout_ms =
    p["capture_timeout_ms"] ? p["capture_timeout_ms"].as<int64_t>() : 60000;
  act.auto_calibrate_param.timeout_ms =
    p["timeout_ms"] ? p["timeout_ms"].as<int64_t>() : 600000;

  // 解析目标关节角池：pose_pool 为浮点数组序列（每组 7 个关节角，单位 rad）
  auto pp = p["pose_pool"];
  if (pp.IsDefined() && !pp.IsNull() && pp.IsSequence())
  {
    for (auto row : pp)
    {
      act.auto_calibrate_param.pose_pool.push_back(row.as<std::vector<float>>());
    }
  }
  LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
    "auto_calibrate: arm_id=" << static_cast<int>(act.auto_calibrate_param.arm_id)
    << ", mode=" << act.auto_calibrate_param.mode
    << ", sample_num=" << act.auto_calibrate_param.sample_num
    << ", joint_name_size=" << act.auto_calibrate_param.joint_name.size()
    << ", enable_collision_check=" << static_cast<unsigned>(act.auto_calibrate_param.enable_collision_check)
    << ", head_move_enable=" << (act.auto_calibrate_param.head_move_enable ? "true" : "false")
    << ", head_position_size=" << act.auto_calibrate_param.head_position.size()
    << ", post_move_delay_ms=" << act.auto_calibrate_param.post_move_delay_ms
    << ", pose_pool_size=" << act.auto_calibrate_param.pose_pool.size());
  return true;
}

/// @brief 解析复合动作参数（move_compound）
/// 遍历 sub_actions，每个子动作的 params 字段结构与普通同名单体动作完全相同，
/// 复用 parse_move_pose_params / parse_move_joints_params 解析（以临时 AtomicAction 为载体），
/// 保证子动作参数控制与普通 move_pose / move_joints 完全一致。
bool TaskConfigParser::parse_compound_move_params(YAML::Node p, AtomicAction& act)
{
  // 复合整体超时（可选，默认 30000ms）
  act.compound_move_param.timeout_ms =
    p["timeout_ms"] ? p["timeout_ms"].as<int64_t>() : 30000;

  auto subs = p["sub_actions"];
  if (!subs.IsDefined() || subs.IsNull() || !subs.IsSequence() || subs.size() == 0)
  {
    LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
      "move_compound requires non-empty sub_actions list");
    return false;
  }

  int idx = 0;
  for (auto snode : subs)
  {
    if (!snode.IsDefined() || snode.IsNull() || !snode["action_type"])
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "move_compound sub_actions[" << idx << "] missing action_type");
      return false;
    }
    const std::string sub_type = snode["action_type"].as<std::string>();
    auto sp = snode["params"];
    if (!sp.IsDefined() || sp.IsNull())
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "move_compound sub_actions[" << idx << "] (" << sub_type << ") missing params");
      return false;
    }

    CompoundSubAction sub;
    sub.action_type = sub_type;
    if (sub_type == "move_pose")
    {
      // 复用普通 move_pose 参数解析（参数控制与普通 move_pose 一致，含 ARM_BOTH）
      AtomicAction tmp;
      if (!parse_move_pose_params(sp, tmp))
      {
        LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
          "Failed to parse move_compound sub move_pose at index " << idx);
        return false;
      }
      sub.move_pose_param = tmp.arm_param;
    }
    else if (sub_type == "move_joints")
    {
      // 复用普通 move_joints 参数解析（参数控制与普通 move_joints 一致）
      AtomicAction tmp;
      if (!parse_move_joints_params(sp, tmp))
      {
        LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
          "Failed to parse move_compound sub move_joints at index " << idx);
        return false;
      }
      sub.move_joints_param = tmp.move_joints_param;
    }
    else
    {
      LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
        "move_compound unsupported sub action_type: " << sub_type
        << " (supported: move_pose / move_joints)");
      return false;
    }
    act.compound_move_param.sub_actions.push_back(sub);
    ++idx;
  }

  const auto& actions = act.compound_move_param.sub_actions;
  const bool has_leader = std::any_of(actions.begin(), actions.end(), [](const auto& sub) {
    return sub.action_type == "move_pose" && com_interfaces::isRigidGraspLeaderGoal(sub.move_pose_param);
  });
  if (has_leader && std::count_if(actions.begin(), actions.end(), [](const auto& sub) {
        const auto id = sub.action_type == "move_pose" ? sub.move_pose_param.arm_id : sub.move_joints_param.arm_id;
        return id != com_interfaces::action::MovePose::Goal::HEAD;
      }) > 1) {
    throw std::invalid_argument("rigid leader MoveP owns both arms and the leg/waist resource in move_compound");
  }
  LOG_INFO(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
    "move_compound: sub_actions=" << act.compound_move_param.sub_actions.size()
    << ", timeout_ms=" << act.compound_move_param.timeout_ms);
  return true;
}

/// @brief 解析前置条件节点（preconditions）
bool TaskConfigParser::parse_preconditions(YAML::Node pre_node, PreconditionList& pre)
{
  pre.has = true;
  pre.logical_op = pre_op_from_str(pre_node["logical_op"] ? pre_node["logical_op"].as<std::string>() : "and");
  pre.policy = pre_policy_from_str(pre_node["policy"] ? pre_node["policy"].as<std::string>() : "wait");
  pre.max_wait_ms = pre_node["max_wait_ms"] ? pre_node["max_wait_ms"].as<int64_t>() : 0;
  pre.check_interval_ms = pre_node["check_interval_ms"] ? pre_node["check_interval_ms"].as<int64_t>() : 100;

  auto conds = pre_node["conditions"];
  if (conds.IsDefined() && !conds.IsNull() && conds.IsSequence())
  {
    for (auto c : conds)
    {
      PreconditionParam cond;
      if (parse_precondition(c, cond))
      {
        pre.conditions.push_back(std::move(cond));
      }
      else
      {
        return false;
      }
    }
  }

  LOG_INFO(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
    "Preconditions parsed: logical_op=" << (pre.logical_op == PreLogicalOp::OR ? "or" : "and")
    << ", policy=" << (pre.policy == PrePolicy::FAIL_FAST ? "fail_fast" : (pre.policy == PrePolicy::SKIP ? "skip" : "wait"))
    << ", max_wait_ms=" << pre.max_wait_ms
    << ", check_interval_ms=" << pre.check_interval_ms
    << ", conditions=" << pre.conditions.size());
  return true;
}

/// @brief 解析单个前置条件
bool TaskConfigParser::parse_precondition(YAML::Node cond_node, PreconditionParam& cond)
{
  std::string type_str = cond_node["type"] ? cond_node["type"].as<std::string>() : "";
  cond.type = pre_type_from_str(type_str);
  if (cond.type == PreconditionType::UNKNOWN)
  {
    LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
      "Unknown precondition type: '" << type_str << "'");
    return false;
  }
  cond.enable = cond_node["enable"] ? cond_node["enable"].as<bool>() : true;
  cond.on_timeout = pre_timeout_from_str(cond_node["on_timeout"] ? cond_node["on_timeout"].as<std::string>() : "fail");

  switch (cond.type)
  {
    case PreconditionType::BATTERY_LEVEL:
      // 数值类条件
      cond.field = cond_node["field"] ? cond_node["field"].as<std::string>() : "percentage";
      cond.operator_str = cond_node["operator"] ? cond_node["operator"].as<std::string>() : ">=";
      cond.threshold = cond_node["threshold"] ? cond_node["threshold"].as<double>() : 0.0;
      break;

    case PreconditionType::ROBOT_STATUS:
    case PreconditionType::SAFETY_STATUS:
    {
      // 状态集合类条件：expected 支持数值或枚举名
      auto exp = cond_node["expected"];
      if (exp.IsDefined() && !exp.IsNull())
      {
        if (exp.IsSequence())
        {
          for (auto e : exp)
          {
            int32_t v = status_value_from_node(e);
            if (v >= 0) { cond.expected.push_back(v); }
          }
        }
        else
        {
          int32_t v = status_value_from_node(exp);
          if (v >= 0) { cond.expected.push_back(v); }
        }
      }
      break;
    }

    case PreconditionType::ARM_IDLE:
    case PreconditionType::GRIPPER_IDLE:
    case PreconditionType::SUCK_IDLE:
    case PreconditionType::DEXTEROUS_HAND_IDLE:
      // 部件空闲类条件（分左右）
      cond.part = pre_part_from_str(cond_node["part"] ? cond_node["part"].as<std::string>() : "both");
      cond.expected_bool = cond_node["expected"] ? cond_node["expected"].as<bool>() : true;
      break;

    case PreconditionType::HEAD_IDLE:
    case PreconditionType::WAIST_IDLE:
      // 部件空闲类条件（单部件）
      cond.expected_bool = cond_node["expected"] ? cond_node["expected"].as<bool>() : true;
      break;

    default:
      break;
  }

  LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
    "Precondition parsed: type=" << type_str << ", enable=" << (cond.enable ? "true" : "false")
    << ", on_timeout=" << static_cast<int>(cond.on_timeout)
    << ", conditions_num=" << cond.expected.size());
  return true;
}

/// @brief 解析结果分支节点（on_success / on_failure）
bool TaskConfigParser::parse_result_branch(YAML::Node branch_node, ResultBranchParam& branch, int depth)
{
  branch.has = true;
  std::string policy_str = branch_node["branch_policy"] ? branch_node["branch_policy"].as<std::string>() : "";
  if (policy_str == "continue")        { branch.policy = ResultPolicy::BRANCH_CONTINUE; }
  else if (policy_str == "override")   { branch.policy = ResultPolicy::BRANCH_OVERRIDE; }
  else if (policy_str == "fail")       { branch.policy = ResultPolicy::BRANCH_FAIL; }
  else if (policy_str == "recover")    { branch.policy = ResultPolicy::BRANCH_RECOVER; }
  else
  {
    LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
      "Unknown branch_policy: '" << policy_str << "'");
    return false;
  }

  auto acts = branch_node["actions"];
  if (acts.IsDefined() && !acts.IsNull() && acts.IsSequence())
  {
    for (auto a : acts)
    {
      auto branch_act = std::make_shared<AtomicAction>();
      if (parse_action(a, *branch_act, depth + 1))
      {
        ResultActionParam rap;
        rap.action = branch_act;
        branch.actions.push_back(std::move(rap));
      }
      else
      {
        LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
          "Failed to parse action in result branch (policy=" << policy_str << ")");
        return false;
      }
    }
  }

  LOG_INFO(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
    "Result branch parsed: policy=" << policy_str << ", actions=" << branch.actions.size());
  return true;
}

/// @brief 解析单个任务节点
/// @param task_node YAML 任务节点
/// @param item [输出] 解析后的任务映射表项
/// @return true 解析成功
bool TaskConfigParser::parse_task(YAML::Node task_node, TaskMapItem& item)
{
  auto seqnum = task_node["action_sequencenum"].as<uint32_t>();
  LOG_INFO_FLOW(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
    "Parsing task with " << seqnum << " sequences");

  // 解析每个动作序列
  for (uint32_t i = 0; i < seqnum; i++)
  {
    auto node_name = "action_sequence" + std::to_string(i+1);
    auto seq = task_node[node_name];
    LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "Parsing sequence: " << node_name);

    std::vector<AtomicAction> action_list;
    for(auto a : seq)
    {
      AtomicAction act{};
      const auto parallel = a["parallel_actions"];
      if (parallel.IsDefined())
      {
        if (!parallel.IsSequence() || parallel.size() < 2U)
        {
          LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
            node_name << " parallel_actions must contain at least two actions");
          return false;
        }

        act.action_type = "parallel_actions";
        act.is_parallel_group = true;
        for (const auto& parallel_action_node : parallel)
        {
          if (parallel_action_node["parallel_actions"].IsDefined())
          {
            LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
              node_name << " nested parallel_actions are not supported");
            return false;
          }

          AtomicAction parallel_action{};
          if (!parse_action(parallel_action_node, parallel_action))
          {
            return false;
          }
          act.parallel_actions.push_back(std::move(parallel_action));
        }
        LOG_INFO(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
          "Parsed local parallel group with " << act.parallel_actions.size()
          << " actions in " << node_name);
      }
      else if (!parse_action(a, act))
      {
        return false;
      }
      action_list.push_back(act);
    }
    item.action_seq.push_back(action_list);
  }

  LOG_INFO(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
    "Task parsed, sequences: " << item.action_seq.size());
  return true;
}

/// @brief 解析启动任务节点（自检 + 复位）
/// @param boot_node YAML 启动任务节点
/// @param boot [输出] 解析后的启动任务集
/// @return true 解析成功
bool TaskConfigParser::parse_boot(YAML::Node boot_node, BootTaskSet& boot)
{
  // 解析自检序列
  auto sc_seq = boot_node["self_check"]["action_sequence"];
  for(auto a : sc_seq)
  {
    AtomicAction act;
    parse_action(a, act);
    boot.self_check_seq.push_back(act);
  }

  // 解析复位序列
  auto rst_seq = boot_node["reset"]["action_sequence"];
  for(auto a : rst_seq)
  {
    AtomicAction act;
    parse_action(a, act);
    boot.reset_seq.push_back(act);
  }

  LOG_INFO(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
    "Boot tasks parsed: self_check=" << boot.self_check_seq.size() << ", reset=" << boot.reset_seq.size());
  return true;
}
}
