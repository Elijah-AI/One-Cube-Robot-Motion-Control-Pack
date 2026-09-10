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
#include "task_manager/log_helper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <cmath>
#include <stdexcept>

namespace task_cfg
{

// 配置解析器统一日志器名称
static const char* kConfigLogger = "task_config_parser";

/// @brief 加载并解析 YAML 配置文件
/// @param file_path 配置文件路径
/// @param out_cfg [输出] 解析后的全局配置
/// @return true 解析成功，false 解析失败
bool TaskConfigParser::load_config(const std::string& file_path, GlobalConfig& out_cfg)
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

    // 解析任务列表
    auto tasks_node = root["tasks"];
    for(auto it = tasks_node.begin(); it != tasks_node.end(); ++it)
    {
      auto name = it->first.as<std::string>();
      LOG_INFO_FLOW(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "Parsing task: " << name);

      TaskMapItem item;
      item.trigger_reg = it->second["trigger_reg"].as<uint32_t>();
      item.status_reg = it->second["status_reg"].as<uint32_t>();

      if(!parse_task(it->second, item))
      {
        LOG_ERROR(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
          "Failed to parse task: " << name);
        return false;
      }
      out_cfg.task_dict[it->first.as<std::string>()] = item;
    }

    // 解析启动任务
    parse_boot(root["boot_task"], out_cfg.boot_tasks);

    LOG_SUCCESS(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
      "Config loaded successfully, tasks: " << out_cfg.task_dict.size());
    return true;

  }catch(std::exception& e){
    LOG_ERROR_CRITICAL(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "YAML parse error: " << e.what());
    return false;
  }
}

/// @brief 解析单个动作节点
/// @param action_node YAML 动作节点
/// @param act [输出] 解析后的原子动作
/// @return true 解析成功
bool TaskConfigParser::parse_action(YAML::Node action_node, AtomicAction& act)
{
  act.action_type = action_node["action_type"].as<std::string>();
  auto p = action_node["params"];
  auto c = action_node["condition"];

  const auto scale = [&](const char* key) {
    const float value = p[key] ? p[key].as<float>() : 0.0f;
    if (!std::isfinite(value) || value < 0 || value > 1) {
      throw std::invalid_argument(std::string(key) + " must be in [0,1]");
    }
    return value;
  };
  // 根据动作类型分别解析对应参数
  if(act.action_type == "vision_detect")
  {
    // 视觉检测参数
    act.vision_param.target_obj_name = p["target_obj_name"].as<std::string>();
    act.vision_param.expected_frame_id = p["expected_frame_id"].as<std::string>();
    act.vision_param.need_6d_pose = p["need_6d_pose"].as<bool>();
    act.vision_param.need_env_point_cloud = p["need_env_point_cloud"].as<bool>();
    act.vision_param.downsample_res = p["downsample_res"].as<float>();
    act.vision_param.priority = p["priority"].as<int32_t>();

    // 解析执行条件（可选）
    if (c.IsDefined() && !c.IsNull())
    {
      LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "Parsing condition for: " << act.action_type);
      act.action_execut_condition.left_arm_state = c["left_arm_state"].as<std::string>();
      act.action_execut_condition.power_level = c["power_level"].as<std::string>();
      act.action_execut_condition.right_arm_state = c["right_arm_state"].as<std::string>();
      act.action_execut_condition.body_waist_state = c["body_waist_state"].as<std::string>();
      act.action_execut_condition.vision_state = c["Vision_state"].as<std::string>();
    } else {
      LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "No condition defined for: " << act.action_type);
    }
  }
  else if(act.action_type == "move_pose")
  {
    // 机械臂位姿移动参数（对应 MovePose.action）
    act.arm_param.arm_id = p["arm_id"].as<uint8_t>();
    act.arm_param.enable_collision_check = p["enable_collision_check"].as<bool>();
    act.arm_param.use_config_pose = p["use_config_pose"].as<bool>();
    act.arm_param.use_force_control = p["use_force_control"].as<bool>();
    act.arm_param.enable_move_line = p["enable_move_line"] ?
      p["enable_move_line"].as<bool>() : false;
    act.arm_param.acceleration_scale = scale("acceleration_scale");
    act.arm_param.jerk_scale = scale("jerk_scale");
    act.arm_param.via_poses.clear();
    if (p["via_poses"]) {
      act.arm_param.via_poses = p["via_poses"].as<std::vector<std::vector<double>>>();
      if (act.arm_param.arm_id > 1 || act.arm_param.enable_move_line ||
          act.arm_param.via_poses.size() > 64 || !act.arm_param.use_config_pose) {
        throw std::invalid_argument("via_poses requires single-arm configured MoveP, at most 64 poses");
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
    act.arm_param.hand_action_type = p["hand_action_type"] ?
      p["hand_action_type"].as<std::vector<uint8_t>>() : std::vector<uint8_t>{};
    act.arm_param.timeout_ms = p["timeout_ms"].as<int64_t>();
    act.arm_param.frame_id = p["frame_id"] ? p["frame_id"].as<std::string>() : "";

    // com_interfaces/action/MovePose.action 定义：
    //   ARM_LEFT/ARM_RIGHT/HEAD/LEG → 使用 target_pose + bound_size
    //   ARM_BOTH → 使用 left_target_pose/left_bound_size + right_target_pose/right_bound_size + synchronization_mode
    if (act.arm_param.arm_id == 2) // ARM_BOTH
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

    // 解析执行条件（可选）
    if (c.IsDefined() && !c.IsNull())
    {
      LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "Parsing condition for: " << act.action_type);
      act.action_execut_condition.left_arm_state = c["left_arm_state"].as<std::string>();
      act.action_execut_condition.power_level = c["power_level"].as<std::string>();
      act.action_execut_condition.right_arm_state = c["right_arm_state"].as<std::string>();
      act.action_execut_condition.body_waist_state = c["body_waist_state"].as<std::string>();
      act.action_execut_condition.vision_state = c["Vision_state"].as<std::string>();
    } else {
      LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "No condition defined for: " << act.action_type);
    }
  }
  else if(act.action_type == "gripper_ctrl")
  {
    // 夹爪控制参数
    act.gripper_param.gripper_id = p["gripper_id"].as<uint8_t>();
    act.gripper_param.com_port = p["comport"].as<std::vector<uint8_t>>();
    act.gripper_param.position = p["position"].as<std::vector<uint8_t>>();
    act.gripper_param.speed = p["speed"].as<std::vector<uint8_t>>();
    act.gripper_param.max_torque = p["max_torque"].as<std::vector<uint8_t>>();
    act.gripper_param.block = p["block"].as<bool>();
    act.gripper_param.timeout_ms = p["timeout_ms"].as<int64_t>();

    // 解析执行条件（可选）
    if (c.IsDefined() && !c.IsNull())
    {
      LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "Parsing condition for: " << act.action_type);
      act.action_execut_condition.left_arm_state = c["left_arm_state"].as<std::string>();
      act.action_execut_condition.power_level = c["power_level"].as<std::string>();
      act.action_execut_condition.right_arm_state = c["right_arm_state"].as<std::string>();
      act.action_execut_condition.body_waist_state = c["body_waist_state"].as<std::string>();
      act.action_execut_condition.vision_state = c["Vision_state"].as<std::string>();
    } else {
      LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "No condition defined for: " << act.action_type);
    }
  }
  else if (act.action_type == "vision_check_self")
  {
    act.vision_check_self_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  }
  else if (act.action_type == "control_check_self")
  {
    act.control_check_self_param.timeout_ms = p["timeout_ms"].as<int64_t>();
    // HardwareCheck.action 定义 com_port 为 int32[]，使用 int32_t 解析
    if (p["comport"].IsDefined() && !p["comport"].IsNull())
    {
      act.control_check_self_param.com_port = p["comport"].as<std::vector<int32_t>>();
    }
  }
  else if (act.action_type == "control_reset")
  {
    act.control_reset_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  }
  else if (act.action_type == "vision_reset")
  {
    act.vision_reset_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  }
  else if (act.action_type == "move_joints")
  {
    // 关节空间移动参数
    act.move_joints_param.arm_id = p["arm_id"].as<uint8_t>();
    act.move_joints_param.enable_collision_check = p["enable_collision_check"].as<bool>();
    act.move_joints_param.acceleration_scale = scale("acceleration_scale");
    act.move_joints_param.jerk_scale = scale("jerk_scale");
    act.move_joints_param.speed = p["speed"].as<float>();
    act.move_joints_param.timeout_ms = p["timeout_ms"].as<int64_t>();

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
      act.move_joints_param.joint_name = p["joint_name"].as<std::vector<std::string>>();
      act.move_joints_param.position = p["position"].as<std::vector<double>>();
    }

    // 解析执行条件（可选）
    if (c.IsDefined() && !c.IsNull())
    {
      LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "Parsing condition for: " << act.action_type);
      act.action_execut_condition.left_arm_state = c["left_arm_state"].as<std::string>();
      act.action_execut_condition.power_level = c["power_level"].as<std::string>();
      act.action_execut_condition.right_arm_state = c["right_arm_state"].as<std::string>();
      act.action_execut_condition.body_waist_state = c["body_waist_state"].as<std::string>();
      act.action_execut_condition.vision_state = c["Vision_state"].as<std::string>();
    } else {
      LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "No condition defined for: " << act.action_type);
    }
  }
  else if (act.action_type == "control_suck")
  {
    // 吸盘控制参数
    act.suck_param.suck_id = p["suck_id"].as<uint8_t>();
    act.suck_param.active = p["active"].as<std::vector<bool>>();
    act.suck_param.timeout_ms = p["timeout_ms"].as<int64_t>();

    // 解析执行条件（可选）
    if (c.IsDefined() && !c.IsNull())
    {
      LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "Parsing condition for: " << act.action_type);
      act.action_execut_condition.left_arm_state = c["left_arm_state"].as<std::string>();
      act.action_execut_condition.power_level = c["power_level"].as<std::string>();
      act.action_execut_condition.right_arm_state = c["right_arm_state"].as<std::string>();
      act.action_execut_condition.body_waist_state = c["body_waist_state"].as<std::string>();
      act.action_execut_condition.vision_state = c["Vision_state"].as<std::string>();
    } else {
      LOG_DEBUG(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG, "No condition defined for: " << act.action_type);
    }
  }
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
    AtomicAction act{};
    parse_action(a, act);
    boot.self_check_seq.push_back(act);
  }

  // 解析复位序列
  auto rst_seq = boot_node["reset"]["action_sequence"];
  for(auto a : rst_seq)
  {
    AtomicAction act{};
    parse_action(a, act);
    boot.reset_seq.push_back(act);
  }

  LOG_INFO(rclcpp::get_logger(kConfigLogger), task_log::Tag::CONFIG,
    "Boot tasks parsed: self_check=" << boot.self_check_seq.size() << ", reset=" << boot.reset_seq.size());
  return true;
}
}
