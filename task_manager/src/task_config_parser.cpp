#include "task_manager/task_config_parser.hpp"
#include <rclcpp/rclcpp.hpp>

namespace task_cfg
{
bool TaskConfigParser::load_config(const std::string& file_path, GlobalConfig& out_cfg)
{
  try{
    YAML::Node root = YAML::LoadFile(file_path);

    // 第一步判断文件是否为空 / 根节点无效
    if(!root.IsMap())
    {
      RCLCPP_ERROR(rclcpp::get_logger("config_parser"), "yaml file empty or invalid map");
      return false;
    }

    out_cfg.node_id = root["node_id"].as<int32_t>();
    RCLCPP_ERROR(rclcpp::get_logger("load_config"), "out_cfg.node_id: %d", out_cfg.node_id);
    auto mb_node = root["modbus_slave"];
    out_cfg.modbus_listen_ip = mb_node["listen_ip"].as<std::string>();
    out_cfg.modbus_port = mb_node["listen_port"].as<uint16_t>();
    auto reg_map = root["register_map"];
    out_cfg.task_trigger_base = reg_map["task_trigger_base_addr"].as<uint32_t>();
    out_cfg.task_status_base = reg_map["task_status_base_addr"].as<uint32_t>();

    auto tasks_node = root["tasks"];

    for(auto it = tasks_node.begin(); it != tasks_node.end(); ++it)
    {
      TaskMapItem item;
      if(parse_task(it->second, item))
      {
        out_cfg.task_dict[it->first.as<std::string>()] = item;
      }
    }

    parse_boot(root["boot_task"], out_cfg.boot_tasks);
    return true;

  }catch(std::exception& e){

    RCLCPP_ERROR(rclcpp::get_logger("config_parser"), "yaml parse err: %s", e.what());
    return false;
    
  }
}

bool TaskConfigParser::parse_action(YAML::Node action_node, AtomicAction& act)
{
  act.action_type = action_node["action_type"].as<std::string>();
  auto p = action_node["params"];
  if(act.action_type == "vision_detect")
  {
    act.vision_param.target_obj_name = p["target_obj_name"].as<std::string>();
    act.vision_param.expected_frame_id = p["expected_frame_id"].as<std::string>();
    act.vision_param.need_6d_pose = p["need_6d_pose"].as<bool>();
    act.vision_param.need_env_point_cloud = p["need_env_point_cloud"].as<bool>();
    act.vision_param.downsample_res = p["downsample_res"].as<float>();
    act.vision_param.priority = p["priority"].as<int32_t>();
  }
  else if(act.action_type == "move_arm")
  {
    act.arm_param.arm_id = p["arm_id"].as<std::string>();
    act.arm_param.use_planning = p["use_planning"].as<bool>();
    act.arm_param.collision_check = p["collision_check"].as<bool>();
    act.arm_param.safety_distance = p["safety_distance"].as<std::vector<float>>();
    act.arm_param.avoid_obstacle =  p["avoid_obstacle"].as<bool>();
    act.arm_param.use_config_pose = p["use_config_pose"].as<bool>();
    act.arm_param.config_pose = p["target_pose"].as<std::vector<float>>();

  }
  else if(act.action_type == "gripper_ctrl")
  {
    act.gripper_param.gripper_id = p["gripper_id"].as<uint8_t>();
    act.gripper_param.com_port = p["comport"].as<std::vector<uint8_t>>();
    act.gripper_param.position = p["position"].as<std::vector<uint8_t>>();
    RCLCPP_INFO(rclcpp::get_logger("act.gripper_param.position size"), "%d", act.gripper_param.position.size());
    act.gripper_param.speed = p["speed"].as<std::vector<uint8_t>>();
    act.gripper_param.max_torque = p["max_torque"].as<std::vector<uint8_t>>();
    act.gripper_param.block = p["block"].as<bool>();
    act.gripper_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  }
  else if (act.action_type == "vision_check_self")
  {
    act.vision_check_self_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  }
  else if (act.action_type == "control_check_self")
  {
    act.control_check_self_param.timeout_ms = p["timeout_ms"].as<int64_t>();
    act.control_check_self_param.com_port = p["comport"].as<std::vector<uint8_t>>();
  }
  else if (act.action_type == "control_reset")
  {
    act.control_reset_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  }
  else if (act.action_type == "vision_reset")
  {
    act.vision_reset_param.timeout_ms = p["timeout_ms"].as<int64_t>();
  }
  return true;
}

bool TaskConfigParser::parse_task(YAML::Node task_node, TaskMapItem& item)
{
  item.trigger_reg = task_node["trigger_reg"].as<uint32_t>();
  item.status_reg = task_node["status_reg"].as<uint32_t>();
  auto seq = task_node["action_sequence"];
  for(auto a : seq)
  {
    AtomicAction act;
    RCLCPP_INFO(rclcpp::get_logger("parse_task"), "parse_action");
    parse_action(a, act);
    item.action_seq.push_back(act);
  }
  return true;
}

bool TaskConfigParser::parse_boot(YAML::Node boot_node, BootTaskSet& boot)
{
  auto sc_seq = boot_node["self_check"]["action_sequence"];
  for(auto a : sc_seq)
  {
    AtomicAction act;
    parse_action(a, act);
    boot.self_check_seq.push_back(act);
  }
  auto rst_seq = boot_node["reset"]["action_sequence"];
  for(auto a : rst_seq)
  {
    AtomicAction act;
    parse_action(a, act);
    boot.reset_seq.push_back(act);
  }
  return true;
}
}