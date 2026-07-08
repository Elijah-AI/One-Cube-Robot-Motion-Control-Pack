#ifndef TASK_MANAGER__TASK_CONFIG_PARSER_HPP_
#define TASK_MANAGER__TASK_CONFIG_PARSER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace task_cfg
{
struct VisionActionParam
{
  std::string target_obj_name;
  std::string expected_frame_id;
  bool need_6d_pose;
  bool need_env_point_cloud;
  float downsample_res;
  int8_t  type;
  int32_t priority;
};

struct MoveArmParam
{
  std::string arm_id;
  bool use_planning;
  bool collision_check;
  std::vector<float> safety_distance;
  std::vector<float> config_pose;
  bool avoid_obstacle;
  bool use_config_pose;
  
};

struct GripperParam
{
  uint8_t gripper_id;
  std::vector<uint8_t> com_port;
  std::vector<uint8_t> position;
  std::vector<uint8_t> speed;
  std::vector<uint8_t> max_torque;
  bool block;
  int64_t timeout_ms;
};

struct VisionCheckSelfParam
{
  int64_t timeout_ms;
};

struct ControlCheckSelfParam
{
  int64_t timeout_ms;
  std::vector<uint8_t> com_port;
};


struct VisionResetParam
{
  int64_t timeout_ms;
};

struct ControlResetParam
{
  int64_t timeout_ms;
};

struct AtomicAction
{
  std::string action_type;
  VisionActionParam vision_param;
  MoveArmParam arm_param;
  GripperParam gripper_param;
  VisionCheckSelfParam vision_check_self_param;
  ControlCheckSelfParam control_check_self_param;
  VisionResetParam vision_reset_param;
  ControlResetParam control_reset_param;
};

struct TaskMapItem
{
  uint32_t trigger_reg;
  uint32_t status_reg;
  std::vector<AtomicAction> action_seq;
};

struct BootTaskSet
{
  std::vector<AtomicAction> self_check_seq;
  std::vector<AtomicAction> reset_seq;
};

struct GlobalConfig
{
  int32_t node_id;
  std::string modbus_listen_ip;
  uint16_t modbus_port;
  uint32_t task_trigger_base;
  uint32_t task_status_base;
  std::unordered_map<std::string, TaskMapItem> task_dict;
  BootTaskSet boot_tasks;
};

class TaskConfigParser
{
public:
  TaskConfigParser() = default;
  bool load_config(const std::string& file_path, GlobalConfig& out_cfg);
private:
  bool parse_action(YAML::Node action_node, AtomicAction& act);
  bool parse_task(YAML::Node task_node, TaskMapItem& item);
  bool parse_boot(YAML::Node boot_node, BootTaskSet& boot);
};
}

#endif