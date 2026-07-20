// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Author: gene.wang
// Date: 2026
//
// Description:
// 任务配置文件解析器头文件。
// 定义 YAML 配置文件中各动作类型的参数结构体（VisionActionParam、MoveArmParam、GripperParam 等），
// 以及 TaskConfigParser 类，用于加载和解析任务配置文件。

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
/// @brief 视觉检测动作参数
struct VisionActionParam
{
  std::string target_obj_name;       ///< 目标物体名称
  std::string expected_frame_id;     ///< 期望的坐标系 ID
  bool need_6d_pose;                 ///< 是否需要 6D 位姿
  bool need_env_point_cloud;         ///< 是否需要环境点云
  float downsample_res;              ///< 点云降采样分辨率
  int8_t  type;                      ///< 视觉检测类型
  int32_t priority;                  ///< 任务优先级
};

/// @brief 机械臂移动动作参数
struct MoveArmParam
{
  std::string arm_id;                ///< 机械臂 ID
  bool use_planning;                 ///< 是否使用运动规划
  bool collision_check;              ///< 是否启用碰撞检测
  std::vector<float> safety_distance; ///< 安全距离参数
  std::vector<float> config_pose;    ///< 配置位姿 [x, y, z, qx, qy, qz, qw]
  bool avoid_obstacle;               ///< 是否避障
  bool use_config_pose;              ///< 是否使用配置中的位姿
};

/// @brief 夹爪控制动作参数
struct GripperParam
{
  uint8_t gripper_id;                ///< 夹爪 ID
  std::vector<uint8_t> com_port;     ///< 串口通道
  std::vector<uint8_t> position;     ///< 目标位置
  std::vector<uint8_t> speed;        ///< 运动速度
  std::vector<uint8_t> max_torque;   ///< 最大扭矩
  bool block;                        ///< 是否阻塞等待完成
  int64_t timeout_ms;                ///< 超时时间（毫秒）
};

/// @brief 动作执行前置条件
struct condition
{
  std::string  left_arm_state;       ///< 左臂状态要求
  std::string  power_level;          ///< 电量水平要求
  std::string  right_arm_state;      ///< 右臂状态要求
  std::string  body_waist_state;     ///< 腰部状态要求
  std::string  vision_state;         ///< 视觉状态要求
};

/// @brief 视觉自检参数
struct VisionCheckSelfParam
{
  int64_t timeout_ms;                ///< 超时时间（毫秒）
};

/// @brief 控制自检参数
struct ControlCheckSelfParam
{
  int64_t timeout_ms;                ///< 超时时间（毫秒）
  std::vector<uint8_t> com_port;    ///< 串口通道
};

/// @brief 视觉复位参数
struct VisionResetParam
{
  int64_t timeout_ms;                ///< 超时时间（毫秒）
};

/// @brief 控制复位参数
struct ControlResetParam
{
  int64_t timeout_ms;                ///< 超时时间（毫秒）
};

/// @brief 原子动作：包含动作类型及所有可能的参数
struct AtomicAction
{
  std::string action_type;                    ///< 动作类型标识
  VisionActionParam vision_param;             ///< 视觉检测参数
  MoveArmParam arm_param;                     ///< 机械臂移动参数
  GripperParam gripper_param;                 ///< 夹爪控制参数
  VisionCheckSelfParam vision_check_self_param;   ///< 视觉自检参数
  ControlCheckSelfParam control_check_self_param; ///< 控制自检参数
  VisionResetParam vision_reset_param;         ///< 视觉复位参数
  ControlResetParam control_reset_param;       ///< 控制复位参数
  condition action_execut_condition;           ///< 动作执行前置条件
};

/// @brief 任务映射表项
struct TaskMapItem
{
  uint32_t trigger_reg;                                    ///< 触发寄存器地址
  uint32_t status_reg;                                     ///< 状态寄存器地址
  std::vector<std::vector<AtomicAction>> action_seq;       ///< 动作序列（支持并行分组）
};

/// @brief 启动任务集（自检 + 复位）
struct BootTaskSet
{
  std::vector<AtomicAction> self_check_seq;  ///< 开机自检动作序列
  std::vector<AtomicAction> reset_seq;       ///< 开机复位动作序列
};

/// @brief 全局配置结构体
struct GlobalConfig
{
  int32_t node_id;                                         ///< 节点 ID
  std::string modbus_listen_ip;                            ///< Modbus 监听 IP
  uint16_t modbus_port;                                    ///< Modbus 监听端口
  std::unordered_map<std::string, TaskMapItem> task_dict;  ///< 任务字典
  BootTaskSet boot_tasks;                                  ///< 启动任务集
};

/// @brief 任务配置解析器类
class TaskConfigParser
{
public:
  TaskConfigParser() = default;

  /// @brief 加载并解析 YAML 配置文件
  /// @param file_path 配置文件路径
  /// @param out_cfg [输出] 解析后的全局配置
  /// @return true 解析成功，false 解析失败
  bool load_config(const std::string& file_path, GlobalConfig& out_cfg);

private:
  /// @brief 解析单个动作节点
  bool parse_action(YAML::Node action_node, AtomicAction& act);

  /// @brief 解析单个任务节点
  bool parse_task(YAML::Node task_node, TaskMapItem& item);

  /// @brief 解析启动任务节点
  bool parse_boot(YAML::Node boot_node, BootTaskSet& boot);
};
}

#endif // TASK_MANAGER__TASK_CONFIG_PARSER_HPP_
