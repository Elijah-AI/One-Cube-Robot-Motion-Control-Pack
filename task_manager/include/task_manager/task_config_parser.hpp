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

/// @brief 机械臂位姿移动动作参数（对应 MovePose.action）
struct MovePoseParam
{
  uint8_t arm_id;                         ///< 部件编号，与 com_interfaces/action/MovePose 一致：0=ARM_LEFT, 1=ARM_RIGHT, 2=ARM_BOTH, 3=HEAD, 4=LEG
  bool enable_collision_check;            ///< 是否启用碰撞检测
  std::vector<float> config_pose;         ///< 目标位姿 [x, y, z, qx, qy, qz, qw]（对应 YAML target_pose；ARM_LEFT/ARM_RIGHT/HEAD/LEG 使用）
  bool use_config_pose;                   ///< 是否使用配置中的位姿
  std::vector<float> bound_size;          ///< 包围盒尺寸 [x, y, z]（ARM_LEFT/ARM_RIGHT/HEAD/LEG 使用）
  /// @name ARM_BOTH 专用字段（与 com_interfaces/action/MovePose.action 一致）
  /// @{
  std::vector<float> left_target_pose;    ///< 左臂目标位姿 [x, y, z, qx, qy, qz, qw]（对应 YAML left_target_pose；ARM_BOTH 使用）
  std::vector<float> right_target_pose;   ///< 右臂目标位姿 [x, y, z, qx, qy, qz, qw]（对应 YAML right_target_pose；ARM_BOTH 使用）
  std::vector<float> left_bound_size;     ///< 左臂包围盒尺寸 [x, y, z]（对应 YAML left_bound_size；ARM_BOTH 使用）
  std::vector<float> right_bound_size;    ///< 右臂包围盒尺寸 [x, y, z]（对应 YAML right_bound_size；ARM_BOTH 使用）
  uint8_t synchronization_mode;           ///< 同步模式：0=SYNC_START_AND_FINISH, 1=SYNC_START_ONLY（对应 YAML synchronization_mode；ARM_BOTH 使用）
  /// @}
  bool use_force_control;                 ///< 是否使用力控模式
  bool enable_move_line;                  ///< 是否启用笛卡尔直线运动
  float speed;                            ///< 控制速度，范围0～1
  std::vector<uint8_t> hand_action_type;  ///< 灵巧手动作类型
  std::string frame_id;                   ///< 坐标系 ID
  int64_t timeout_ms;                     ///< 超时时间(毫秒), 默认5000ms
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
  std::vector<int32_t> com_port;    ///< 串口通道（与 HardwareCheck.action 的 int32[] com_port 一致）
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

/// @brief 关节空间移动动作参数
struct MoveJointsParam
{
  uint8_t arm_id;                         ///< 部件编号，与 com_interfaces/action/MoveJoints 一致：0=ARM_LEFT, 1=ARM_RIGHT, 2=ARM_BOTH, 3=HEAD, 4=LEG
  std::vector<std::string> joint_name;    ///< 关节名称列表（ARM_LEFT/ARM_RIGHT/HEAD/LEG 使用）
  std::vector<double> position;           ///< 关节位置（单位 rad）（ARM_LEFT/ARM_RIGHT/HEAD/LEG 使用）
  /// @name ARM_BOTH 专用字段（与 com_interfaces/action/MoveJoints.action 一致）
  /// @{
  std::vector<std::string> left_joint_name;   ///< 左臂关节名称列表（ARM_BOTH 使用）
  std::vector<double> left_position;          ///< 左臂关节位置（ARM_BOTH 使用）
  std::vector<std::string> right_joint_name;  ///< 右臂关节名称列表（ARM_BOTH 使用）
  std::vector<double> right_position;         ///< 右臂关节位置（ARM_BOTH 使用）
  uint8_t synchronization_mode;               ///< 同步模式：0=SYNC_START_AND_FINISH, 1=SYNC_START_ONLY（ARM_BOTH 使用）
  /// @}
  bool enable_collision_check;            ///< 是否启用碰撞检测
  float speed;                            ///< 控制速度，范围0～1
  int64_t timeout_ms;                     ///< 操作超时时间 (毫秒)，默认 5000ms
};

/// @brief 吸盘控制动作参数（对应 ControlSuck.action）
struct SuckParam
{
  uint8_t suck_id;                        ///< 部件编号，与 com_interfaces/action/ControlSuck 一致：0=SUCK_LEFT, 1=SUCK_RIGHT, 2=SUCK_BOTH
  std::vector<bool> active;               ///< 打开/关闭
  int64_t timeout_ms;                     ///< 操作超时时间 (毫秒)，默认 5000ms
};

/// @brief 原子动作：包含动作类型及所有可能的参数
struct AtomicAction
{
  std::string action_type;                    ///< 动作类型标识
  VisionActionParam vision_param;             ///< 视觉检测参数
  MovePoseParam arm_param;                    ///< 机械臂位姿移动参数
  GripperParam gripper_param;                 ///< 夹爪控制参数
  VisionCheckSelfParam vision_check_self_param;   ///< 视觉自检参数
  ControlCheckSelfParam control_check_self_param; ///< 控制自检参数
  VisionResetParam vision_reset_param;         ///< 视觉复位参数
  ControlResetParam control_reset_param;       ///< 控制复位参数
  MoveJointsParam move_joints_param;           ///< 关节移动参数
  SuckParam suck_param;                        ///< 吸盘控制参数
  condition action_execut_condition;           ///< 动作执行前置条件
  bool is_parallel_group{false};               ///< true 时本节点为局部并行动作组
  std::vector<AtomicAction> parallel_actions;  ///< 组内动作并发执行，全部结束后继续
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
