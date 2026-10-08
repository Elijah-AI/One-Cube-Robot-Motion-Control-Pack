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
#include <memory>
#include <cstdint>

namespace task_cfg
{
// 前向声明：用于 FeedbackCheckParam 中的 retry_actions（shared_ptr 支持不完整类型）
struct AtomicAction;

/// @brief 视觉检测动作参数
struct VisionActionParam
{
  std::string target_obj_name;       ///< 目标物体名称
  std::string expected_frame_id;     ///< 期望的坐标系 ID
  int8_t  type;                      ///< 视觉检测类型
  int32_t priority;                  ///< 任务优先级
  bool require_single_target{false}; ///< 要求本次检测恰好返回一个目标
  bool require_expected_frame{false}; ///< 检查目标实际坐标系；不执行坐标变换
  std::string result_key;             ///< 可选：在本次任务的同序列内命名保存结果
  std::vector<double> canonical_y_direction; ///< 可选：期望坐标系内的局部+Y参考方向；空则禁用
  double canonical_y_min_alignment{0.5}; ///< |Y dot reference| 下限；方向不明确时拒绝
};

/// 平移默认沿 frame_id 轴，translation_frame=vision 时沿视觉局部轴；姿态右乘。
struct VisionPoseOffset
{
  bool enabled{false};
  std::string translation_frame{"base"}; ///< base 或 vision；base 保持旧配置行为
  std::vector<double> translation{0.0, 0.0, 0.0};
  std::vector<double> rotation{0.0, 0.0, 0.0, 1.0}; ///< [x,y,z,w]
  bool force_vertical_down{false}; ///< TCP Z -> base_link -Z; preserve projected TCP X heading
  bool keep_current_tcp_height{false}; ///< translation[2]=current_tcp_height: derive local Z from measured base Z
  double minimum_z{0.0}; ///< Minimum vision-local approach height in current_tcp_height mode
};

/// @brief 机械臂位姿移动动作参数（对应 MovePose.action）
struct MovePoseParam
{
  uint8_t arm_id;                         ///< 部件编号，与 com_interfaces/action/MovePose 一致：0=ARM_LEFT, 1=ARM_RIGHT, 2=ARM_BOTH, 3=HEAD, 4=LEG
  uint8_t enable_collision_check{0};            ///< 是否启用碰撞检测
  std::vector<float> config_pose;         ///< 目标位姿 [x, y, z, qx, qy, qz, qw]（对应 YAML target_pose；ARM_LEFT/ARM_RIGHT/HEAD/LEG 使用）
  bool use_config_pose;                   ///< 是否使用配置中的位姿
  VisionPoseOffset vision_pose_offset;    ///< 可选：基于同序列视觉原始位姿计算目标
  std::string vision_result_key;          ///< 空值使用最近结果，非空只读取同名结果
  VisionPoseOffset left_vision_pose_offset;  ///< 双臂视觉目标的左臂偏移
  VisionPoseOffset right_vision_pose_offset; ///< 双臂视觉目标的右臂偏移（必须成对配置）
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
  std::vector<std::vector<double>> via_poses; ///< 连续 MoveP 中间 TCP 位姿 [x,y,z,qx,qy,qz,qw]
  bool enable_move_line;                  ///< 是否启用笛卡尔直线运动
  bool whole_body_waist_only{false};       ///< 全身 MoveP 仅使用腰部 J4 和所选手臂
  float acceleration_scale{0.0f};         ///< 0=默认，(0,1]额外加速度比例
  float jerk_scale{0.0f};                 ///< 0=默认，(0,1]额外 jerk 比例
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

/// @brief 反馈字段比较条件（用于根据反馈参数选择调整动作）
/// 支持字段：pose_confidence、target_6d_pose.pose.position.x/y/z、
///         target_6d_pose.pose.orientation.x/y/z/w
struct FeedbackCompareCondition
{
  bool enable{false};                       ///< 是否启用该条件（未启用则始终匹配）
  std::string field{"pose_confidence"};     ///< 反馈字段名
  std::string operator_str{">="};           ///< 比较运算符：">=", ">", "<=", "<", "==", "!="
  double threshold{0.0};                    ///< 比较阈值
};

/// @brief 单个重试（调整）动作：可携带基于反馈参数的选择条件（when）
struct RetryActionParam
{
  FeedbackCompareCondition when;            ///< 选择条件（when 满足时才执行该调整动作；未配置则始终执行）
  std::shared_ptr<AtomicAction> action;     ///< 调整动作本体
};

/// @brief 动作反馈条件检查配置（条件重试）
/// 用于在动作执行成功后检查其反馈结果（如视觉置信度）是否满足阈值，
/// 若不满足则按 when 条件选择 retry_actions 中的调整动作执行，然后重新执行本动作。
struct FeedbackCheckParam
{
  bool enable{false};                       ///< 是否启用反馈条件检查
  std::string check_field{"pose_confidence"}; ///< 检查的反馈字段名（如 pose_confidence）
  std::string operator_str{">="};           ///< 比较运算符：">=", ">", "<=", "<", "==", "!="
  double threshold{0.0};                    ///< 置信度阈值
  int32_t max_retries{1};                   ///< 最大重试次数（每次先执行 retry_actions 再重新执行本动作）
  std::string on_exhaust{"fail"};           ///< 重试次数用尽后的行为："fail" 返回失败，"pass" 强制通过
  std::vector<RetryActionParam> retry_actions; ///< 调整动作序列（条件不满足时按 when 条件选择执行）
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
  uint8_t leg_motion_mode{0}; ///< 0=关节目标；1=绝对身体倾角；2=相对升降；3=绝对身体高度
  double body_tilt_deg{0};    ///< 身体 +Z 向 base_link +X 的有符号倾角（度）
  double body_height_m{0};    ///< 相对 Z 距离或 body_link 原点的绝对 Z（米）
  bool relative_position{false}; ///< 仅 LEG/body_joint：position[0] 为相对当前反馈的腰转角
  // LEG: four joint targets retain ordinary MoveJ; two distinct leg_joint1/2/3
  // targets request automatic third-joint compensation with fixed body orientation.
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
  uint8_t enable_collision_check{0};            ///< 是否启用碰撞检测
  float acceleration_scale{0.0f};         ///< 0=默认，(0,1]额外加速度比例
  float jerk_scale{0.0f};                 ///< 0=默认，(0,1]额外 jerk 比例
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

/// @brief 视觉标定控制动作参数（对应 com_interfaces/action/CalibrationControl.action）
struct CalibrationControlParam
{
  uint8_t camera_id{0};                   ///< 相机 ID：0=头部相机，1=腰部相机，2+预留
  uint8_t arm_id{0};                      ///< 机械臂 ID：0=左臂，1=右臂
  std::string command;                    ///< 命令类型："CHECK_STATUS", "TRIGGER_CAPTURE", "EXECUTE_CALIBRATION"
  bool auto_execute{false};               ///< 采集完成后自动执行标定（仅 TRIGGER_CAPTURE 时有效）
  std::string mode{"TO_HAND"};            ///< 标定模式："TO_HAND", "IN_HAND"
  int64_t timeout_ms{60000};              ///< 等待结果超时时间 (毫秒)，默认 60000ms
};

/// @brief 自动标定任务参数（复合动作，由 task_action_client 后台编排执行）
struct AutoCalibrateParam
{
  uint8_t camera_id{0};                   ///< 相机 ID（默认头部相机）
  uint8_t arm_id{0};                      ///< 机械臂 ID（0=左臂）
  std::string mode{"TO_HAND"};            ///< 标定模式（默认 TO_HAND）
  int32_t sample_num{12};                 ///< 按顺序从位姿池取前 sample_num 组（默认 12）

  // —— 循环内 move_joints 子步骤（单臂关节空间移动）通用参数 ——
  std::vector<std::string> joint_name;    ///< 关节名称列表（需 7 个；未配置时按 arm_id 回退默认名 Joint1_R..Joint7_R / Joint1_L..Joint7_L）
  uint8_t enable_collision_check{0};     ///< 是否启用自碰撞检测（自动标定流程下默认关闭）
  float speed{0.2f};                      ///< 移动速度 0~1

  // —— 头部初始到位（可选）：自动标定开始时先将头部 move_joints 到指定关节角 ——
  bool head_move_enable{false};           ///< 是否在自动标定开始时先将头部 move_joints 到指定位置（默认关闭）
  std::vector<std::string> head_joint_name; ///< 头部关节名称列表（未配置时回退默认 head_yaw_joint / head_pitch_joint）
  std::vector<float> head_position;       ///< 头部目标关节角（单位 rad，顺序对应 head_joint_name）
  float head_speed{0.2f};                 ///< 头部移动速度 0~1
  int64_t head_move_timeout_ms{15000};    ///< 头部 move_joints 超时（毫秒）

  // —— 各阶段超时 ——
  int64_t move_timeout_ms{15000};         ///< 单次 move_joints 超时（毫秒）
  int64_t capture_timeout_ms{60000};      ///< 单次 TRIGGER_CAPTURE 等待超时（毫秒）
  int64_t timeout_ms{600000};             ///< 整个自动标定总超时（毫秒）

  /// 每次 move_joints 到位后、触发 TRIGGER_CAPTURE 采集前的稳定等待时长（毫秒），0=不等待
  int64_t post_move_delay_ms{0};

  /// 目标关节角池：每个元素为一组 7 个关节角（单位 rad，顺序对应 joint_name / 默认关节名）
  std::vector<std::vector<float>> pose_pool;
};

// ============================================================================
// 复合动作（move_compound）相关结构体
// ============================================================================

/// @brief 复合动作子项：动作类型 + 与普通单体动作完全一致的参数
/// 子动作支持 move_pose / move_joints，其 params 字段结构与普通同名单体动作完全相同
/// （即 parse_action 中 move_pose / move_joints 的 params 节点），
/// 由对应的 parse_move_pose_params / parse_move_joints_params 复用解析，保证参数控制一致。
struct CompoundSubAction
{
  std::string action_type;              ///< 子动作类型：move_pose / move_joints
  MovePoseParam move_pose_param;        ///< move_pose 子动作完整参数（与普通 move_pose 一致，含 ARM_BOTH）
  MoveJointsParam move_joints_param;    ///< move_joints 子动作完整参数（与普通 move_joints 一致）
};

/// @brief 复合动作（move_compound）参数：在单个动作内部并行驱动多个单体动作
/// 典型场景：双臂 move_pose（ARM_BOTH）+ 腿部 move_joints（LEG）同时运动。
/// 由 task_action_client 后台 worker 线程对每个子动作使用独立子 state_key 异步发送
/// goal（各动作 Server 相互独立，天然并行），再并发等待全部子动作完成：
/// 全部成功才判成功，任一失败/整体超时即取消其余并判失败。
struct CompoundMoveParam
{
  std::vector<CompoundSubAction> sub_actions;   ///< 并行执行的子动作列表（至少 1 个）
  int64_t timeout_ms{30000};                    ///< 复合整体超时（毫秒），默认 30000ms
};

// ============================================================================
// 前置条件（preconditions）相关结构体
// ============================================================================

/// @brief 前置条件类型（10 类）
enum class PreconditionType : int8_t
{
  BATTERY_LEVEL,          ///< 电池电量（数值条件）
  ROBOT_STATUS,           ///< 机器人状态（状态集合条件）
  SAFETY_STATUS,          ///< 安全状态（状态集合条件）
  ARM_IDLE,               ///< 机械臂空闲（部件空闲条件）
  HEAD_IDLE,              ///< 头部空闲（部件空闲条件）
  WAIST_IDLE,             ///< 腰部空闲（部件空闲条件）
  GRIPPER_IDLE,           ///< 夹爪空闲（部件空闲条件）
  SUCK_IDLE,              ///< 吸盘空闲（部件空闲条件）
  DEXTEROUS_HAND_IDLE,    ///< 灵巧手空闲（部件空闲条件）
  UNKNOWN                 ///< 未知/未配置
};

/// @brief 前置条件组合逻辑
enum class PreLogicalOp : int8_t
{
  AND,       ///< 全部满足（默认）
  OR,        ///< 任一满足
  UNKNOWN    ///< 未知/未配置
};

/// @brief 前置条件不满足时的处理策略
enum class PrePolicy : int8_t
{
  WAIT,       ///< 周期性重查直至满足或超时（默认）
  FAIL_FAST,  ///< 立即判定本动作失败
  SKIP,       ///< 跳过本动作，视为成功继续
  UNKNOWN     ///< 未知/未配置
};

/// @brief 等待超时后的处理行为
enum class PreTimeoutAction : int8_t
{
  FAIL,     ///< 超时后判定失败（默认）
  PASS,     ///< 超时后放行
  ABORT,    ///< 超时后中止整个任务
  UNKNOWN   ///< 未知/未配置
};

/// @brief 部件方位
enum class PrePart : int8_t
{
  LEFT,     ///< 左
  RIGHT,    ///< 右
  BOTH,     ///< 双侧（默认）
  UNKNOWN   ///< 未知/未配置
};

/// @brief 单个前置条件参数
struct PreconditionParam
{
  PreconditionType type{PreconditionType::UNKNOWN}; ///< 条件类型
  bool enable{true};                                ///< 是否启用（false 时该条件恒通过）
  // —— 状态集合类条件（robot_status / safety_status）——
  std::vector<int32_t> expected;                    ///< 期望状态集合（状态枚举值，解析时字符串/数值统一映射为枚举值）
  // —— 数值类条件（battery_level）——
  std::string field{"percentage"};                  ///< 数值字段名：percentage / voltage，默认 percentage
  std::string operator_str{">="};                   ///< 比较运算符：>=, >, <=, <, ==, !=，默认 >=
  double threshold{0.0};                            ///< 数值阈值
  // —— 部件空闲类条件（arm/head/waist/gripper/suck/dexterous_hand_idle）——
  PrePart part{PrePart::BOTH};                      ///< 部件方位：left / right / both，默认 both
  bool expected_bool{true};                         ///< 期望空闲（true=空闲 / false=占用），默认 true
  // —— 等待超时行为 ——
  PreTimeoutAction on_timeout{PreTimeoutAction::FAIL}; ///< wait 超时后：fail / pass / abort，默认 fail
};

/// @brief 前置条件列表（挂在原子动作节点下）
struct PreconditionList
{
  bool has{false};                                  ///< 是否配置了前置条件
  PreLogicalOp logical_op{PreLogicalOp::AND};       ///< 组合逻辑：and / or，默认 and
  PrePolicy policy{PrePolicy::WAIT};                ///< 不满足时处理策略：wait / fail_fast / skip，默认 wait
  int64_t max_wait_ms{0};                           ///< wait 模式最长等待时间（毫秒），0=无限等待
  int64_t check_interval_ms{100};                   ///< wait 模式重查周期（毫秒），默认 100
  std::vector<PreconditionParam> conditions;        ///< 条件列表
};

// ============================================================================
// 结果分支（on_success / on_failure）相关结构体
// ============================================================================

/// @brief 结果分支策略
enum class ResultPolicy : int8_t
{
  BRANCH_CONTINUE,  ///< on_success: continue（分支成功后本动作仍算成功；分支失败则本动作失败）
  BRANCH_OVERRIDE,  ///< on_success: override（分支的最终结果作为本动作结果）
  BRANCH_FAIL,      ///< on_failure: fail（分支执行后本动作仍判失败，任务终止）
  BRANCH_RECOVER,   ///< on_failure: recover（分支全部成功则本动作转为成功，主序列继续）
  UNKNOWN           ///< 未知/未配置
};

/// @brief 结果分支动作（可递归嵌套 preconditions / 结果分支）
struct ResultActionParam
{
  std::shared_ptr<AtomicAction> action;   ///< 分支动作本体（shared_ptr 支持递归）
};

/// @brief 结果分支（on_success / on_failure）
struct ResultBranchParam
{
  bool has{false};                                  ///< 是否配置了该分支
  ResultPolicy policy{ResultPolicy::UNKNOWN};       ///< 分支策略
  std::vector<ResultActionParam> actions;           ///< 分支动作序列
};

/// @brief 原子动作：包含动作类型及所有可能的参数
struct IKLegLiftRecoveryParam
{
  bool enabled{false};
  std::string key;
  double lift_m{0.05};
  float speed{0.10f};
  int64_t timeout_ms{30000};
  uint8_t collision_mode{1};
};

struct AtomicAction
{
  IKLegLiftRecoveryParam ik_failure_recovery;
  std::string if_recovery_applied;
  std::string action_type;                    ///< 动作类型标识
  int64_t delay_ms;                           ///< 执行前延迟时间（毫秒），>0 时先延迟再执行，默认 0 不延迟
  VisionActionParam vision_param;             ///< 视觉检测参数
  MovePoseParam arm_param;                    ///< 机械臂位姿移动参数
  GripperParam gripper_param;                 ///< 夹爪控制参数
  VisionCheckSelfParam vision_check_self_param;   ///< 视觉自检参数
  ControlCheckSelfParam control_check_self_param; ///< 控制自检参数
  VisionResetParam vision_reset_param;         ///< 视觉复位参数
  ControlResetParam control_reset_param;       ///< 控制复位参数
  MoveJointsParam move_joints_param;           ///< 关节移动参数
  SuckParam suck_param;                        ///< 吸盘控制参数
  bool is_parallel_group{false};               ///< true 时本节点为局部并行动作组
  std::vector<AtomicAction> parallel_actions;  ///< 组内动作并发执行，全部结束后继续
  CalibrationControlParam calibration_param;   ///< 视觉标定控制参数
  AutoCalibrateParam auto_calibrate_param;     ///< 自动标定任务参数
  CompoundMoveParam compound_move_param;       ///< 复合动作参数（move_compound：多单体动作并行）
  FeedbackCheckParam feedback_check;           ///< 动作反馈条件检查（条件重试）
  PreconditionList preconditions;              ///< 前置条件（可选，见 4.2）
  ResultBranchParam on_success;                ///< 成功分支（可选，见 4.3）
  ResultBranchParam on_failure;                ///< 失败分支（可选，见 4.3）
};

/// @brief 任务映射表项
struct TaskMapItem
{
  uint32_t trigger_reg;                                    ///< 触发寄存器地址
  uint32_t status_reg;                                     ///< 状态寄存器地址
  std::string config_file;                                 ///< 动作列表来源配置文件（外链格式时为相对主配置目录的路径；内联时为空）
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
  uint8_t modbus_slave_id{1};                              ///< Modbus 从站 ID（映射 modbus_set_slave）
  std::unordered_map<std::string, TaskMapItem> task_dict;  ///< 任务字典
  BootTaskSet boot_tasks;                                  ///< 启动任务集
};

/// @brief 任务配置解析器类
class TaskConfigParser
{
public:
  TaskConfigParser() = default;

  /// @brief 加载并解析 YAML 配置文件（兼容单文件格式，等价于 load_all）
  /// @param file_path 主配置文件路径
  /// @param out_cfg [输出] 解析后的全局配置
  /// @return true 解析成功，false 解析失败
  bool load_config(const std::string& file_path, GlobalConfig& out_cfg);

  /// @brief 加载全部配置（主配置 + 级联加载外链的次级任务配置文件）
  /// @param main_file_path 主配置文件路径
  /// @param out_cfg [输出] 解析后的全局配置
  /// @return true 解析成功，false 解析失败
  bool load_all(const std::string& main_file_path, GlobalConfig& out_cfg);

  /// @brief 加载主配置文件（节点 ID、Modbus 参数、任务注册表、启动任务）
  /// 任务条目支持双格式：
  ///   - 内联：直接包含 action_sequencenum / action_sequenceN
  ///   - 外链：通过 config_file 指向次级任务动作配置文件（相对主配置文件所在目录解析）
  /// @param file_path 主配置文件路径
  /// @param out_cfg [输出] 解析后的全局配置
  /// @return true 解析成功，false 解析失败
  bool load_main_config(const std::string& file_path, GlobalConfig& out_cfg);

  /// @brief 加载次级任务动作配置文件（含 action_sequencenum / action_sequenceN），合并到 out_cfg
  /// @param file_path 次级配置文件路径
  /// @param out_cfg [输入/输出] 全局配置（task_dict[task_name] 将被填充）
  /// @param task_name 任务名称（用于合并到 task_dict）
  /// @return true 解析成功，false 解析失败
  bool load_task_config(const std::string& file_path, GlobalConfig& out_cfg, const std::string& task_name);

private:
  /// @brief 解析单个动作节点（递归深度受 depth 限制，防止配置错误导致无限递归）
  /// @param depth 当前递归深度（结果分支嵌套时递增，上限 kMaxBranchDepth）
  bool parse_action(YAML::Node action_node, AtomicAction& act, int depth = 0);

  // --------------------------------------------------------------------------
  // 各动作类型参数解析（由 parse_action 按 action_type 分发调用）
  // 每个函数仅负责解析 params 节点并填充对应的动作参数结构体；
  // YAML 字段缺失/类型不符时 .as<>() 抛异常，由上层 load_* 的 try/catch 统一处理。
  // 返回 true 表示解析成功（预留校验能力）。
  // --------------------------------------------------------------------------

  /// @brief 解析视觉检测动作参数（vision_detect）
  bool parse_vision_detect_params(YAML::Node params_node, AtomicAction& act);

  /// @brief 解析机械臂位姿移动动作参数（move_pose）
  bool parse_move_pose_params(YAML::Node params_node, AtomicAction& act);

  /// @brief 解析夹爪控制动作参数（gripper_ctrl）
  bool parse_gripper_ctrl_params(YAML::Node params_node, AtomicAction& act);

  /// @brief 解析视觉自检动作参数（vision_check_self）
  bool parse_vision_check_self_params(YAML::Node params_node, AtomicAction& act);

  /// @brief 解析控制自检动作参数（control_check_self）
  bool parse_control_check_self_params(YAML::Node params_node, AtomicAction& act);

  /// @brief 解析控制复位动作参数（control_reset）
  bool parse_control_reset_params(YAML::Node params_node, AtomicAction& act);

  /// @brief 解析视觉复位动作参数（vision_reset）
  bool parse_vision_reset_params(YAML::Node params_node, AtomicAction& act);

  /// @brief 解析关节空间移动动作参数（move_joints）
  bool parse_move_joints_params(YAML::Node params_node, AtomicAction& act);

  /// @brief 解析吸盘控制动作参数（control_suck）
  bool parse_control_suck_params(YAML::Node params_node, AtomicAction& act);

  /// @brief 解析视觉标定控制动作参数（calibration_control）
  bool parse_calibration_control_params(YAML::Node params_node, AtomicAction& act);

  /// @brief 解析自动标定任务动作参数（auto_calibrate）
  bool parse_auto_calibrate_params(YAML::Node params_node, AtomicAction& act);

  /// @brief 解析复合动作参数（move_compound）：遍历 sub_actions 并按子类型复用
  /// parse_move_pose_params / parse_move_joints_params 解析，保证与普通单体动作参数一致
  bool parse_compound_move_params(YAML::Node params_node, AtomicAction& act);

  /// @brief 解析前置条件节点（preconditions）
  bool parse_preconditions(YAML::Node pre_node, PreconditionList& pre);

  /// @brief 解析单个前置条件
  bool parse_precondition(YAML::Node cond_node, PreconditionParam& cond);

  /// @brief 解析结果分支节点（on_success / on_failure）
  bool parse_result_branch(YAML::Node branch_node, ResultBranchParam& branch, int depth);

  /// @brief 解析单个任务节点
  bool parse_task(YAML::Node task_node, TaskMapItem& item);

  /// @brief 解析启动任务节点
  bool parse_boot(YAML::Node boot_node, BootTaskSet& boot);

  /// @brief 结果分支递归深度上限（防止配置错误导致无限递归）
  static constexpr int kMaxBranchDepth = 5;
};
}

#endif // TASK_MANAGER__TASK_CONFIG_PARSER_HPP_
