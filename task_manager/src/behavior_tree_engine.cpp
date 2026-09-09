// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Author: gene.wang
// Date: 2026
//
// Description:
// 行为树引擎实现。
// 实现自定义行为树框架，包含 FallbackNode（回退选择）、
// ActionNode（动作执行）、ConditionNode（条件判断）、
// SequenceNode（顺序执行）、ParallelNode（并行执行）等节点类型，
// 以及 BehaviorTreeEngine 引擎的加载和执行调度逻辑。

#include "task_manager/behavior_tree_engine.hpp"
#include "task_manager/log_helper.hpp"
#include "com_interfaces/msg/target_detection.hpp"
#include <rclcpp/logging.hpp>
#include <iostream>

BehaviorTreeEngine::BehaviorTreeEngine(rclcpp::Logger logger)
: logger_(std::move(logger)),
  root_node_(nullptr),
  running_(false),
  last_status_(BTStatus::IDLE)
{}

void BehaviorTreeEngine::setPreconditionManager(task_mgr::PreconditionManager* pm)
{
  precondition_mgr_ = pm;
}

void BehaviorTreeEngine::setPreconditionFailCb(std::function<void()> cb)
{
  precondition_fail_cb_ = std::move(cb);
}

void BehaviorTreeEngine::setRoot(NodePtr root)
{
  root_node_ = std::move(root);
  if (root_node_)
  {
    root_node_->reset();
  }
  last_status_ = BTStatus::IDLE;
  running_ = false;
}

void BehaviorTreeEngine::start()
{
  if (!root_node_)
  {
    LOG_ERROR_CRITICAL(logger_, task_log::Tag::BT, "BehaviorTree root node is empty, cannot start!");
    return;
  }
  root_node_->reset();
  running_ = true;
  LOG_INFO(logger_, task_log::Tag::BT, "BehaviorTree started");
}

void BehaviorTreeEngine::abort()
{
  running_ = false;
  if (root_node_)
  {
    root_node_->reset();
  }
  last_status_ = BTStatus::IDLE;
  LOG_WARN(logger_, task_log::Tag::BT, "BehaviorTree aborted");
}




void BehaviorTreeEngine::tick_once()
{
  if (!running_ || !root_node_)
  {
    return;
  }

  last_status_ = root_node_->tick();
  if (last_status_ == BTStatus::SUCCESS || last_status_ == BTStatus::FAILURE)
  {
    running_ = false;
    LOG_INFO(logger_, task_log::Tag::BT, "Tree finished, final status: " << (last_status_ == BTStatus::SUCCESS ? "SUCCESS" : "FAILURE"));
  }
}

// ============================================================================
// 辅助函数：将 ActionCallResult 映射为 BTStatus
// ============================================================================
namespace {

/// @brief 根据 ActionCallResult 决定行为树节点的返回状态。
///        对于 FAIL_SERVER_BUSY 返回 RUNNING（允许重试），
///        对于 FAIL_PARAM_INVALID / FAIL_UNKNOWN_TYPE 直接返回 FAILURE（无需重试），
///        其他失败也返回 FAILURE。
inline BTStatus action_result_to_bt_status(task_act::ActionCallResult result)
{
    switch (result)
    {
        case task_act::ActionCallResult::SUCCESS:
            return BTStatus::SUCCESS;

        // 服务端繁忙：返回 RUNNING，让行为树下次 tick 重试
        case task_act::ActionCallResult::FAIL_SERVER_BUSY:
            return BTStatus::RUNNING;

        // 本地资源被占用：返回 RUNNING，让行为树下次 tick 重试
        case task_act::ActionCallResult::FAIL_RESOURCE_BUSY:
            return BTStatus::RUNNING;

        // 参数无效或未知类型：直接失败，无需重试
        case task_act::ActionCallResult::FAIL_PARAM_INVALID:
        case task_act::ActionCallResult::FAIL_UNKNOWN_TYPE:
            return BTStatus::FAILURE;

        // 其他失败（超时、被拒绝、通用错误）：也返回 FAILURE
        case task_act::ActionCallResult::FAIL_SERVER_TIMEOUT:
        case task_act::ActionCallResult::FAIL_GOAL_REJECTED:
        case task_act::ActionCallResult::FAIL_TIMEOUT:
        case task_act::ActionCallResult::FAIL_GENERAL:
        default:
            return BTStatus::FAILURE;
    }
}

/// @brief 检查 action 是否正在运行（超时检测）
/// @param state_key 可选的状态 key，默认为 "default"
inline BTStatus check_action_running(
    task_act::TaskActionClient& act_client,
    const std::string& action_type,
    rclcpp::Logger logger,
    const std::string& state_key = "default")
{
    rclcpp::Time now = act_client.get_ros_now_time();
    rclcpp::Time start_time = act_client.get_action_start_time(state_key);
    
    // 如果 start_time 为 0，说明还未开始计时，返回 RUNNING
    if (start_time.nanoseconds() == 0)
    {
        return BTStatus::RUNNING;
    }
    
    rclcpp::Duration diff = now - start_time;

    double start_ms = static_cast<double>(start_time.nanoseconds()) / 1000000.0;
    double now_ms   = static_cast<double>(now.nanoseconds()) / 1000000.0;

    int64_t delta_ns = diff.nanoseconds();
    double elapsed_ms = std::fabs(delta_ns / 1000000.0);
    uint32_t exec_time = act_client.get_action_execution_time(state_key);
    
    LOG_DEBUG(logger, task_log::Tag::BT,
        "check_action_running [" << action_type << "][" << state_key << "]: elapsed=" << elapsed_ms << "ms, limit=" << exec_time << "ms");
    
    if (elapsed_ms > exec_time)
    {
      LOG_WARN(logger, task_log::Tag::BT,
        "Action timeout: start=" << start_ms << "ms, now=" << now_ms << "ms, elapsed=" << elapsed_ms << "ms, limit=" << exec_time << "ms");
      LOG_WARN(logger, task_log::Tag::BT, "Action [" << action_type << "] execution timed out");
      return BTStatus::FAILURE;
    }
    else
    {
      return BTStatus::RUNNING;
    }
}

/// @brief 检查 action 是否成功完成
inline BTStatus check_action_success(
    task_act::TaskActionClient& act_client,
    const std::string& action_type,
    rclcpp::Logger logger)
{
    LOG_SUCCESS(logger, task_log::Tag::BT, "Action [" << action_type << "] executed successfully");
    act_client.reset_state();
    return BTStatus::SUCCESS;
}

/// @brief 检查 action 是否失败，并根据详细原因返回对应的 BTStatus
inline BTStatus check_action_failure(
    task_act::TaskActionClient& act_client,
    const std::string& action_type,
    rclcpp::Logger logger)
{
    task_act::ActionCallResult detail = act_client.get_last_call_result();
    LOG_ERROR(logger, task_log::Tag::BT, "Action [" << action_type << "] execution failed: " << task_act::action_call_result_to_str(detail));
    return action_result_to_bt_status(detail);
}

/// @brief 发送 goal 并返回对应的 BTStatus
inline BTStatus send_action_goal(
    task_act::TaskActionClient& act_client,
    const task_cfg::AtomicAction& atomic_act,
    const std::string& action_type,
    rclcpp::Logger logger)
{
    LOG_INFO_TITLE(logger, task_log::Tag::BT, "===== Execute Action Type: " << action_type << " =====");
    if (action_type == "vision_detect")
    {
      LOG_INFO(logger, task_log::Tag::BT, "VisionParam -> target: " << atomic_act.vision_param.target_obj_name
        << ", frame_id: " << atomic_act.vision_param.expected_frame_id);
    }
    else if (action_type == "move_pose")
    {
      LOG_INFO(logger, task_log::Tag::BT, "MovePoseParam -> arm_id: " << static_cast<int>(atomic_act.arm_param.arm_id)
        << ", collision_check: " << (atomic_act.arm_param.enable_collision_check ? "true" : "false")
        << ", speed: " << atomic_act.arm_param.speed << ", timeout_ms: " << atomic_act.arm_param.timeout_ms);
    }
    else if (action_type == "move_joints")
    {
      LOG_INFO(logger, task_log::Tag::BT, "MoveJointsParam -> arm_id: " << static_cast<int>(atomic_act.move_joints_param.arm_id)
        << ", collision_check: " << (atomic_act.move_joints_param.enable_collision_check ? "true" : "false")
        << ", speed: " << atomic_act.move_joints_param.speed << ", timeout_ms: " << atomic_act.move_joints_param.timeout_ms);
    }
    else if (action_type == "gripper_ctrl")
    {
      LOG_INFO(logger, task_log::Tag::BT, "GripperParam -> gripper_id: " << static_cast<int>(atomic_act.gripper_param.gripper_id)
        << ", block: " << (atomic_act.gripper_param.block ? "true" : "false"));
    }
    else if (action_type == "control_suck")
    {
      std::ostringstream oss;
      oss << "SuckParam -> suck_id: " << static_cast<int>(atomic_act.suck_param.suck_id)
          << ", active=[";
      for (size_t i = 0; i < atomic_act.suck_param.active.size(); ++i)
        oss << (i ? "," : "") << (atomic_act.suck_param.active[i] ? "true" : "false");
      oss << "], timeout_ms: " << atomic_act.suck_param.timeout_ms;
      LOG_INFO(logger, task_log::Tag::BT, oss.str());
    }
    else if (action_type == "vision_check_self")
    {
      LOG_INFO(logger, task_log::Tag::BT, "VisionCheckSelfParam -> timeout_ms: " << atomic_act.vision_check_self_param.timeout_ms);
    }
    else if (action_type == "control_check_self")
    {
      LOG_INFO(logger, task_log::Tag::BT, "ControlCheckSelfParam -> com_port[0]: " << static_cast<int>(atomic_act.control_check_self_param.com_port[0])
        << ", com_port[1]: " << static_cast<int>(atomic_act.control_check_self_param.com_port[1])
        << ", timeout_ms: " << atomic_act.control_check_self_param.timeout_ms);
    }
    else if (action_type == "vision_reset")
    {
      LOG_INFO(logger, task_log::Tag::BT, "VisionResetParam -> timeout_ms: " << atomic_act.vision_reset_param.timeout_ms);
    }
    else if (action_type == "control_reset")
    {
      LOG_INFO(logger, task_log::Tag::BT, "ControlResetParam -> timeout_ms: " << atomic_act.control_reset_param.timeout_ms);
    }

    task_act::ActionCallResult send_ret = act_client.send_goal(atomic_act);
    if (!task_act::action_call_result_is_ok(send_ret))
    {
        LOG_ERROR(logger, task_log::Tag::BT, "Action [" << action_type << "] send goal failed: " << task_act::action_call_result_to_str(send_ret));
        return action_result_to_bt_status(send_ret);
    }
    else
    {
        return BTStatus::RUNNING;
    }
}

/// @brief 构建并行分支动作节点（ActionNode）
/// 复刻 load_parallel_task 中原有的动作回调逻辑：
/// 基于 state_key 查询动作状态（运行/成功/失败），
/// vision_detect 成功后提取结果并存储到序列级 key，最后重置动作状态。
/// 供普通动作节点与条件重试节点的探测/调整动作复用。
/// @param act_client Action 客户端引用
/// @param atomic_act 原子动作共享指针
/// @param state_key 状态 key
/// @param branch_index 分支序号（日志）
/// @param node_name 节点名称
/// @param logger ROS2 日志记录器
/// @return 构建好的 ActionNode 共享指针
BTNode::Ptr build_action_node(
    task_act::TaskActionClient& act_client,
    const std::shared_ptr<task_cfg::AtomicAction>& atomic_act,
    const std::string& state_key,
    long branch_index,
    const std::string& node_name,
    rclcpp::Logger logger)
{
    auto action_cb = [&act_client, atomic_act, state_key, branch_index, logger]() -> BTStatus
    {
      // 使用独立状态 key 查询，不干扰其他并行分支的动作状态
      if (act_client.is_action_running(state_key))
      {
        return check_action_running(act_client, atomic_act->action_type, logger, state_key);
      }
      else if (act_client.is_action_success(state_key))
      {
        LOG_SUCCESS(logger, task_log::Tag::BT, "[Branch " << branch_index << "] Action [" << atomic_act->action_type << "] executed successfully");

        // ========== vision_detect 成功后提取并存储到序列级 key ==========
        if (atomic_act->action_type == "vision_detect" && act_client.has_vision_result(state_key))
        {
          task_act::VisionDet::Result vis_res;
          if (act_client.get_vision_result(state_key, vis_res))
          {
            // 推导序列级 key（如 "seq1", "seq2"），用于同序列后续动作共享
            std::string seq_key = task_act::derive_seq_key(state_key);
            // 新接口 DetectionPoseCloud 以 detected_targets[0] 作为主目标输出
            std::string vis_pos = "[0, 0, 0]";
            double vis_conf = 0.0;
            std::string vis_frame = "";
            if (!vis_res.detected_targets.empty())
            {
              const auto& vis_tgt = vis_res.detected_targets[0];
              std::ostringstream oss_pos;
              oss_pos << "[" << vis_tgt.pose.pose.position.x << ", "
                      << vis_tgt.pose.pose.position.y << ", "
                      << vis_tgt.pose.pose.position.z << "]";
              vis_pos = oss_pos.str();
              vis_conf = static_cast<double>(vis_tgt.confidence);
              vis_frame = vis_tgt.pose.header.frame_id;
            }
            LOG_INFO(logger, task_log::Tag::BT,
              "[Branch " << branch_index << "] Vision result stored: target="
              << atomic_act->vision_param.target_obj_name
              << ", seq_key=" << seq_key
              << ", count=" << vis_res.total_detected_count
              << ", pos=" << vis_pos
              << ", confidence=" << vis_conf
              << ", frame_id=" << vis_frame);
            // 存储到序列级 key，供同序列的 move_pose/move_joints 使用
            act_client.store_vision_result_at_seq_key(seq_key, vis_res);
          }
        }

        act_client.reset_action_state(state_key);
        return BTStatus::SUCCESS;
      }
      else if (act_client.is_action_failure(state_key))
      {
        task_act::ActionCallResult detail = act_client.get_last_call_result(state_key);
        LOG_ERROR(logger, task_log::Tag::BT, "[Branch " << branch_index << "] Action [" << atomic_act->action_type << "] execution failed: " << task_act::action_call_result_to_str(detail));
        return action_result_to_bt_status(detail);
      }

      LOG_INFO_TITLE(logger, task_log::Tag::BT, "===== [Branch " << branch_index << "] Execute Action Type: " << atomic_act->action_type << " =====");
      if (atomic_act->action_type == "vision_detect")
      {
        LOG_INFO(logger, task_log::Tag::BT, "[Branch " << branch_index << "] VisionParam -> target: " << atomic_act->vision_param.target_obj_name
          << ", frame_id: " << atomic_act->vision_param.expected_frame_id);
      }
      else if (atomic_act->action_type == "move_pose")
      {
        LOG_INFO(logger, task_log::Tag::BT, "[Branch " << branch_index << "] MovePoseParam -> arm_id: " << static_cast<int>(atomic_act->arm_param.arm_id)
          << ", collision_check: " << (atomic_act->arm_param.enable_collision_check ? "true" : "false")
          << ", speed: " << atomic_act->arm_param.speed << ", timeout_ms: " << atomic_act->arm_param.timeout_ms);
      }
      else if (atomic_act->action_type == "move_joints")
      {
        LOG_INFO(logger, task_log::Tag::BT, "[Branch " << branch_index << "] MoveJointsParam -> arm_id: " << static_cast<int>(atomic_act->move_joints_param.arm_id)
          << ", collision_check: " << (atomic_act->move_joints_param.enable_collision_check ? "true" : "false")
          << ", speed: " << atomic_act->move_joints_param.speed << ", timeout_ms: " << atomic_act->move_joints_param.timeout_ms);
      }
      else if (atomic_act->action_type == "gripper_ctrl")
      {
        LOG_INFO(logger, task_log::Tag::BT, "[Branch " << branch_index << "] GripperParam -> gripper_id: " << static_cast<int>(atomic_act->gripper_param.gripper_id)
          << ", block: " << (atomic_act->gripper_param.block ? "true" : "false"));
      }
      else if (atomic_act->action_type == "control_suck")
      {
        std::ostringstream oss;
        oss << "[Branch " << branch_index << "] SuckParam -> suck_id: " << static_cast<int>(atomic_act->suck_param.suck_id)
            << ", active=[";
        for (size_t i = 0; i < atomic_act->suck_param.active.size(); ++i)
          oss << (i ? "," : "") << (atomic_act->suck_param.active[i] ? "true" : "false");
        oss << "], timeout_ms: " << atomic_act->suck_param.timeout_ms;
        LOG_INFO(logger, task_log::Tag::BT, oss.str());
      }
      else if (atomic_act->action_type == "vision_check_self")
      {
        LOG_INFO(logger, task_log::Tag::BT, "[Branch " << branch_index << "] VisionCheckSelfParam -> timeout_ms: " << atomic_act->vision_check_self_param.timeout_ms);
      }
      else if (atomic_act->action_type == "control_check_self")
      {
        LOG_INFO(logger, task_log::Tag::BT, "[Branch " << branch_index << "] ControlCheckSelfParam -> com_port[0]: " << static_cast<int>(atomic_act->control_check_self_param.com_port[0])
          << ", com_port[1]: " << static_cast<int>(atomic_act->control_check_self_param.com_port[1])
          << ", timeout_ms: " << atomic_act->control_check_self_param.timeout_ms);
      }
      else if (atomic_act->action_type == "vision_reset")
      {
        LOG_INFO(logger, task_log::Tag::BT, "[Branch " << branch_index << "] VisionResetParam -> timeout_ms: " << atomic_act->vision_reset_param.timeout_ms);
      }
      else if (atomic_act->action_type == "control_reset")
      {
        LOG_INFO(logger, task_log::Tag::BT, "[Branch " << branch_index << "] ControlResetParam -> timeout_ms: " << atomic_act->control_reset_param.timeout_ms);
      }
      else if (atomic_act->action_type == "move_compound")
      {
        std::ostringstream oss;
        oss << "[Branch " << branch_index << "] CompoundMoveParam -> sub_actions: "
            << atomic_act->compound_move_param.sub_actions.size()
            << ", timeout_ms: " << atomic_act->compound_move_param.timeout_ms << " [";
        for (size_t i = 0; i < atomic_act->compound_move_param.sub_actions.size(); ++i)
        {
          const auto& sub = atomic_act->compound_move_param.sub_actions[i];
          oss << (i ? ", " : "");
          if (sub.action_type == "move_pose")
          {
            oss << "move_pose(arm_id=" << static_cast<int>(sub.move_pose_param.arm_id) << ")";
          }
          else if (sub.action_type == "move_joints")
          {
            oss << "move_joints(arm_id=" << static_cast<int>(sub.move_joints_param.arm_id) << ")";
          }
          else
          {
            oss << sub.action_type;
          }
        }
        oss << "]";
        LOG_INFO(logger, task_log::Tag::BT, oss.str());
      }

      LOG_DEBUG(logger, task_log::Tag::BT, "[Branch " << branch_index << "] ActionClient state_key: " << state_key);

      // 使用 ActionCallResult 替代 bool
      task_act::ActionCallResult send_ret = act_client.send_goal(*atomic_act, state_key);
      if (!task_act::action_call_result_is_ok(send_ret))
      {
        LOG_ERROR(logger, task_log::Tag::BT, "[Branch " << branch_index << "] Action [" << atomic_act->action_type << "] send goal failed: " << task_act::action_call_result_to_str(send_ret));
        return action_result_to_bt_status(send_ret);
      }
      else
      {
        return BTStatus::RUNNING;
      }
    };

    return std::make_shared<ActionNode>(node_name, std::move(action_cb), logger);
}

/// @brief 从视觉反馈结果中提取指定字段的数值（double）
/// 支持字段（点分路径）：
///   - 新接口 DetectionPoseCloud（多目标，取 detected_targets[0] 作为主目标）：
///     - detected_targets[0].confidence
///     - detected_targets[0].pose.pose.position.x / .y / .z
///     - detected_targets[0].pose.pose.orientation.x / .y / .z / .w
///     - total_detected_count
///   - 旧接口 VisionDetection 兼容别名（保留，避免已有配置失效）：
///     - pose_confidence
///     - target_6d_pose.pose.position.x / .y / .z
///     - target_6d_pose.pose.orientation.x / .y / .z / .w
/// @param vis_res 视觉检测结果
/// @param field 字段名（点分路径）
/// @param logger ROS2 日志记录器
/// @return 提取的数值；字段不支持或目标列表为空时返回 0.0
double extract_feedback_field(
    const task_act::VisionDet::Result& vis_res,
    const std::string& field,
    rclcpp::Logger logger)
{
    // 新接口下取第一个检测目标作为主目标（多目标场景可扩展数组下标）
    const com_interfaces::msg::TargetDetection* tgt = nullptr;
    if (!vis_res.detected_targets.empty())
    {
        tgt = &vis_res.detected_targets[0];
    }

    if (field == "total_detected_count")
    {
        return static_cast<double>(vis_res.total_detected_count);
    }
    else if (field == "detected_targets[0].confidence" || field == "pose_confidence")
    {
        if (tgt == nullptr) { return 0.0; }
        return static_cast<double>(tgt->confidence);
    }
    else if (field == "detected_targets[0].pose.pose.position.x" || field == "target_6d_pose.pose.position.x")
    {
        if (tgt == nullptr) { return 0.0; }
        return static_cast<double>(tgt->pose.pose.position.x);
    }
    else if (field == "detected_targets[0].pose.pose.position.y" || field == "target_6d_pose.pose.position.y")
    {
        if (tgt == nullptr) { return 0.0; }
        return static_cast<double>(tgt->pose.pose.position.y);
    }
    else if (field == "detected_targets[0].pose.pose.position.z" || field == "target_6d_pose.pose.position.z")
    {
        if (tgt == nullptr) { return 0.0; }
        return static_cast<double>(tgt->pose.pose.position.z);
    }
    else if (field == "detected_targets[0].pose.pose.orientation.x" || field == "target_6d_pose.pose.orientation.x")
    {
        if (tgt == nullptr) { return 0.0; }
        return static_cast<double>(tgt->pose.pose.orientation.x);
    }
    else if (field == "detected_targets[0].pose.pose.orientation.y" || field == "target_6d_pose.pose.orientation.y")
    {
        if (tgt == nullptr) { return 0.0; }
        return static_cast<double>(tgt->pose.pose.orientation.y);
    }
    else if (field == "detected_targets[0].pose.pose.orientation.z" || field == "target_6d_pose.pose.orientation.z")
    {
        if (tgt == nullptr) { return 0.0; }
        return static_cast<double>(tgt->pose.pose.orientation.z);
    }
    else if (field == "detected_targets[0].pose.pose.orientation.w" || field == "target_6d_pose.pose.orientation.w")
    {
        if (tgt == nullptr) { return 0.0; }
        return static_cast<double>(tgt->pose.pose.orientation.w);
    }

    LOG_WARN(logger, task_log::Tag::BT, "FeedbackCheck: unsupported field '" << field << "', using value=0.0");
    return 0.0;
}

/// @brief 数值与阈值按运算符比较
/// 支持运算符：>=, >, <=, <, ==, !=
/// @param value 反馈数值
/// @param op 比较运算符
/// @param threshold 阈值
/// @param logger ROS2 日志记录器
/// @return 比较结果；运算符不支持时默认按 >= 处理
bool compare_feedback_value(double value, const std::string& op, double threshold, rclcpp::Logger logger)
{
    if (op == ">=")      { return value >= threshold; }
    else if (op == ">")  { return value > threshold; }
    else if (op == "<=") { return value <= threshold; }
    else if (op == "<")  { return value < threshold; }
    else if (op == "==") { return value == threshold; }
    else if (op == "!=") { return value != threshold; }

    LOG_WARN(logger, task_log::Tag::BT, "FeedbackCheck: unsupported operator '" << op << "', default to '>='");
    return value >= threshold;
}

/// @brief 构建反馈条件判断节点（ConditionNode）
/// 从序列级 key（seq_key）读取最近一次 vision_detect 结果的 check_field 字段，
/// 使用配置的比较运算符与阈值比较，满足条件返回 true。
/// 注意：条件必须从 seq_key 读取，因为 reset_action_state 会清除 state_key 上的视觉结果。
BTNode::Ptr build_confidence_condition_node(
    task_act::TaskActionClient& act_client,
    const std::string& state_key,
    const task_cfg::AtomicAction& atomic_act,
    rclcpp::Logger logger)
{
    std::string seq_key = task_act::derive_seq_key(state_key);
    std::string field = atomic_act.feedback_check.check_field;
    std::string op = atomic_act.feedback_check.operator_str;
    double threshold = atomic_act.feedback_check.threshold;

    auto cond_cb = [&act_client, seq_key, field, op, threshold, logger]() -> bool
    {
        task_act::VisionDet::Result vis_res;
        if (!act_client.get_vision_result_from_seq_key(seq_key, vis_res))
        {
            LOG_WARN(logger, task_log::Tag::BT, "FeedbackCheck: no vision result at seq_key=" << seq_key << ", condition not satisfied");
            return false;
        }

        double value = extract_feedback_field(vis_res, field, logger);
        bool satisfied = compare_feedback_value(value, op, threshold, logger);

        LOG_INFO(logger, task_log::Tag::BT, "FeedbackCheck[" << seq_key << "]: " << field << "=" << value << " " << op << " " << threshold << " -> " << (satisfied ? "PASS" : "FAIL"));
        return satisfied;
    };

    return std::make_shared<ConditionNode>("FeedbackCond_" + seq_key, std::move(cond_cb), logger);
}

/// @brief 构建单个调整动作的选择条件节点（ConditionNode）
/// 依据 RetryActionParam.when（反馈字段比较条件）判断是否采用该调整动作：
///   - when.enable 为 false：无条件采用（作为兜底/默认调整动作），恒返回 true
///   - when.enable 为 true：读取反馈字段并比较，满足条件才返回 true
/// @note 条件从 seq_key 读取（derive_seq_key 推导），与探测动作共享同一视觉结果。
BTNode::Ptr build_retry_condition_node(
    task_act::TaskActionClient& act_client,
    const std::string& state_key,
    const task_cfg::RetryActionParam& ra,
    rclcpp::Logger logger)
{
    std::string seq_key = task_act::derive_seq_key(state_key);
    const task_cfg::FeedbackCompareCondition& when = ra.when;

    auto cond_cb = [&act_client, seq_key, when, logger]() -> bool
    {
        if (!when.enable)
        {
            // 未配置 when：兜底调整动作，恒采用
            return true;
        }

        task_act::VisionDet::Result vis_res;
        if (!act_client.get_vision_result_from_seq_key(seq_key, vis_res))
        {
            LOG_WARN(logger, task_log::Tag::BT, "RetryCond: no vision result at seq_key=" << seq_key << ", skip this retry action");
            return false;
        }

        double value = extract_feedback_field(vis_res, when.field, logger);
        bool satisfied = compare_feedback_value(value, when.operator_str, when.threshold, logger);

        LOG_INFO(logger, task_log::Tag::BT, "RetryCond[" << seq_key << "]: " << when.field << "=" << value << " " << when.operator_str << " " << when.threshold << " -> " << (satisfied ? "MATCH" : "NO"));
        return satisfied;
    };

    return std::make_shared<ConditionNode>("RetryCond_" + seq_key, std::move(cond_cb), logger);
}

/// @brief 构建条件重试节点（RetryUntilConditionNode）
/// 将配置了 feedback_check 的原子动作包装成条件重试节点：
///   - 探测动作：该动作本身（复用其 state_key）
///   - 条件节点：读取反馈字段（如 pose_confidence）比较阈值
///   - 调整动作序列：feedback_check.retry_actions 中的动作（使用独立 state_key）
BTNode::Ptr build_feedback_check_node(
    task_act::TaskActionClient& act_client,
    const task_cfg::AtomicAction& atomic_act,
    const std::string& state_key,
    long branch_index,
    rclcpp::Logger logger)
{
    auto node = std::make_shared<RetryUntilConditionNode>("RetryCheck_" + state_key, logger);
    node->setMaxRetries(atomic_act.feedback_check.max_retries);
    node->setOnExhaust(atomic_act.feedback_check.on_exhaust == "pass");

    // 1. 探测动作：即该动作本身（复用原始 state_key）
    auto probe_act = std::make_shared<task_cfg::AtomicAction>(atomic_act);
    std::string probe_name = atomic_act.action_type + std::to_string(branch_index);
    node->setProbeAction(build_action_node(act_client, probe_act, state_key, branch_index, probe_name, logger));

    // 2. 反馈条件节点
    node->setConditionNode(build_confidence_condition_node(act_client, state_key, atomic_act, logger));

    // 3. 调整动作选择（条件不满足时按 when 条件挑选并执行）
    //    采用 FallbackNode：逐个尝试 SequenceNode[选择条件, 调整动作]，
    //    第一个条件满足（或未配置 when 的兜底动作）的分支会执行其调整动作，
    //    若该调整动作执行失败，则继续尝试下一个候选分支。
    auto retry_fallback = std::make_shared<FallbackNode>("RetryActions_" + state_key, logger);
    for (size_t j = 0; j < atomic_act.feedback_check.retry_actions.size(); ++j)
    {
        const task_cfg::RetryActionParam& ra = atomic_act.feedback_check.retry_actions[j];
        // 为调整动作生成独立 state_key，避免与主干动作冲突。
        // 注意：derive_seq_key 仍能推导出原序列级 key（如 "seq1"），
        // 因此调整动作中的 vision_detect 结果同样会存储到该序列级 key。
        std::string retry_state_key = "seq" + std::to_string(branch_index) + "_retry_" + std::to_string(j) + "_" + ra.action->action_type + "_0";
        std::string retry_name = "Retry_" + ra.action->action_type + "_" + retry_state_key;

        auto branch = std::make_shared<SequenceNode>(retry_name + "_Seq", logger);
        branch->addChild(build_retry_condition_node(act_client, retry_state_key, ra, logger));
        branch->addChild(build_action_node(act_client, ra.action, retry_state_key, branch_index, retry_name, logger));
        retry_fallback->addChild(branch);
    }
    node->setRetrySequence(retry_fallback);

    LOG_INFO(logger, task_log::Tag::BT, "FeedbackCheck node built for state_key=" << state_key
      << ", field=" << atomic_act.feedback_check.check_field
      << ", op=" << atomic_act.feedback_check.operator_str
      << ", threshold=" << atomic_act.feedback_check.threshold
      << ", max_retries=" << atomic_act.feedback_check.max_retries
      << ", retry_actions=" << atomic_act.feedback_check.retry_actions.size());
    return node;
}

/// @brief 结果分支序列递归构建（前向声明）
BTNode::Ptr build_branch_sequence(
    task_act::TaskActionClient& act_client,
    const std::vector<task_cfg::ResultActionParam>& branch_actions,
    const std::string& parent_key,
    const std::string& suffix,
    long branch_index,
    rclcpp::Logger logger,
    task_mgr::PreconditionManager* pm,
    const std::function<void()>& pre_fail_cb,
    int depth);

/// @brief 前置条件/结果分支包装（设计方案 §6.3）
/// 仅当动作配置了 preconditions / on_success / on_failure 时才包装为 GuardedActionNode；
/// 未配置时返回 nullptr（调用方保持原节点，完全向后兼容）。
/// 递归深度超过 kMaxBranchDepth 或前置条件已配置但未注入 PreconditionManager 时，
/// 告警并返回 nullptr（取消包装，防止配置错误导致无限递归或误放行）。
/// @param action_node 主动作叶子节点（ActionNode 或 RetryUntilConditionNode）
/// @param state_key 主动作状态 key（用于分支动作独立 state_key 生成）
/// @param depth 当前递归深度（分支嵌套每层 +1）
BTNode::Ptr wrap_guarded_action(
    task_act::TaskActionClient& act_client,
    const std::shared_ptr<task_cfg::AtomicAction>& atomic_act,
    BTNode::Ptr action_node,
    const std::string& state_key,
    long branch_index,
    const std::string& node_name,
    rclcpp::Logger logger,
    task_mgr::PreconditionManager* pm,
    const std::function<void()>& pre_fail_cb,
    int depth)
{
    // 未配置前置条件/结果分支：不包装
    if (!atomic_act->preconditions.has && !atomic_act->on_success.has && !atomic_act->on_failure.has)
    {
        return nullptr;
    }

    // 递归深度保护（防止配置错误导致无限递归）
    if (depth > kMaxBranchDepth)
    {
        LOG_ERROR(logger, task_log::Tag::BT, "wrap_guarded_action: branch depth exceeds limit "
            << kMaxBranchDepth << ", unwrapping node [" << node_name << "]");
        return nullptr;
    }

    // 前置条件已配置但未注入 PreconditionManager：无法评估，告警并整组取消包装
    if (atomic_act->preconditions.has && !pm)
    {
        LOG_WARN(logger, task_log::Tag::BT, "wrap_guarded_action: preconditions configured but "
            "PreconditionManager not injected, guarding disabled for node [" << node_name << "]");
        return nullptr;
    }

    auto guarded = std::make_shared<GuardedActionNode>(node_name + "_Guarded", logger);

    // 1. 前置条件门
    if (atomic_act->preconditions.has && pm)
    {
        auto gate = std::make_shared<PreconditionGateNode>(node_name + "_Gate", logger);
        gate->setConfig(atomic_act->preconditions);
        auto pre_copy = atomic_act->preconditions;
        task_mgr::PreconditionManager* pm_ptr = pm;
        task_act::TaskActionClient* act_ptr = &act_client;
        gate->setEvaluator([pm_ptr, act_ptr, pre_copy]() -> task_mgr::PrecheckResult
        {
            return pm_ptr->evaluate(pre_copy, *act_ptr);
        });
        guarded->setPreconditionGate(std::move(gate));
    }

    // 2. 主动作叶子节点
    guarded->setAction(std::move(action_node));

    // 3. 结果分支（递归包装，独立 state_key：父 key + "_success"/"_failure" + 序号）
    if (atomic_act->on_success.has && !atomic_act->on_success.actions.empty())
    {
        guarded->setOnSuccessBranch(build_branch_sequence(
            act_client, atomic_act->on_success.actions, state_key, "success",
            branch_index, logger, pm, pre_fail_cb, depth));
    }
    if (atomic_act->on_failure.has && !atomic_act->on_failure.actions.empty())
    {
        guarded->setOnFailureBranch(build_branch_sequence(
            act_client, atomic_act->on_failure.actions, state_key, "failure",
            branch_index, logger, pm, pre_fail_cb, depth));
    }

    guarded->setBranchPolicy(atomic_act->on_success.policy, atomic_act->on_failure.policy);
    guarded->setPreconditionFailCb(pre_fail_cb);

    LOG_INFO(logger, task_log::Tag::BT, "GuardedActionNode built: [" << node_name << "]"
        << " preconditions=" << (atomic_act->preconditions.has ? "yes" : "no")
        << " on_success=" << (atomic_act->on_success.has ? "yes" : "no")
        << " on_failure=" << (atomic_act->on_failure.has ? "yes" : "no")
        << " depth=" << depth);

    return guarded;
}

/// @brief 递归构建结果分支序列（on_success / on_failure 分支动作）
/// 分支动作同样用 wrap_guarded_action 包装，支持递归嵌套 preconditions / 结果分支。
/// state_key 约定：父状态 key + "_success"/"_failure" + 分支内序号。
BTNode::Ptr build_branch_sequence(
    task_act::TaskActionClient& act_client,
    const std::vector<task_cfg::ResultActionParam>& branch_actions,
    const std::string& parent_key,
    const std::string& suffix,
    long branch_index,
    rclcpp::Logger logger,
    task_mgr::PreconditionManager* pm,
    const std::function<void()>& pre_fail_cb,
    int depth)
{
    auto seq = std::make_shared<SequenceNode>(parent_key + "_" + suffix + "_Seq", logger);
    size_t j = 0;
    for (const auto& ra : branch_actions)
    {
        if (!ra.action)
        {
            LOG_ERROR(logger, task_log::Tag::BT, "Branch action is null in [" << parent_key << "_" << suffix << "]");
            ++j;
            continue;
        }
        std::string branch_state_key = parent_key + "_" + suffix + "_" + std::to_string(j);
        std::string branch_name = ra.action->action_type + "_" + suffix + "_" + std::to_string(j);
        auto branch_act = std::make_shared<task_cfg::AtomicAction>(*ra.action);

        // 先构建分支动作叶子节点（支持 feedback_check 条件重试）
        BTNode::Ptr leaf;
        if (ra.action->feedback_check.enable)
        {
            leaf = build_feedback_check_node(act_client, *ra.action, branch_state_key, branch_index, logger);
        }
        else
        {
            leaf = build_action_node(act_client, branch_act, branch_state_key, branch_index, branch_name, logger);
        }

        // 再递归包装前置条件/结果分支（未配置时 wrap_guarded_action 返回 nullptr，保持叶子节点）
        BTNode::Ptr wrapped = wrap_guarded_action(act_client, branch_act, leaf, branch_state_key,
                                                  branch_index, branch_name, logger, pm, pre_fail_cb, depth + 1);
        seq->addChild(wrapped ? wrapped : leaf);
        ++j;
    }
    return seq;
}

} // anonymous namespace

// ============================================================================
// 串行动作序列加载
// ============================================================================
void BehaviorTreeEngine::load_action_group(
    const std::vector<task_cfg::AtomicAction>& action_list,
    task_act::TaskActionClient& act_client)
{
  auto sequence_node = std::make_shared<SequenceNode>("Action_Group_Root", logger_);
  sequence_node->clearAllChildren();
  for (size_t i = 0; i < action_list.size(); i++)
  {
    auto atomic_act = std::make_shared<task_cfg::AtomicAction>(action_list[i]);
    std::string node_name = "Action_" + atomic_act->action_type;
    LOG_INFO_FLOW(logger_, task_log::Tag::BT, "Loading action node: " << node_name);
    auto action_cb = [&act_client, atomic_act, this]() -> BTStatus
    {
       // 适配客户端新接口
      if (act_client.is_action_running())
      {
        return check_action_running(act_client, atomic_act->action_type, logger_);
      }
      else if (act_client.is_action_success())
      {
        return check_action_success(act_client, atomic_act->action_type, logger_);
      }
      else if (act_client.is_failure())
      {
        return check_action_failure(act_client, atomic_act->action_type, logger_);
      }
     
      // IDLE 状态：发送 goal
      return send_action_goal(act_client, *atomic_act, atomic_act->action_type, logger_);
    };

    auto act_node = std::make_shared<ActionNode>(node_name, std::move(action_cb), logger_);
    // 前置条件/结果分支包装（未配置时 wrap_guarded_action 返回 nullptr，保持原节点，完全向后兼容）
    // 串行路径主动作使用默认状态 key；state_key 仅用于生成分支动作的独立状态 key
    std::string state_key = "seq0_" + atomic_act->action_type + "_" + std::to_string(i);
    auto wrapped = wrap_guarded_action(act_client, atomic_act, act_node, state_key, 0, node_name,
                                       logger_, precondition_mgr_, precondition_fail_cb_, 0);
    // 安全添加子节点，不再直接访问 protected children_
    sequence_node->addChild(wrapped ? wrapped : act_node);
  }
  this->setRoot(std::move(sequence_node));
  LOG_SUCCESS(logger_, task_log::Tag::BT, "Action group loaded, total actions: " << action_list.size());
}

// ============================================================================
// 并行动作序列加载
// ============================================================================
void BehaviorTreeEngine::load_parallel_task(std::vector<std::vector<task_cfg::AtomicAction>> & parallel_action_groups, task_act::TaskActionClient& act_client)
{
    auto parallel_root = std::make_shared<ParallelNode>("Parallel_Task_Root", logger_);
    long Sequence_index = 0;
    for (const auto& single_group : parallel_action_groups) {
        Sequence_index++;
        LOG_INFO_FLOW(logger_, task_log::Tag::BT, "Loading parallel branch [" << Sequence_index << "] with " << single_group.size() << " actions");
        
        // 每个分组生成一个独立的Sequence顺序节点，作为并行分支
        auto group_seq = std::make_shared<SequenceNode>("Sequence_Branch_" + std::to_string(Sequence_index), logger_);
        for (size_t i = 0; i < single_group.size(); i++) {
            auto atomic_act = std::make_shared<task_cfg::AtomicAction>(single_group[i]);
            // 生成唯一状态 key：seq{序号}_{动作类型}_{索引}
            std::string state_key = "seq" + std::to_string(Sequence_index) + "_" + atomic_act->action_type + "_" + std::to_string(i);
            // 记录当前分支序号，用于日志输出
            long branch_index = Sequence_index;
            LOG_INFO(logger_, task_log::Tag::BT, "[Branch " << branch_index << "] Action [" << atomic_act->action_type << "] added");
            auto node_name = atomic_act->action_type + std::to_string(Sequence_index);

            // 若该动作配置了 feedback_check（反馈条件检查），则构建条件重试节点：
            // 动作执行成功后检查反馈置信度（如 pose_confidence），
            // 若不满足则先执行调整动作序列，再重新执行该动作，直至条件满足或重试次数用尽。
            if (atomic_act->feedback_check.enable)
            {
                auto retry_node = build_feedback_check_node(act_client, *atomic_act, state_key, branch_index, logger_);
                // 前置条件/结果分支包装（未配置时 wrap_guarded_action 返回 nullptr，保持原节点）
                auto wrapped = wrap_guarded_action(act_client, atomic_act, retry_node, state_key,
                                                   branch_index, node_name, logger_,
                                                   precondition_mgr_, precondition_fail_cb_, 0);
                group_seq->addChild(wrapped ? wrapped : retry_node);
                continue;
            }

            // 普通动作：构建标准 ActionNode（复用统一构建逻辑）
            auto act_node = build_action_node(act_client, atomic_act, state_key, branch_index, node_name, logger_);
            // 前置条件/结果分支包装（未配置时 wrap_guarded_action 返回 nullptr，保持原节点）
            auto wrapped = wrap_guarded_action(act_client, atomic_act, act_node, state_key,
                                               branch_index, node_name, logger_,
                                               precondition_mgr_, precondition_fail_cb_, 0);
            group_seq->addChild(wrapped ? wrapped : act_node);
        }
        parallel_root->addChild(group_seq);
    }
    
    this->setRoot(parallel_root);
    
    LOG_SUCCESS(logger_, task_log::Tag::BT, "Parallel task tree loaded, total branches: " << parallel_action_groups.size());
}
