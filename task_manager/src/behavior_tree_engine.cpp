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
#include <rclcpp/logging.hpp>
#include <iostream>

// 构造函数实现
FallbackNode::FallbackNode(const std::string& name, rclcpp::Logger logger)
    : BTNode(name), current_child_idx_(0), logger_(std::move(logger)) {
}

// tick 方法实现
BTStatus FallbackNode::tick() {

    
    // 遍历子节点，从当前索引开始
    for (; current_child_idx_ < children_.size(); ++current_child_idx_) {
        auto& child = children_[current_child_idx_];
        
        // 执行子节点
        BTStatus child_status = child->tick();

        if (child_status == BTStatus::RUNNING) {
            status_ = BTStatus::RUNNING;
            return BTStatus::RUNNING;
        } 
        else if (child_status == BTStatus::SUCCESS) {
            current_child_idx_ = 0; // 成功则重置索引，下次从头开始
            status_ = BTStatus::SUCCESS;
            return BTStatus::SUCCESS;
        }
        // 如果是 FAILURE，循环继续，尝试下一个子节点
    }

    // 所有子节点都失败
    current_child_idx_ = 0; // 重置索引
    status_ = BTStatus::FAILURE;
    return BTStatus::FAILURE;
}

// reset 方法实现
void FallbackNode::reset() {
    current_child_idx_ = 0;
    BTNode::reset(); // 调用基类的 reset
}


ActionNode::ActionNode(const std::string& name,
                       ActionCallback callback,
                       rclcpp::Logger logger)
    : BTNode(name),
      callback_(std::move(callback)),
      logger_(std::move(logger))
{
}

BTStatus ActionNode::tick()
{

    if (!callback_)
    {
        LOG_ERROR(logger_, task_log::Tag::BT, "ActionNode [" << name_ << "] has null callback");
        return BTStatus::FAILURE;
    }
    status_ = callback_();
    return status_;
}


ConditionNode::ConditionNode(const std::string& name, 
                                  ConditionCallback callback, 
                                  rclcpp::Logger logger)
: BTNode(name), 
  callback_(callback),  
  logger_(std::move(logger))
  {

  }



 BTStatus ConditionNode::tick() 
 {
    LOG_DEBUG(logger_, task_log::Tag::BT, "ConditionNode [" << name_ << "] tick");
    if (!callback_)
    {
        LOG_ERROR(logger_, task_log::Tag::BT, "ConditionNode [" << name_ << "] has null callback");
        return BTStatus::FAILURE;
    }
    // 条件节点通常瞬间完成，不返回 RUNNING
    status_ = callback_() ? BTStatus::SUCCESS : BTStatus::FAILURE;
    return status_;
}

// 构造函数实现
SequenceNode::SequenceNode(const std::string& name, rclcpp::Logger logger)
    : BTNode(name), current_child_idx_(0), logger_(std::move(logger)) {
}

// tick 函数实现
BTStatus SequenceNode::tick() {

   
    // 从当前索引开始遍历子节点
    for (; current_child_idx_ < children_.size(); ++current_child_idx_) {
        auto& child = children_[current_child_idx_];
        
        // 执行子节点
        BTStatus child_status = child->tick();
        
        if (child_status == BTStatus::RUNNING) {
            // 如果子节点正在运行，保持当前索引，下次继续执行该子节点
            status_ = BTStatus::RUNNING;
            return BTStatus::RUNNING;
        } else if (child_status == BTStatus::FAILURE) {
            // 如果子节点失败，整个序列失败，重置索引以便下次重新从头开始
            LOG_WARN(logger_, task_log::Tag::BT, "SequenceNode [" << name_ << "] child [" << child->getName() << "] failed");
            status_ = BTStatus::FAILURE;
            current_child_idx_ = 0;
            return BTStatus::FAILURE;
        } else if (child_status == BTStatus::SUCCESS) {
            continue;
        }
        // 如果是 SUCCESS，循环继续，执行下一个子节点
    }
    LOG_SUCCESS(logger_, task_log::Tag::BT, "SequenceNode [" << name_ << "] all children executed successfully");
    // 所有子节点都成功执行完毕
    current_child_idx_ = 0; // 重置索引，为下一次执行做准备


    status_ = BTStatus::SUCCESS;
    return BTStatus::SUCCESS;
}

// reset 函数实现
void SequenceNode::reset() {
    current_child_idx_ = 0;
    BTNode::reset(); // 调用父类的重置方法
}

// 清空所有子节点
void SequenceNode::clearAllChildren()
{
    BTNode::children_.clear();
    current_child_idx_ = 0;
}


ParallelNode::ParallelNode(const std::string& name, rclcpp::Logger logger)
    : BTNode(name), logger_(std::move(logger))
{
    // 分支状态在 addChild 和 reset 时初始化，构造函数中 children_ 为空无需操作
}

BTStatus ParallelNode::tick()
{
    
    bool all_success = true;

    for (auto& child : children_) {

        
        const std::string& child_name = child->getName();
        auto current_status = branch_status_[child_name];
       
        // 已经执行完成的分支跳过，不需要重复tick
        if (current_status == BTStatus::SUCCESS || current_status == BTStatus::FAILURE) {
            continue;
        }
        
        // 驱动当前活跃分支执行tick

        //child->name_ = child_name;
        BTStatus child_run_status = child->tick();
        branch_status_[child_name] = child_run_status;

        // 任意分支返回失败，立刻标记全局失败
        if (child_run_status == BTStatus::FAILURE) {
            LOG_ERROR(logger_, task_log::Tag::BT, "Parallel branch [" << child_name << "] failed, whole parallel node abort");
            status_ = BTStatus::FAILURE;
            return BTStatus::FAILURE;
        }
        // 当前分支还在运行，标记整体还未全部成功
        if (child_run_status != BTStatus::SUCCESS) {
            all_success = false;
        }
    }

    // 所有分支全部执行成功，返回成功状态
    if (all_success) {
        LOG_SUCCESS(logger_, task_log::Tag::BT, "All parallel branches executed successfully");
        status_ = BTStatus::SUCCESS;
        return BTStatus::SUCCESS;
    }

    // 仍有分支处于运行态，返回RUNNING
    status_ = BTStatus::RUNNING;
    return BTStatus::RUNNING;
}

void ParallelNode::reset()
{
    // 重置所有分支的状态，递归调用所有子节点的reset
    branch_status_.clear();
    for (auto& child : children_) {
        child->reset();
        branch_status_[child->getName()] = BTStatus::IDLE;
    }
    BTNode::reset();
}

void ParallelNode::addChild(BTNode::Ptr child)
{
    if (!child) {
        LOG_ERROR(logger_, task_log::Tag::BT, "Cannot add null child to ParallelNode");
        return;
    }
    
    // 直接拷贝共享指针，局部变量 child 依然有效
    children_.push_back(child);
    
    // 安全访问
    branch_status_[child->getName()] = BTStatus::IDLE;
}



BehaviorTreeEngine::BehaviorTreeEngine(rclcpp::Logger logger)
: logger_(std::move(logger)),
  root_node_(nullptr),
  running_(false),
  last_status_(BTStatus::IDLE)
{}

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
    LOG_INFO(logger, task_log::Tag::BT, "VisionParam -> target: " << atomic_act.vision_param.target_obj_name << ", need_6d: " << (atomic_act.vision_param.need_6d_pose ? "true" : "false"));
    LOG_INFO(logger, task_log::Tag::BT, "ArmParam -> arm_id: " << atomic_act.arm_param.arm_id << ", use_planning: " << (atomic_act.arm_param.use_planning ? "true" : "false"));
    LOG_INFO(logger, task_log::Tag::BT, "GripperParam -> gripper_id: " << (int)atomic_act.gripper_param.gripper_id << ", block: " << (atomic_act.gripper_param.block ? "true" : "false"));

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
    // 安全添加子节点，不再直接访问 protected children_
    sequence_node->addChild(std::move(act_node));
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
            auto action_cb = [&act_client, atomic_act, state_key, branch_index, this]() -> BTStatus
            {
              // 使用独立状态 key 查询，不干扰其他并行分支的动作状态
              if (act_client.is_action_running(state_key))
              {
                return check_action_running(act_client, atomic_act->action_type, logger_, state_key);
              }
              else if (act_client.is_action_success(state_key))
              {
                LOG_SUCCESS(logger_, task_log::Tag::BT, "[Branch " << branch_index << "] Action [" << atomic_act->action_type << "] executed successfully");
                act_client.reset_action_state(state_key);
                return BTStatus::SUCCESS;

              }
              else if (act_client.is_action_failure(state_key))
              {
                task_act::ActionCallResult detail = act_client.get_last_call_result(state_key);
                LOG_ERROR(logger_, task_log::Tag::BT, "[Branch " << branch_index << "] Action [" << atomic_act->action_type << "] execution failed: " << task_act::action_call_result_to_str(detail));
                return action_result_to_bt_status(detail);
              }
            

              LOG_INFO_TITLE(logger_, task_log::Tag::BT, "===== [Branch " << branch_index << "] Execute Action Type: " << atomic_act->action_type << " =====");
              LOG_INFO(logger_, task_log::Tag::BT, "[Branch " << branch_index << "] VisionParam -> target: " << atomic_act->vision_param.target_obj_name << ", need_6d: " << (atomic_act->vision_param.need_6d_pose ? "true" : "false"));
              LOG_INFO(logger_, task_log::Tag::BT, "[Branch " << branch_index << "] ArmParam -> arm_id: " << atomic_act->arm_param.arm_id << ", use_planning: " << (atomic_act->arm_param.use_planning ? "true" : "false"));
              LOG_INFO(logger_, task_log::Tag::BT, "[Branch " << branch_index << "] GripperParam -> gripper_id: " << (int)atomic_act->gripper_param.gripper_id << ", block: " << (atomic_act->gripper_param.block ? "true" : "false"));
              
              LOG_DEBUG(logger_, task_log::Tag::BT, "[Branch " << branch_index << "] ActionClient state_key: " << state_key);
              
              // 使用 ActionCallResult 替代 bool
              task_act::ActionCallResult send_ret = act_client.send_goal(*atomic_act, state_key);
              if (!task_act::action_call_result_is_ok(send_ret))
              {
                LOG_ERROR(logger_, task_log::Tag::BT, "[Branch " << branch_index << "] Action [" << atomic_act->action_type << "] send goal failed: " << task_act::action_call_result_to_str(send_ret));
                return action_result_to_bt_status(send_ret);
              }
              else
              {
                return BTStatus::RUNNING;
              }
            };
            
            auto node_name = atomic_act->action_type;
            auto act_node = std::make_shared<ActionNode>(node_name + std::to_string(Sequence_index), std::move(action_cb), logger_);
            group_seq->addChild(act_node);
        }
        parallel_root->addChild(group_seq);
    }
    
    this->setRoot(parallel_root);
    
    LOG_SUCCESS(logger_, task_log::Tag::BT, "Parallel task tree loaded, total branches: " << parallel_action_groups.size());
}
