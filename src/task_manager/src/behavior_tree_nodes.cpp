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

// ============================================================================
// 条件重试节点实现
// ============================================================================
RetryUntilConditionNode::RetryUntilConditionNode(const std::string& name, rclcpp::Logger logger)
    : BTNode(name), logger_(std::move(logger))
{
}

void RetryUntilConditionNode::setProbeAction(BTNode::Ptr probe)
{
    probe_ = std::move(probe);
}

void RetryUntilConditionNode::setConditionNode(BTNode::Ptr condition)
{
    condition_ = std::move(condition);
}

void RetryUntilConditionNode::setRetrySequence(BTNode::Ptr retry_seq)
{
    retry_seq_ = std::move(retry_seq);
}

void RetryUntilConditionNode::setMaxRetries(int32_t max_retries)
{
    max_retries_ = max_retries > 0 ? max_retries : 0;
}

void RetryUntilConditionNode::setOnExhaust(bool pass_on_exhaust)
{
    pass_on_exhaust_ = pass_on_exhaust;
}

BTStatus RetryUntilConditionNode::tick()
{
    switch (phase_)
    {
        case RetryPhase::PROBE:
        {
            if (!probe_)
            {
                LOG_ERROR(logger_, task_log::Tag::BT, "RetryUntilConditionNode [" << name_ << "] has null probe action");
                phase_ = RetryPhase::DONE;
                status_ = BTStatus::FAILURE;
                return status_;
            }
            BTStatus probe_status = probe_->tick();
            if (probe_status == BTStatus::RUNNING)
            {
                status_ = BTStatus::RUNNING;
                return status_;
            }
            if (probe_status == BTStatus::FAILURE)
            {
                LOG_ERROR(logger_, task_log::Tag::BT, "RetryUntilConditionNode [" << name_ << "] probe action failed");
                phase_ = RetryPhase::DONE;
                status_ = BTStatus::FAILURE;
                return status_;
            }
            // 探测成功，进入条件检查阶段
            LOG_INFO(logger_, task_log::Tag::BT, "RetryUntilConditionNode [" << name_ << "] probe succeeded, checking feedback condition");
            phase_ = RetryPhase::CHECK;
            status_ = BTStatus::RUNNING;
            return status_;
        }

        case RetryPhase::CHECK:
        {
            if (!condition_)
            {
                LOG_ERROR(logger_, task_log::Tag::BT, "RetryUntilConditionNode [" << name_ << "] has null condition node");
                phase_ = RetryPhase::DONE;
                status_ = BTStatus::FAILURE;
                return status_;
            }
            BTStatus cond_status = condition_->tick();
            if (cond_status == BTStatus::SUCCESS)
            {
                LOG_SUCCESS(logger_, task_log::Tag::BT, "RetryUntilConditionNode [" << name_ << "] feedback condition satisfied, node succeeds");
                phase_ = RetryPhase::DONE;
                status_ = BTStatus::SUCCESS;
                return status_;
            }
            // 条件不满足：执行调整动作后重试（重试次数未用尽）
            if (retry_count_ < max_retries_)
            {
                retry_count_++;
                LOG_WARN(logger_, task_log::Tag::BT, "RetryUntilConditionNode [" << name_ << "] condition not satisfied, retry " << retry_count_ << "/" << max_retries_);
                phase_ = RetryPhase::RETRY;
                status_ = BTStatus::RUNNING;
                return status_;
            }
            // 重试次数用尽
            LOG_WARN(logger_, task_log::Tag::BT, "RetryUntilConditionNode [" << name_ << "] retries exhausted, result=" << (pass_on_exhaust_ ? "PASS(forced)" : "FAIL"));
            phase_ = RetryPhase::DONE;
            status_ = pass_on_exhaust_ ? BTStatus::SUCCESS : BTStatus::FAILURE;
            return status_;
        }

        case RetryPhase::RETRY:
        {
            if (!retry_seq_)
            {
                LOG_ERROR(logger_, task_log::Tag::BT, "RetryUntilConditionNode [" << name_ << "] has null retry sequence");
                phase_ = RetryPhase::DONE;
                status_ = BTStatus::FAILURE;
                return status_;
            }
            BTStatus retry_status = retry_seq_->tick();
            if (retry_status == BTStatus::RUNNING)
            {
                status_ = BTStatus::RUNNING;
                return status_;
            }
            if (retry_status == BTStatus::FAILURE)
            {
                LOG_ERROR(logger_, task_log::Tag::BT, "RetryUntilConditionNode [" << name_ << "] retry action sequence failed");
                phase_ = RetryPhase::DONE;
                status_ = BTStatus::FAILURE;
                return status_;
            }
            // 调整动作完成，重新执行探测动作
            LOG_INFO(logger_, task_log::Tag::BT, "RetryUntilConditionNode [" << name_ << "] retry actions done, re-running probe");
            phase_ = RetryPhase::PROBE;
            status_ = BTStatus::RUNNING;
            return status_;
        }

        case RetryPhase::DONE:
        default:
        {
            return status_;
        }
    }
}

void RetryUntilConditionNode::reset()
{
    retry_count_ = 0;
    phase_ = RetryPhase::PROBE;
    status_ = BTStatus::IDLE;
    if (probe_) probe_->reset();
    if (condition_) condition_->reset();
    if (retry_seq_) retry_seq_->reset();
}

// ============================================================================
// 前置条件门节点实现
// ============================================================================
PreconditionGateNode::PreconditionGateNode(const std::string& name, rclcpp::Logger logger)
    : BTNode(name), logger_(std::move(logger))
{
}

void PreconditionGateNode::setConfig(const task_cfg::PreconditionList& pre)
{
    pre_ = pre;
    configured_ = true;
}

void PreconditionGateNode::setEvaluator(PreEvalFn fn)
{
    eval_ = std::move(fn);
}

BTStatus PreconditionGateNode::tick()
{
    if (!configured_ || !eval_)
    {
        LOG_ERROR(logger_, task_log::Tag::BT, "PreconditionGateNode [" << name_ << "] not configured (config or evaluator missing)");
        gate_result_ = GateResult::FAILED;
        status_ = BTStatus::FAILURE;
        return status_;
    }

    // 已结算则保持结果
    if (gate_result_ != GateResult::NONE)
    {
        return status_;
    }

    // 首次 tick 初始化等待计时
    if (wait_start_ == std::chrono::steady_clock::time_point{})
    {
        wait_start_ = std::chrono::steady_clock::now();
        last_check_ = wait_start_;
    }

    const auto now = std::chrono::steady_clock::now();

    // WAIT 模式：先检查是否超时（即使处于节流周期内也要按时结算）
    if (pre_.policy == task_cfg::PrePolicy::WAIT && pre_.max_wait_ms > 0)
    {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - wait_start_);
        if (elapsed.count() >= pre_.max_wait_ms)
        {
            // 门级超时行为：取第一个启用条件的 on_timeout（同一列表内条件通常配置一致），缺省 fail
            task_cfg::PreTimeoutAction timeout_action = task_cfg::PreTimeoutAction::FAIL;
            for (const auto& cond : pre_.conditions)
            {
                if (cond.enable)
                {
                    timeout_action = cond.on_timeout;
                    break;
                }
            }
            switch (timeout_action)
            {
                case task_cfg::PreTimeoutAction::PASS:
                    LOG_WARN(logger_, task_log::Tag::BT, "PreconditionGateNode [" << name_ << "] wait timeout, forced PASS");
                    gate_result_ = GateResult::PASSED;
                    status_ = BTStatus::SUCCESS;
                    break;
                case task_cfg::PreTimeoutAction::ABORT:
                    LOG_ERROR(logger_, task_log::Tag::BT, "PreconditionGateNode [" << name_ << "] wait timeout, ABORT");
                    gate_result_ = GateResult::ABORTED;
                    status_ = BTStatus::FAILURE;
                    break;
                case task_cfg::PreTimeoutAction::FAIL:
                default:
                    LOG_ERROR(logger_, task_log::Tag::BT, "PreconditionGateNode [" << name_ << "] wait timeout, FAIL");
                    gate_result_ = GateResult::FAILED;
                    status_ = BTStatus::FAILURE;
                    break;
            }
            return status_;
        }
    }

    // WAIT 模式按 check_interval_ms 节流评估；其余策略每次评估
    bool need_eval = true;
    if (pre_.policy == task_cfg::PrePolicy::WAIT && pre_.check_interval_ms > 0)
    {
        const auto interval = std::chrono::milliseconds(pre_.check_interval_ms);
        if ((now - last_check_) < interval)
        {
            need_eval = false;
        }
    }

    if (need_eval)
    {
        last_check_ = now;
        task_mgr::PrecheckResult result = eval_();
        if (result.satisfied)
        {
            LOG_INFO(logger_, task_log::Tag::BT, "PreconditionGateNode [" << name_ << "] conditions satisfied, gate passed");
            gate_result_ = GateResult::PASSED;
            status_ = BTStatus::SUCCESS;
            return status_;
        }

        switch (pre_.policy)
        {
            case task_cfg::PrePolicy::FAIL_FAST:
                LOG_ERROR(logger_, task_log::Tag::BT, "PreconditionGateNode [" << name_ << "] conditions not satisfied (fail_fast): " << result.fail_reason);
                gate_result_ = GateResult::FAILED;
                status_ = BTStatus::FAILURE;
                return status_;

            case task_cfg::PrePolicy::SKIP:
                LOG_WARN(logger_, task_log::Tag::BT, "PreconditionGateNode [" << name_ << "] conditions not satisfied (skip), action skipped: " << result.fail_reason);
                gate_result_ = GateResult::SKIPPED;
                status_ = BTStatus::SUCCESS;
                return status_;

            case task_cfg::PrePolicy::WAIT:
            default:
                // 继续等待（超时已在上面处理）
                status_ = BTStatus::RUNNING;
                return status_;
        }
    }

    // 节流期间：继续等待
    status_ = BTStatus::RUNNING;
    return status_;
}

void PreconditionGateNode::reset()
{
    gate_result_ = GateResult::NONE;
    wait_start_ = {};
    last_check_ = {};
    status_ = BTStatus::IDLE;
    BTNode::reset();
}

// ============================================================================
// 前置条件门 + 主动作 + 结果分支 包装节点实现
// ============================================================================
GuardedActionNode::GuardedActionNode(const std::string& name, rclcpp::Logger logger)
    : BTNode(name), logger_(std::move(logger))
{
}

void GuardedActionNode::setPreconditionGate(BTNode::Ptr gate)
{
    gate_ = std::move(gate);
}

void GuardedActionNode::setAction(BTNode::Ptr action)
{
    action_ = std::move(action);
}

void GuardedActionNode::setOnSuccessBranch(BTNode::Ptr branch)
{
    on_success_ = std::move(branch);
}

void GuardedActionNode::setOnFailureBranch(BTNode::Ptr branch)
{
    on_failure_ = std::move(branch);
}

void GuardedActionNode::setBranchPolicy(task_cfg::ResultPolicy on_success_policy,
                                        task_cfg::ResultPolicy on_failure_policy)
{
    // UNKNOWN → 按 continue / fail 缺省处理
    on_success_policy_ = (on_success_policy == task_cfg::ResultPolicy::UNKNOWN)
                             ? task_cfg::ResultPolicy::BRANCH_CONTINUE
                             : on_success_policy;
    on_failure_policy_ = (on_failure_policy == task_cfg::ResultPolicy::UNKNOWN)
                             ? task_cfg::ResultPolicy::BRANCH_FAIL
                             : on_failure_policy;
}

void GuardedActionNode::setPreconditionFailCb(std::function<void()> cb)
{
    pre_fail_cb_ = std::move(cb);
}

BTStatus GuardedActionNode::tick()
{
    switch (phase_)
    {
        case Phase::GATE_CHECK:
        {
            if (!gate_)
            {
                // 无前置条件门：直接执行主动作
                phase_ = Phase::EXECUTE;
                status_ = BTStatus::RUNNING;
                return status_;
            }
            BTStatus gate_status = gate_->tick();
            if (gate_status == BTStatus::RUNNING)
            {
                status_ = BTStatus::RUNNING;
                return status_;
            }
            if (gate_status == BTStatus::FAILURE)
            {
                // 门失败/中止：触发失败回调，本动作失败
                if (pre_fail_cb_)
                {
                    pre_fail_cb_();
                }
                auto gate_node = std::dynamic_pointer_cast<PreconditionGateNode>(gate_);
                if (gate_node && gate_node->getGateResult() == PreconditionGateNode::GateResult::ABORTED)
                {
                    LOG_ERROR(logger_, task_log::Tag::BT, "GuardedActionNode [" << name_ << "] precondition gate ABORTED");
                }
                else
                {
                    LOG_ERROR(logger_, task_log::Tag::BT, "GuardedActionNode [" << name_ << "] precondition gate failed");
                }
                phase_ = Phase::DONE;
                status_ = BTStatus::FAILURE;
                return status_;
            }
            // gate SUCCESS：区分 SKIP 与 PASSED
            auto gate_node = std::dynamic_pointer_cast<PreconditionGateNode>(gate_);
            if (gate_node && gate_node->getGateResult() == PreconditionGateNode::GateResult::SKIPPED)
            {
                LOG_WARN(logger_, task_log::Tag::BT, "GuardedActionNode [" << name_ << "] precondition SKIP, bypassing action as SUCCESS");
                phase_ = Phase::DONE;
                status_ = BTStatus::SUCCESS;
                return status_;
            }
            LOG_INFO(logger_, task_log::Tag::BT, "GuardedActionNode [" << name_ << "] precondition gate passed, executing action");
            phase_ = Phase::EXECUTE;
            status_ = BTStatus::RUNNING;
            return status_;
        }

        case Phase::EXECUTE:
        {
            if (!action_)
            {
                LOG_ERROR(logger_, task_log::Tag::BT, "GuardedActionNode [" << name_ << "] has null action");
                phase_ = Phase::DONE;
                status_ = BTStatus::FAILURE;
                return status_;
            }
            action_result_ = action_->tick();
            if (action_result_ == BTStatus::RUNNING)
            {
                status_ = BTStatus::RUNNING;
                return status_;
            }
            phase_ = (action_result_ == BTStatus::SUCCESS) ? Phase::ON_SUCCESS : Phase::ON_FAILURE;
            status_ = BTStatus::RUNNING;
            return status_;
        }

        case Phase::ON_SUCCESS:
        {
            if (!on_success_)
            {
                phase_ = Phase::DONE;
                status_ = BTStatus::SUCCESS;
                return status_;
            }
            BTStatus branch_status = on_success_->tick();
            if (branch_status == BTStatus::RUNNING)
            {
                status_ = BTStatus::RUNNING;
                return status_;
            }
            phase_ = Phase::DONE;
            if (on_success_policy_ == task_cfg::ResultPolicy::BRANCH_OVERRIDE)
            {
                // override：分支最终结果作为本动作结果
                status_ = branch_status;
            }
            else
            {
                // continue：分支成功→成功；分支失败→本动作失败
                status_ = (branch_status == BTStatus::SUCCESS) ? BTStatus::SUCCESS : BTStatus::FAILURE;
            }
            return status_;
        }

        case Phase::ON_FAILURE:
        {
            if (!on_failure_)
            {
                phase_ = Phase::DONE;
                status_ = BTStatus::FAILURE;
                return status_;
            }
            BTStatus branch_status = on_failure_->tick();
            if (branch_status == BTStatus::RUNNING)
            {
                status_ = BTStatus::RUNNING;
                return status_;
            }
            phase_ = Phase::DONE;
            if (on_failure_policy_ == task_cfg::ResultPolicy::BRANCH_RECOVER)
            {
                // recover：分支全部成功→本动作转为成功
                status_ = (branch_status == BTStatus::SUCCESS) ? BTStatus::SUCCESS : BTStatus::FAILURE;
            }
            else
            {
                // fail：分支执行后本动作仍判失败
                status_ = BTStatus::FAILURE;
            }
            return status_;
        }

        case Phase::DONE:
        default:
        {
            return status_;
        }
    }
}

void GuardedActionNode::reset()
{
    phase_ = Phase::GATE_CHECK;
    action_result_ = BTStatus::IDLE;
    if (gate_) gate_->reset();
    if (action_) action_->reset();
    if (on_success_) on_success_->reset();
    if (on_failure_) on_failure_->reset();
    status_ = BTStatus::IDLE;
    BTNode::reset();
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
    bool all_terminal = true;
    bool any_failure = false;
    bool stop_starting_idle_branches = false;

    for (auto& child : children_) {
        const std::string& child_name = child->getName();
        auto current_status = branch_status_[child_name];

        if (current_status == BTStatus::SUCCESS) {
            continue;
        }

        if (current_status == BTStatus::FAILURE) {
            any_failure = true;
            stop_starting_idle_branches = true;
            continue;
        }

        // 某分支失败后，不再启动尚未发送 Goal 的空闲分支。已经 RUNNING
        // 的分支继续 tick，等待其进入终态后并行节点才结束。
        if (stop_starting_idle_branches && current_status == BTStatus::IDLE) {
            branch_status_[child_name] = BTStatus::FAILURE;
            any_failure = true;
            continue;
        }

        BTStatus child_run_status = child->tick();
        branch_status_[child_name] = child_run_status;

        if (child_run_status == BTStatus::FAILURE) {
            LOG_ERROR(logger_, task_log::Tag::BT,
                "Parallel branch [" << child_name
                << "] failed; waiting for already-running sibling branches");
            any_failure = true;
            stop_starting_idle_branches = true;
        }
        else if (child_run_status != BTStatus::SUCCESS) {
            all_terminal = false;
        }
    }

    if (!all_terminal) {
        status_ = BTStatus::RUNNING;
        return BTStatus::RUNNING;
    }

    if (any_failure) {
        LOG_ERROR(logger_, task_log::Tag::BT,
            "Parallel node failed after all started branches reached a terminal state");
        status_ = BTStatus::FAILURE;
        return BTStatus::FAILURE;
    }

    LOG_SUCCESS(logger_, task_log::Tag::BT, "All parallel branches executed successfully");
    status_ = BTStatus::SUCCESS;
    return BTStatus::SUCCESS;
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




BTStatus IKLegLiftRecoveryNode::tick() {
    if (phase_ == Phase::DONE) return status_;
    auto child = phase_ == Phase::PRIMARY ? primary_ : phase_ == Phase::LIFT ? lift_ : retry_;
    const auto state = child->tick();
    if (state == BTStatus::RUNNING || state == BTStatus::IDLE) return status_ = BTStatus::RUNNING;
    if (phase_ == Phase::PRIMARY && state == BTStatus::FAILURE && may_recover_()) {
        phase_ = Phase::LIFT;
        return status_ = BTStatus::RUNNING;
    }
    if (phase_ == Phase::LIFT && state == BTStatus::SUCCESS) {
        mark_lifted_();
        phase_ = Phase::RETRY;
        return status_ = BTStatus::RUNNING;
    }
    phase_ = Phase::DONE;
    return status_ = state;
}
void IKLegLiftRecoveryNode::reset() {
    phase_ = Phase::PRIMARY;
    status_ = BTStatus::IDLE;
    primary_->reset(); lift_->reset(); retry_->reset();
}

BTStatus ConditionalLegRestoreNode::tick() {
    if (status_ == BTStatus::SUCCESS || status_ == BTStatus::FAILURE) return status_;
    if (!decided_) { run_ = pending_(); decided_ = true; }
    if (!run_) return status_ = BTStatus::SUCCESS;
    const auto state = restore_->tick();
    if (state == BTStatus::SUCCESS) clear_();
    return status_ = state;
}
void ConditionalLegRestoreNode::reset() {
    decided_ = run_ = false;
    status_ = BTStatus::IDLE;
    restore_->reset();
}
