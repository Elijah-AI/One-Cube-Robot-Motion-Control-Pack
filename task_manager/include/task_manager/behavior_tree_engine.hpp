// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Author: gene.wang
// Date: 2026
//
// Description:
// 行为树引擎头文件。
// 实现自定义行为树框架，包含以下节点类型：
// - BTNode: 节点基类
// - ActionNode: 动作执行节点（叶子节点）
// - ConditionNode: 条件判断节点（叶子节点）
// - SequenceNode: 顺序执行节点（控制流节点）
// - ParallelNode: 并行执行节点（控制流节点）
// - FallbackNode: 回退选择节点（控制流节点）
// - BehaviorTreeEngine: 行为树引擎，负责树的构建、加载和执行调度。

#ifndef BEHAVIOR_TREE_ENGINE_HPP_
#define BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <unordered_map>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "task_manager/task_config_parser.hpp"
#include "task_manager/task_action_client.hpp"
#include "task_manager/precondition_manager.hpp"

/// @brief 行为树节点状态枚举
enum class BTStatus {
    SUCCESS,  ///< 执行成功
    FAILURE,  ///< 执行失败
    RUNNING,  ///< 执行中（需要下次 tick 继续）
    IDLE      ///< 空闲（未执行）
};

/// @brief 行为树节点基类
/// 所有节点类型的基类，提供名称、状态、子节点管理等基础功能。
class BTNode {
public:
    using Ptr = std::shared_ptr<BTNode>;

    /// @brief 构造函数
    /// @param name 节点名称
    explicit BTNode(const std::string& name) : name_(name), status_(BTStatus::IDLE) {}
    virtual ~BTNode() = default;

    /// @brief 核心接口：每次 tick 调用执行节点逻辑
    virtual BTStatus tick() = 0;

    /// @brief 重置节点状态（递归重置所有子节点）
    virtual void reset() {
        status_ = BTStatus::IDLE;
        for (auto& child : children_) {
            if (child) child->reset();
        }
    }

    std::string getName() const { return name_; }
    BTStatus getStatus() const { return status_; }

protected:
    std::string name_;           ///< 节点名称
    BTStatus status_;            ///< 当前节点状态
    std::vector<Ptr> children_;  ///< 子节点列表（用于控制流节点）

public:
    /// @brief 添加子节点
    virtual void addChild(Ptr child) { children_.push_back(std::move(child)); }
};

/// @brief 动作执行节点（叶子节点）
/// 封装一个可执行的动作回调，每次 tick 调用回调函数获取执行状态。
class ActionNode : public BTNode
{
public:
    using ActionCallback = std::function<BTStatus()>;

    /// @brief 构造函数
    /// @param name 节点名称
    /// @param callback 执行回调
    /// @param logger ROS2 日志记录器
    explicit ActionNode(const std::string& name,
                        ActionCallback callback,
                        rclcpp::Logger logger);

    BTStatus tick() override;

private:
    ActionCallback callback_;  ///< 动作执行回调
    rclcpp::Logger logger_;   ///< ROS2 日志记录器
};

/// @brief 条件判断节点（叶子节点）
/// 封装一个条件判断回调，返回 true 表示条件满足（SUCCESS），false 表示不满足（FAILURE）。
class ConditionNode : public BTNode {
public:
    using ConditionCallback = std::function<bool()>;

    /// @brief 构造函数
    /// @param name 节点名称
    /// @param callback 条件判断回调
    /// @param logger ROS2 日志记录器
    explicit ConditionNode(const std::string& name, ConditionCallback callback, rclcpp::Logger logger);

    BTStatus tick() override;

private:
    ConditionCallback callback_;  ///< 条件判断回调
    rclcpp::Logger logger_;      ///< ROS2 日志记录器
};

/// @brief 顺序执行节点（控制流节点）
/// 按顺序依次执行所有子节点，全部成功才返回 SUCCESS，任一失败立即返回 FAILURE。
class SequenceNode : public BTNode {
public:
    /// @brief 构造函数
    /// @param name 节点名称
    /// @param logger ROS2 日志记录器
    explicit SequenceNode(const std::string& name, rclcpp::Logger logger);

    BTStatus tick() override;
    void reset() override;

    void clearAllChildren();
    std::string parent_name;

private:
    size_t current_child_idx_;   ///< 当前正在执行的子节点索引
    rclcpp::Logger logger_;      ///< ROS2 日志记录器
};

/// @brief 并行执行节点（控制流节点）
/// 所有子节点并行执行，全部成功才返回 SUCCESS，任一失败立即返回 FAILURE。
class ParallelNode : public BTNode
{
public:
    using Ptr = std::shared_ptr<ParallelNode>;

    /// @brief 构造函数
    /// @param name 节点名称
    /// @param logger ROS2 日志记录器
    ParallelNode(const std::string& name, rclcpp::Logger logger);

    BTStatus tick() override;
    void reset() override;

    /// @brief 添加子节点
    void addChild(BTNode::Ptr child);

private:
    rclcpp::Logger logger_;                              ///< ROS2 日志记录器
    std::unordered_map<std::string, BTStatus> branch_status_;  ///< 各分支执行状态
};

/// @brief 回退选择节点（控制流节点）
/// 按顺序尝试子节点，遇到 SUCCESS 立即返回，全部失败才返回 FAILURE。
class FallbackNode : public BTNode {
public:
    /// @brief 构造函数
    /// @param name 节点名称
    /// @param logger ROS2 日志记录器
    explicit FallbackNode(const std::string& name, rclcpp::Logger logger);

    BTStatus tick() override;
    void reset() override;

private:
    size_t current_child_idx_;  ///< 当前正在尝试的子节点索引
    rclcpp::Logger logger_;     ///< ROS2 日志记录器
};

/// @brief 条件重试节点（控制流节点）
/// 用于对动作执行结果进行反馈条件检查与自动重试。
/// 执行流程（状态机）：
///   1. PROBE：执行探测动作（probe，通常是 vision_detect）；
///      成功 → CHECK，失败 → FAILURE；
///   2. CHECK：执行条件判断（condition，读取反馈字段如 pose_confidence 比较阈值）；
///      满足 → SUCCESS；不满足且重试次数未用尽 → 执行调整动作序列（RETRY），
///      不满足且重试次数用尽 → 按 on_exhaust 返回 FAILURE 或 SUCCESS（pass）；
///   3. RETRY：依次执行调整动作序列（retry_actions，如 move_pose 调整姿态）；
///      完成后回到 PROBE 重新执行探测动作。
class RetryUntilConditionNode : public BTNode {
public:
    /// @brief 节点内部状态机阶段
    enum class RetryPhase {
        PROBE,  ///< 执行探测动作
        CHECK,  ///< 检查反馈条件
        RETRY,  ///< 执行调整动作序列
        DONE    ///< 已结束（条件满足或重试次数用尽）
    };

    /// @brief 构造函数
    /// @param name 节点名称
    /// @param logger ROS2 日志记录器
    explicit RetryUntilConditionNode(const std::string& name, rclcpp::Logger logger);

    BTStatus tick() override;
    void reset() override;

    /// @brief 设置探测动作节点（通常是 vision_detect）
    void setProbeAction(BTNode::Ptr probe);
    /// @brief 设置条件判断节点（读取反馈字段比较阈值）
    void setConditionNode(BTNode::Ptr condition);
    /// @brief 设置调整动作序列节点（条件不满足时执行）
    void setRetrySequence(BTNode::Ptr retry_seq);
    /// @brief 设置最大重试次数
    void setMaxRetries(int32_t max_retries);
    /// @brief 设置重试次数用尽后的行为：true=强制通过(SUCCESS)，false=失败(FAILURE)
    void setOnExhaust(bool pass_on_exhaust);

private:
    rclcpp::Logger logger_;               ///< ROS2 日志记录器
    BTNode::Ptr probe_;                   ///< 探测动作节点
    BTNode::Ptr condition_;               ///< 条件判断节点
    BTNode::Ptr retry_seq_;               ///< 调整动作序列节点
    int32_t max_retries_{1};              ///< 最大重试次数
    int32_t retry_count_{0};              ///< 已重试次数
    RetryPhase phase_{RetryPhase::PROBE}; ///< 当前阶段
    bool pass_on_exhaust_{false};         ///< 重试次数用尽后是否强制通过
};

/// @brief 结果分支（on_success / on_failure）递归包装深度上限
/// 防止配置错误导致无限递归
constexpr int kMaxBranchDepth = 5;

/// @brief 前置条件门节点
/// 评估一组前置条件，并按策略处理等待 / 超时 / 跳过：
///   - WAIT:     周期性重查直至满足或超时（未满足且未超时 → RUNNING，由外部 tick 驱动）；
///                超时后按 on_timeout 处理：FAIL → 门失败 / PASS → 放行 / ABORT → 中止
///   - FAIL_FAST:立即判定，不满足 → 门失败
///   - SKIP:     不满足时门结果置为 SKIPPED（GuardedActionNode 按"跳过动作、视为成功"处理）
/// 返回 SUCCESS 表示门通过（满足 / PASS 放行 / SKIP 跳过）；FAILURE 表示门不通过。
class PreconditionGateNode : public BTNode
{
public:
    /// @brief 门结算结果（供 GuardedActionNode 区分通过 / 跳过 / 失败 / 中止）
    enum class GateResult
    {
        NONE,     ///< 未结算
        PASSED,   ///< 通过（条件满足或 PASS 超时放行）
        SKIPPED,  ///< 跳过（SKIP 策略，视为成功）
        FAILED,   ///< 失败（FAIL_FAST 或 WAIT 超时 FAIL）
        ABORTED   ///< 中止（WAIT 超时 ABORT）
    };

    /// @brief 条件评估回调（封装 PreconditionManager::evaluate）
    using PreEvalFn = std::function<task_mgr::PrecheckResult()>;

    explicit PreconditionGateNode(const std::string& name, rclcpp::Logger logger);

    BTStatus tick() override;
    void reset() override;

    /// @brief 设置前置条件配置（拷贝快照，避免外部引用失效）
    void setConfig(const task_cfg::PreconditionList& pre);

    /// @brief 设置条件评估回调
    void setEvaluator(PreEvalFn fn);

    /// @brief 门结算后读取结果
    GateResult getGateResult() const { return gate_result_; }

private:
    rclcpp::Logger logger_;                         ///< ROS2 日志记录器
    task_cfg::PreconditionList pre_;                ///< 前置条件配置快照
    PreEvalFn eval_;                                ///< 条件评估回调
    std::chrono::steady_clock::time_point wait_start_{};  ///< 等待开始时刻
    std::chrono::steady_clock::time_point last_check_{};  ///< 上次评估时刻
    GateResult gate_result_{GateResult::NONE};      ///< 门结算结果
    bool configured_{false};                        ///< 是否已配置
};

/// @brief 前置条件门 + 主动作 + 结果分支 包装节点（控制流节点）
/// 内部阶段状态机：GATE_CHECK → EXECUTE → (ON_SUCCESS | ON_FAILURE) → DONE
/// 结算规则（branch_policy，见 task_config_parser.hpp ResultPolicy）：
///   on_success: continue（分支成功→成功；分支失败→本动作失败）
///               override（分支的最终结果作为本动作结果）
///   on_failure: fail（分支执行后本动作仍判失败，任务终止）
///               recover（分支全部成功则本动作转为成功，主序列继续）
/// 前置条件门 SKIP → 跳过整个动作（视为成功，不执行结果分支）；
/// 门 FAIL/ABORT → 本动作失败（触发可选的 precondition_fail_cb）。
class GuardedActionNode : public BTNode
{
public:
    enum class Phase { GATE_CHECK, EXECUTE, ON_SUCCESS, ON_FAILURE, DONE };

    explicit GuardedActionNode(const std::string& name, rclcpp::Logger logger);

    BTStatus tick() override;
    void reset() override;

    /// @brief 设置前置条件门（内部可含等待循环；nullptr 表示无前置条件，直接执行主动作）
    void setPreconditionGate(BTNode::Ptr gate);
    /// @brief 设置主动作节点（叶子，复用 build_action_node / feedback_check 节点）
    void setAction(BTNode::Ptr action);
    /// @brief 设置成功分支序列（可选；通常为 SequenceNode）
    void setOnSuccessBranch(BTNode::Ptr branch);
    /// @brief 设置失败分支序列（可选；通常为 SequenceNode）
    void setOnFailureBranch(BTNode::Ptr branch);
    /// @brief 设置结果分支策略（UNKNOWN 时分别按 continue / fail 缺省处理）
    void setBranchPolicy(task_cfg::ResultPolicy on_success_policy,
                         task_cfg::ResultPolicy on_failure_policy);
    /// @brief 可选：前置条件判定失败/中止回调（如置 ERR_PRECONDITION_FAIL 错误位）
    void setPreconditionFailCb(std::function<void()> cb);

private:
    rclcpp::Logger logger_;                         ///< ROS2 日志记录器
    Phase phase_{Phase::GATE_CHECK};                ///< 当前阶段
    BTNode::Ptr gate_;                              ///< 前置条件门
    BTNode::Ptr action_;                            ///< 主动作节点
    BTNode::Ptr on_success_;                        ///< 成功分支序列
    BTNode::Ptr on_failure_;                        ///< 失败分支序列
    task_cfg::ResultPolicy on_success_policy_{task_cfg::ResultPolicy::BRANCH_CONTINUE};
    task_cfg::ResultPolicy on_failure_policy_{task_cfg::ResultPolicy::BRANCH_FAIL};
    std::function<void()> pre_fail_cb_;             ///< 前置条件失败回调（可选）
    BTStatus action_result_{BTStatus::IDLE};        ///< 主动作结果（供结算）
};

/// @brief 行为树引擎
/// 负责行为树的构建、根节点管理、执行调度（tick）和状态查询。
class BehaviorTreeEngine {
public:
    using NodePtr = BTNode::Ptr;

    /// @brief 构造函数
    /// @param logger ROS2 日志记录器
    BehaviorTreeEngine(rclcpp::Logger logger);
    ~BehaviorTreeEngine() = default;

    /// @brief 设置前置条件管理器（供前置条件门评估；可空，未注入时条件门不生效）
    void setPreconditionManager(task_mgr::PreconditionManager* pm);
    /// @brief 设置前置条件失败回调（可空，如置 ERR_PRECONDITION_FAIL 错误位）
    void setPreconditionFailCb(std::function<void()> cb);

    /// @brief 设置根节点
    void setRoot(NodePtr root);

    /// @brief 启动行为树执行
    void start();

    /// @brief 中止行为树执行
    void abort();

    /// @brief 执行一次 tick（驱动整棵树执行一步）
    void tick_once();

    /// @brief 查询任务是否正在运行
    bool isTaskRunning() const { return running_; }

    /// @brief 查询任务是否成功完成
    bool isTaskSuccess() const { return last_status_ == BTStatus::SUCCESS; }

    /// @brief 查询是否已设置根节点
    bool hasRootNode() const  { return root_node_ != nullptr; }

    /// @brief 加载串行动作序列
    /// @param action_list 原子动作列表
    /// @param act_client Action 客户端引用
    void load_action_group(
        const std::vector<task_cfg::AtomicAction>& action_list,
        task_act::TaskActionClient& act_client);

    /// @brief 加载并行任务（多组动作序列并行执行）
    /// @param parallel_action_groups 并行动作分组
    /// @param act_client Action 客户端引用
    void load_parallel_task(
        std::vector<std::vector<task_cfg::AtomicAction>> & parallel_action_groups,
        task_act::TaskActionClient& act_client);

private:
    rclcpp::Logger logger_;               ///< ROS2 日志记录器
    NodePtr root_node_;                   ///< 行为树根节点
    bool running_;                        ///< 运行标志
    BTStatus last_status_;                ///< 上次 tick 的执行状态
    task_mgr::PreconditionManager* precondition_mgr_{nullptr};  ///< 前置条件管理器（构建期注入）
    std::function<void()> precondition_fail_cb_;                ///< 前置条件失败回调（可选）
};

#endif // BEHAVIOR_TREE_ENGINE_H
