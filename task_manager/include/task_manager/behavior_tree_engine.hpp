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
#include <rclcpp/rclcpp.hpp>

#include "task_manager/task_config_parser.hpp"
#include "task_manager/task_action_client.hpp"

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
/// 所有子节点并行执行并等待已启动分支进入终态；全部成功才返回 SUCCESS。
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

/// @brief 行为树引擎
/// 负责行为树的构建、根节点管理、执行调度（tick）和状态查询。
class BehaviorTreeEngine {
public:
    using NodePtr = BTNode::Ptr;

    /// @brief 构造函数
    /// @param logger ROS2 日志记录器
    BehaviorTreeEngine(rclcpp::Logger logger);
    ~BehaviorTreeEngine() = default;

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
    rclcpp::Logger logger_;   ///< ROS2 日志记录器
    NodePtr root_node_;       ///< 行为树根节点
    bool running_;            ///< 运行标志
    BTStatus last_status_;    ///< 上次 tick 的执行状态
};

#endif // BEHAVIOR_TREE_ENGINE_H
