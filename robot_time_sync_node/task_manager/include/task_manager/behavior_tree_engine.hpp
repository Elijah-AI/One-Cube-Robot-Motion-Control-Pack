#ifndef BEHAVIOR_TREE_ENGINE_HPP_
#define BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

// 引入你的任务配置头文件（必须）
#include "task_manager/task_config_parser.hpp"
#include "task_manager/task_action_client.hpp"

// 1. 定义标准节点状态
enum class BTStatus {
    SUCCESS,
    FAILURE,
    RUNNING,
    IDLE
};

// 2. 定义节点基类 (核心扩展点)
class BTNode {
public:
    using Ptr = std::shared_ptr<BTNode>;

    explicit BTNode(const std::string& name) : name_(name), status_(BTStatus::IDLE) {}
    virtual ~BTNode() = default;

    // 核心接口：每次 tick 调用
    virtual BTStatus tick() = 0;

    // 重置节点状态
    virtual void reset() {
        status_ = BTStatus::IDLE;
        for (auto& child : children_) {
            if (child) child->reset();
        }
    }

    std::string getName() const { return name_; }
    BTStatus getStatus() const { return status_; }

protected:
    std::string name_;
    BTStatus status_;
    std::vector<Ptr> children_; // 子节点列表用于控制流节点
};

// 3. 叶子节点：动作节点 (Action)
// class ActionNode : public BTNode {
// public:
//     // 传入具体的执行函数回调
//     using ActionCallback = std::function<BTStatus()>;

//     explicit ActionNode(const std::string& name, ActionCallback callback, rclcpp::Logger logger)
//         : logger_(std::move(logger)), BTNode(name), callback_(callback) {}

//     BTStatus tick() override {
//         if (!callback_) return BTStatus::FAILURE;
//         status_ = callback_(); // 执行具体逻辑
//         return status_;
//     }

// private:
//     ActionCallback callback_;
//     rclcpp::Logger logger_;    ///< ROS2 日志记录器    
// };

class ActionNode : public BTNode
{
public:
    /// 执行回调类型：无入参，返回节点状态
    using ActionCallback = std::function<BTStatus()>;

    /**
     * @brief 构造函数
     * @param name 节点名
     * @param callback 节点tick执行回调
     * @param logger ROS日志对象
     */
    explicit ActionNode(const std::string& name,
                        ActionCallback callback,
                        rclcpp::Logger logger);

    /// 行为树核心tick函数
    BTStatus tick() override;

private:
    ActionCallback callback_;
    rclcpp::Logger logger_;    ///< ROS2 日志记录器
};




// 4. 叶子节点：条件节点 (Condition)
class ConditionNode : public BTNode {
public:
    using ConditionCallback = std::function<bool()>;

    explicit ConditionNode(const std::string& name, ConditionCallback callback)
        : BTNode(name), callback_(callback) {}

    BTStatus tick() override {
     
        if (!callback_) return BTStatus::FAILURE;
        // 条件节点通常瞬间完成，不返回 RUNNING
        status_ = callback_() ? BTStatus::SUCCESS : BTStatus::FAILURE;
        return status_;
    }

private:
    ConditionCallback callback_;
    
};

//5. 控制流节点：序列 (Sequence) - 所有子节点成功才成功，遇到失败立即停止
class SequenceNode : public BTNode {
    public:    
    /**
         * @brief 构造函数
         * @param name 节点名称
         * @param logger ROS2 日志记录器
         */    
    explicit SequenceNode(const std::string& name, rclcpp::Logger logger);
        /**
         * @brief 执行节点逻辑
         * @return BTStatus 返回当前执行状态 (SUCCESS, FAILURE, RUNNING)
         */    
    BTStatus tick() override;
        /**
         * @brief 重置节点状态
         */    
    void reset() override;        
    /**
         * @brief 添加子节点
         * @param child 子节点智能指针
         */    
    void addChild(Ptr child);

    void clearAllChildren();
    private:    
    size_t current_child_idx_; ///< 当前正在执行的子节点索引    
    rclcpp::Logger logger_;    ///< ROS2 日志记录器    
    std::vector<Ptr> children_; ///< 子节点列表
};


class FallbackNode : public BTNode {
public:
    // 构造函数声明
    explicit FallbackNode(const std::string& name, rclcpp::Logger logger);

    // 虚函数声明
    BTStatus tick() override;
    void reset() override;

private:
    size_t current_child_idx_;
    rclcpp::Logger logger_;
    // 假设 children_ 和 status_ 定义在基类 BTNode 中，
    // 如果不在基类，需在此处声明:
    // std::vector<std::shared_ptr<BTNode>> children_;
    // BTStatus status_;
};


// 7. 行为树引擎
class BehaviorTreeEngine {
public:
    using NodePtr = BTNode::Ptr;

    BehaviorTreeEngine(rclcpp::Logger logger);
    ~BehaviorTreeEngine() = default;

    void setRoot(NodePtr root);
    void start();
    void abort();
    void tick_once();
    bool isTaskRunning() const { return running_; }
    bool isTaskSuccess() const { return last_status_ == BTStatus::SUCCESS; }

    // ===================== 改造后接口 =====================
    /**
     * @brief 加载原子动作序列（内含视觉/机械臂/夹爪全套参数）
     * @param action_list 原子动作数组
     * @param act_client ROS2 Action 客户端
     */
    void load_action_group(
        const std::vector<task_cfg::AtomicAction>& action_list,
        task_act::TaskActionClient& act_client);

private:
    rclcpp::Logger logger_;
    NodePtr root_node_;
    bool running_;
    BTStatus last_status_;
};

#endif // BEHAVIOR_TREE_ENGINE_H