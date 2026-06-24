#include "task_manager/behavior_tree_engine.hpp"
#include <rclcpp/logging.hpp>
#include <iostream> // 如果需要日志输出

// 构造函数实现
FallbackNode::FallbackNode(const std::string& name, rclcpp::Logger logger) 
    : logger_(std::move(logger)), BTNode(name), current_child_idx_(0) {
}

// tick 方法实现
BTStatus FallbackNode::tick() {

    RCLCPP_INFO(logger_, "FallbackNode tick");
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
        return BTStatus::FAILURE;
    }
    status_ = callback_();
    return status_;
}


// 构造函数实现
SequenceNode::SequenceNode(const std::string& name, rclcpp::Logger logger) 
    : logger_(std::move(logger)), BTNode(name), current_child_idx_(0) {
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
            status_ = BTStatus::FAILURE;
            current_child_idx_ = 0;
            return BTStatus::FAILURE;
        } else if (child_status == BTStatus::SUCCESS) {
            continue;
        }
        // 如果是 SUCCESS，循环继续，执行下一个子节点
    }
    
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

// addChild 函数实现
void SequenceNode::addChild(Ptr child) {
    children_.push_back(std::move(child));
}


// 清空所有子节点
void SequenceNode::clearAllChildren()
{
    children_.clear();
    current_child_idx_ = 0;
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
    RCLCPP_ERROR(logger_, "BehaviorTree root node is empty!");
    return;
  }
  root_node_->reset();
  running_ = true;
  RCLCPP_INFO(logger_, "BehaviorTree started");
}

void BehaviorTreeEngine::abort()
{
  running_ = false;
  if (root_node_)
  {
    root_node_->reset();
  }
  last_status_ = BTStatus::IDLE;
  RCLCPP_INFO(logger_, "BehaviorTree aborted");
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
    RCLCPP_INFO(logger_, "Tree finished, final status: %d", static_cast<int>(last_status_));
  }
}

void BehaviorTreeEngine::load_action_group(
    const std::vector<task_cfg::AtomicAction>& action_list,
    task_act::TaskActionClient& act_client)
{
  auto root_sequence = std::make_shared<SequenceNode>("Action_Group_Root", logger_);
  root_sequence->clearAllChildren();
  for (const auto& atomic_act : action_list)
  {
    std::string node_name = "Action_" + atomic_act.action_type;
    RCLCPP_INFO(logger_, "node_name: %s",node_name.c_str());
    auto action_cb = [&act_client, atomic_act, this]() -> BTStatus
    {
       // 适配客户端新接口
      if (act_client.is_action_running())
      {
        rclcpp::Time now = act_client.get_ros_now_time();
        rclcpp::Duration diff = now - act_client.get_task_start_time();

        double start_ms = static_cast<double>(act_client.get_task_start_time().nanoseconds()) / 1000000.0;
        double now_ms   = static_cast<double>(now.nanoseconds()) / 1000000.0;

        int64_t delta_ns = diff.nanoseconds();
        double elapsed_ms = std::fabs(delta_ns / 1000000.0);
       
        

        
        if (elapsed_ms > act_client.get_task_execution_time())
        {
          RCLCPP_INFO(logger_,
          "start total ms: %.3f | end total ms: %.3f, elapsed_ms: %.3f, execution_time:%ld",
          start_ms, now_ms, elapsed_ms, act_client.get_task_execution_time());

          RCLCPP_INFO(logger_, "Action execution time out");
          return BTStatus::FAILURE;
        }
        else
        {
          return BTStatus::RUNNING;
        }
        return BTStatus::RUNNING;
      }
      else if (act_client.is_action_success())
      {
        RCLCPP_INFO(logger_, "Action [%s] execute success", atomic_act.action_type.c_str());
        act_client.reset_state();
        return BTStatus::SUCCESS;

      }
      else if (act_client.is_failure())
      {
        RCLCPP_ERROR(logger_, "Action [%s] execute failed", atomic_act.action_type.c_str());
        
        return BTStatus::FAILURE;
      } 
     

      RCLCPP_INFO(logger_, "===== Execute Action Type: %s =====", atomic_act.action_type.c_str());
      RCLCPP_INFO(logger_, "VisionParam -> target: %s, need_6d: %d",
          atomic_act.vision_param.target_obj_name.c_str(),
          atomic_act.vision_param.need_6d_pose);
      RCLCPP_INFO(logger_, "ArmParam -> arm_id: %s, use_planning: %d",
          atomic_act.arm_param.arm_id.c_str(),
          atomic_act.arm_param.use_planning);
      RCLCPP_INFO(logger_, "GripperParam -> gripper_id: %d, block: %d",
          atomic_act.gripper_param.gripper_id,
          atomic_act.gripper_param.block);
      
      RCLCPP_INFO(logger_, "act_client.is_action_idle = %d", act_client.get_action_state());
      
      
      bool send_ok = act_client.send_goal(atomic_act);
      if (!send_ok)
      {
        RCLCPP_ERROR(logger_, "Action [%s] send goal failed", atomic_act.action_type.c_str());
        return BTStatus::FAILURE;
      }
      else 
      {
        return BTStatus::RUNNING;
      }
    };

    auto act_node = std::make_shared<ActionNode>(node_name, std::move(action_cb), logger_);
    // 安全添加子节点，不再直接访问 protected children_
    root_sequence->addChild(std::move(act_node));
  }

  this->setRoot(std::move(root_sequence));
  RCLCPP_INFO(logger_, "Load action group complete, total actions: %zu", action_list.size());
}