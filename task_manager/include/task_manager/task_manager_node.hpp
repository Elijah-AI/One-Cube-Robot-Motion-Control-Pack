// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Author: gene.wang
// Date: 2026
//
// Description:
// 任务管理器 ROS2 节点头文件。
// 定义 TaskManagerNode 类，继承自 rclcpp::Node，
// 负责系统初始化、配置加载、Modbus 通信、状态机管理、
// 行为树调度和心跳发布等核心功能。

#ifndef TASK_MANAGER__TASK_MANAGER_NODE_HPP_
#define TASK_MANAGER__TASK_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "task_manager/behavior_tree_engine.hpp"
#include "task_manager/fsm_state.hpp"
#include "task_manager/task_config_parser.hpp"
#include "task_manager/modbus_slave.hpp"
#include "task_manager/task_action_client.hpp"

#include "com_interfaces/msg/heart_beat.hpp"
#include "com_interfaces/msg/node_status.hpp"
#include "com_interfaces/srv/get_system_status.hpp"
#include "task_manager/safety_task_types.hpp"
#include <chrono>
#include <unordered_map>
#include <atomic>
#include <string>
#include <queue>
#include <mutex>
#include <thread>

namespace task_mgr
{
// ==================== 时间常量（毫秒） ====================
constexpr int64_t HEARTBEAT_INTERVAL_MS = 1000;   ///< 心跳发布间隔
constexpr int64_t BT_TICK_INTERVAL_MS = 100;      ///< 行为树 tick 间隔
constexpr int64_t SERVICE_TIMEOUT_MS = 500;       ///< 服务等待超时

// ==================== Modbus 寄存器常量 ====================
constexpr uint16_t MODBUS_ADDR_OFFSET = 40001;    ///< Modbus 协议地址偏移
constexpr uint16_t MODBUS_STATUS_OFFSET = 10000;  ///< 任务状态寄存器偏移（相对触发寄存器）

// ==================== 任务执行状态码 ====================
constexpr uint16_t TASK_STATUS_RUNNING = 1;        ///< 任务运行中
constexpr uint16_t TASK_STATUS_SUCCESS = 2;        ///< 任务执行成功
constexpr uint16_t TASK_STATUS_FAILED = 3;         ///< 任务执行失败
using HeartbeatMsg = com_interfaces::msg::HeartBeat;
using NodeStatusMsg = com_interfaces::msg::NodeStatus;
using GetSystemStatusSrv = com_interfaces::srv::GetSystemStatus;

/// @brief 任务管理器 ROS2 节点
/// 系统核心节点，负责：
/// - 加载 YAML 任务配置文件
/// - 启动 Modbus TCP 从站（与 PLC 通信）
/// - 管理系统状态机（FSM）
/// - 调度行为树执行任务
/// - 发布心跳和状态信息
/// - 提供 /Task_manager/add_task Action 服务接收安全控制任务
class TaskManagerNode : public rclcpp::Node
{
public:
  TaskManagerNode();
  ~TaskManagerNode() override;

  /// @brief 节点主运行函数（初始化并进入 spin）
  void run();

private:
  /// @brief 初始化配置（加载 YAML 文件）
  bool init_config();

  /// @brief 初始化 Modbus 从站
  bool init_modbus();

  /// @brief 查询系统状态（开机自检阶段）
  void boot_check_system_state();

  /// @brief 执行系统复位（开机复位阶段）
  void boot_reset_system_state();

  /// @brief 执行启动任务序列
  void run_boot_task(const std::vector<task_cfg::AtomicAction>& seq);

  /// @brief 创建动作节点（预留接口）
  BTNode::Ptr create_action_node(const task_cfg::AtomicAction& action);

  /// @brief 心跳定时器回调
  void heartbeat_timer_cb();

  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp::Publisher<HeartbeatMsg>::SharedPtr hb_pub_;

  /// @brief 系统状态服务响应回调
  void request_result_callback(rclcpp::Client<GetSystemStatusSrv>::SharedFuture result_future);

  /// @brief 发布状态变更消息
  void publish_state_change();

  rclcpp::Publisher<NodeStatusMsg>::SharedPtr state_pub_;
  rclcpp::Client<GetSystemStatusSrv>::SharedPtr sys_state_client_;

  /// @brief 生成请求 ID
  std::string gen_request_id();
  uint64_t req_id_counter_{0};

  /// @brief Modbus 寄存器写入回调
  void on_modbus_reg_write(uint32_t addr, uint16_t old_val, uint16_t new_val);

  /// @brief 处理触发任务（根据 Modbus 寄存器地址查找并执行任务）
  void handle_trigger_task(uint32_t trigger_addr);

  /// @brief 写入任务状态到 Modbus 寄存器
  void write_task_status_reg(uint32_t status_addr, uint16_t result);

  // ==================== 状态机相关 ====================
  task_fsm::SystemState current_state_;  ///< 当前系统状态
  uint64_t error_flag_;                  ///< 错误标志位

  /// @brief 切换系统状态
  void switch_state(task_fsm::SystemState new_state, uint64_t err_mask = 0);

  /// @brief 清除错误标志
  void clear_error_flag(uint64_t mask);

  /// @brief 设置错误标志
  void set_error_flag(uint64_t mask);

  // ==================== 核心组件 ====================
  task_cfg::GlobalConfig cfg_;                              ///< 全局配置
  std::unique_ptr<task_modbus::ModbusTcpSlave> modbus_slave_;   ///< Modbus 从站
  std::unique_ptr<task_act::TaskActionClient> action_client_;   ///< Action 客户端
  std::unique_ptr<BehaviorTreeEngine> bt_engine_;               ///< 行为树引擎

  // ==================== 行为树调度 ====================
  rclcpp::TimerBase::SharedPtr bt_tick_timer_;  ///< 行为树 tick 定时器

  /// @brief 行为树 tick 回调（定时驱动）
  void bt_tick_cb();

  /// @brief 开始记录普通业务任务总耗时
  void start_normal_task_timing(const std::string& task_name);

  /// @brief 输出普通业务任务总耗时并结束计时
  void log_normal_task_total_time(const char* result);

  std::string current_task_id_;            ///< 当前任务 ID
  std::atomic<bool> task_running_{false};  ///< 任务运行标志
  std::chrono::steady_clock::time_point normal_task_start_time_;  ///< 普通任务开始时间

  /// @brief 防止重复发送系统状态请求
  std::atomic<bool> boot_state_req_pending_{false};

  // ==================== AddTask Action Server ====================
  /// @brief AddTask Action Server，接收 safety_monitor 下发的安全控制任务
  rclcpp_action::Server<AddTaskAction>::SharedPtr add_task_action_server_;

  /// @brief 处理 AddTask Goal 请求（验证参数、优先级队列插入）
  rclcpp_action::GoalResponse handle_add_task_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const AddTaskAction::Goal> goal);

  /// @brief 处理 AddTask Cancel 请求
  rclcpp_action::CancelResponse handle_add_task_cancel(
    const std::shared_ptr<GoalHandleAddTask> goal_handle);

  /// @brief 接受 Goal 后的执行逻辑（优先级队列调度入口）
  void handle_add_task_accepted(const std::shared_ptr<GoalHandleAddTask> goal_handle);

  /// @brief 执行安全任务（在行为树中以最高优先级调度）
  /// @param task 优先级任务
  /// @param goal_handle 关联的 goal handle，用于发送反馈和结果
  void execute_safety_task(
    const PrioritizedTask& task,
    std::shared_ptr<GoalHandleAddTask> goal_handle);

  /// @brief 安全任务执行线程
  void safety_task_execution_thread(std::shared_ptr<GoalHandleAddTask> goal_handle);

  /// @brief 处理优先级任务队列中的待处理任务
  void process_queued_tasks();

  // ==================== 优先级任务队列 ====================
  /// @brief 优先级任务队列（线程安全）
  std::priority_queue<PrioritizedTask> safety_task_queue_;
  std::mutex task_queue_mutex_;             ///< 队列互斥锁
  std::atomic<bool> safety_task_running_{false};  ///< 安全任务执行标志
  std::thread safety_task_thread_;          ///< 安全任务执行线程

  /// @brief 反馈定时器（用于周期性发送执行进度）
  rclcpp::TimerBase::SharedPtr safety_feedback_timer_;
  std::shared_ptr<GoalHandleAddTask> current_safety_goal_handle_;
  rclcpp::Time safety_task_start_time_;
};
}

#endif // TASK_MANAGER__TASK_MANAGER_NODE_HPP_
