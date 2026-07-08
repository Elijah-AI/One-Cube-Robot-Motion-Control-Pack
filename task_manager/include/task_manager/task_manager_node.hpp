#ifndef TASK_MANAGER__TASK_MANAGER_NODE_HPP_
#define TASK_MANAGER__TASK_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
// #include <task_manager/msg/heartbeat.hpp>
// #include <task_manager/msg/node_status.hpp>
// #include <task_manager/srv/get_system_status.hpp>
#include "task_manager/behavior_tree_engine.hpp"
#include "task_manager/fsm_state.hpp"
#include "task_manager/task_config_parser.hpp"
#include "task_manager/modbus_slave.hpp"
#include "task_manager/task_action_client.hpp"
#include "com_interfaces/msg/heartbeat.hpp"
#include "com_interfaces/msg/node_status.hpp"
#include "com_interfaces/srv/get_system_status.hpp"
#include <chrono>
#include <unordered_map>
#include <atomic>
#include <string>

namespace task_mgr
{
using HeartbeatMsg = com_interfaces::msg::Heartbeat;
using NodeStatusMsg = com_interfaces::msg::NodeStatus;
using GetSystemStatusSrv = com_interfaces::srv::GetSystemStatus;

class TaskManagerNode : public rclcpp::Node
{
public:
  TaskManagerNode();
  ~TaskManagerNode() override;
  void run();

private:
  bool init_config();
  bool init_modbus();
  void boot_check_system_state();
  void boot_reset_system_state();
  void run_boot_task(const std::vector<task_cfg::AtomicAction>& seq);
  BTNode::Ptr create_action_node(const task_cfg::AtomicAction& action);
  void heartbeat_timer_cb();
  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp::Publisher<HeartbeatMsg>::SharedPtr hb_pub_;
  void request_result_callback(rclcpp::Client<GetSystemStatusSrv>::SharedFuture result_future);
  void publish_state_change();
  rclcpp::Publisher<NodeStatusMsg>::SharedPtr state_pub_;

  rclcpp::Client<GetSystemStatusSrv>::SharedPtr sys_state_client_;
  std::string gen_request_id();
  uint64_t req_id_counter_{0};

  void on_modbus_reg_write(uint32_t addr, uint16_t old_val, uint16_t new_val);
  void handle_trigger_task(uint32_t trigger_addr);
  void write_task_status_reg(uint32_t status_addr, uint16_t result);

  task_fsm::SystemState current_state_;
  uint64_t error_flag_;
  void switch_state(task_fsm::SystemState new_state, uint64_t err_mask = 0);
  void clear_error_flag(uint64_t mask);
  void set_error_flag(uint64_t mask);

  task_cfg::GlobalConfig cfg_;
  std::unique_ptr<task_modbus::ModbusTcpSlave> modbus_slave_;
  std::unique_ptr<task_act::TaskActionClient> action_client_;
  std::unique_ptr<BehaviorTreeEngine> bt_engine_;

  rclcpp::TimerBase::SharedPtr bt_tick_timer_;

  void bt_tick_cb();
  std::string current_task_id_;
  std::atomic<bool> task_running_{false};
};
}

#endif