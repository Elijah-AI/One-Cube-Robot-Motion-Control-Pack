#include "task_manager/task_manager_node.hpp"
#include <sstream>
#include <uuid/uuid.h>

namespace task_mgr
{
TaskManagerNode::TaskManagerNode()
: Node("task_manager_node"), current_state_(task_fsm::SystemState::UNKNOWN), error_flag_(0)
{
  hb_pub_ = this->create_publisher<HeartbeatMsg>("/SafeNode/Heartbeat", 10);
  hb_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
    std::bind(&TaskManagerNode::heartbeat_timer_cb, this));

  state_pub_ = this->create_publisher<NodeStatusMsg>("/SafeNode/status", 10);
  sys_state_client_ = this->create_client<GetSystemStatusSrv>("/Safety/get_system_status");
}

TaskManagerNode::~TaskManagerNode()
{
  if(modbus_slave_) modbus_slave_->stop();
}

void TaskManagerNode::run()
{
  RCLCPP_DEBUG(this->get_logger(), "Task manager");
  action_client_ = std::make_unique<task_act::TaskActionClient>(shared_from_this());
  bt_engine_ = std::make_unique<BehaviorTreeEngine>(this->get_logger());

  bt_tick_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
    std::bind(&TaskManagerNode::bt_tick_cb, this));


  switch_state(task_fsm::SystemState::INIT);
  if(!init_config())
  {
    set_error_flag(task_fsm::ERR_CONFIG_PARSE_FAIL);
    switch_state(task_fsm::SystemState::ERROR);
    return;
  }
  if(!init_modbus())
  {
    set_error_flag(task_fsm::ERR_MODBUS_FAULT);
    switch_state(task_fsm::SystemState::ERROR);
    return;
  }
  boot_check_system_state();
  rclcpp::spin(shared_from_this());
}

bool TaskManagerNode::init_config()
{
  task_cfg::TaskConfigParser parser;
  std::string cfg_path = this->declare_parameter<std::string>("config_path", "./config/task_config.yaml");
  return parser.load_config(cfg_path, cfg_);
}

bool TaskManagerNode::init_modbus()
{
  modbus_slave_ = std::make_unique<task_modbus::ModbusTcpSlave>(cfg_.modbus_listen_ip, cfg_.modbus_port, this->get_logger());
  modbus_slave_->register_write_callback(std::bind(&TaskManagerNode::on_modbus_reg_write, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  modbus_slave_->start();
  return true;
}

void TaskManagerNode::boot_check_system_state()
{
  switch_state(task_fsm::SystemState::SELF_CHECK);
  auto req = std::make_shared<GetSystemStatusSrv::Request>();
  req->header.stamp = this->now();
  req->request_id = gen_request_id();
  if(!sys_state_client_->wait_for_service(std::chrono::seconds(3)))
  {
    RCLCPP_WARN(this->get_logger(), "sys state service not online, enter idle");
    switch_state(task_fsm::SystemState::IDLE);
    return;
  }
  auto fut = sys_state_client_->async_send_request(req);
  if(rclcpp::spin_until_future_complete(shared_from_this(), fut, std::chrono::seconds(3)) != rclcpp::FutureReturnCode::SUCCESS)
  {
    set_error_flag(task_fsm::ERR_SERVICE_TIMEOUT);
    switch_state(task_fsm::SystemState::ERROR);
    return;
  }
  auto res = fut.get();
  if(res->status == static_cast<int8_t>(task_fsm::SystemState::SELF_CHECK))
  {
    run_boot_task(cfg_.boot_tasks.self_check_seq);
    run_boot_task(cfg_.boot_tasks.reset_seq);
    switch_state(task_fsm::SystemState::RESET_DONE);
  }
  switch_state(task_fsm::SystemState::IDLE);
  clear_error_flag(task_fsm::ERR_SERVICE_TIMEOUT);
}

// task_manager_node.cpp

BTNode::Ptr TaskManagerNode::create_action_node(const task_cfg::AtomicAction& action)
{
    std::string name = action.action_type; // 用于日志和调试
}



// run root task

void TaskManagerNode::run_boot_task(const std::vector<task_cfg::AtomicAction>& seq)
{
}





void TaskManagerNode::heartbeat_timer_cb()
{
  HeartbeatMsg msg;
  msg.header.stamp = this->now();
  msg.node_id = cfg_.node_id; // nodeId → node_id
  hb_pub_->publish(msg);
}

void TaskManagerNode::publish_state_change()
{
  NodeStatusMsg msg;
  msg.header.stamp = this->now();
  msg.node_id = cfg_.node_id;       // nodeId → node_id
  msg.status = static_cast<int32_t>(current_state_);
  msg.error_flag = error_flag_;     // errorFlag → error_flag
  state_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "FSM switch to %s, errFlag: %lu", task_fsm::state_to_str(current_state_).c_str(), error_flag_);
}
std::string TaskManagerNode::gen_request_id()
{
  std::stringstream ss;
  ss << this->get_name() << "_" << req_id_counter_++;
  return ss.str();
}

void TaskManagerNode::on_modbus_reg_write(uint32_t addr, uint16_t old_val, uint16_t new_val)
{

  RCLCPP_INFO(this->get_logger(), "new_val %u, old_val %u", new_val, old_val);
  //if(new_val == 1 && old_val != 1)
  {
    handle_trigger_task(addr);
  }
}

void TaskManagerNode::handle_trigger_task(uint32_t trigger_addr)
{
  if(current_state_ != task_fsm::SystemState::IDLE)
  {
    RCLCPP_WARN(this->get_logger(), "system busy, ignore task reg %u", trigger_addr);
    modbus_slave_->set_reg(trigger_addr, 0);
    return;
  }

  auto node = rclcpp::Node::make_shared("yaml_bt_runner");

  for(auto& [name, item] : cfg_.task_dict)
  {
    if(item.trigger_reg == trigger_addr)
    {
      switch_state(task_fsm::SystemState::RUNNING);
      auto node = rclcpp::Node::make_shared("manual_bt_demo");
      action_client_->reset_state();
      bt_engine_->load_action_group(item.action_seq, *action_client_);
      bt_engine_->start();
    }
  }
}

void TaskManagerNode::write_task_status_reg(uint32_t status_addr, uint16_t result)
{
  modbus_slave_->set_reg(status_addr, result);
}

void TaskManagerNode::switch_state(task_fsm::SystemState new_state, uint64_t err_mask)
{
  if(err_mask != 0) set_error_flag(err_mask);
  if(current_state_ == new_state) return;
  current_state_ = new_state;
  publish_state_change();
}

void TaskManagerNode::clear_error_flag(uint64_t mask)
{
  error_flag_ &= (~mask);
}

void TaskManagerNode::set_error_flag(uint64_t mask)
{
  error_flag_ |= mask;
}

void TaskManagerNode::bt_tick_cb()
{
  
  //RCLCPP_INFO(this->get_logger(), "bt_tick_cb, running state:%d", bt_engine_->isTaskRunning());
  if(rclcpp::ok() && bt_engine_->isTaskRunning())
  {
     bt_engine_->tick_once();
  }
  else
  {
    switch_state(task_fsm::SystemState::IDLE);
  }
}

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<task_mgr::TaskManagerNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}