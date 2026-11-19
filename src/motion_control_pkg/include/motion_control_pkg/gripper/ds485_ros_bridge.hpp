#pragma once
#include "ds485_driver/gripper_config.h"
#include "ds485_driver/embedded_torque.h"
#include "ds485_driver/gripper_controller.h"
#include <com_interfaces/action/control_gripper.hpp>
#include <ds485_interfaces/msg/gripper_command.hpp>
#include <ds485_interfaces/msg/gripper_result.hpp>
#include <ds485_interfaces/msg/gripper_stop.hpp>
#include <ds485_interfaces/msg/servo_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace motion_control_pkg::gripper {
using Ds485Executor = ds485_driver::GripperController;
class Ds485RosBridge {
public:
  using Action = com_interfaces::action::ControlGripper;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  // Called by the topic frontend; the owner keeps its admission lock during dispatch.
  using Admission = std::function<bool(unsigned, const std::function<void()> &, std::string &)>;
  Ds485RosBridge(rclcpp::Node::SharedPtr, const ds485_driver::DriverConfig &,
                 Ds485Executor::PortFactory factory = {});
  Ds485RosBridge(rclcpp::Node::SharedPtr, const std::string& device_config,
                 const std::array<bool, 2>& gripper_tools);
  ~Ds485RosBridge();
  void start();
  void shutdown();
  void stopActiveTorque() {
    if (torque_) torque_->requestStop();
  }
  void setAdmission(Admission guard) {
    std::atomic_store(&admission_, std::make_shared<const Admission>(std::move(guard)));
  }
  bool busy(unsigned i) const {
    return torque_ ? torque_->busy(i) : executor_->busy(i);
  }
  bool validGoal(const Action::Goal &, std::string &) const;
  void runAction(std::shared_ptr<GoalHandle>);

private:
  void onCommand(const ds485_interfaces::msg::GripperCommand &);
  void publishResult(const Ds485Executor::Command &, const Ds485Executor::Outcome &);
  void publishState(unsigned, const Ds485Executor::State &);
  rclcpp::Node::WeakPtr node_;
  bool admit(unsigned, const std::function<void()>&, std::string&) const;
  std::shared_ptr<const Admission> admission_;
  std::unique_ptr<ds485_driver::EmbeddedTorque> torque_;
  std::array<std::string, 2> last_state_warning_;
  int64_t default_timeout_ms_ = 10000;
  rclcpp::Publisher<ds485_interfaces::msg::GripperResult>::SharedPtr result_pub_;
  rclcpp::Publisher<ds485_interfaces::msg::ServoState>::SharedPtr state_pub_;
  rclcpp::Subscription<ds485_interfaces::msg::GripperCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<ds485_interfaces::msg::GripperStop>::SharedPtr stop_sub_;
  // Destroy the executor before its publishers and callbacks.
  std::unique_ptr<Ds485Executor> executor_;
};
} // namespace motion_control_pkg::gripper
