// Offline ROS host for the same bridge created by DualArmModule. Only PTYs are accepted here.
#include "motion_control_pkg/gripper/ds485_ros_bridge.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 4 || std::string(argv[2]) != "--params-file") return 2;
  const auto config = YAML::LoadFile(argv[3])["ds485_node"]["ros__parameters"];
  for (const auto* side : {"left", "right"}) {
    const auto values = config[side];
    if (values["port_type"].as<std::string>() != "serial" ||
        values["serial_device"].as<std::string>().rfind("/dev/pts/",0) != 0) return 2;
  }
  rclcpp::init(0, nullptr);
  auto host = std::make_shared<rclcpp::Node>("offline_motion_torque_host");
  using Bridge = motion_control_pkg::gripper::Ds485RosBridge;
  Bridge bridge(host, argv[3], {true, true});
  Bridge::Action::Goal goal;
  std::string why;
  if (bridge.validGoal(goal, why) || why.find("POSITION_MODE_REQUIRED") == std::string::npos) return 1;
  std::atomic<bool> admitted{false};
  using Gate = std_srvs::srv::SetBool;
  auto gate = host->create_service<Gate>("/offline/gripper_admit",
      [&](Gate::Request::SharedPtr q, Gate::Response::SharedPtr r) { admitted=q->data; r->success=true; });
  using Busy = std_srvs::srv::Trigger;
  auto busy = host->create_service<Busy>("/offline/left_busy",
      [&](Busy::Request::SharedPtr, Busy::Response::SharedPtr r) { r->success = bridge.busy(0); });
  auto stop = host->create_service<Busy>("/offline/host_stop",
      [&](Busy::Request::SharedPtr, Busy::Response::SharedPtr r) {
        admitted = false;
        bridge.stopActiveTorque();
        r->success = true;
      });
  bridge.setAdmission([&](unsigned, const std::function<void()>& dispatch, std::string& error) {
    if (!admitted) { error="HOST_DENIED"; return false; }
    dispatch(); return true;
  });
  bridge.start();
  rclcpp::spin(host);
  bridge.shutdown();
  return 0;
}
