// Parse production YAML and capture production clients' Goals on isolated mock Actions.
#include "task_manager/task_config_parser.hpp"
#include "task_manager/task_action_client.hpp"
#include <future>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}
task_cfg::AtomicAction parse(const YAML::Node& params, bool pose,
    const std::string& file, bool expected = true) {
  YAML::Node root, action;
  root["action_sequencenum"] = 1;
  action["action_type"] = pose ? "move_pose" : "move_joints";
  action["params"] = params;
  root["action_sequence1"].push_back(action);
  { std::ofstream out(file); out << root; }
  task_cfg::TaskConfigParser parser;
  task_cfg::GlobalConfig config{};
  check(parser.load_task_config(file, config, "offline") == expected,
        "precision YAML acceptance mismatch");
  if (!expected) return {};
  return config.task_dict.at("offline").action_seq.at(0).at(0);
}
YAML::Node params(bool pose, unsigned id) {
  auto p = YAML::Load(R"({speed: 0.2, timeout_ms: 5000, enable_collision_check: 1,
    use_config_pose: true, use_force_control: false, frame_id: base_link,
    target_pose: [0.3, 0, 1, 0, 0, 0, 1], bound_size: [0, 0, 0],
    left_target_pose: [0.3, 0.2, 1, 0, 0, 0, 1], right_target_pose: [0.3, -0.2, 1, 0, 0, 0, 1],
    left_bound_size: [0, 0, 0], right_bound_size: [0, 0, 0], synchronization_mode: 0,
    left_joint_name: [Joint1_L, Joint2_L, Joint3_L, Joint4_L, Joint5_L, Joint6_L, Joint7_L],
    right_joint_name: [Joint1_R, Joint2_R, Joint3_R, Joint4_R, Joint5_R, Joint6_R, Joint7_R],
    left_position: [0, 0, 0, 0, 0, 0, 0], right_position: [0, 0, 0, 0, 0, 0, 0]})");
  p["arm_id"] = id;
  if (!pose) {
    p["joint_name"] = id == 4 ? YAML::Load("[leg_joint1,leg_joint2,leg_joint3,body_joint]") :
      id == 3 ? YAML::Load("[head_yaw_joint,head_pitch_joint]") : p[id == 1 ? "right_joint_name" : "left_joint_name"];
    p["position"] = std::vector<double>(p["joint_name"].size(), 0.0);
  }
  return p;
}
template<class Action> void capture(const task_cfg::AtomicAction& action, bool enabled) {
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  const bool pose = action.action_type == "move_pose";
  const std::string endpoint = "/Control/" + action.action_type;
  const std::string offline = "/offline/precision_" + action.action_type;
  rclcpp::NodeOptions options;
  std::vector<std::string> arguments{"--ros-args"};
  for (const auto* suffix : {"send_goal", "get_result", "cancel_goal", "feedback", "status"}) {
    arguments.push_back("-r");
    arguments.push_back(endpoint + "/_action/" + suffix + ":=" + offline + "/_action/" + suffix);
  }
  options.arguments(arguments);
  auto client_node = std::make_shared<rclcpp::Node>("offline_precision_client", options);
  auto server_node = std::make_shared<rclcpp::Node>("offline_precision_server");
  task_act::TaskActionClient client(client_node);
  std::shared_ptr<const typename Action::Goal> captured;
  auto server = rclcpp_action::create_server<Action>(server_node, offline,
    [](const rclcpp_action::GoalUUID&, std::shared_ptr<const typename Action::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }, [](std::shared_ptr<Handle>) { return rclcpp_action::CancelResponse::ACCEPT; },
    [&](std::shared_ptr<Handle> handle) {
      captured = handle->get_goal();
      auto result = std::make_shared<typename Action::Result>();
      result->success = true;
      handle->succeed(result);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client_node);
  executor.add_node(server_node);
  auto sent = std::async(std::launch::async, [&] {
    return pose ? client.call_move_pose("offline", action.arm_param,
      geometry_msgs::msg::PoseStamped{}, sensor_msgs::msg::PointCloud2{}, "base_link") :
      client.call_move_joints("offline", action.move_joints_param);
  });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (sent.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready || !captured ||
      client.get_context("default").state != task_act::ActionRunState::SUCCESS) {
    executor.spin_some();
    check(std::chrono::steady_clock::now() < deadline, "offline Action timed out");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  check(sent.get() == task_act::ActionCallResult::SUCCESS, "client rejected test Goal");
  check(captured->precision_arrival == enabled, "precision flag lost on Action wire");
  check(captured->arm_id == (pose ? action.arm_param.arm_id : action.move_joints_param.arm_id) &&
    captured->enable_collision_check == 1 && captured->timeout_ms == 5000 &&
    captured->control_mode == 0, "unrelated Goal routing changed");
}
}

int main(int argc, char** argv) {
  try {
    check(argc == 2, "expected scratch YAML path");
    rclcpp::init(argc, argv);
    unsigned accepted = 0, rejected = 0;
    for (bool pose : {false, true}) for (unsigned id = 0; id <= (pose ? 7U : 4U); ++id) {
      const bool supported = pose ? id >= 4 : id >= 3;
      for (int setting : {-1, 0, 1}) {
        auto yaml = params(pose, id);
        if (setting >= 0) yaml["precision_arrival"] = setting == 1;
        const bool enabled = setting == 1;
        const bool valid = !enabled || supported;
        const auto action = parse(yaml, pose, argv[1], valid);
        if (!valid) { ++rejected; continue; }
        check((pose ? action.arm_param.precision_arrival : action.move_joints_param.precision_arrival)
              == enabled, "parser flag mismatch");
        if (pose) capture<com_interfaces::action::MovePose>(action, enabled);
        else capture<com_interfaces::action::MoveJoints>(action, enabled);
        ++accepted;
      }
      if (supported) {
        auto invalid = params(pose, id);
        invalid["precision_arrival"] = true;
        invalid["control_mode"] = 1;
        parse(invalid, pose, argv[1], false); ++rejected;
        invalid.remove("control_mode");
        invalid["precision_arrival"] = "not_a_bool";
        parse(invalid, pose, argv[1], false); ++rejected;
      }
    }
    rclcpp::shutdown();
    std::cout << "PASS precision_arrival: " << accepted << " YAML-to-Action cases, "
              << rejected << " invalid configurations rejected; localhost mock only\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
