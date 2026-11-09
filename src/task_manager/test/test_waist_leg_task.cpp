// Production task tree and client, isolated fake Action endpoints only.
#include "task_manager/behavior_tree_engine.hpp"
#include <future>
#include <iostream>
#include <stdexcept>

void check(bool ok, const std::string& why) { if (!ok) throw std::runtime_error(why); }

int main(int argc, char** argv) try {
  check(argc == 2, "expected task YAML");
  task_cfg::GlobalConfig config;
  task_cfg::TaskConfigParser parser;
  check(parser.load_task_config(argv[1], config, "laptop"), "actual YAML rejected");
  std::vector<task_cfg::AtomicAction> groups;
  for (const auto& a : config.task_dict.at("laptop").action_seq.at(0))
    if (a.is_parallel_group && std::any_of(a.parallel_actions.begin(), a.parallel_actions.end(),
        [](const auto& child) { return child.action_type == "gripper_ctrl"; })) groups.push_back(a);
  check(groups.size() == 3, "expected three parallel pre-position groups");
  rclcpp::init(argc, argv);
  using J = com_interfaces::action::MoveJoints;
  using G = com_interfaces::action::ControlGripper;
  using JH = rclcpp_action::ServerGoalHandle<J>;
  using GH = rclcpp_action::ServerGoalHandle<G>;
  for (std::size_t scenario=0; scenario<5; ++scenario) {
    const std::size_t round=scenario<3 ? scenario : scenario-2;
    const bool failure=scenario>=3;
    rclcpp::NodeOptions options;
    std::vector<std::string> args{"--ros-args"};
    for (const auto& action : {"move_joints", "control_gripper"})
      for (const auto& suffix : {"send_goal", "get_result", "cancel_goal", "feedback", "status"}) {
        args.push_back("-r");
        args.push_back(std::string("/Control/") + action + "/_action/" + suffix +
                       ":=/offline/waist_leg/" + action + "/_action/" + suffix);
      }
    options.arguments(args);
    options.parameter_overrides({{"task_joint_state_topic", "/offline/waist_leg/joints"}});
    auto node = std::make_shared<rclcpp::Node>("waist_leg_task_client_" + std::to_string(scenario), options);
    auto server = std::make_shared<rclcpp::Node>("waist_leg_fake_server_" + std::to_string(scenario));
    task_act::TaskActionClient client(node);
    std::shared_ptr<JH> pending_joints, pending_arms;
    std::shared_ptr<GH> pending_grip;
    unsigned after_calls = 0, joint_calls = 0, grip_calls = 0, arm_calls = 0;
    auto joints_server = rclcpp_action::create_server<J>(server, "/offline/waist_leg/move_joints",
      [](const auto&, auto) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
      [&](std::shared_ptr<JH> h) {
        const auto goal = h->get_goal();
        if (goal->arm_id == 2) {
          ++arm_calls;check(round>0,"unexpected arms in initial turn group");
          const auto& expected=groups[round].parallel_actions[0].move_joints_param;
          check(std::equal(goal->left_position.begin(),goal->left_position.end(),expected.left_position.begin()) &&
                std::equal(goal->right_position.begin(),goal->right_position.end(),expected.right_position.begin()),
                "pre-grasp arm targets changed in dispatch");
          pending_arms=h;return;
        }
        if (goal->arm_id == 0) {
          ++after_calls;
          auto result = std::make_shared<J::Result>(); result->success = true; h->succeed(result);
          return;
        }
        ++joint_calls;
        check(goal->arm_id == 4 && goal->position.back() == -1.5707963267948966, "waist target changed");
        check(goal->joint_name.size() == (round == 0 ? 4U : 3U), "coordinated goal not forwarded");
        if (round != 0) check(goal->position == std::vector<double>{-1.0, 2.2, -1.5707963267948966}, "retreat target changed");
        pending_joints = h;
      });
    auto grip_server = rclcpp_action::create_server<G>(server, "/offline/waist_leg/control_gripper",
      [](const auto&, auto) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
      [&](std::shared_ptr<GH> h) {
        ++grip_calls;
        check(h->get_goal()->gripper_id == 2 && h->get_goal()->position == std::vector<uint8_t>{140,140},
              "pre-position target changed");
        pending_grip = h;
      });
    auto pub = server->create_publisher<sensor_msgs::msg::JointState>("/offline/waist_leg/joints", 10);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node); executor.add_node(server);
    task_cfg::AtomicAction after{}; after.action_type = "move_joints";
    after.move_joints_param.arm_id = 0;
    after.move_joints_param.joint_name = {"Joint1_L","Joint2_L","Joint3_L","Joint4_L","Joint5_L","Joint6_L","Joint7_L"};
    after.move_joints_param.position = {0,0,0,0,0,0,0};
    after.move_joints_param.speed = .1; after.move_joints_param.timeout_ms = 30000;
    BehaviorTreeEngine tree(node->get_logger());
    std::vector<std::vector<task_cfg::AtomicAction>> sequence{{groups[round], after}};
    tree.load_parallel_task(sequence, client); tree.start();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(12);
    const auto spin = [&] {
      sensor_msgs::msg::JointState state; state.header.stamp = server->now();
      state.name = {"leg_joint1","leg_joint2","leg_joint3","body_joint"};
      state.position = {-1,2.2,-1.2258046865463257,0}; state.velocity = {0,0,0,0};
      pub->publish(state); executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(2));
      check(std::chrono::steady_clock::now() < deadline, "parallel scheduling timed out");
    };
    const auto tick = [&] {
      auto f = std::async(std::launch::async, [&] { tree.tick_once(); });
      while (f.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) spin();
      f.get(); spin();
    };
    for (unsigned i = 0; i < 25; ++i) spin();
    while (!pending_joints || !pending_grip || (round>0 && !pending_arms)) tick();
    check(after_calls==0 && joint_calls==1 && grip_calls==1 && arm_calls==(round>0?1U:0U),
          "three branches did not start concurrently or dispatched duplicate goals");
    auto succeed_joints=[](const std::shared_ptr<JH>& h) {
      auto result=std::make_shared<J::Result>();result->success=true;h->succeed(result);
    };
    auto succeed_grip=[&] {
      auto result=std::make_shared<G::Result>();result->success=true;
      result->final_position={140,140};result->final_torque={0,0};pending_grip->succeed(result);
    };
    if (failure) {
      auto result=std::make_shared<J::Result>();result->success=false;
      result->error_message="offline injected parallel branch failure";
      if (scenario==3) pending_arms->abort(result);else pending_joints->abort(result);
      for (unsigned i=0;i<10;++i) tick();
      check(tree.isTaskRunning() && after_calls==0,"failure did not wait for started siblings");
      if (scenario==3) succeed_joints(pending_joints);else succeed_joints(pending_arms);
      succeed_grip();
    } else if (round==2) {
      succeed_grip();succeed_joints(pending_joints);
      for (unsigned i=0;i<10;++i) tick();
      check(tree.isTaskRunning() && after_calls==0,"next action did not wait for arms");
      succeed_joints(pending_arms);
    } else {
      if (pending_arms) succeed_joints(pending_arms);
      succeed_joints(pending_joints);
      for (unsigned i=0;i<10;++i) tick();
      check(tree.isTaskRunning() && after_calls==0,"next action did not wait for gripper");
      succeed_grip();
    }
    while (tree.isTaskRunning()) tick();
    check(tree.isTaskSuccess()==!failure && after_calls==(failure?0U:1U),
          "parallel success/failure barrier did not control next action");
    executor.remove_node(node); executor.remove_node(server);
  }
  rclcpp::shutdown();
  std::cout << "PASS five cases: three parallel groups, arms/body/gripper concurrent dispatch, last-child barriers, failure drains siblings and blocks next action\n";
  return 0;
} catch (const std::exception& e) {
  std::cerr << "FAIL " << e.what() << '\n';
  if (rclcpp::ok()) rclcpp::shutdown();
  return 1;
}
