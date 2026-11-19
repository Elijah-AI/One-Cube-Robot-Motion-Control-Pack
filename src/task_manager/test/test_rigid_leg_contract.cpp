// YAML + real ROS Action wire capture in an isolated localhost test namespace.
// The mock server never constructs a robot module or dispatches motor commands.
#include "task_manager/task_config_parser.hpp"
#include "task_manager/task_action_client.hpp"
#include <future>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
task_cfg::AtomicAction parseAction(const std::string& type, const YAML::Node& params,
                                  const std::string& file, bool expected=true) {
  YAML::Node root;
  root["action_sequencenum"]=1;
  YAML::Node action;
  action["action_type"]=type;
  action["params"]=params;
  root["action_sequence1"].push_back(action);
  {std::ofstream stream(file); stream << root;}
  task_cfg::TaskConfigParser parser;
  task_cfg::GlobalConfig config{};
  const bool loaded=parser.load_task_config(file,config,"offline");
  check(loaded==expected,type+" YAML validation mismatch");
  if (!loaded) return {};
  return config.task_dict.at("offline").action_seq.at(0).at(0);
}
task_cfg::MoveJointsParam parse(const YAML::Node& params, const std::string& file, bool expected=true) {
  return parseAction("move_joints",params,file,expected).move_joints_param;
}
void testLeaderPose(const std::string& file) {
  using namespace std::chrono_literals;
  using Action=com_interfaces::action::MovePose;
  const auto base=YAML::Load("{use_config_pose: true, use_force_control: false, target_pose: [0.4, 0.1, 0.8, 0, 0, 0, 1], bound_size: [0, 0, 0], frame_id: base_link, speed: 0.2, timeout_ms: 30000, keep_grasp_rigid: true, grasp_position_tolerance_m: 0.002}");
  auto server_node=std::make_shared<rclcpp::Node>("offline_rigid_pose_server");
  std::shared_ptr<const Action::Goal> captured;
  auto server=rclcpp_action::create_server<Action>(server_node,"/offline/leader",
      [](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto){return rclcpp_action::CancelResponse::ACCEPT;},
      [&](auto handle){captured=handle->get_goal();auto result=std::make_shared<Action::Result>();
                       result->success=true;handle->succeed(result);});
  std::vector<std::string> args{"--ros-args"};
  for (const auto* endpoint:{"send_goal","get_result","cancel_goal","feedback","status"}) {
    args.push_back("-r");
    args.push_back(std::string("/Control/move_pose/_action/")+endpoint+":=/offline/leader/_action/"+endpoint);
  }
  auto node=std::make_shared<rclcpp::Node>("offline_rigid_pose_client",rclcpp::NodeOptions().arguments(args));
  task_act::TaskActionClient client(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);executor.add_node(node);
  for (const int id:{0,1,5,6}) {
    auto yaml=YAML::Clone(base);yaml["arm_id"]=id;
    const auto pose=parseAction("move_pose",yaml,file).arm_param;
    auto invalid=YAML::Clone(yaml);invalid["enable_move_line"]=true;
    parseAction("move_pose",invalid,file,false);
    auto compound=YAML::Load("{sub_actions: [{action_type: move_pose}, {action_type: move_pose}]}");
    compound["sub_actions"][0]["params"]=yaml;
    compound["sub_actions"][1]["params"]=yaml;
    parseAction("move_compound",compound,file,false);
    captured.reset();
    auto sent=std::async(std::launch::async,[&]{return client.call_move_pose("leader",pose,{},{}, "base_link");});
    const auto deadline=std::chrono::steady_clock::now()+8s;
    while (!captured || sent.wait_for(0s)!=std::future_status::ready ||
           client.get_context("default").state!=task_act::ActionRunState::SUCCESS) {
      executor.spin_some();
      check(std::chrono::steady_clock::now()<deadline,"leader MoveP wire capture timeout");
      std::this_thread::sleep_for(2ms);
    }
    check(sent.get()==task_act::ActionCallResult::SUCCESS && captured->arm_id==id &&
          captured->keep_grasp_rigid && captured->grasp_position_tolerance_m==.002 &&
          captured->target_pose.header.frame_id=="base_link" && std::abs(captured->target_pose.pose.position.x-.4)<1e-6 &&
          captured->left_target_pose.header.frame_id.empty() && captured->right_target_pose.header.frame_id.empty(),
          "leader Goal did not contain exactly one configured TCP target");
  }
  std::cout<<"PASS four single-target MoveP YAML/ROS Action modes and compound conflicts\n";
}
}
int main(int argc,char** argv) {
  try {
    check(argc==2,"Expected temporary YAML path");
    const std::string file=argv[1];
    const auto base=YAML::Load("{arm_id: 4, speed: 0.15, timeout_ms: 30000, enable_collision_check: 1, keep_grasp_rigid: true, grasp_position_tolerance_m: 0.002, grasp_orientation_tolerance_rad: 0.007}");
    std::vector<task_cfg::MoveJointsParam> cases;
    for (const auto& mode : {"body_tilt","vertical_relative","vertical_absolute"}) {
      auto yaml=YAML::Clone(base);
      yaml["leg_motion_mode"]=mode;
      if (std::string(mode)=="vertical_relative") yaml["body_height_m"]=-.02;
      if (std::string(mode)=="vertical_absolute") yaml["body_height_m"]=1.05;
      const auto parsed=parse(yaml,file);
      check(parsed.joint_name.empty() && parsed.position.empty(),"Body mode introduced joint targets");
      check(parsed.body_tilt_deg==0,"Missing tilt no longer defaults to upright");
      cases.push_back(parsed);
    }
    auto tilt=YAML::Clone(base); tilt["leg_motion_mode"]=1; tilt["body_tilt_deg"]=-7.5;
    cases.push_back(parse(tilt,file));
    auto legacy=YAML::Clone(base);
    legacy["joint_name"]=YAML::Load("[leg_joint1,leg_joint2,leg_joint3,body_joint]");
    legacy["position"]=YAML::Load("[-1,2.2,-1.2,0]");
    cases.push_back(parse(legacy,file));
    check(cases.back().leg_motion_mode==0 && cases.back().joint_name.size()==4,"Legacy YAML behavior changed");
    auto defaults=YAML::Clone(legacy);
    defaults.remove("grasp_position_tolerance_m");
    cases.push_back(parse(defaults,file));
    check(cases.back().grasp_position_tolerance_m==.002,"LEG YAML default must be 2 mm");
    auto tighter=YAML::Clone(legacy);
    tighter["grasp_position_tolerance_m"]=.001;
    cases.push_back(parse(tighter,file));
    check(cases.back().grasp_position_tolerance_m==.001,"explicit 1 mm setting lost");
    for(const auto* names:{"[body_joint]","[leg_joint1,leg_joint2]","[leg_joint1,leg_joint2,body_joint]"}) {
      auto yaml=YAML::Clone(base);yaml["joint_name"]=YAML::Load(names);
      for(std::size_t i=0;i<yaml["joint_name"].size();++i) yaml["position"].push_back(.1);
      cases.push_back(parse(yaml,file));
    }
    auto unchanged=YAML::Clone(legacy);unchanged.remove("keep_grasp_rigid");
    cases.push_back(parse(unchanged,file));check(!cases.back().keep_grasp_rigid,"legacy default changed");
    auto relative=YAML::Clone(base);relative["joint_name"]=YAML::Load("[body_joint]");
    relative["position"]=YAML::Load("[0.1]");relative["relative_position"]=true;
    cases.push_back(parse(relative,file));
    for (unsigned kind=0;kind<15;++kind) {
      auto invalid=YAML::Clone(base); invalid["leg_motion_mode"]="body_tilt";
      switch(kind) {
        case 0: invalid["leg_motion_mode"]="unknown"; break;
        case 1: invalid["arm_id"]=0; break;
        case 2: invalid["joint_name"]=YAML::Load("[leg_joint3]"); invalid["position"]=YAML::Load("[0]"); break;
        case 3: invalid["body_tilt_deg"]=181; break;
        case 4: invalid["body_tilt_deg"]=YAML::Load(".nan"); break;
        case 5: invalid["body_height_m"]=.01; break;
        case 6: invalid["leg_motion_mode"]=2; invalid["body_tilt_deg"]=1; break;
        case 7: invalid["relative_position"]=true; break;
        case 8: invalid["speed"]=0; break;
        case 9: invalid["control_mode"]=1; break;
        case 10: invalid["acceleration_scale"]=.5; break;
        case 11: invalid["keep_tcp_level"]=true; break;
        case 12: invalid["synchronization_mode"]=1; break;
        case 13: invalid["grasp_position_tolerance_m"]=0; break;
        case 14: invalid["grasp_orientation_tolerance_rad"]=.2; break;
      }
      parse(invalid,file,false);
    }
    rclcpp::init(argc,argv);
    testLeaderPose(file);
    using Action=com_interfaces::action::MoveJoints;
    using Handle=rclcpp_action::ServerGoalHandle<Action>;
    rclcpp::executors::SingleThreadedExecutor executor;
    auto server_node=std::make_shared<rclcpp::Node>("offline_rigid_leg_contract_server");
    auto publisher=server_node->create_publisher<sensor_msgs::msg::JointState>(
        "/offline/rigid_leg_state",rclcpp::SensorDataQoS());
    auto timer=server_node->create_wall_timer(std::chrono::milliseconds(10),[&] {
      sensor_msgs::msg::JointState state;state.header.stamp=server_node->now();
      state.name={"leg_joint1","leg_joint2","leg_joint3","body_joint"};
      state.position={0,0,0,.2};state.velocity={0,0,0,0};publisher->publish(state);
    });
    std::shared_ptr<const Action::Goal> captured;
    auto server=rclcpp_action::create_server<Action>(server_node,"/offline/rigid_leg_contract",
        [](const rclcpp_action::GoalUUID&,std::shared_ptr<const Action::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<Handle>) {return rclcpp_action::CancelResponse::ACCEPT;},
        [&](std::shared_ptr<Handle> handle) {
          captured=handle->get_goal();
          auto result=std::make_shared<Action::Result>();
          result->success=true;
          result->error_message="offline wire capture only";
          handle->succeed(result);
        });
    executor.add_node(server_node);
    for (std::size_t n=0;n<cases.size();++n) {
      captured.reset();
      rclcpp::NodeOptions options;
      options.parameter_overrides({rclcpp::Parameter("task_joint_state_topic","/offline/rigid_leg_state")});
      // Humble remaps the five Action services/topics after suffix expansion.
      // Remap each endpoint explicitly so no /Control endpoint is contacted.
      options.arguments({"--ros-args",
          "-r","/Control/move_joints/_action/send_goal:=/offline/rigid_leg_contract/_action/send_goal",
          "-r","/Control/move_joints/_action/get_result:=/offline/rigid_leg_contract/_action/get_result",
          "-r","/Control/move_joints/_action/cancel_goal:=/offline/rigid_leg_contract/_action/cancel_goal",
          "-r","/Control/move_joints/_action/feedback:=/offline/rigid_leg_contract/_action/feedback",
          "-r","/Control/move_joints/_action/status:=/offline/rigid_leg_contract/_action/status"});
      auto node=std::make_shared<rclcpp::Node>("offline_rigid_leg_contract_client_"+std::to_string(n),options);
      task_act::TaskActionClient client(node);
      executor.add_node(node);
      if(cases[n].relative_position) {
        const auto ready=std::chrono::steady_clock::now()+std::chrono::milliseconds(300);
        while(std::chrono::steady_clock::now()<ready) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
      }
      auto sent=std::async(std::launch::async,[&] {return client.call_move_joints("offline",cases[n]);});
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
      while (sent.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready || !captured) {
        executor.spin_some();
        check(std::chrono::steady_clock::now()<deadline,"Mock Action transport deadline");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      check(sent.get()==task_act::ActionCallResult::SUCCESS,"Client refused body mode");
      while (client.get_context("default").state!=task_act::ActionRunState::SUCCESS) {
        executor.spin_some();
        check(std::chrono::steady_clock::now()<deadline,"Mock Action result deadline");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      check(captured->keep_grasp_rigid==cases[n].keep_grasp_rigid &&
          captured->grasp_position_tolerance_m==cases[n].grasp_position_tolerance_m &&
          captured->grasp_orientation_tolerance_rad==cases[n].grasp_orientation_tolerance_rad,
          "Rigid compensation fields lost on Action wire");
      check(captured->leg_motion_mode==cases[n].leg_motion_mode &&
          captured->body_tilt_deg==cases[n].body_tilt_deg && captured->body_height_m==cases[n].body_height_m,
          "YAML body target was lost on the Action wire");
      check(captured->joint_name==cases[n].joint_name,"Joint target names changed");
      if(cases[n].relative_position) check(captured->position.size()==1 && std::abs(captured->position[0]-.3)<1e-12,
          "Relative rigid waist must become one absolute waist target");
      else check(captured->position==cases[n].position,"Joint target forwarding changed");
      check(captured->control_mode==0 && captured->arm_id==4 && captured->enable_collision_check==1,
          "Body mode routing or collision policy changed");
      if (cases[n].leg_motion_mode) check(captured->header.frame_id=="base_link","Body target frame lost");
      executor.remove_node(node);
    }
    executor.remove_node(server_node);
    rclcpp::shutdown();
    std::cout << "PASS " << cases.size() << " YAML/ROS Action contracts and 15 invalid YAML cases\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
