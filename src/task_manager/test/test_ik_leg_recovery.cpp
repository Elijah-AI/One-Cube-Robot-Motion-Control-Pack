// Isolated Action servers exercise the production task tree. No robot module
// is constructed; every actuator endpoint and joint topic is remapped offline.
#include "task_manager/behavior_tree_engine.hpp"
#include "task_manager/ik_lift_recovery.hpp"
#include <algorithm>
#include <future>
#include <iostream>
#include <stdexcept>

void check(bool condition,const std::string& message) {if(!condition)throw std::runtime_error(message);}
std::vector<std::string> remaps() {
  std::vector<std::string> args{"--ros-args"};
  for (const auto& action:{"move_pose","move_joints"})
    for (const auto& suffix:{"send_goal","get_result","cancel_goal","feedback","status"}) {
      args.push_back("-r");
      args.push_back(std::string("/Control/")+action+"/_action/"+suffix+":=/offline/ik_recovery/"+action+"/_action/"+suffix);
    }
  return args;
}

int main(int argc,char** argv) {
 try {
  check(argc==2,"Expected task YAML");
  task_cfg::GlobalConfig config{};task_cfg::TaskConfigParser parser;
  check(parser.load_task_config(argv[1],config,"laptop"),"Actual task YAML failed to parse");
  const auto& all=config.task_dict.at("laptop").action_seq.at(0);
  std::vector<std::size_t> lifts,restores;
  unsigned grips=0,closed=0;
  for(std::size_t i=0;i<all.size();++i) {
    if(all[i].ik_failure_recovery.enabled)lifts.push_back(i);
    if(!all[i].if_recovery_applied.empty() && all[i].move_joints_param.leg_motion_mode==2)restores.push_back(i);
    for(const auto& a:all[i].is_parallel_group?all[i].parallel_actions:std::vector<task_cfg::AtomicAction>{all[i]})
      if(a.action_type=="gripper_ctrl") {
        ++grips;check(a.gripper_param.speed==std::vector<uint8_t>{255,255},"Grip speed not 255");
        if(a.gripper_param.position==std::vector<uint8_t>{255,255}) {
          ++closed;check(a.gripper_param.max_torque.size()==2 && a.gripper_param.max_torque[0]>0 && a.gripper_param.max_torque[1]>0,"Closing torque missing");
        }
      }
  }
  check(lifts.size()==3 && restores.size()==3 && grips==12 && closed==3,"Task recovery/grip stage counts");
  for(unsigned r=0;r<3;++r) {
    check(all[lifts[r]].ik_failure_recovery.key==all[restores[r]].if_recovery_applied,"Unpaired round keys");
    check(restores[r]==lifts[r]+2,"Descent is not immediately after turn");
    const auto& turn=all[restores[r]-1].move_joints_param;
    check(turn.joint_name==std::vector<std::string>{"body_joint"} && turn.position==std::vector<double>{-.3},"Wrong restoration turn");
    check(all[restores[r]-1].if_recovery_applied==all[restores[r]].if_recovery_applied,"Turn must be recovery-only");
    check(all[restores[r]+1].move_joints_param.joint_name==std::vector<std::string>{"leg_joint1","leg_joint2","body_joint"},"Missing coordinated leg/waist goal");
  }
  rclcpp::init(argc,argv);
  using Pose=com_interfaces::action::MovePose;
  using Leg=com_interfaces::action::MoveJoints;
  using PH=rclcpp_action::ServerGoalHandle<Pose>;
  using LH=rclcpp_action::ServerGoalHandle<Leg>;
  unsigned scenarios=0;
  // 0 normal, 1 pre-motion IK, 2 completion timeout, 3 invalid feedback,
  // 4 lift failure, 5 retry failure, 6 descent failure, 7 accepted lift delay,
  // 8 IK-looking failure after EXECUTING feedback.
  for(int scenario=0;scenario<9;++scenario) {
    rclcpp::executors::SingleThreadedExecutor executor;
    auto server_node=std::make_shared<rclcpp::Node>("offline_ik_recovery_server_"+std::to_string(scenario));
    rclcpp::NodeOptions options;options.arguments(remaps());
    options.parameter_overrides({{"task_joint_state_topic","/offline/ik_recovery/joints"}});
    auto client_node=std::make_shared<rclcpp::Node>("offline_ik_recovery_client_"+std::to_string(scenario),options);
    task_act::TaskActionClient client(client_node);
    auto joints=server_node->create_publisher<sensor_msgs::msg::JointState>("/offline/ik_recovery/joints",10);
    std::vector<std::string> events;
    std::vector<Pose::Goal> pose_goals;
    unsigned arm_calls=0;
    double lifted_metres=0;
    std::shared_ptr<LH> delayed_lift;
    std::shared_ptr<PH> delayed_pose;
    auto poses=rclcpp_action::create_server<Pose>(server_node,"/offline/ik_recovery/move_pose",
      [](const rclcpp_action::GoalUUID&,std::shared_ptr<const Pose::Goal>){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](std::shared_ptr<PH>){return rclcpp_action::CancelResponse::ACCEPT;},
      [&](std::shared_ptr<PH> h){
        const auto g=h->get_goal();pose_goals.push_back(*g);
        auto result=std::make_shared<Pose::Result>();
        if(g->arm_id==7) {
          ++arm_calls;events.push_back(arm_calls==1?"lift_arms":"retry_arms");
          if(arm_calls==1 && scenario!=0) {
            if(scenario==8) {
              auto feedback=std::make_shared<Pose::Feedback>();feedback->dual_task_status=Pose::Feedback::STATUS_EXECUTING;
              h->publish_feedback(feedback);delayed_pose=h;return;
            }
            result->error_message=scenario==2?"whole-body completion timed out: max_position_error_rad=0.016186":
              scenario==3?"WBC feedback incomplete, stale or time-skewed":
              "failed to solve dual-arm whole-body IK at Cartesian substep 6/14";
            h->abort(result);return;
          }
          if(arm_calls>1 && scenario==5) {
            result->error_message="failed to solve dual-arm whole-body IK at Cartesian substep 1/14";h->abort(result);return;
          }
        } else events.push_back("place");
        result->success=true;h->succeed(result);
      });
    auto legs=rclcpp_action::create_server<Leg>(server_node,"/offline/ik_recovery/move_joints",
      [](const rclcpp_action::GoalUUID&,std::shared_ptr<const Leg::Goal>){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](std::shared_ptr<LH>){return rclcpp_action::CancelResponse::ACCEPT;},
      [&](std::shared_ptr<LH> h){
        const auto g=h->get_goal();auto result=std::make_shared<Leg::Result>();
        if(g->leg_motion_mode==2) {
          check(g->arm_id==4 && g->joint_name.empty() && g->position.empty(),"Recovery incorrectly sent joint-space targets");
          check(g->header.frame_id=="base_link" && g->enable_collision_check==1,"Recovery lost frame or collision mode");
          if(g->body_height_m>0) {
            events.push_back("leg_up");check(g->body_height_m==.05,"Lift not 5 cm");
            if(scenario==4){result->error_message="Unreachable body height";h->abort(result);return;}
            if(scenario==7){delayed_lift=h;return;}
          } else {
            events.push_back("leg_down");check(g->body_height_m==-.05,"Descent not 5 cm");
            if(scenario==6){result->error_message="Body target not reached";h->abort(result);return;}
          }
          lifted_metres+=g->body_height_m;
        } else if (g->joint_name.size()==3) {
          events.push_back("coordinated");
          check(g->joint_name==std::vector<std::string>{"leg_joint1","leg_joint2","body_joint"} &&
                g->position==std::vector<double>{-.65,2.0,-.3},"Coordinated target changed in dispatch");
        } else {events.push_back("turn");check(g->position.size()==4 && g->position[3]==-.3,"Turn target changed");}
        result->success=true;h->succeed(result);
      });
    executor.add_node(server_node);executor.add_node(client_node);
    com_interfaces::action::DetectionPoseCloud::Result vision;
    vision.success=true;vision.header.frame_id="base_link";vision.total_detected_count=1;
    com_interfaces::msg::TargetDetection target;target.obj_name="rack";target.confidence=.99;
    target.pose.header.frame_id="base_link";target.pose.pose.position.x=.75;target.pose.pose.position.z=.8;
    target.pose.pose.orientation.w=1;vision.detected_targets.push_back(target);
    client.store_vision_result_at_seq_key("seq1",vision,"rack");
    auto place=task_cfg::AtomicAction{};place.action_type="move_pose";place.delay_ms=0;
    place.arm_param.arm_id=0;place.arm_param.use_config_pose=true;
    place.arm_param.config_pose={1,0,1,0,0,0,1};place.arm_param.bound_size={0,0,0};
    place.arm_param.frame_id="base_link";place.arm_param.speed=.1;place.arm_param.timeout_ms=30000;
    std::vector<std::vector<task_cfg::AtomicAction>> group{{all[lifts[0]],all[restores[0]-1],all[restores[0]],all[restores[0]+1],place}};
    BehaviorTreeEngine tree(client_node->get_logger());tree.load_parallel_task(group,client);tree.start();
    const auto spin=[&]{
      sensor_msgs::msg::JointState state;state.header.stamp=server_node->now();
      state.name={"leg_joint1","leg_joint2","leg_joint3","body_joint"};
      state.position={-1,2.2,-1.2258046865463257,0};state.velocity={0,0,0,0};joints->publish(state);
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));
    };
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(12);
    unsigned delayed_ticks=0;
    while(tree.isTaskRunning()) {
      auto tick=std::async(std::launch::async,[&]{tree.tick_once();});
      while(tick.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready) {
        spin();check(std::chrono::steady_clock::now()<deadline,"Task Action tick deadline");
      }
      tick.get();spin();
      if(delayed_lift) {
        check(arm_calls==1 && client.recovery_offset("seq1",all[lifts[0]].ik_failure_recovery.key)==0,
          "Accepted lift triggered arm retry or marked recovery too early");
        if(++delayed_ticks==10) {
          auto result=std::make_shared<Leg::Result>();result->success=true;lifted_metres+=.05;
          delayed_lift->succeed(result);delayed_lift.reset();
        }
      }
      if (delayed_pose) {
        auto feedback=std::make_shared<Pose::Feedback>();
        feedback->dual_task_status=Pose::Feedback::STATUS_EXECUTING;
        delayed_pose->publish_feedback(feedback);
      }
      if(delayed_pose && ++delayed_ticks==10) {
        auto result=std::make_shared<Pose::Result>();result->error_message="failed to solve dual-arm whole-body IK at Cartesian substep 6/14";
        delayed_pose->abort(result);delayed_pose.reset();
      }
      check(std::chrono::steady_clock::now()<deadline,"Recovery task deadline");
    }
    const bool success=scenario==0 || scenario==1 || scenario==7;
    check(tree.isTaskSuccess()==success,"Recovery final task status");
    if(scenario==0)check(events==std::vector<std::string>{"lift_arms","coordinated","place"},"Normal path performed leg compensation");
    if(scenario==1 || scenario==7) {
      check(events==std::vector<std::string>{"lift_arms","leg_up","retry_arms","turn","leg_down","coordinated","place"},"Recovery ordering changed");
      check(lifted_metres==0 && client.recovery_offset("seq1",all[lifts[0]].ik_failure_recovery.key)==0,"Recovery height not restored");
      check(pose_goals[0].left_target_pose.pose==pose_goals[1].left_target_pose.pose &&
            pose_goals[0].right_target_pose.pose==pose_goals[1].right_target_pose.pose &&
            pose_goals[0].left_target_pose.header.frame_id==pose_goals[1].left_target_pose.header.frame_id,
            "Retry changed vision-derived targets");
    }
    if(scenario==2 || scenario==3 || scenario==8)check(events==std::vector<std::string>{"lift_arms"},"Non-planning failure sent recovery motion");
    if(scenario==4)check(events==std::vector<std::string>{"lift_arms","leg_up"} && lifted_metres==0,"Failed lift continued arms");
    if(scenario==5)check(events==std::vector<std::string>{"lift_arms","leg_up","retry_arms"} && lifted_metres==.05,"Failed retry looped or continued to place");
    if(scenario==6)check(events==std::vector<std::string>{"lift_arms","leg_up","retry_arms","turn","leg_down"},"Failed descent continued to place");
    check(client.recovery_offset("seq2",all[lifts[0]].ik_failure_recovery.key)==0,"Recovery leaked between sequences");
    client.reset_all_states();check(client.recovery_offset("seq1",all[lifts[0]].ik_failure_recovery.key)==0,"Task reset retained a stale recovery flag");
    executor.remove_node(client_node);executor.remove_node(server_node);++scenarios;
  }
  rclcpp::shutdown();
  std::cout<<"IK_LEG_RECOVERY_INTEGRATION_PASS scenarios="<<scenarios<<" grips="<<grips<<" closed="<<closed<<"\n";
 }catch(const std::exception& error){std::cerr<<"FAIL: "<<error.what()<<"\n";if(rclcpp::ok())rclcpp::shutdown();return 1;}
}
