#include "whole_body_hardware_traps.hpp"
#include "rigid_grasp_fixture.hpp"
#include "motion_control_pkg/execution/motion_safety.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <mutex>
#include <iostream>

using namespace motion_control_pkg::execution;
static void require(bool ok,const std::string& e) {if(!ok) throw std::runtime_error(e);}

class FakeLegBackend final : public WholeBodyExecutionBackend {
public:
  std::shared_ptr<MiracleMAlgo> model;
  rclcpp::Node::SharedPtr node;
  std::vector<double> q;
  std::mutex mutex;
  std::string scenario;
  std::array<std::vector<std::string>,4> names;
  std::array<std::uint64_t,4> seq{};
  std::atomic<int> reservations{0},dispatches{0},canceled_children{0};
  std::atomic<bool> injected{false};
  Time arm_start{},leg_start{};
  FakeLegBackend(std::shared_ptr<MiracleMAlgo> m,rclcpp::Node::SharedPtr n,std::vector<double> start,std::string mode)
    :model(std::move(m)),node(std::move(n)),q(std::move(start)),scenario(std::move(mode)) {
    for(unsigned a=0;a<2;++a) for(int j=1;j<=7;++j) names[a].push_back("Joint"+std::to_string(j)+(a?"_R":"_L"));
    names[2]={"leg_joint1","leg_joint2","leg_joint3","body_joint"};
    names[3]={"head_yaw_joint","head_pitch_joint"};
  }
  int index(const std::string& name) const {const auto& m=model->getModel();return m.joints[m.getJointId(name)].idx_q();}
  bool reserve_arms(bool l,bool r,std::string&) override {require(l&&r,"must reserve both arms");reservations+=2;return true;}
  bool reserve_legs(std::string&) override {++reservations;return true;}
  bool reserve_head(std::string& e) override {
    if(scenario=="head_busy") {e="injected head busy";return false;}
    ++reservations;return true;
  }
  void release_arms(bool,bool) noexcept override {reservations-=2;}
  void release_legs() noexcept override {--reservations;}
  void release_head() noexcept override {--reservations;}
  bool arm_limits(bool,std::vector<double>& v,std::vector<double>& a,std::vector<double>& j) const override {
    v.assign(7,.6);a.assign(7,1.2);j.assign(7,6);return true;
  }
  bool read_body_state(sensor_msgs::msg::JointState& out,std::string&) override {
    std::lock_guard<std::mutex> lock(mutex);
    out.header.stamp=node->now()-rclcpp::Duration::from_seconds(scenario=="stale_start"?1.:0.);
    for(const auto& channel:names) for(const auto& name:channel) {
      if(scenario=="missing_start" && name=="Joint7_R") continue;
      out.name.push_back(name);out.position.push_back(q[index(name)]);
      out.velocity.push_back(scenario=="moving_start"?.1:0);
    }
    return true;
  }
  bool prepare_dispatch(const Path& l,const Path& r,const Path& p,std::uint8_t mode,std::string& e) override {
    require(reservations==4 && mode==0,"dispatch without all resource reservations");
    require(l.size()==r.size() && l.size()==p.size(),"unsynchronized path lengths");
    if(scenario=="prepare_failure") {e="injected preparation failure";return false;}
    if(scenario=="start_changed") {std::lock_guard<std::mutex> lock(mutex);q[index("body_joint")]+=.03;}
    return true;
  }
  bool send(unsigned channel,const std::vector<double>& target,std::string& error) {
    auto permit=motionSafety()->command(1U<<channel,error);
    if(!permit || !permit.validate(1U<<channel,target)) return false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if(scenario!="not_at_target") for(std::size_t j=0;j<target.size();++j) q[index(names[channel][j])]=target[j];
    }
    return permit.committed(1U<<channel,target);
  }
  bool delay(const Cancel& cancel,std::string& error) {
    for(int i=0;i<40;++i) {
      if(i==5 && (scenario=="grasp_violation" || scenario=="head_moved") && !injected.exchange(true)) {
        std::lock_guard<std::mutex> lock(mutex);
        q[index(scenario=="head_moved"?"head_pitch_joint":"Joint2_R")]+=.03;
      }
      if(cancel->requested()) {++canceled_children;error="injected child observed cancel";return false;}
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
  }
  bool execute_arms(const Path& l,const Path& r,std::uint8_t,Time start,const Cancel& cancel,std::string& error) override {
    arm_start=start;++dispatches;std::this_thread::sleep_until(start);
    if(scenario=="arm_failure") {error="injected arm transport failure";return false;}
    return delay(cancel,error) && send(0,l.back(),error) && send(1,r.back(),error);
  }
  bool execute_legs(const Path& p,Time start,const Cancel& cancel,std::string& error) override {
    leg_start=start;++dispatches;std::this_thread::sleep_until(start);
    if(scenario=="child_failure") {error="injected leg transport failure";return false;}
    return delay(cancel,error) && send(2,p.back(),error);
  }
  bool safety_sample(unsigned channel,SafetySample& s,std::string&) {
    std::lock_guard<std::mutex> lock(mutex);
    s.acquired=SafetyClock::now()-std::chrono::milliseconds(scenario=="stale_runtime" && dispatches>0?1000:0);
    s.sequence=++seq[channel];
    for(const auto& name:names[channel]) {s.position.push_back(q[index(name)]);s.velocity.push_back(0);}
    return true;
  }
  bool read_completion_feedback(WholeBodyCompletionFeedback& out,std::string&) override {
    std::lock_guard<std::mutex> lock(mutex);
    for(unsigned arm=0;arm<2;++arm) {
      auto& f=out.arms.arms[arm];f.current_state=1;f.low_speed=scenario!="not_stopped";
      f.feedback_sequence=++seq[arm];f.acquired=SafetyClock::now();
      for(unsigned j=0;j<7;++j) f.joint_position[j]=q[index(names[arm][j])]*180/3.141592653589793;
    }
    out.leg_sequence=++seq[2];
    for(unsigned j=0;j<4;++j) out.leg_positions[j]=q[index(names[2][j])];
    return true;
  }
};

int main(int argc,char** argv) try {
  require(argc==2,"expected repository path");
  const std::string repo=argv[1];
  auto model=std::make_shared<MiracleMAlgo>(repo+"/src/onecube_robot_description/urdf/onecube_robot_description.urdf",repo+"/src");
  model->setCollisionCheckMode(0);
  RigidFixture fixture(*model);
  const auto initial=fixture.start;
  rclcpp::init(0,nullptr);
  unsigned cases=0;
  for(const std::string scenario:{"success","head_busy","stale_start","missing_start","moving_start","prepare_failure","start_changed","child_failure","arm_failure","grasp_violation","head_moved","stale_runtime","cancel","not_at_target","not_stopped"}) {
    auto node=std::make_shared<rclcpp::Node>("leg_rigid_offline_"+scenario);
    auto backend=std::make_shared<FakeLegBackend>(model,node,initial,scenario);
    auto safety=std::make_shared<MotionSafety>();
    const auto& pm=model->getModel();
    for(unsigned channel=0;channel<4;++channel) {
      std::vector<SafetyJoint> joints;
      for(const auto& n:backend->names[channel]) {
        const auto& j=pm.joints[pm.getJointId(n)];
        joints.push_back({n,pm.lowerPositionLimit[j.idx_q()],pm.upperPositionLimit[j.idx_q()],pm.velocityLimit[j.idx_v()]});
      }
      safety->addChannel(1U<<channel,joints,[backend,channel](auto& s,auto& e){return backend->safety_sample(channel,s,e);});
    }
    installMotionSafety(safety);safety->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    WholeBodyModule whole(node,model,nullptr,nullptr,"base_link",fixture.left_tool,fixture.right_tool,backend);
    auto server=rclcpp_action::create_server<MoveJoints>(node,"/offline_leg_rigid",
      [](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto){return rclcpp_action::CancelResponse::ACCEPT;},
      [&](auto handle){whole.start_execution_move_joints(handle);});
    auto client=rclcpp_action::create_client<MoveJoints>(node,"/offline_leg_rigid");
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
    std::thread spin([&]{executor.spin();});
    const auto clean=[&] {executor.cancel();if(spin.joinable()) spin.join();safety->stop();installMotionSafety(nullptr);};
    try {
      require(client->wait_for_action_server(std::chrono::seconds(3)),"no local test action server");
      MoveJoints::Goal goal;goal.arm_id=4;goal.keep_grasp_rigid=true;goal.joint_name={"body_joint"};
      goal.position={scenario=="not_at_target"?.05:.01};goal.speed=.5;goal.timeout_ms=8000;
      auto accepted=client->async_send_goal(goal);
      require(accepted.wait_for(std::chrono::seconds(3))==std::future_status::ready,"goal acceptance timeout");
      auto handle=accepted.get();require(bool(handle),"test goal rejected");
      auto result=client->async_get_result(handle);
      if(scenario=="cancel") {
        const auto until=Time::clock::now()+std::chrono::seconds(3);
        while(backend->dispatches<2 && Time::clock::now()<until) std::this_thread::sleep_for(std::chrono::milliseconds(2));
        auto canceled=client->async_cancel_goal(handle);
        require(canceled.wait_for(std::chrono::seconds(2))==std::future_status::ready,"cancel response timeout");
      }
      require(result.wait_for(std::chrono::seconds(10))==std::future_status::ready,"Action terminal timeout");
      const auto outcome=result.get();
      const auto until=Time::clock::now()+std::chrono::seconds(1);
      while(!whole.is_idle() && Time::clock::now()<until) std::this_thread::sleep_for(std::chrono::milliseconds(2));
      require(whole.is_idle() && backend->reservations==0,"leaked action resources");
      require(outcome.result->success==(scenario=="success"),"unexpected result: "+scenario+": "+outcome.result->error_message);
      if(scenario=="success") require(outcome.code==rclcpp_action::ResultCode::SUCCEEDED && backend->arm_start==backend->leg_start,"common start or success result missing");
      if(scenario=="cancel") require(outcome.code==rclcpp_action::ResultCode::CANCELED && backend->canceled_children==2,"children did not both cancel");
      if(scenario=="child_failure") require(backend->canceled_children>=1,"arm worker continued after leg failure");
      if(scenario=="arm_failure") require(backend->canceled_children>=1,"leg worker continued after arm failure");
      if(scenario=="grasp_violation" || scenario=="head_moved" || scenario=="stale_runtime")
        require(backend->canceled_children==2,"both workers must cancel on measured state fault");
      if(scenario=="head_busy" || scenario=="stale_start" || scenario=="missing_start" || scenario=="moving_start" || scenario=="prepare_failure" || scenario=="start_changed")
        require(backend->dispatches==0,"sent samples after pre-dispatch rejection");
      std::cout<<"PASS LEG Action "<<scenario<<" dispatches="<<backend->dispatches<<" result="<<outcome.result->error_message<<std::endl;
      clean();++cases;
    } catch(...) {clean();throw;}
  }
  rclcpp::shutdown();std::cout<<"TOTAL "<<cases<<" LEG Action cases PASS; simulated feedback only\n";return 0;
} catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';if(rclcpp::ok()) rclcpp::shutdown();return 1;}
