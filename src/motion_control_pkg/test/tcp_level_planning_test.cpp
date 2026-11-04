#include "motion_control_pkg/whole_body_module.hpp"
#include "motion_control_pkg/motion/tcp_level_plan.hpp"
#include "motion_control_pkg/kinematics/transform_utils.hpp"
#include <iostream>
#include <stdexcept>

// Hardware entry points are traps; this executable links no vendor drivers.
[[noreturn]] static void hardware() {throw std::runtime_error("UNEXPECTED HARDWARE ACCESS");}
using Path=std::vector<std::vector<double>>;
using Time=std::chrono::steady_clock::time_point;
using Cancel=std::shared_ptr<motion_control_pkg::execution::CancellationToken>;
bool DualArmModule::reserve_preplanned_trajectory_resources(bool,bool,std::string&) {hardware();}
void DualArmModule::release_preplanned_trajectory_resources(bool,bool) noexcept {std::terminate();}
bool DualArmModule::get_joint_motion_limits(bool,std::vector<double>&,std::vector<double>&,std::vector<double>&) const {hardware();}
bool DualArmModule::prepare_preplanned_dispatch(const Path&,const Path&,std::uint8_t,std::string&) {hardware();}
bool DualArmModule::dispatch_preplanned(const Time&,const Cancel&,std::string&) {hardware();}
bool DualArmModule::read_preplanned_completion_feedback(motion_control_pkg::sdk::DualArmDeviceState&,std::string&) {hardware();}
namespace miracle {
bool MiracleLowerBodyModule::reserve_preplanned_trajectory_resource(std::string&) {hardware();}
void MiracleLowerBodyModule::release_preplanned_trajectory_resource() noexcept {std::terminate();}
bool MiracleLowerBodyModule::prepare_preplanned_dispatch(const Path&,std::string&) {hardware();}
bool MiracleLowerBodyModule::dispatch_preplanned(const Path&,const Time&,const Cancel&,std::string&) {hardware();}
bool MiracleLowerBodyModule::read_body_orientation_feedback(std::array<double,4>&,std::array<double,4>&,uint64_t&,std::string&) {hardware();}
}
static void require(bool ok,const std::string& e) {if(!ok) throw std::runtime_error(e);}
int main(int argc,char** argv) try {
  require(argc==2,"expected repository path");
  const std::string repo=argv[1];
  auto model=std::make_shared<MiracleMAlgo>(
    repo+"/src/onecube_robot_description/urdf/onecube_robot_description.urdf",repo+"/src");
  model->setCollisionCheckMode(0);
  const std::vector<double> l{.8045046960995934,-.7106088931426268,.5295298256393933,-.9450260159378988,
    -.37056347598195566,-.16073897211859947,-.34476481302261197};
  const std::vector<double> r{-.5737369649852467,-.8629368873651962,-.5753896267099474,-.8682243487403323,
    .7008482975591221,.034796401984532814,.5726308588998467};
  require(model->updateChainJointsAngle("left_arm",l)==0 && model->updateChainJointsAngle("right_arm",r)==0,"set model state");
  std::vector<double> start;
  require(model->getFullJointState(start),"get model state");
  Eigen::Isometry3d lt=Eigen::Isometry3d::Identity(),rt=lt;
  lt.translation().z()=.159;rt.translation().z()=.2205;
  std::string error;
  motion_control_pkg::motion::TcpLevelPlan single(true,model->getModel(),start,true,false,true,lt,rt,.02);
  require(single.initialize(error),error);
  Path arm_path{l,{1.163726990368042,-.48249328062982544,-.24799976047626499,-.5345004962897334,
    -.8622864552426153,.18381645312232883,-.8945851342157822}};
  for(auto& point:arm_path) point.insert(point.end(),{0,0,0,0});
  require(single.project(arm_path,error),"WBC arm path projection: "+error);
  Path samples;
  require(model->toppraPlan(arm_path,.004,std::vector<double>(11,.6),std::vector<double>(11,1.2),samples,
    MiracleMAlgo::ToppraSolverMode::kStableCartesian,60,std::vector<double>(11,6)),"constrained WBC TOPPRA");
  require(single.validate(samples,error),"retimed WBC arm path: "+error);
  std::cout<<"PASS actual 4 ms TOPPRA WBC arm path samples="<<samples.size()<<'\n';

  // A deliberately large leg pitch would tilt BOTH stationary TCPs by 0.15 rad.
  // Keep that leg path and compensate with the arms, then retime the complete
  // 18-DOF path through the same TOPPRA implementation used by WBC.
  motion_control_pkg::motion::TcpLevelPlan compensation(true,model->getModel(),start,
    true,true,true,lt,rt,.02);
  require(compensation.initialize(error),error);
  std::vector<double> rest=l;
  rest.insert(rest.end(),r.begin(),r.end());
  rest.insert(rest.end(),{0,0,0,0});
  Path bent_path{rest,rest,rest};
  bent_path[1][14]=.15;
  require(!compensation.validate(bent_path,error),"uncompensated leg pitch should tilt TCPs");
  const bool projected=compensation.project(bent_path,error);
  require(projected,"leg compensation: "+error);
  Path compensated_samples;
  require(model->toppraPlan(bent_path,.004,std::vector<double>(18,.6),
    std::vector<double>(18,1.2),compensated_samples,MiracleMAlgo::ToppraSolverMode::kStableCartesian,
    60,std::vector<double>(18,6)),"retime leg compensation");
  const bool horizontal=compensation.validate(compensated_samples,error);
  require(horizontal,"retimed leg compensation: "+error);
  double peak_leg=0,peak_arm=0;
  for(const auto& q:compensated_samples) {
    peak_leg=std::max(peak_leg,q[14]);
    for(unsigned j=0;j<14;++j) peak_arm=std::max(peak_arm,std::abs(q[j]-rest[j]));
    for(unsigned j=15;j<18;++j) require(std::abs(q[j])<1e-12,"frozen leg/waist changed");
  }
  require(peak_leg>.14 && peak_arm>.02,"leg motion or arm compensation was lost");
  std::cout<<"PASS full-trajectory WBC leg compensation: peak_leg_rad="<<peak_leg
    <<" peak_arm_compensation_rad="<<peak_arm<<" samples="<<compensated_samples.size()<<'\n';

  rclcpp::init(0,nullptr);
  auto node=std::make_shared<rclcpp::Node>("tcp_level_offline_planning");
  WholeBodyModule whole(node,model,nullptr,nullptr,"base_link",lt,rt);
  const auto pose=[&](bool left) {
    Eigen::Isometry3d p;
    require(model->getFramePoseAtConfiguration("base_link",left?"gripper_base_l_link":"gripper_base_r_link",start,p),"FK");
    p=p*(left?lt:rt);
    p.translation().z()+=.006;
    p.linear()=Eigen::AngleAxisd(.025,Eigen::Vector3d::UnitZ()).toRotationMatrix()*p.rotation();
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id="base_link";
    out.pose=motion_control_pkg::kinematics::isometryToPose(p);
    return out;
  };
  unsigned passed=2;
  for(int part:{5,6,7}) for(bool line:{false,true}) {
    auto goal=std::make_shared<MovePose::Goal>();
    goal->keep_tcp_level=true;
    goal->arm_id=part;goal->speed=.2;goal->timeout_ms=60000;goal->enable_move_line=line;
    goal->target_pose=pose(part!=6);goal->left_target_pose=pose(true);goal->right_target_pose=pose(false);
    Path left,right,legs;
    std::optional<Eigen::Isometry3d> lgoal,rgoal;
    require(whole.plan_trajectory(goal,start,left,right,legs,error,lgoal,rgoal),"WBC "+std::to_string(part)+": "+error);
    require(!legs.empty(),"no WBC samples");
    for(std::size_t i=0;i<legs.size();++i) {
      std::vector<double> active,full;
      if(part!=6) active.insert(active.end(),left[i].begin(),left[i].end());
      if(part!=5) active.insert(active.end(),right[i].begin(),right[i].end());
      active.insert(active.end(),legs[i].begin(),legs[i].end());
      require(model->mergeWholeBodyJoints(start,part!=6,part!=5,active,full),"merge");
      for(bool larm:{true,false}) {
        if((larm&&part==6)||(!larm&&part==5)) continue;
        Eigen::Isometry3d actual;
        require(model->getFramePoseAtConfiguration("base_link",larm?"gripper_base_l_link":"gripper_base_r_link",full,actual),"FK validation");
        actual=actual*(larm?lt:rt);
        require(std::acos(std::clamp(-actual.rotation()(2,2),-1.0,1.0))<=.02,"actual TCP not horizontal");
      }
    }
    std::cout<<"PASS WBC part="<<part<<" line="<<line<<" samples="<<legs.size()<<'\n';++passed;
    if(part==7 && !line) {
      goal->whole_body_waist_only=true;
      require(whole.plan_trajectory(goal,start,left,right,legs,error,lgoal,rgoal),"waist-only: "+error);
      std::vector<double> original;
      require(model->extractLegJoints(start,original),"extract legs");
      for(const auto& q:legs) for(unsigned j=0;j<3;++j) require(q[j]==original[j],"frozen leg moved");
      std::cout<<"PASS waist-only preserves frozen legs\n";++passed;
    }
    goal->target_pose.pose.orientation=geometry_msgs::msg::Quaternion();
    goal->target_pose.pose.orientation.w=1;
    goal->left_target_pose=goal->right_target_pose=goal->target_pose;
    require(!whole.plan_trajectory(goal,start,left,right,legs,error,lgoal,rgoal),"WBC normal flip accepted");
  }
  // Omitted keep_tcp_level preserves the original WBC ability to start tilted.
  // Explicit opt-in must reject that same pose before planning compensation.
  auto tilted=start;
  const auto& pm=model->getModel();
  tilted[pm.joints[pm.getJointId("leg_joint1")].idx_q()]=.05;
  auto default_goal=std::make_shared<MovePose::Goal>();
  require(!default_goal->keep_tcp_level,"WBC must default off");
  default_goal->arm_id=5;default_goal->speed=.2;default_goal->timeout_ms=60000;
  Eigen::Isometry3d target;
  require(model->getFramePoseAtConfiguration("base_link","gripper_base_l_link",tilted,target),"tilted FK");
  target=target*lt;target.translation().z()+=.006;
  default_goal->target_pose.header.frame_id="base_link";
  default_goal->target_pose.pose=motion_control_pkg::kinematics::isometryToPose(target);
  Path left,right,legs;
  std::optional<Eigen::Isometry3d> lgoal,rgoal;
  const bool default_planned=whole.plan_trajectory(default_goal,tilted,left,right,legs,error,lgoal,rgoal);
  require(default_planned,"default-off WBC from tilted start: "+error);
  default_goal->keep_tcp_level=true;
  require(!whole.plan_trajectory(default_goal,tilted,left,right,legs,error,lgoal,rgoal),
    "enabled WBC accepted tilted start");
  require(error.find("TCP")!=std::string::npos,"rejection must identify TCP level constraint");
  std::cout<<"PASS default-off WBC accepts tilted start; explicit true rejects it\n";++passed;
  rclcpp::shutdown();
  std::cout<<"TOTAL "<<passed<<" planning cases PASS; no hardware linked or executed\n";
  return 0;
} catch(const std::exception& e) {
  std::cerr<<"FAIL "<<e.what()<<'\n';if(rclcpp::ok()) rclcpp::shutdown();return 1;
}
