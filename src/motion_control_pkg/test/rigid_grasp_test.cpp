#include "motion_control_pkg/miracle_m_algo.hpp"
#include "motion_control_pkg/motion/tcp_level_plan.hpp"
#include "com_interfaces/action/move_pose.hpp"
#include "com_interfaces/rigid_grasp_goal.hpp"
#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace motion_control_pkg;
using Q=std::vector<double>;
using Path=std::vector<Q>;
void require(bool ok,const std::string& e) {if(!ok) throw std::runtime_error(e);}
int main(int argc,char** argv) try {
  require(argc==2,"expected repository");std::string root=argv[1];
  MiracleMAlgo algo(root+"/src/onecube_robot_description/urdf/onecube_robot_description.urdf",root+"/src");
  const auto& m=algo.getModel();Json::Value f;
  std::ifstream input(root+"/src/motion_control_pkg/tools/mujoco_validation/rigid_grasp_fixture.json");input>>f;
  Eigen::Isometry3d lt=Eigen::Isometry3d::Identity(),rt=lt;
  for(bool left:{true,false}) {
    Q a;for(const auto& v:f[left?"left_joints":"right_joints"]) a.push_back(v.asDouble());
    require(algo.updateChainJointsAngle(left?"left_arm":"right_arm",a)==0,"fixture joints");
    const auto& p=f[left?"left_tool_tf":"right_tool_tf"];auto& t=left?lt:rt;
    t.translation()<<p[0].asDouble(),p[1].asDouble(),p[2].asDouble();
    t.linear()=Eigen::Quaterniond(p[6].asDouble(),p[3].asDouble(),p[4].asDouble(),p[5].asDouble()).toRotationMatrix();
  }
  Q start;require(algo.getFullJointState(start),"start state");
  const auto idx=[&](const std::string& n){return m.joints[m.getJointId(n)].idx_q();};
  const motion::RigidGraspOptions options{true,.001,.005};std::string error;
  motion::RigidGraspConstraint grasp(m,start,lt,rt,options);
  require(grasp.initialize(error),error);
  Eigen::Isometry3d l,r;
  require(algo.getFramePoseAtConfiguration("base_link","gripper_base_l_link",start,l),"FK");
  require(algo.getFramePoseAtConfiguration("base_link","gripper_base_r_link",start,r),"FK");
  l=l*lt;r=r*rt;
  require(grasp.checkTargets(l,r,error),error);
  auto shifted=r;shifted.translation().y()-=.01;
  require(!grasp.checkTargets(l,shifted,error),"stretch must reject");
  shifted=r;shifted.translation().z()+=.01;
  require(!grasp.checkTargets(l,shifted,error),"different contact heights must reject");
  shifted=r;shifted.linear()=Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitZ()).toRotationMatrix()*r.rotation();
  require(!grasp.checkTargets(l,shifted,error),"relative yaw change must reject");
  motion::TcpLevelConstraint legacy(m,start,true,true,lt,rt,.02);
  require(legacy.initialize(error)&&legacy.checkTarget(false,shifted,error),"legacy yaw is unconstrained control");
  Eigen::Isometry3d common=Eigen::Isometry3d::Identity();
  common.linear()=Eigen::AngleAxisd(3.,Eigen::Vector3d::UnitZ()).toRotationMatrix();common.translation().z()=.1;
  require(grasp.checkTargets(common*l,common*r,error),"common yaw/translation must stay free");
  std::cout<<"PASS rigid geometry and rotated calibrated tools\n";

  Q bent=start;bent[idx("leg_joint1")]=.2;
  Path path{start,bent,start};const auto raw=path;
  require(!grasp.validatePath(path,error),"uncompensated leg path must fail");
  const bool projected=grasp.projectPath(path,{},error);require(projected,error);
  require(path.front()==start&&path.back()==start,"endpoints changed");
  require(grasp.validatePath(path,error),error);
  double peak=0;for(const auto& q:path) {
    peak=std::max(peak,q[idx("leg_joint1")]);
    for(const char* n:{"leg_joint2","leg_joint3","body_joint","head_pitch_joint","head_yaw_joint"})
      require(q[idx(n)]==start[idx(n)],"inactive joint changed");
  }
  require(std::abs(peak-.2)<1e-10,"leg waypoint lost");
  Path invalid{start,bent};const auto before=invalid;
  require(!grasp.projectPath(invalid,{},error)&&invalid==before,"invalid endpoint must reject transactionally");
  motion::RigidGraspConstraint wrong(m,start,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity(),options);
  require(!wrong.initialize(error),"wrong tool calibration must reject");
  std::cout<<"PASS coupled projection, interpolation, immutable legs and transactional errors\n";

  com_interfaces::action::MovePose::Goal goal;require(!goal.keep_grasp_rigid,"default off");
  goal.keep_grasp_rigid=true;goal.arm_id=7;require(com_interfaces::validateRigidGraspGoal(goal,error),error);
  for(unsigned id:{0,1,2,3,4,5,6}) {goal.arm_id=id;require(!com_interfaces::validateRigidGraspGoal(goal,error),"unsupported part");}
  goal.arm_id=7;goal.keep_tcp_level=true;require(!com_interfaces::validateRigidGraspGoal(goal,error),"conflicting flag");
  goal.keep_tcp_level=false;goal.control_mode=1;require(!com_interfaces::validateRigidGraspGoal(goal,error),"impedance");
  goal.control_mode=0;goal.synchronization_mode=1;require(!com_interfaces::validateRigidGraspGoal(goal,error),"unsynchronized");
  goal.synchronization_mode=0;goal.grasp_position_tolerance_m=0;require(!com_interfaces::validateRigidGraspGoal(goal,error),"zero tolerance");
  std::cout<<"PASS Action admission and default compatibility\n";

  Q measured=start;auto safety=std::make_shared<execution::MotionSafety>();
  std::uint64_t seq=0;bool stale=false;
  for(unsigned channel:{execution::kLeftSafety,execution::kRightSafety,execution::kLegSafety}) {
    std::vector<std::string> names;
    if(channel==execution::kLegSafety) names={"leg_joint1","leg_joint2","leg_joint3","body_joint"};
    else for(int j=1;j<=7;++j) names.push_back("Joint"+std::to_string(j)+(channel==execution::kLeftSafety?"_L":"_R"));
    std::vector<execution::SafetyJoint> specs;
    for(const auto& n:names) specs.push_back({n,m.lowerPositionLimit[idx(n)],m.upperPositionLimit[idx(n)],1.});
    safety->addChannel(channel,specs,[&,names](auto& sample,std::string&) {
      sample.position.clear();for(const auto& n:names) sample.position.push_back(measured[idx(n)]);
      sample.velocity.assign(names.size(),0.);sample.sequence=++seq;
      sample.acquired=execution::SafetyClock::now()-std::chrono::milliseconds(stale?1000:1);return true;
    });
  }
  safety->poll();execution::installMotionSafety(safety);
  {
    motion::TcpLevelPlan plan(false,m,start,true,true,true,lt,rt,.02,options);
    require(plan.initialize(error)&&plan.startMonitoring(error),error);
    measured[idx("Joint2_R")]+=.03;safety->poll();
    require(safety->latched(),"measured grasp violation must latch command fence");
    require(!safety->command(execution::kArmSafety,error),"fault must block commands");
  }
  execution::installMotionSafety(nullptr);
  std::vector<std::string> missing{"Joint1_L"};
  require(!grasp.checkMeasured(missing,{start[idx("Joint1_L")]},error),"incomplete feedback accepted");
  std::cout<<"PASS independent measured grasp monitor, missing feedback and command fence\n";
  return 0;
} catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
