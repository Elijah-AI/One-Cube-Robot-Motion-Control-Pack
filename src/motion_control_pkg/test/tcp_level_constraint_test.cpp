#include "motion_control_pkg/motion/tcp_level_plan.hpp"
#include "com_interfaces/action/move_joints.hpp"
#include "com_interfaces/action/move_pose.hpp"
#include "com_interfaces/tcp_level_goal.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <iostream>
#include <algorithm>
#include <limits>
#include <stdexcept>

using namespace motion_control_pkg;
using Q=motion::TcpLevelConstraint::Configuration;
using Path=motion::TcpLevelConstraint::Path;
static void require(bool ok,const std::string& reason) {if(!ok) throw std::runtime_error(reason);}
int main(int argc,char** argv) try {
  require(argc==2,"expected robot URDF path");
  pinocchio::Model model;pinocchio::urdf::buildModel(argv[1],model);
  const auto neutral=pinocchio::neutral(model);
  Q start(neutral.data(),neutral.data()+neutral.size());
  const auto idx=[&](const std::string& name){return model.joints[model.getJointId(name)].idx_q();};
  // Independently solved horizontal poses of the actual robot, identity TCP rotation.
  const Q left{.8045046960995934,-.7106088931426268,.5295298256393933,-.9450260159378988,
    -.37056347598195566,-.16073897211859947,-.34476481302261197};
  const Q left_goal{1.163726990368042,-.48249328062982544,-.24799976047626499,-.5345004962897334,
    -.8622864552426153,.18381645312232883,-.8945851342157822};
  const Q right{-.5737369649852467,-.8629368873651962,-.5753896267099474,-.8682243487403323,
    .7008482975591221,.034796401984532814,.5726308588998467};
  const auto assign=[&](Q& q,bool l,const Q& values){
    for(int j=0;j<7;++j) q[idx("Joint"+std::to_string(j+1)+(l?"_L":"_R"))]=values[j];
  };
  assign(start,true,left);assign(start,false,right);
  Q goal=start;assign(goal,true,left_goal);
  const auto tool=Eigen::Isometry3d::Identity();
  std::string error;
  unsigned passed=0;
  const auto pass=[&](const char* name){std::cout<<"PASS "<<name<<'\n';++passed;};

  require(!com_interfaces::action::MoveJoints::Goal{}.keep_tcp_level &&
    !com_interfaces::action::MovePose::Goal{}.keep_tcp_level,"constraint must default off");
  for(std::uint8_t part=0;part<=8;++part) {
    require(!com_interfaces::validateTcpLevelGoal(true,.02,part,false,0,error),"reject every MoveJ constraint");
    require(com_interfaces::validateTcpLevelGoal(true,.02,part,true,0,error)==(part>=5 && part<=7),
      "only WBC MovePose may enable TCP leveling");
    require(com_interfaces::validateTcpLevelGoal(false,.02,part,false,0,error),"disabled MoveJ unchanged");
    require(com_interfaces::validateTcpLevelGoal(false,.02,part,true,0,error),"disabled MovePose unchanged");
  }
  require(!com_interfaces::validateTcpLevelGoal(true,.02,7,true,1,error),"reject impedance");
  require(!com_interfaces::validateTcpLevelGoal(true,std::numeric_limits<double>::quiet_NaN(),0,false,0,error),"reject NaN");
  require(!com_interfaces::validateTcpLevelGoal(true,0,0,false,0,error),"reject zero tolerance");
  pass("default-off and WBC-only action contract; malformed inputs rejected");

  motion::TcpLevelConstraint level(model,start,true,false,tool,tool,.02);
  require(level.initialize(error),"start: "+error);
  require(level.check(goal,error),"goal: "+error);
  Path path{start,goal};
  require(!level.validatePath(path,error),"endpoint-only check would miss 0.096 rad midpoint tilt");
  require(level.projectPath(path,{},error),"projection: "+error);
  require(path.front()==start && path.back()==goal,"WBC path endpoints must be exact");
  require(level.validatePath(path,error),"projected interpolation: "+error);
  for(const auto& q:path) for(int j=0;j<model.nq;++j) {
    bool arm=false;
    for(int k=1;k<=7;++k) arm|=j==idx("Joint"+std::to_string(k)+"_L");
    if(!arm) require(q[j]==start[j],"projection moved inactive joints");
  }
  pass("WBC arm endpoints horizontal but middle tilted; projection preserves endpoints and inactive joints");

  Q tilted=goal;tilted[idx("Joint2_L")]+=.3;
  Path invalid{start,tilted},original=invalid;
  require(!level.projectPath(invalid,{},error) && invalid==original,"invalid endpoint must reject transactionally");
  Q bad=start;bad[0]=std::numeric_limits<double>::infinity();
  require(!level.check(bad,error),"nonfinite joint rejected");
  motion::TcpLevelConstraint bad_start(model,Q(neutral.data(),neutral.data()+neutral.size()),true,false,tool,tool,.02);
  require(!bad_start.initialize(error),"nonlevel start rejected");
  pass("invalid start, goal and nonfinite state fail without changing targets");

  pinocchio::Data data(model);
  pinocchio::framesForwardKinematics(model,data,Eigen::Map<const Eigen::VectorXd>(start.data(),start.size()));
  Eigen::Isometry3d tcp=Eigen::Isometry3d::Identity();
  tcp.linear()=data.oMf[model.getFrameId("gripper_base_l_link")].rotation();
  tcp.linear()=Eigen::AngleAxisd(.9,Eigen::Vector3d::UnitZ()).toRotationMatrix()*tcp.rotation();
  require(level.checkTarget(true,tcp,error),"horizontal yaw must be free");
  tcp.linear()=tcp.rotation()*Eigen::AngleAxisd(3.141592653589793,Eigen::Vector3d::UnitX()).toRotationMatrix();
  require(!level.checkTarget(true,tcp,error),"normal flip must reject");
  Eigen::Isometry3d rotated_tool=tool;
  rotated_tool.linear()=Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitX()).toRotationMatrix();
  motion::TcpLevelConstraint rotated(model,start,true,false,rotated_tool,tool,.02);
  require(!rotated.initialize(error),"must include tool rotation, not only flange");
  pass("free yaw, preserved downward branch and calibrated tool rotation");

  Path via{start,goal,start};
  require(level.projectPath(via,{goal},error),"via projection: "+error);
  require(std::find(via.begin(),via.end(),goal)!=via.end(),"via joint target must stay exact");
  pass("path projection preserves pinned configurations");

  motion::TcpLevelConstraint both(model,start,true,true,tool,tool,.02);
  require(both.initialize(error),error);
  Q bent=start;bent[idx("leg_joint1")]+=.15;
  Path wbc{start,bent,start};
  require(both.projectPath(wbc,{},error),"WBC compensation: "+error);
  require(both.validatePath(wbc,error),error);
  double peak=0;
  for(const auto& q:wbc) {
    peak=std::max(peak,q[idx("leg_joint1")]-start[idx("leg_joint1")]);
    for(const auto* name:{"leg_joint2","leg_joint3","body_joint"})
      require(q[idx(name)]==start[idx(name)],"projection changed frozen leg/waist joint");
  }
  require(std::abs(peak-.15)<1e-10,"WBC leg motion lost");
  pass("both TCPs stay level during leg bending; inactive leg axes unchanged");

  Path active{left,left_goal},disabled_path=active;
  motion::TcpLevelPlan disabled(false,model,start,true,false,false,tool,tool,.02);
  require(disabled.initialize(error) && disabled.project(active,error) && active==disabled_path &&
    disabled.validate(active,error) && disabled.startMonitoring(error),"explicit false preserves old pipeline");
  pass("explicit disable preserves old path and needs no runtime safety instance");

  auto monitored=std::make_shared<motion::TcpLevelConstraint>(model,start,true,false,tool,tool,.02);
  require(monitored->initialize(error),error);
  Q measured=start;
  auto stamp=execution::SafetyClock::now()-std::chrono::milliseconds(1);
  std::uint64_t seq=1;
  const auto makeSafety=[&]{
    auto safety=std::make_shared<execution::MotionSafety>();
    for(unsigned channel:{execution::kLeftSafety,execution::kLegSafety}) {
      std::vector<std::string> names;
      if(channel==execution::kLeftSafety)
        for(int j=1;j<=7;++j) names.push_back("Joint"+std::to_string(j)+"_L");
      else names={"leg_joint1","leg_joint2","leg_joint3","body_joint"};
      std::vector<execution::SafetyJoint> specs;
      for(const auto& name:names) {
        const auto jid=model.getJointId(name);
        specs.push_back({name,model.lowerPositionLimit[idx(name)],model.upperPositionLimit[idx(name)],
          model.velocityLimit[model.joints[jid].idx_v()]});
      }
      safety->addChannel(channel,specs,[&,names](execution::SafetySample& out,std::string&){
        out.acquired=stamp;out.sequence=seq;out.velocity.assign(names.size(),0);
        for(const auto& name:names) out.position.push_back(measured[idx(name)]);
        return true;
      });
    }
    safety->poll();return safety;
  };
  auto safety=makeSafety();
  auto id=safety->addStateConstraint(5,[&](const auto& names,const auto& positions,std::string& e){
    return monitored->checkMeasured(names,positions,e);},error);
  require(id!=0,error);
  measured[idx("Joint2_L")]+=.2;stamp=execution::SafetyClock::now();++seq;
  safety->poll();
  require(safety->latched() && safety->status().find("TCP level")!=std::string::npos,
    "measured tilt must latch the shared command fence");
  safety->removeStateConstraint(id);
  pass("measured tilt trips shared MotionSafety fence");

  require(!monitored->checkMeasured({"Joint1_L"},{0},error),"missing leg/arm feedback must reject");
  pass("missing measured ancestors rejected");

  measured=start;stamp=execution::SafetyClock::now()-std::chrono::seconds(1);++seq;
  auto stale_safety=makeSafety();
  require(stale_safety->addStateConstraint(5,[&](const auto& names,const auto& positions,std::string& e){
    return monitored->checkMeasured(names,positions,e);},error)==0,"stale state must reject monitor registration");
  pass("stale ancestor feedback refuses constraint registration");
  std::cout<<"TOTAL "<<passed<<" PASS\n";
  return 0;
} catch(const std::exception& e) {
  std::cerr<<"FAIL "<<e.what()<<'\n';return 1;
}
