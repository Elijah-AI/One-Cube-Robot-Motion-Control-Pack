#include "whole_body_hardware_traps.hpp"
#include "rigid_grasp_fixture.hpp"
#include "motion_control_pkg/motion/rigid_grasp_constraint.hpp"
#include "motion_control_pkg/motion/leg_body_motion.hpp"
#include "motion_control_pkg/motion/leg_body_orientation.hpp"
#include <iostream>

using namespace motion_control_pkg::motion;
static void require(bool ok,const std::string& e) {if(!ok) throw std::runtime_error(e);}
int main(int argc,char** argv) try {
  require(argc==2,"expected repository path");
  const std::string repo=argv[1];
  auto model=std::make_shared<MiracleMAlgo>(repo+"/src/onecube_robot_description/urdf/onecube_robot_description.urdf",repo+"/src");
  model->setCollisionCheckMode(0);
  RigidFixture f(*model);
  const auto& pm=model->getModel();
  auto index=[&](const std::string& n){return pm.joints[pm.getJointId(n)].idx_q();};
  rclcpp::init(0,nullptr);
  auto node=std::make_shared<rclcpp::Node>("rigid_leg_offline_planning");
  WholeBodyModule whole(node,model,nullptr,nullptr,"base_link",f.left_tool,f.right_tool);
  const auto goal=[] {
    auto g=std::make_shared<MoveJoints::Goal>();g->arm_id=4;g->keep_grasp_rigid=true;
    g->speed=.3;g->timeout_ms=120000;return g;
  };
  std::string error;unsigned cases=0;
  auto tcp=[&](const std::vector<double>& q,bool left) {
    Eigen::Isometry3d p;
    require(model->getFramePoseAtConfiguration("base_link",left?"gripper_base_l_link":"gripper_base_r_link",q,p),"FK");
    return Eigen::Isometry3d(p*(left?f.left_tool:f.right_tool));
  };
  auto run=[&](const std::string& label,const auto& start,const auto& g) {
    Path l,r,legs;
    const bool ok=whole.plan_leg_trajectory(g,start,l,r,legs,error);
    require(ok,label+": "+error);
    require(!legs.empty() && l.size()==legs.size() && r.size()==legs.size(),"missing synchronized trajectory");
    const Eigen::Isometry3d relative=tcp(start,true).inverse()*tcp(start,false);
    double height=0,pos=0,angle=0,tilt=0,arm_change=0;
    auto inspect=[&](const auto& q) {
      const auto a=tcp(q,true),b=tcp(q,false);
      const Eigen::Isometry3d rel=a.inverse()*b;
      height=std::max(height,std::abs(a.translation().z()-b.translation().z()));
      pos=std::max(pos,(relative.translation()-rel.translation()).norm());
      angle=std::max(angle,Eigen::AngleAxisd(relative.rotation().transpose()*rel.rotation()).angle());
      for(const auto& p:{a,b}) tilt=std::max(tilt,std::atan2(std::hypot(p.rotation()(0,2),p.rotation()(1,2)),-p.rotation()(2,2)));
      for(unsigned j=1;j<pm.joints.size();++j) {
        const auto k=pm.joints[j].idx_q();
        require(q[k]>=pm.lowerPositionLimit[k]-1e-9 && q[k]<=pm.upperPositionLimit[k]+1e-9,"joint limit");
      }
      for(const auto* n:{"head_yaw_joint","head_pitch_joint"}) require(q[index(n)]==start[index(n)],"head changed");
    };
    std::vector<double> previous;
    for(std::size_t i=0;i<legs.size();++i) {
      auto active=l[i];active.insert(active.end(),r[i].begin(),r[i].end());
      active.insert(active.end(),legs[i].begin(),legs[i].end());
      std::vector<double> q;
      require(model->mergeWholeBodyJoints(start,true,true,active,q),"merge");inspect(q);
      if(!previous.empty()) {auto mid=q;for(unsigned j=0;j<q.size();++j) mid[j]=.5*(q[j]+previous[j]);inspect(mid);}
      previous=q;
      for(unsigned j=0;j<7;++j) arm_change=std::max({arm_change,std::abs(l[i][j]-l.front()[j]),std::abs(r[i][j]-r.front()[j])});
    }
    require(height<=g->grasp_position_tolerance_m && pos<=g->grasp_position_tolerance_m &&
      angle<=g->grasp_orientation_tolerance_rad && tilt<=g->grasp_orientation_tolerance_rad,"independent FK grasp violation");
    std::cout<<"PASS "<<label<<" samples="<<legs.size()<<" height_mm="<<height*1000
      <<" relative_mm="<<pos*1000<<" relative_rad="<<angle<<" tilt_rad="<<tilt<<" arm_change_rad="<<arm_change<<std::endl;
    ++cases;return legs;
  };
  for(double yaw:{175.*M_PI/180.,-175.*M_PI/180.}) {
    auto g=goal();g->joint_name={"body_joint"};g->position={yaw};
    const auto legs=run("waist "+std::to_string(yaw),f.start,g);
    require(std::abs(legs.back()[3]-yaw)<1e-8,"waist endpoint changed");
    for(const auto& p:legs) for(unsigned j=0;j<3;++j) require(std::abs(p[j])<1e-10,"waist-only moved legs");
  }
  auto mixed=goal();mixed->joint_name={"body_joint","leg_joint3","leg_joint1","leg_joint2"};
  mixed->position={175.*M_PI/180.,.025,.15,-.075};
  const auto combined=run("four named joints and 175 degree waist",f.start,mixed);
  require(std::abs(combined.back()[0]-.15)<1e-8 && std::abs(combined.back()[1]+.075)<1e-8 &&
    std::abs(combined.back()[2]-.025)<1e-8 && std::abs(combined.back()[3]-mixed->position[0])<1e-8,"named target mapping");
  for(bool waist:{false,true}) {
    auto g=goal();g->joint_name={"leg_joint1","leg_joint2"};g->position={.2,-.4};
    if(waist) {g->joint_name.push_back("body_joint");g->position.push_back(175.*M_PI/180.);}
    const auto legs=run(waist?"two legs plus waist":"two legs with third compensation",f.start,g);
    require(std::abs(legs.back()[0]-.2)<1e-8 && std::abs(legs.back()[1]+.4)<1e-8,"paired leg target changed");
    LegBodyMotionPlanner bp(pm);const auto initial=bp.pose({0,0,0,0});
    for(const auto& p:legs) require(Eigen::AngleAxisd(initial.rotation().transpose()*bp.pose({p[0],p[1],p[2],0}).rotation()).angle()<=.02,"paired body tilt");
  }
  auto tilt=goal();tilt->leg_motion_mode=1;tilt->body_tilt_deg=5.;run("body tilt",f.start,tilt);
  LegBodyOrientationPlanner op(pm);const auto folded=op.plan({0,0,0,0},{"leg_joint1","leg_joint2"},{.3,-.6},.3,.004,60);
  auto folded_start=f.start;const std::array<std::string,4> names{{"leg_joint1","leg_joint2","leg_joint3","body_joint"}};
  for(unsigned i=0;i<4;++i) folded_start[index(names[i])]=folded.target[i];
  LegBodyMotionPlanner bp(pm);const auto z0=bp.pose(folded.target).translation().z();
  for(unsigned mode:{2U,3U}) {
    auto g=goal();g->leg_motion_mode=mode;g->body_height_m=mode==2?-.02:z0+.02;
    const auto legs=run(mode==2?"relative height":"absolute height",folded_start,g);
    LegJoints end;std::copy(legs.back().begin(),legs.back().end(),end.begin());
    require(std::abs(bp.pose(end).translation().z()-(mode==2?z0-.02:z0+.02))<1e-5,"height target changed");
  }
  auto noop=goal();noop->joint_name={"body_joint"};noop->position={0};run("no movement",f.start,noop);
  auto reject=[&](const std::string& label,const auto& g,const auto& q) {
    Path a,b,c;require(!whole.plan_leg_trajectory(g,q,a,b,c,error),"accepted invalid goal "+label);
    require(a.empty() && b.empty() && c.empty(),"partial trajectory escaped");
    std::cout<<"PASS rejects "<<label<<": "<<error<<std::endl;++cases;
  };
  auto bad=goal();bad->joint_name={"body_joint"};bad->position={M_PI};reject("waist beyond URDF",bad,f.start);
  bad->position={.1};bad->control_mode=1;reject("impedance",bad,f.start);
  bad->control_mode=0;bad->synchronization_mode=1;reject("unsynchronized",bad,f.start);
  bad->synchronization_mode=0;bad->keep_tcp_level=true;reject("conflicting constraint",bad,f.start);
  bad->keep_tcp_level=false;bad->timeout_ms=1;reject("insufficient timeout",bad,f.start);
  bad->timeout_ms=120000;bad->joint_name={"head_yaw_joint"};reject("unsupported joint",bad,f.start);
  bad->joint_name={"body_joint"};auto tilted=f.start;tilted[index("leg_joint1")]=.1;reject("tilted suction start",bad,tilted);
  bad->enable_collision_check=2;reject("missing collision cloud",bad,f.start);
  bad->enable_collision_check=0;bad->acceleration_scale=.5;reject("unsupported scale",bad,f.start);
  bad->acceleration_scale=0;bad->grasp_position_tolerance_m=0;reject("invalid tolerance",bad,f.start);
  rclcpp::shutdown();std::cout<<"TOTAL "<<cases<<" rigid LEG planning cases PASS\n";return 0;
} catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';if(rclcpp::ok()) rclcpp::shutdown();return 1;}
