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
  RigidFixture f(*model,repo);
  const auto& pm=model->getModel();
  auto index=[&](const std::string& n){return pm.joints[pm.getJointId(n)].idx_q();};
  rclcpp::init(0,nullptr);
  auto node=std::make_shared<rclcpp::Node>("rigid_leg_offline_planning");
  WholeBodyModule whole(node,model,nullptr,nullptr,"base_link",f.left_tool,f.right_tool);
  const auto goal=[] {
    auto g=std::make_shared<MoveJoints::Goal>();g->arm_id=4;g->keep_grasp_rigid=true;
    require(g->grasp_position_tolerance_m==.002,"LEG default grasp position tolerance must be 2 mm");
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
    const auto left_start=tcp(start,true),right_start=tcp(start,false);
    const Eigen::Isometry3d relative=left_start.inverse()*right_start;
    const double initial_height=left_start.translation().z()-right_start.translation().z();
    double height=0,pos=0,angle=0,tilt=0,arm_change=0;
    auto inspect=[&](const auto& q) {
      const auto a=tcp(q,true),b=tcp(q,false);
      const Eigen::Isometry3d rel=a.inverse()*b;
      height=std::max(height,std::abs(a.translation().z()-b.translation().z()-initial_height));
      pos=std::max(pos,(relative.translation()-rel.translation()).norm());
      angle=std::max(angle,Eigen::AngleAxisd(relative.rotation().transpose()*rel.rotation()).angle());
      for(bool left:{true,false}) {
        const Eigen::Vector3d up=(left?a:b).rotation()*(left?left_start:right_start).rotation().transpose()*Eigen::Vector3d::UnitZ();
        tilt=std::max(tilt,std::atan2(up.head<2>().norm(),up.z()));
      }
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
  LegBodyMotionPlanner body(pm);
  const LegJoints neutral{0,0,0,0};
  const auto neutral_rotation=body.pose(neutral).rotation().eval();
  std::array<Eigen::Vector3d,3> pitch_axes;
  for (unsigned j=0;j<3;++j) {
    auto probe=neutral;
    probe[j]=.1;
    pitch_axes[j]=Eigen::AngleAxisd(body.pose(probe).rotation()*neutral_rotation.transpose()).axis();
  }
  const bool parallel=pitch_axes[0].cross(pitch_axes[1]).norm()<1e-4 &&
                      pitch_axes[0].cross(pitch_axes[2]).norm()<1e-4;
  for(bool waist:{false,true}) {
    auto g=goal();g->joint_name={"leg_joint1","leg_joint2"};g->position={.2,-.4};
    if(waist) {g->joint_name.push_back("body_joint");g->position.push_back(175.*M_PI/180.);}
    if (!parallel) {
      Path left,right,legs;
      require(!whole.plan_leg_trajectory(g,f.start,left,right,legs,error) &&
              error.find("not parallel")!=std::string::npos &&
              left.empty() && right.empty() && legs.empty(), "nonparallel leg axes must reject paired compensation");
      ++cases;
      continue;
    }
    const auto legs=run(waist?"two legs plus waist":"two legs with third compensation",f.start,g);
    require(std::abs(legs.back()[0]-.2)<1e-8 && std::abs(legs.back()[1]+.4)<1e-8,"paired leg target changed");
    LegBodyMotionPlanner bp(pm);const auto initial=bp.pose({0,0,0,0});
    for(const auto& p:legs) require(Eigen::AngleAxisd(initial.rotation().transpose()*bp.pose({p[0],p[1],p[2],0}).rotation()).angle()<=.02,"paired body tilt");
  }
  // On a calibrated nonplanar chain the underlying LEG planner can reject an
  // exact pure inclination or vertical path. Compensation must return the same
  // geometry error without releasing any arm or leg samples.
  auto feasibleBodyGoal=[&](const LegJoints& legs,const std::vector<double>& start,const auto& g) {
    try {
      body.plan(legs,static_cast<LegBodyMotionMode>(g->leg_motion_mode),
                g->body_tilt_deg,g->body_height_m,g->speed,.004,120.);
      return true;
    } catch (const std::invalid_argument& ex) {
      const std::string reason=ex.what();
      require(!parallel && (reason=="J3 alone cannot produce the requested inclination in this URDF" ||
                            reason=="Loaded URDF cannot preserve the complete body pose on a vertical path"),reason);
      Path left,right,path;
      require(!whole.plan_leg_trajectory(g,start,left,right,path,error) && error==reason &&
              left.empty() && right.empty() && path.empty(),"unsafe dispatch after infeasible LEG request: "+reason);
      ++cases;
      std::cout<<"PASS calibrated LEG rejection: "<<reason<<'\n';
      return false;
    }
  };
  auto tilt=goal();tilt->leg_motion_mode=1;tilt->body_tilt_deg=5.;
  if (feasibleBodyGoal(neutral,f.start,tilt)) run("body tilt",f.start,tilt);
  const LegJoints folded{.3,-.6,.3,0};
  auto folded_start=f.start;const std::array<std::string,4> names{{"leg_joint1","leg_joint2","leg_joint3","body_joint"}};
  for(unsigned i=0;i<4;++i) folded_start[index(names[i])]=folded[i];
  LegBodyMotionPlanner bp(pm);const auto z0=bp.pose(folded).translation().z();
  for(unsigned mode:{2U,3U}) {
    auto g=goal();g->leg_motion_mode=mode;g->body_height_m=mode==2?-.02:z0+.02;
    if (!feasibleBodyGoal(folded,folded_start,g)) continue;
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
  bad->joint_name={"body_joint"};
  auto tilted=f.start;tilted[index("leg_joint1")]=.1;tilted[index("Joint2_R")]+=.1;
  require(std::abs(tcp(tilted,true).translation().z()-tcp(tilted,false).translation().z())>.002,"test requires unequal TCP heights");
  run("capture tilted grasp at different heights",tilted,bad);
  bad->enable_collision_check=2;reject("missing collision cloud",bad,f.start);
  bad->enable_collision_check=0;bad->acceleration_scale=.5;reject("unsupported scale",bad,f.start);
  bad->acceleration_scale=0;bad->grasp_position_tolerance_m=0;reject("invalid tolerance",bad,f.start);
  rclcpp::shutdown();std::cout<<"TOTAL "<<cases<<" rigid LEG planning cases PASS\n";return 0;
} catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';if(rclcpp::ok()) rclcpp::shutdown();return 1;}
