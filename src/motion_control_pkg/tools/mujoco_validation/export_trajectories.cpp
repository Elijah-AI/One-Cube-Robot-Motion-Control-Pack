#include "motion_control_pkg/whole_body_module.hpp"
#include "motion_control_pkg/motion/tcp_level_plan.hpp"
#include "motion_control_pkg/kinematics/transform_utils.hpp"
#include <jsoncpp/json/json.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

using Q=std::vector<double>;
using Path=std::vector<Q>;
constexpr double dt=.004, tolerance=.02;
void require(bool ok,const std::string& why) {if(!ok) throw std::runtime_error(why);}
Json::Value array(const Q& q) {Json::Value a(Json::arrayValue);for(double v:q) a.append(v);return a;}
Json::Value transform(const Eigen::Isometry3d& p) {
  Json::Value a(Json::arrayValue);
  for(int i=0;i<4;++i) for(int j=0;j<4;++j) a.append(p.matrix()(i,j));
  return a;
}

int main(int argc,char** argv) try {
  require(argc==3||(argc==4&&std::string(argv[3])=="--rigid"),"usage: wbc_mujoco_export REPOSITORY OUTPUT.json [--rigid]");
  const bool rigid=argc==4;
  const motion_control_pkg::motion::RigidGraspOptions grasp{rigid,.001,.005};
  const auto root=std::filesystem::canonical(argv[1]);
  const auto urdf=root/"src/onecube_robot_description/urdf/onecube_robot_description.urdf";
  auto algo=std::make_shared<MiracleMAlgo>(urdf.string(),(root/"src").string());
  algo->setCollisionCheckMode(0);
  const auto& model=algo->getModel();
  Q l{.8045046960995934,-.7106088931426268,.5295298256393933,-.9450260159378988,
    -.37056347598195566,-.16073897211859947,-.34476481302261197};
  Q r{-.5737369649852467,-.8629368873651962,-.5753896267099474,-.8682243487403323,
    .7008482975591221,.034796401984532814,.5726308588998467};
  Eigen::Isometry3d lt=Eigen::Isometry3d::Identity(),rt=lt;
  lt.translation().z()=rt.translation().z()=.1785;
  if(rigid) {
    Json::Value fixture;std::ifstream f(root/"src/motion_control_pkg/tools/mujoco_validation/rigid_grasp_fixture.json");
    require(bool(f),"missing rigid fixture");f>>fixture;
    for(int j=0;j<7;++j) {l[j]=fixture["left_joints"][j].asDouble();r[j]=fixture["right_joints"][j].asDouble();}
    for(bool left:{true,false}) {
      const auto& a=fixture[left?"left_tool_tf":"right_tool_tf"];
      auto& t=left?lt:rt;t.translation()<<a[0].asDouble(),a[1].asDouble(),a[2].asDouble();
      t.linear()=Eigen::Quaterniond(a[6].asDouble(),a[3].asDouble(),a[4].asDouble(),a[5].asDouble()).normalized().toRotationMatrix();
    }
  }
  require(algo->updateChainJointsAngle("left_arm",l)==0 &&
    algo->updateChainJointsAngle("right_arm",r)==0,"set horizontal fixture");
  Q start;require(algo->getFullJointState(start),"read full fixture");
  const auto index=[&](const std::string& n){
    require(model.existJointName(n),"missing joint "+n);
    return model.joints[model.getJointId(n)].idx_q();
  };
  // Explicit source-default TCP fixture, not a claim about a robot's runtime tools.
  std::vector<std::string> names(model.nq),active;
  for(pinocchio::JointIndex i=1;i<model.joints.size();++i) {
    require(model.joints[i].nq()==1,"only scalar URDF joints supported");
    names[model.joints[i].idx_q()]=model.names[i];
  }
  for(const char* side:{"_L","_R"}) for(int j=1;j<=7;++j) active.push_back("Joint"+std::to_string(j)+side);
  for(const char* n:{"leg_joint1","leg_joint2","leg_joint3","body_joint"}) active.emplace_back(n);
  const auto expand=[&](const Path& path){
    Path full;for(const auto& a:path) {
      require(a.size()==active.size(),"active layout");Q q=start;
      for(std::size_t j=0;j<a.size();++j) q[index(active[j])]=a[j];
      full.push_back(std::move(q));
    }return full;
  };
  Q rest;for(const auto& n:active) rest.push_back(start[index(n)]);
  Json::Value out;
  out["schema_version"]=1;out["urdf"]=urdf.string();out["dt_sec"]=dt;
  std::ifstream urdf_file(urdf,std::ios::binary);
  out["urdf_xml"]=std::string(std::istreambuf_iterator<char>(urdf_file),{});
  out["tolerance_rad"]=tolerance;out["waist_target_deg"]=175.;
  out["mode"]="fixed_base_kinematic_validation";
  out["constraint_mode"]=rigid?"rigid_grasp":"tcp_level";
  out["grasp_position_tolerance_m"]=grasp.position_tolerance_m;
  out["grasp_orientation_tolerance_rad"]=grasp.orientation_tolerance_rad;
  out["collision_check_enabled"]=false;out["hardware_dispatch_enabled"]=false;
  out["tool_profile"]=rigid?"user calibrated suction tools: z=.167, left xyzw=[.5,-.5,.5,.5], right=[.5,.5,.5,-.5]":"main_node source default: translation z=.1785, identity rotation";
  out["left_tool"]=transform(lt);out["right_tool"]=transform(rt);
  out["joint_names"]=Json::Value(Json::arrayValue);
  for(const auto& n:names) out["joint_names"].append(n);
  const auto fk=[&](const Q& q,bool left){
    Eigen::Isometry3d p;
    require(algo->getFramePoseAtConfiguration("base_link",left?"gripper_base_l_link":"gripper_base_r_link",q,p),"FK");
    return Eigen::Isometry3d(p*(left?lt:rt));
  };
  const auto save=[&](const std::string& name,const std::string& pipeline,const Path& full,
                      const std::optional<Eigen::Isometry3d>& left_goal=std::nullopt,
                      const std::optional<Eigen::Isometry3d>& right_goal=std::nullopt) {
    require(full.size()>1,"empty or static scenario");
    motion_control_pkg::motion::TcpLevelConstraint c(model,full.front(),true,true,lt,rt,tolerance);
    std::string error;require(c.initialize(error),error);
    const bool valid=c.validatePath(full,error);require(valid,name+": "+error);
    if(rigid) {
      motion_control_pkg::motion::RigidGraspConstraint coupled(model,full.front(),lt,rt,grasp);
      require(coupled.initialize(error),error);
      const bool ok=coupled.validatePath(full,error);require(ok,name+": "+error);
    }
    Json::Value scene;scene["name"]=name;scene["pipeline"]=pipeline;
    scene["duration_sec"]=(full.size()-1)*dt;
    if(left_goal) scene["left_target"]=transform(*left_goal);
    if(right_goal) scene["right_target"]=transform(*right_goal);
    for(const auto& q:full) {
      Json::Value s;s["q"]=array(q);
      s["left_tcp"]=transform(fk(q,true));s["right_tcp"]=transform(fk(q,false));
      if(rigid) {
        Q legacy=full.front();
        for(const char* n:{"leg_joint1","leg_joint2","leg_joint3","body_joint"}) legacy[index(n)]=q[index(n)];
        const bool ok=c.projectConfiguration(legacy,error);require(ok,"legacy comparison: "+error);
        s["legacy_q"]=array(legacy);
      }
      scene["samples"].append(s);
    }
    out["scenarios"].append(scene);
    std::cout<<"EXPORTED "<<name<<" samples="<<full.size()<<std::endl;
  };
  const double yaw=175.*std::acos(-1.)/180.;
  for(const std::string kind:{"leg_bend","waist_175","leg_waist_175"}) {
    Path path{rest,rest,rest};
    if(kind!="waist_175") {path[1][14]=.30;path[1][15]=-.15;path[1][16]=.05;}
    if(kind!="leg_bend") {path[1][17]=yaw/2;path[2][17]=yaw;}
    motion_control_pkg::motion::TcpLevelPlan level(!rigid,model,start,true,true,true,lt,rt,tolerance,grasp);
    std::string error;require(level.initialize(error),error);
    const bool projected=level.project(path,error);require(projected,kind+": "+error);
    Q velocity,acceleration,jerk;
    for(const auto& n:active) {
      velocity.push_back(std::min(.6,model.velocityLimit[model.joints[model.getJointId(n)].idx_v()]*.3));
      acceleration.push_back(1.2);jerk.push_back(6.);
    }
    Path samples;
    require(algo->toppraPlan(path,dt,velocity,acceleration,samples,
      MiracleMAlgo::ToppraSolverMode::kStableCartesian,120,jerk),"TOPPRA "+kind);
    require(samples.front().size()==18 && samples.back().size()==18,"sample layout");
    for(int j=14;j<18;++j) {
      require(std::abs(samples.front()[j]-path.front()[j])<1e-9,"start leg changed");
      require(std::abs(samples.back()[j]-path.back()[j])<1e-9,"goal leg changed");
    }
    save(kind,rigid?"production coupled rigid-grasp projection + TOPPRA":"production TcpLevelPlan + TOPPRA, prescribed lower-body path (not a MoveJoints Action)",expand(samples));
  }
  // Explicit invalid-target regression: never silently clip 180 degrees.
  motion_control_pkg::motion::TcpLevelConstraint c(model,start,true,true,lt,rt,tolerance);
  std::string error;require(c.initialize(error),error);
  Q outside=start;outside[index("body_joint")]=std::acos(-1.);
  require(!c.check(outside,error),"URDF must reject zero-to-180 target");
  out["negative_checks"]["waist_180_rejected"]=true;
  out["negative_checks"]["waist_180_reason"]=error;
  outside=start;outside[index("leg_joint1")]=.15;
  require(!c.check(outside,error),"uncompensated tilt must be detected");
  out["negative_checks"]["uncompensated_tilt_rejected"]=true;

  rclcpp::init(0,nullptr);
  rclcpp::NodeOptions options;options.enable_rosout(false);options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  auto node=std::make_shared<rclcpp::Node>("mujoco_offline_planner",options);
  WholeBodyModule whole(node,algo,nullptr,nullptr,"base_link",lt,rt);
  for(bool line:{false,true}) {
    auto goal=std::make_shared<MovePose::Goal>();goal->arm_id=7;
    goal->keep_tcp_level=!rigid;goal->tcp_level_tolerance_rad=tolerance;
    goal->keep_grasp_rigid=rigid;
    goal->speed=.2;goal->timeout_ms=60000;goal->enable_move_line=line;
    Eigen::Isometry3d common=Eigen::Isometry3d::Identity();
    common.linear()=Eigen::AngleAxisd(rigid?.15:.025,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d center=.5*(fk(start,true).translation()+fk(start,false).translation());
    common.translation()=center-common.rotation()*center+Eigen::Vector3d(0,0,.03);
    for(bool left:{true,false}) {
      auto p=fk(start,left);p.translation().z()+=.03;
      p.linear()=Eigen::AngleAxisd(.025,Eigen::Vector3d::UnitZ()).toRotationMatrix()*p.rotation();
      if(rigid) p=common*fk(start,left);
      auto& target=left?goal->left_target_pose:goal->right_target_pose;
      target.header.frame_id="base_link";target.pose=motion_control_pkg::kinematics::isometryToPose(p);
    }
    Path lp,rp,bp;std::optional<Eigen::Isometry3d> lg,rg;
    const bool ok=whole.plan_trajectory(goal,start,lp,rp,bp,error,lg,rg);
    require(ok,"WBC planning: "+error);require(lp.size()==bp.size() && rp.size()==bp.size(),"WBC time alignment");
    Path full;
    for(std::size_t i=0;i<bp.size();++i) {
      Q a=lp[i],q;a.insert(a.end(),rp[i].begin(),rp[i].end());a.insert(a.end(),bp[i].begin(),bp[i].end());
      require(algo->mergeWholeBodyJoints(start,true,true,a,q),"merge WBC");full.push_back(q);
    }
    save(line?"wbc_movel":"wbc_movep","production WholeBodyModule::plan_trajectory, arm_id=7",full,lg,rg);
  }
  rclcpp::shutdown();
  std::ofstream file(argv[2]);require(bool(file),"cannot open output");
  Json::StreamWriterBuilder writer;writer["indentation"]="";
  file<<Json::writeString(writer,out)<<'\n';require(bool(file),"cannot write output");
  return 0;
} catch(const std::exception& e) {
  std::cerr<<"FAIL "<<e.what()<<std::endl;if(rclcpp::ok()) rclcpp::shutdown();return 1;
}
