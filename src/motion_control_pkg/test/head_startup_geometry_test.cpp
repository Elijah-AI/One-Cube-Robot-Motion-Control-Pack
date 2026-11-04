#include "motion_control_pkg/miracle_m_algo.hpp"
#include <iostream>
#include <stdexcept>
void require(bool ok,const std::string& message){if(!ok)throw std::runtime_error(message);}
int main(int argc,char** argv)try {
  require(argc==2,"expected repository path");
  const std::string root=argv[1];
  MiracleMAlgo algo(root+"/src/onecube_robot_description/urdf/onecube_robot_description.urdf",root+"/src");
  const auto& model=algo.getModel();
  for(const char* name:{"body_joint","head_yaw_joint","head_pitch_joint"}) {
    const auto& joint=model.joints[model.getJointId(name)];
    const double expected_velocity=std::string(name)=="body_joint"?1.31:.61;
    require(std::abs(model.velocityLimit[joint.idx_v()]-expected_velocity)<1e-12,
        std::string(name)+" must load the requested URDF speed "+std::to_string(expected_velocity));
  }
  const auto idx=[&](const char* name){return model.joints[model.getJointId(name)].idx_q();};
  std::vector<double> full;require(algo.getFullJointState(full),"full state");
  const auto lower=model.lowerPositionLimit,upper=model.upperPositionLimit;
  full[idx("head_pitch_joint")]=-.7;
  const std::vector<std::vector<double>> path{{0,-.7},{0,-.6},{0,-.3},{0,0}};
  std::string error;
  const bool accepted=algo.validateHeadStartupPath(full,path,{-2.92,-.7},{2.92,.41},error);
  require(accepted,error);
  std::cout<<"INFO neutral body recovery geometry: "<<(accepted?"accepted":error)<<'\n';
  require(!algo.isFullQValid(algo.toEigenConfig(full)),"normal validator acquired recovery exception");
  require(!algo.validateHeadStartupPath(full,path,{-2.92,-.62},{2.92,.41},error),"unextended recovery bounds accepted invalid head");
  auto bad=full;bad[idx("body_joint")]=upper[idx("body_joint")]+.1;
  require(!algo.validateHeadStartupPath(bad,path,{-2.92,-.7},{2.92,.41},error),"waist bounds bypassed");
  require(!algo.validateHeadStartupPath(full,{{0,-.7},{0,-.71},{0,0}},{-2.92,-.7},{2.92,.41},error),"outward path accepted");
  require((lower-model.lowerPositionLimit).norm()==0 && (upper-model.upperPositionLimit).norm()==0,"model limits mutated");
  std::cout<<"PASS actual URDF recovery-only head bounds, geometry check, inward path, unchanged model\n";
  return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
