#pragma once
#include <cmath>
#include <string>
#include "com_interfaces/leg_joint_goal.hpp"
namespace com_interfaces {
inline bool validateRigidLegGraspRequest(bool enabled,double position_tolerance,
    double orientation_tolerance,unsigned arm_id,unsigned control_mode,unsigned sync,
    bool keep_tcp_level,std::string& error) {
  if(!std::isfinite(position_tolerance)||position_tolerance<1e-5||position_tolerance>.01||
     !std::isfinite(orientation_tolerance)||orientation_tolerance<1e-4||orientation_tolerance>.05) {
    error="grasp tolerances must be position [0.00001,0.01] m, orientation [0.0001,0.05] rad";return false;
  }
  if(enabled && (arm_id!=4||control_mode!=0||sync!=0||keep_tcp_level)) {
    error="rigid LEG MoveJ requires arm_id=4, synchronized position control and keep_tcp_level=false";return false;
  }
  error.clear();return true;
}
template<class Goal> bool validateRigidLegGraspGoal(const Goal& goal,std::string& error) {
  return validateRigidLegGraspRequest(goal.keep_grasp_rigid,goal.grasp_position_tolerance_m,
    goal.grasp_orientation_tolerance_rad,goal.arm_id,goal.control_mode,
    goal.synchronization_mode,goal.keep_tcp_level,error);
}
inline bool validateRigidLegTargets(const std::vector<std::string>& names,
    const std::vector<double>& values,unsigned mode,std::string& error) {
  if(mode!=0) {
    if(names.empty()&&values.empty()) {error.clear();return true;}
  } else if(names.size()==1 && names[0]=="body_joint" && values.size()==1 && std::isfinite(values[0])) {
    error.clear();return true;
  } else if(names.size()==2 || names.size()==3) {
    LegJointSelection selection;return validateLegOrientationGoal(names,values,selection,error);
  } else if(names.size()==4 && values.size()==4) {
    std::array<bool,4> used{};
    const std::array<std::string,4> joints{{"leg_joint1","leg_joint2","leg_joint3","body_joint"}};
    for(std::size_t i=0;i<4;++i) {
      std::size_t j=0;while(j<4&&joints[j]!=names[i]) ++j;
      if(j==4||used[j]||!std::isfinite(values[i])) {error="invalid or duplicate rigid LEG joint target";return false;}
      used[j]=true;
    }
    error.clear();return true;
  }
  error="rigid LEG MoveJ needs body_joint, two leg joints optionally with waist, or all four named joints";return false;
}
template<class Goal> bool isRigidGraspLeaderGoal(const Goal& goal) {
  return goal.keep_grasp_rigid &&
    (goal.arm_id == 0 || goal.arm_id == 1 || goal.arm_id == 5 || goal.arm_id == 6);
}
template<class Goal> bool validateRigidGraspGoal(const Goal& goal,std::string& error) {
  if(!std::isfinite(goal.grasp_position_tolerance_m)||goal.grasp_position_tolerance_m<1e-5||goal.grasp_position_tolerance_m>.01||
     !std::isfinite(goal.grasp_orientation_tolerance_rad)||goal.grasp_orientation_tolerance_rad<1e-4||goal.grasp_orientation_tolerance_rad>.05) {
    error="grasp tolerances must be position [0.00001,0.01] m, orientation [0.0001,0.05] rad";return false;
  }
  const bool leader = isRigidGraspLeaderGoal(goal);
  if(leader && goal.enable_move_line) {
    error="single-target rigid grasp requires MoveP (enable_move_line=false)";return false;
  }
  if(goal.keep_grasp_rigid && ((!leader && goal.arm_id!=7)||goal.control_mode!=0||goal.synchronization_mode!=0||
      goal.keep_tcp_level||goal.use_force_control)) {
    error="keep_grasp_rigid requires a MoveP leader (arm_id=0/1/5/6) or WBC BOTH (7), synchronized position control, keep_tcp_level=false, use_force_control=false";
    return false;
  }error.clear();return true;
}
}
