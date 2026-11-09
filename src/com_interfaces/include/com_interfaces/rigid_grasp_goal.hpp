#pragma once
#include <cmath>
#include <string>
namespace com_interfaces {
template<class Goal> bool validateRigidGraspGoal(const Goal& goal,std::string& error) {
  if(!std::isfinite(goal.grasp_position_tolerance_m)||goal.grasp_position_tolerance_m<1e-5||goal.grasp_position_tolerance_m>.01||
     !std::isfinite(goal.grasp_orientation_tolerance_rad)||goal.grasp_orientation_tolerance_rad<1e-4||goal.grasp_orientation_tolerance_rad>.05) {
    error="grasp tolerances must be position [0.00001,0.01] m, orientation [0.0001,0.05] rad";return false;
  }
  if(goal.keep_grasp_rigid && (goal.arm_id!=7||goal.control_mode!=0||goal.synchronization_mode!=0||
      goal.keep_tcp_level||goal.use_force_control)) {
    error="keep_grasp_rigid requires WBC BOTH arm_id=7, synchronized position control, keep_tcp_level=false, use_force_control=false";
    return false;
  }error.clear();return true;
}
}
