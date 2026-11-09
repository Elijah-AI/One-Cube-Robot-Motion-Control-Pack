#pragma once
#include "motion_control_pkg/miracle_m_algo.hpp"
#include <stdexcept>

// A 0.50 m top grasp, using the production suction TCP calibration.
struct RigidFixture {
  Eigen::Isometry3d left_tool=Eigen::Isometry3d::Identity();
  Eigen::Isometry3d right_tool=Eigen::Isometry3d::Identity();
  std::vector<double> start;
  explicit RigidFixture(MiracleMAlgo& model) {
    left_tool.translation().z()=right_tool.translation().z()=.167;
    left_tool.linear()=Eigen::Quaterniond(.5,.5,-.5,.5).toRotationMatrix();
    right_tool.linear()=Eigen::Quaterniond(-.5,.5,.5,.5).toRotationMatrix();
    if(model.updateChainJointsAngle("left_arm",{
       1.7779331196597112,-1.2724396582521105,-1.001608629055811,
       -.575803938937869,-1.6538275652205925,.2637903896770379,-1.420791587551045})!=0 ||
       model.updateChainJointsAngle("right_arm",{
       -1.778723785748662,-1.2775189549111559,1.0067136733379585,
       -.5721654336238697,1.6531892920810893,.26169269371552184,1.4207999984788273})!=0 ||
       !model.getFullJointState(start)) throw std::runtime_error("invalid rigid grasp fixture");
  }
};
