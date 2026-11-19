#pragma once
#include "motion_control_pkg/miracle_m_algo.hpp"
#include <jsoncpp/json/json.h>
#include <fstream>
#include <stdexcept>

// All tests and the MuJoCo exporter share this fixed top-grasp reference.
struct RigidFixture {
  Eigen::Isometry3d left_tool = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d right_tool = Eigen::Isometry3d::Identity();
  std::vector<double> start;

  RigidFixture(MiracleMAlgo& model, const std::string& repo) {
    Json::Value fixture;
    std::ifstream in(repo + "/src/motion_control_pkg/tools/mujoco_validation/rigid_grasp_fixture.json");
    if (!(in >> fixture)) throw std::runtime_error("cannot read rigid grasp fixture");
    for (bool left : {true, false}) {
      const auto& joints = fixture[left ? "left_joints" : "right_joints"];
      const auto& tf = fixture[left ? "left_tool_tf" : "right_tool_tf"];
      if (joints.size() != 7 || tf.size() != 7) throw std::runtime_error("invalid rigid grasp fixture");
      std::vector<double> q;
      for (const auto& value : joints) q.push_back(value.asDouble());
      if (model.updateChainJointsAngle(left ? "left_arm" : "right_arm", q) != 0)
        throw std::runtime_error("invalid rigid grasp fixture joints");
      auto& tool = left ? left_tool : right_tool;
      tool.translation() << tf[0].asDouble(), tf[1].asDouble(), tf[2].asDouble();
      tool.linear() = Eigen::Quaterniond(tf[6].asDouble(), tf[3].asDouble(), tf[4].asDouble(),
                                         tf[5].asDouble()).toRotationMatrix();
    }
    if (!model.getFullJointState(start)) throw std::runtime_error("missing rigid grasp fixture state");
  }
};
