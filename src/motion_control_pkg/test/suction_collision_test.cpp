#include "motion_control_pkg/miracle_m_algo.hpp"
#include "motion_control_pkg/motion/suction_collision_pairs.hpp"
#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>

static void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv) try {
  require(argc == 2, "expected workspace root");
  const std::string root = argv[1];
  MiracleMAlgo planner(root + "/src/onecube_robot_description/urdf/onecube_robot_description.urdf", root + "/src");
  const auto& model = planner.getModel();
  const auto pairs = motion_control_pkg::motion::suctionCollisionPairs();
  std::set<std::pair<std::string, std::string>> unique(pairs.begin(), pairs.end());
  require(pairs.size() == 80 && unique.size() == pairs.size(), "suction policy count/duplicates");
  const auto contains = [&](std::string a, std::string b) {
    if (b < a) std::swap(a, b);
    return unique.count({a, b}) != 0;
  };
  for (const std::string side : {"l", "r"}) {
    const std::string arm = side == "l" ? "L" : "R";
    for (const std::string suffix : {"base_link", "Link"}) {
      const std::string name = "sucker_" + side + "_" + suffix;
      require(contains(name, "body_link") && contains(name, "head_pitch_link") &&
              contains(name, "base_link") && contains(name, "Link1_" + arm) &&
              contains(name, "Link5_" + arm), "tool/body/own-arm coverage missing");
      require(!contains(name, "Link6_" + arm) && !contains(name, "Link7_" + arm), "mounting wrist excluded incorrectly");
    }
    require(!contains("sucker_" + side + "_base_link", "sucker_" + side + "_Link"), "rigid attachment must be excluded");
  }
  if (!model.existFrame("sucker_l_Link")) {
    std::cout << "PASS legacy model accepts optional suction pair extension\n";
    return 0;
  }
  for (const std::string name : {"sucker_l_base_link", "sucker_l_Link", "sucker_r_base_link", "sucker_r_Link",
                                 "gripper_base_l_link", "gripper_base_r_link"})
    require(model.existFrame(name), "required tool or compatibility frame missing: " + name);
  require(model.nq == 24, "unintended controlled/passive degree of freedom change");
  std::vector<double> neutral;
  require(planner.getFullJointState(neutral), "neutral state");
  planner.setCollisionCheckMode(1);
  require(planner.isFullQValid(Eigen::Map<Eigen::VectorXd>(neutral.data(), neutral.size())), "neutral suction pose collides");
  Json::Value fixture;
  std::ifstream input(root + "/src/motion_control_pkg/test/fixtures/suction_collision_cases.json");
  require(input.good(), "missing independent Coal collision cases");
  input >> fixture;
  require(fixture["cases"].size() >= 4, "insufficient collision cases");
  std::set<std::string> covered;
  for (const auto& entry : fixture["cases"]) {
    Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd>(neutral.data(), neutral.size());
    for (const auto& name : entry["joints"].getMemberNames()) {
      require(model.existJointName(name), "unknown fixture joint");
      q[model.joints[model.getJointId(name)].idx_q()] = entry["joints"][name].asDouble();
    }
    planner.setCollisionCheckMode(0);
    require(planner.isFullQValid(q), "fixture is invalid without geometry checks");
    planner.setCollisionCheckMode(1);
    require(!planner.isFullQValid(q), "new suction collision was not rejected: " + entry["name"].asString());
    for (const auto& pair : entry["expected_collisions"])
      for (const auto& name : pair)
        if (name.asString().find("sucker_") == 0) covered.insert(name.asString());
  }
  require(covered.size() == 4, "not all four suction geometries exercised");
  std::cout << "PASS suction collision pairs=80 all_four_tools=4 neutral=clear collision_cases="
            << fixture["cases"].size() << "\n";
  return 0;
} catch (const std::exception& e) {
  std::cerr << "FAIL " << e.what() << '\n';
  return 1;
}
