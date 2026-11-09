#include "motion_control_pkg/miracle_m_algo.hpp"
#include "motion_control_pkg/motion/leg_body_orientation.hpp"
#include "com_interfaces/leg_joint_goal.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool value, const std::string& text) {
  if (!value) throw std::runtime_error(text);
}
}

int main(int argc, char** argv) try {
  check(argc == 2, "expected repository path");
  const std::string root = argv[1];
  MiracleMAlgo algo(root + "/src/onecube_robot_description/urdf/onecube_robot_description.urdf", root + "/src");
  const auto& model = algo.getModel();
  using motion_control_pkg::motion::LegJoints;
  motion_control_pkg::motion::LegBodyOrientationPlanner planner(model);
  const LegJoints rest{{-1.0, 2.2, -1.2258046865463257, -1.5707963267948966}};
  std::size_t scenarios = 0;
  auto verify = [&](const LegJoints& start, const std::vector<std::string>& names,
                    const std::vector<double>& goals, bool waist) {
    const auto path = planner.plan(start, names, goals, .8, .004, 60);
    const auto reference = planner.orientation(start);
    check(!path.positions.empty(), "empty path");
    for (std::size_t i = 0; i < path.positions.size(); ++i) {
      LegJoints sample{};
      std::copy(path.positions[i].begin(), path.positions[i].end(), sample.begin());
      planner.validatePosition(sample);
      for (std::size_t j = 0; j < 4; ++j) {
        const double distance = path.target[j] - start[j];
        if (std::abs(distance) > 1e-10) {
          const double phase = (sample[j] - start[j]) / distance;
          const double leg_phase = (sample[0] - start[0]) / (path.target[0] - start[0]);
          check(std::abs(phase - leg_phase) < 1e-9, "joints do not share trajectory progress");
        }
        const std::string joint = j == 3 ? "body_joint" : "leg_joint" + std::to_string(j + 1);
        const auto idx = model.joints[model.getJointId(joint)].idx_v();
        check(std::abs(path.velocities[i][j]) <= std::min(1.5, model.velocityLimit[idx]) * .8 + 1e-8,
              "URDF velocity exceeded");
      }
      if (waist) sample[3] = start[3];
      check(planner.orientationError(sample, reference) <= planner.kOrientationTolerance,
            "leg pitch compensation lost during waist motion");
    }
    for (std::size_t i = 0; i < names.size(); ++i) {
      const std::size_t j = names[i] == "body_joint" ? 3 : static_cast<std::size_t>(names[i].back() - '1');
      check(std::abs(path.target[j] - goals[i]) < 1e-12, "commanded endpoint changed");
    }
    ++scenarios;
    return path.target;
  };
  auto extended = verify(rest, {"leg_joint1", "leg_joint2", "body_joint"}, {-.65, 2.0, -.3}, true);
  extended[3] = .3;  // Both placements finish at the left-side waist angle.
  verify(extended, {"leg_joint1", "leg_joint2", "body_joint"}, {-1.0, 2.2, rest[3]}, true);
  verify(rest, {"body_joint", "leg_joint2", "leg_joint1"}, {-.3, 2.0, -.65}, true);
  verify(rest, {"leg_joint1", "body_joint", "leg_joint2"}, {-.65, -.3, 2.0}, true);
  verify(rest, {"leg_joint1", "leg_joint2"}, {-.65, 2.0}, false);
  auto recovered = rest;
  recovered[3] = -.3;  // Recovery branch turns and descends before extending.
  verify(recovered, {"leg_joint1", "leg_joint2", "body_joint"}, {-.65, 2.0, -.3}, true);
  const auto noop = planner.plan(rest, {"leg_joint1", "leg_joint2", "body_joint"},
                                {rest[0], rest[1], rest[3]}, .8, .004, 60);
  check(noop.duration == 0 && noop.positions.size() == 1, "no-op moved");
  ++scenarios;
  com_interfaces::LegJointSelection selection;
  std::string error;
  for (const auto& names : std::vector<std::vector<std::string>>{
      {"leg_joint1", "body_joint"}, {"leg_joint1", "leg_joint2", "leg_joint3"},
      {"leg_joint1", "leg_joint1", "body_joint"}, {"body_joint", "body_joint", "leg_joint1"},
      {"leg_joint1", "leg_joint2", "unknown"}}) {
    check(!com_interfaces::validateLegOrientationGoal(names, std::vector<double>(names.size(), 0), selection, error),
          "invalid coordinated goal accepted");
    ++scenarios;
  }
  check(!com_interfaces::validateLegOrientationGoal({"leg_joint1", "leg_joint2", "body_joint"},
      {0, 0, std::numeric_limits<double>::quiet_NaN()}, selection, error), "nonfinite waist accepted");
  ++scenarios;
  try {
    planner.plan(rest, {"leg_joint1", "leg_joint2", "body_joint"}, {-.65, 2.0, 100}, .8, .004, 60);
    throw std::runtime_error("waist outside URDF limits accepted");
  } catch (const std::invalid_argument&) { ++scenarios; }
  std::cout << "PASS " << scenarios << " compensated waist/leg trajectory scenarios, actual URDF, no hardware\n";
  return 0;
} catch (const std::exception& e) {
  std::cerr << "FAIL " << e.what() << '\n';
  return 1;
}
