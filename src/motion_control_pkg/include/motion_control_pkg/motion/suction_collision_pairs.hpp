#pragma once

#include <array>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace motion_control_pkg::motion {

// Optional geometry extension: legacy gripper models keep their existing pairs.
// The two rigid parts of each suction tool are checked independently.
inline std::vector<std::pair<std::string, std::string>> suctionCollisionPairs() {
  std::set<std::pair<std::string, std::string>> unique;
  const auto add = [&](std::string first, std::string second) {
    if (second < first) std::swap(first, second);
    unique.emplace(std::move(first), std::move(second));
  };
  const std::array<std::string, 7> body{{
      "base_link", "leg_link1", "leg_link2", "leg_link3", "body_link",
      "head_yaw_link", "head_pitch_link"}};
  for (const std::string side : {"l", "r"}) {
    const std::string own = side == "l" ? "L" : "R";
    const std::string other = side == "l" ? "R" : "L";
    const std::string opposite = side == "l" ? "r" : "l";
    for (const std::string tool : {"sucker_" + side + "_base_link", "sucker_" + side + "_Link"}) {
      for (const auto& link : body) add(tool, link);
      for (int joint = 1; joint <= 7; ++joint)
        add(tool, "Link" + std::to_string(joint) + "_" + other);
      // Link6/Link7 are the mounting wrist; their intended attachment contact is
      // excluded. Earlier links on the same arm remain checked, including Link5.
      for (int joint = 1; joint <= 5; ++joint)
        add(tool, "Link" + std::to_string(joint) + "_" + own);
      add(tool, "sucker_" + opposite + "_base_link");
      add(tool, "sucker_" + opposite + "_Link");
    }
  }
  // Rigid same-tool bracket/cup contact is intentional, not a self-collision pair.
  return {unique.begin(), unique.end()};
}

}  // namespace motion_control_pkg::motion
