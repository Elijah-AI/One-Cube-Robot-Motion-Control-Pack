#include "task_manager/current_tcp_height.hpp"

#include <array>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

using Pose = geometry_msgs::msg::PoseStamped;

void check(bool ok, const std::string& message)
{
  if (!ok) throw std::runtime_error(message);
}

std::array<double, 3> x_axis(const Pose& pose)
{
  const auto& q = pose.pose.orientation;
  return {1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y+q.z*q.w),
    2*(q.x*q.z-q.y*q.w)};
}

void check_vertical(const Pose& level, const Pose& original)
{
  const auto& q = level.pose.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  check(std::abs(norm-1) < 1e-12, "vertical quaternion not unit length");
  check(std::abs(2*(q.x*q.z+q.y*q.w)) < 1e-12 &&
    std::abs(2*(q.y*q.z-q.x*q.w)) < 1e-12 &&
    std::abs(1-2*(q.x*q.x+q.y*q.y)+1) < 1e-12, "TCP Z is not base -Z");
  check(level.pose.position == original.pose.position, "slot position changed");
  check(level.header == original.header, "header changed");
  const auto x = x_axis(original);
  const auto flat_x = x_axis(level);
  const double length = std::hypot(x[0], x[1]);
  check(std::abs(flat_x[0]-x[0]/length) < 1e-12 &&
    std::abs(flat_x[1]-x[1]/length) < 1e-12 &&
    std::abs(flat_x[2]) < 1e-12, "horizontal TCP X heading changed");
}

int main(int argc, char** argv)
{
  try {
    check(argc == 3, "expected task and temporary YAML paths");
    task_cfg::TaskConfigParser parser;
    task_cfg::GlobalConfig config{};
    check(parser.load_task_config(argv[1], config, "laptop"), "task failed to parse");
    const auto& actions = config.task_dict.at("laptop").action_seq.at(0);
    std::vector<task_cfg::MovePoseParam> placement;
    for (const auto& action : actions) {
      if (action.action_type != "move_pose") continue;
      const auto& move = action.arm_param;
      if (move.vision_result_key == "tray") {
        check(move.arm_id <= 1 && move.vision_pose_offset.force_vertical_down,
          "placement stage missing vertical policy");
        placement.push_back(move);
      } else {
        check(!move.left_vision_pose_offset.force_vertical_down &&
          !move.right_vision_pose_offset.force_vertical_down, "grasp policy changed");
      }
    }
    check(placement.size() == 24U, "expected four stages per arm in three rounds");
    const std::array<std::array<double, 4>, 4> measured{{
      {{-.0139122,-.0104235,.0112146,.999786}}, // actual Sept 30 tray tilt
      {{.10,-.15,.45,.88}},
      {{-.2,.1,-.9,.3}},
      {{0,0,0,1}}
    }};
    std::size_t comparisons = 0;
    std::string error;
    for (const auto& measurement : measured) {
      for (double scale : {1.0,-1.0,3.0,-3.0}) {
        Pose reference;
        reference.header.frame_id = "base_link";
        reference.header.stamp.sec = 123;
        reference.pose.position.x = .725507;
        reference.pose.position.y = -.0562869;
        reference.pose.position.z = .793093;
        auto& q = reference.pose.orientation;
        q.x=scale*measurement[0]; q.y=scale*measurement[1];
        q.z=scale*measurement[2]; q.w=scale*measurement[3];
        for (const auto& move : placement) {
          auto legacy = move.vision_pose_offset;
          legacy.force_vertical_down = false;
          Pose original, level;
          check(task_act::offset_vision_pose(reference,legacy,"base_link",original,error), error);
          check(task_act::offset_vision_pose(reference,move.vision_pose_offset,
            "base_link",level,error), error);
          check_vertical(level,original);
          ++comparisons;
          if (move.vision_pose_offset.keep_current_tcp_height) {
            com_interfaces::msg::MotorStatus state;
            state.header = reference.header;
            state.joint_positions.resize(14,0);
            state.joint_velocities.resize(14,0);
            state.end_effector_pose.resize(2);
            for (auto& tcp : state.end_effector_pose) {
              tcp.position.z = 1.3;
              tcp.orientation.w = 1;
            }
            auto legacy_move = move;
            legacy_move.vision_pose_offset.force_vertical_down = false;
            double height = 0, legacy_height = 0;
            check(task_act::resolve_current_tcp_height(reference,legacy_move,state,
              original,legacy_height,error),error);
            check(task_act::resolve_current_tcp_height(reference,move,state,
              level,height,error),error);
            check_vertical(level,original);
            check(std::abs(height-legacy_height) < 1e-12 &&
              std::abs(level.pose.position.z-1.3) < 1e-12,
              "vertical policy changed current height intersection");
          }
        }
      }
    }
    // Reject an undefined heading rather than inventing a gripper yaw.
    Pose ambiguous, output;
    ambiguous.header.frame_id = "base_link";
    ambiguous.pose.orientation.y = std::sqrt(.5);
    ambiguous.pose.orientation.w = std::sqrt(.5);
    task_cfg::VisionPoseOffset offset;
    offset.enabled = true;
    offset.force_vertical_down = true;
    check(!task_act::offset_vision_pose(ambiguous,offset,"base_link",output,error),
      "vertical TCP X heading accepted");
    ambiguous.pose.orientation = geometry_msgs::msg::Quaternion();
    ambiguous.pose.orientation.w = 0;
    check(!task_act::offset_vision_pose(ambiguous,offset,"base_link",output,error),
      "zero vision quaternion accepted");
    ambiguous.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
    check(!task_act::offset_vision_pose(ambiguous,offset,"base_link",output,error),
      "nonfinite vision quaternion accepted");
    ambiguous.pose.orientation.x = 0;
    ambiguous.pose.orientation.w = 1;
    ambiguous.header.frame_id = "camera";
    check(!task_act::offset_vision_pose(ambiguous,offset,"camera",output,error),
      "vertical policy accepted a non-base frame");
    for (int kind = 0; kind < 3; ++kind) {
      auto root = YAML::LoadFile(argv[1]);
      for (auto action : root["action_sequence1"]) {
        auto p = action["params"];
        if (!p || !p["vision_pose_offset"]) continue;
        if (kind == 0) p["frame_id"] = "camera";
        if (kind == 1) p["vision_pose_offset"]["force_vertical_down"] = "invalid";
        if (kind == 2) p["vision_pose_offset"].remove("force_vertical_down");
        break;
      }
      {std::ofstream f(argv[2]); f << root;}
      task_cfg::GlobalConfig parsed{};
      const bool loaded = parser.load_task_config(argv[2],parsed,"variant");
      check(loaded == (kind == 2), "vertical policy parser validation failed");
      if (kind == 2) {
        bool legacy_found = false;
        for (const auto& action : parsed.task_dict.at("variant").action_seq.at(0)) {
          if (action.action_type == "move_pose" && action.arm_param.vision_result_key == "tray") {
            legacy_found = !action.arm_param.vision_pose_offset.force_vertical_down;
            break;
          }
        }
        check(legacy_found, "missing flag no longer defaults to legacy mapping");
      }
    }
    std::cout << "PASS: " << comparisons << " vertical placement targets, 24 stages, "
      "slot/header/heading preservation, current height, sign/scale and invalid policies\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
