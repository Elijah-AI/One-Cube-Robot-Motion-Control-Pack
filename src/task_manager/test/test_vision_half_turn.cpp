#include "task_manager/vision_pose_utils.hpp"

#include <array>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

using Result = com_interfaces::action::DetectionPoseCloud::Result;
using Pose = geometry_msgs::msg::PoseStamped;

void check(bool ok, const std::string& message)
{
  if (!ok) throw std::runtime_error(message);
}

void same_rotation(const Pose& a, const Pose& b)
{
  const auto& q = a.pose.orientation;
  const auto& r = b.pose.orientation;
  const double dot = q.x*r.x + q.y*r.y + q.z*r.z + q.w*r.w;
  check(std::abs(std::abs(dot)-1.0) < 1e-10, "rotation mismatch");
}

void same_target(const Pose& a, const Pose& b)
{
  check(std::abs(a.pose.position.x-b.pose.position.x) < 1e-10 &&
    std::abs(a.pose.position.y-b.pose.position.y) < 1e-10 &&
    std::abs(a.pose.position.z-b.pose.position.z) < 1e-10, "slot position mismatch");
  same_rotation(a, b);
}

int main(int argc, char** argv)
{
  try {
    check(argc == 3, "expected task and temporary YAML paths");
    task_cfg::TaskConfigParser parser;
    task_cfg::GlobalConfig config{};
    check(parser.load_task_config(argv[1], config, "laptop"), "task YAML failed");
    const auto& actions = config.task_dict.at("laptop").action_seq.at(0);
    task_cfg::VisionActionParam param{};
    std::vector<task_cfg::MovePoseParam> moves;
    bool grasp_found = false;
    for (const auto& action : actions) {
      if (action.action_type == "vision_detect") {
        if (action.vision_param.result_key == "rack") {
          param = action.vision_param;
          grasp_found = true;
        } else {
          check(action.vision_param.canonical_y_direction.empty(), "placement policy changed");
        }
      }
      if (action.action_type == "move_pose" && action.arm_param.vision_result_key == "rack") {
        moves.push_back(action.arm_param);
      }
    }
    check(grasp_found && moves.size() == 9U, "missing rack or three-round grasp stages");
    check(param.canonical_y_direction == std::vector<double>({-1,0,0}), "wrong reference");

    // Actual measurements, including both observed half-turn branches and tilt.
    const std::array<std::array<double, 7>, 4> measurements{{
      {{0.3468480706214905,0.5544589757919312,1.025956153869629,
        -0.017291828989982605,-0.011974404565989971,0.7143856287002563,0.6994360089302063}},
      {{0.36174434423446655,0.5441305637359619,0.8877455592155457,
        -0.014383471570909023,-0.017965983599424362,-0.7017512321472168,0.712050199508667}},
      {{0.34021010994911194,0.5693796277046204,0.8869592547416687,
        -0.010842817835509777,-0.022000065073370934,-0.7137033343315125,0.700018584728241}},
      {{0.37228307127952576,0.5306885242462158,1.0246281623840332,
        -0.06281109154224396,0.03729552403092384,0.6937242150306702,0.7165267467498779}}
    }};
    Result last_input;
    std::size_t comparisons = 0;
    for (std::size_t i = 0; i < measurements.size(); ++i) {
      const auto& m = measurements[i];
      Result input;
      input.success = true;
      input.detected_targets.resize(1);
      auto& pose = input.detected_targets[0].pose;
      pose.header.frame_id = "base_link";
      pose.header.stamp.sec = 123;
      pose.pose.position.x=m[0]; pose.pose.position.y=m[1]; pose.pose.position.z=m[2];
      auto& q = pose.pose.orientation;
      q.x=m[3]; q.y=m[4]; q.z=m[5]; q.w=m[6];
      Result canonical;
      bool flipped = false;
      std::string error;
      check(task_act::prepare_vision_result(input,param,canonical,flipped,error), error);
      check(flipped == (i == 1 || i == 2), "wrong measured branch");
      check(canonical.detected_targets[0].pose.header == pose.header, "header changed");
      check(canonical.detected_targets[0].pose.pose.position == pose.pose.position, "origin changed");
      // A pure local-Z translation must not change with the correction.
      task_cfg::VisionPoseOffset z_offset;
      z_offset.enabled=true; z_offset.translation_frame="vision";
      z_offset.translation={0,0,0.1};
      Pose z_raw,z_fixed;
      check(task_act::offset_vision_pose(pose,z_offset,"base_link",z_raw,error), error);
      check(task_act::offset_vision_pose(canonical.detected_targets[0].pose,z_offset,
        "base_link",z_fixed,error), error);
      check(std::abs(z_raw.pose.position.x-z_fixed.pose.position.x)<1e-12 &&
        std::abs(z_raw.pose.position.y-z_fixed.pose.position.y)<1e-12 &&
        std::abs(z_raw.pose.position.z-z_fixed.pose.position.z)<1e-12, "normal changed");

      for (const double scale : {1.0,-1.0,3.0,-3.0}) {
        for (const bool turn : {false,true}) {
          Result variant=input, fixed;
          auto& v=variant.detected_targets[0].pose.pose.orientation;
          v.x=scale*(turn?q.y:q.x); v.y=scale*(turn?-q.x:q.y);
          v.z=scale*(turn?q.w:q.z); v.w=scale*(turn?-q.z:q.w);
          check(task_act::prepare_vision_result(variant,param,fixed,flipped,error), error);
          same_target(canonical.detected_targets[0].pose,fixed.detected_targets[0].pose);
          Result again;
          check(task_act::prepare_vision_result(fixed,param,again,flipped,error), error);
          check(!flipped, "canonicalization is not idempotent");
          same_target(fixed.detected_targets[0].pose,again.detected_targets[0].pose);
          for (const auto& move : moves) {
            Pose left,right;
            for (const auto& offset : {move.left_vision_pose_offset,move.right_vision_pose_offset}) {
              Pose a,b;
              check(task_act::offset_vision_pose(canonical.detected_targets[0].pose,offset,
                "base_link",a,error), error);
              check(task_act::offset_vision_pose(fixed.detected_targets[0].pose,offset,
                "base_link",b,error), error);
              same_target(a,b); ++comparisons;
            }
            check(task_act::offset_vision_pose(fixed.detected_targets[0].pose,
              move.left_vision_pose_offset,"base_link",left,error), error);
            check(task_act::offset_vision_pose(fixed.detected_targets[0].pose,
              move.right_vision_pose_offset,"base_link",right,error), error);
            check(left.pose.position.x < right.pose.position.x, "arms exchanged");
          }
        }
      }
      last_input = input;
    }
    Result output;
    bool flipped=false;
    std::string error;
    auto disabled=param; disabled.canonical_y_direction.clear();
    check(task_act::prepare_vision_result(last_input,disabled,output,flipped,error), error);
    check(output == last_input && !flipped, "disabled policy changed result");
    for (int kind=0; kind<6; ++kind) {
      auto bad=last_input;
      if(kind==0) bad.detected_targets[0].pose.header.frame_id="camera";
      if(kind==1) bad.detected_targets.clear();
      if(kind==2) bad.detected_targets.push_back(bad.detected_targets[0]);
      if(kind==3) bad.detected_targets[0].pose.pose.orientation = geometry_msgs::msg::Quaternion();
      if(kind==4) bad.detected_targets[0].pose.pose.orientation.x=std::numeric_limits<double>::quiet_NaN();
      if(kind==5) { // local +Y perpendicular to base -X: no reliable sign.
        auto& q=bad.detected_targets[0].pose.pose.orientation;
        q.x=0;q.y=0;q.z=0;q.w=1;
      }
      check(!task_act::prepare_vision_result(bad,param,output,flipped,error), "invalid vision accepted");
    }
    // Parser rejects malformed/unsafe policies instead of silently disabling them.
    for (int kind=0; kind<7; ++kind) {
      auto root=YAML::LoadFile(argv[1]);
      for (auto action : root["action_sequence1"]) {
        if (!action["action_type"] || action["action_type"].as<std::string>()!="vision_detect" ||
            action["params"]["result_key"].as<std::string>()!="rack") continue;
        auto p=action["params"];
        if(kind==0) p["canonical_y_direction"]=YAML::Load("[]");
        if(kind==1) p["canonical_y_direction"]=YAML::Load("[0,0,0]");
        if(kind==2) p["canonical_y_direction"]=YAML::Load("[.nan,0,0]");
        if(kind==3) p["canonical_y_min_alignment"]=0.0;
        if(kind==4) p["require_expected_frame"]=false;
        if(kind==5) p["require_single_target"]=false;
        if(kind==6) p.remove("canonical_y_direction");
      }
      {std::ofstream f(argv[2]); f << root;}
      task_cfg::GlobalConfig invalid{};
      check(!parser.load_task_config(argv[2],invalid,"invalid"), "bad policy parsed");
    }
    std::cout << "PASS: " << comparisons << " target comparisons, measured branches, "
      "quaternion sign/scale, normal preservation, idempotence, legacy mode, invalid inputs and parser\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
