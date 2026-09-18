// JSON-lines offline prediction service. No ROS initialization or SDK linkage.
#include "model_report.hpp"
#include "motion_control_pkg/wbc/zmp.hpp"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <type_traits>
#include <set>

template<class T> std::vector<T> field(const Json::Value& input, const char* name) {
  if (!input[name].isArray()) throw std::invalid_argument(std::string("missing array: ")+name);
  std::vector<T> out;
  for (const auto& item : input[name]) {
    if constexpr (std::is_same_v<T, std::string>) {
      if (!item.isString()) throw std::invalid_argument("joint names must be strings");
      out.push_back(item.asString());
    } else {
      if (!item.isNumeric()) throw std::invalid_argument("q/v/a entries must be numbers");
      out.push_back(item.asDouble());
    }
  }
  return out;
}

int main(int argc, char** argv) {
  using namespace motion_control_pkg::wbc;
  if (argc < 6) { std::cerr << "Usage: wbc_zmp_audit URDF SUPPORT_LINK SUPPORT_LINK SUPPORT_LINK SUPPORT_LINK < states.jsonl\n"; return 2; }
  try {
    CentroidalModel model(argv[1]);
    SupportModel support(argv[1], std::vector<std::string>(argv+2, argv+argc));
    Json::StreamWriterBuilder writer; writer["indentation"] = "";
    std::string line;
    while (std::getline(std::cin, line)) {
      Json::Value out;
      out["prediction_valid"] = false;
      out["contact_observed"] = false;
      out["source"] = "offline_model_prediction_explicit_joint_scenario";
      out["root_assumption"] = "stationary_horizontal";
      try {
        if (line.size() > 1024*1024) throw std::invalid_argument("oversized state packet");
        Json::Value input; Json::CharReaderBuilder reader; std::string errors;
        std::istringstream stream(line);
        if (!Json::parseFromStream(reader, stream, &input, &errors) || !input.isObject())
          throw std::invalid_argument("invalid JSON object");
        const std::set<std::string> allowed{"joint_names","positions","velocities","accelerations","support_inset_m","assumed_joints"};
        for (const auto& name : input.getMemberNames())
          if (!allowed.count(name)) throw std::invalid_argument("unsupported field (moving root/external loads are not supported): " + name);
        if (input.isMember("support_inset_m") && !input["support_inset_m"].isNumeric())
          throw std::invalid_argument("support_inset_m must be numeric");
        JointSnapshot state;
        state.names = field<std::string>(input, "joint_names");
        state.positions = field<double>(input, "positions");
        state.velocities = field<double>(input, "velocities");
        state.position_valid = state.velocity_valid = state.synthetic = true;
        if (input.isMember("assumed_joints")) state.assumed = field<std::string>(input, "assumed_joints");
        const auto acceleration = field<double>(input, "accelerations");
        const auto dyn = model.dynamics(state, acceleration);
        const double inset = input.get("support_inset_m", .02).asDouble();
        const auto region = support.evaluate(model.evaluate(state), inset);
        const auto zmp = predictZmp(dyn, region);
        out["prediction_valid"] = zmp.valid;
        out["error"] = zmp.error;
        out["com_m"] = vectorJson(dyn.com);
        out["com_acceleration_mps2"] = vectorJson(dyn.acceleration);
        out["angular_momentum_rate_nm"] = vectorJson(dyn.angular_momentum_rate);
        out["required_force_n"] = vectorJson(zmp.required_force);
        out["required_moment_at_plane_origin_nm"] = vectorJson(zmp.required_moment);
        out["support_plane_z_m"] = region.plane_z;
        out["mass_kg"] = model.mass();
        bool velocity_ok = true;
        for (const auto& spec : model.joints()) {
          const auto i = std::distance(state.names.begin(), std::find(state.names.begin(),state.names.end(),spec.name));
          velocity_ok = velocity_ok && std::abs(state.velocities[i]) <= spec.velocity+1e-8;
        }
        out["urdf_velocity_limits_ok"] = velocity_ok;
        if (region.geometry_valid) out["static_com_margin_m"] = region.signed_margin;
        if (zmp.valid) {
          out["zmp_m"] = vectorJson(zmp.point);
          out["zmp_margin_m"] = zmp.margin;
          out["within_support"] = zmp.margin >= 0;
        }
      } catch (const std::exception& e) { out["error"] = e.what(); }
      std::cout << Json::writeString(writer,out) << '\n';
    }
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
