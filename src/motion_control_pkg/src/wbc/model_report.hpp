#pragma once
#include "motion_control_pkg/wbc/model.hpp"
#include <json/json.h>
#include <cmath>

namespace motion_control_pkg::wbc {
template<class Values> Json::Value jsonArray(const Values& values) {
  Json::Value out(Json::arrayValue);
  for (const auto& value : values) out.append(value);
  return out;
}
inline Json::Value vectorJson(const Eigen::Vector3d& value) {
  Json::Value out(Json::arrayValue);
  for (int i = 0; i < 3; ++i) out.append(value[i]);
  return out;
}
inline Json::Value modelReport(const CentroidalModel& model, const JointSnapshot& state,
                               const CentroidalResult* com, const SupportResult* support) {
  Json::Value out;
  out["schema_version"] = 1;
  out["frame_id"] = model.rootFrame();
  out["robot_properties_source"] = "urdf";
  out["root_assumption"] = "stationary_horizontal";
  out["synthetic_state"] = state.synthetic;
  out["mass_kg"] = model.mass();
  out["position_valid"] = state.position_valid;
  out["velocity_valid"] = state.velocity_valid;
  out["missing_joints"] = jsonArray(state.missing);
  out["stale_joints"] = jsonArray(state.stale);
  out["assumed_joints"] = jsonArray(state.assumed);
  out["estimated_velocity_joints"] = jsonArray(state.estimated_velocity);
  out["all_positions_measured"] = state.position_valid && state.assumed.empty();
  out["oldest_state_stamp_sec"] = state.oldest_stamp;
  out["newest_state_stamp_sec"] = state.newest_stamp;
  out["contact_observed"] = false;
  out["zmp_valid"] = false;
  out["geometry_valid"] = support && support->geometry_valid;
  if (com) {
    out["com_m"] = vectorJson(com->com);
    if (com->velocity_valid) out["com_velocity_mps"] = vectorJson(com->velocity);
    out["jacobian_joint_names"] = jsonArray(com->joint_names);
  }
  if (support) {
    out["support_source"] = "urdf_collision_mesh_extrema_assuming_all_supports_contact";
    out["support_plane_z_m"] = support->plane_z;
    out["support_error"] = support->error;
    if (support->geometry_valid) {
      out["com_margin_after_inset_m"] = support->signed_margin;
      out["com_projection_inside_inset"] = support->signed_margin >= 0.0;
    }
    out["support_polygon_xy_m"] = Json::Value(Json::arrayValue);
    for (const auto& p : support->polygon) {
      Json::Value row(Json::arrayValue); row.append(p.x()); row.append(p.y());
      out["support_polygon_xy_m"].append(row);
    }
    for (const auto& p : support->contact_candidates)
      out["contact_candidates_m"][p.first] = vectorJson(p.second);
  }
  return out;
}
}  // namespace motion_control_pkg::wbc
