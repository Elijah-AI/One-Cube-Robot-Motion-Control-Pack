#include "motion_control_pkg/wbc/zmp.hpp"
#include <cmath>
#include <limits>

namespace motion_control_pkg::wbc {
ZmpPrediction predictZmp(const CentroidalDynamics& d, const SupportResult& s,
                        double gravity, double min_force) {
  ZmpPrediction out;
  out.margin = std::numeric_limits<double>::quiet_NaN();
  if (!s.geometry_valid || s.A.cols() != 2 || s.A.rows() < 3 || s.b.size() != s.A.rows() ||
      !s.A.allFinite() || !s.b.allFinite() || !std::isfinite(s.plane_z) ||
      !std::isfinite(d.mass) || d.mass <= 0 || !d.com.allFinite() || !d.acceleration.allFinite() ||
      !d.angular_momentum_rate.allFinite() || !std::isfinite(gravity) || gravity <= 0 ||
      !std::isfinite(min_force) || min_force <= 0) {
    out.error = "invalid dynamics, horizontal support or gravity policy"; return out;
  }
  out.required_force = d.mass * (d.acceleration + Eigen::Vector3d(0, 0, gravity));
  const Eigen::Vector3d origin(0, 0, s.plane_z);
  out.required_moment = d.angular_momentum_rate + (d.com-origin).cross(out.required_force);
  if (!out.required_force.allFinite() || !out.required_moment.allFinite() ||
      out.required_force.z() <= min_force) {
    out.error = "insufficient positive normal force; ZMP undefined"; return out;
  }
  out.point = Eigen::Vector3d(-out.required_moment.y()/out.required_force.z(),
                              out.required_moment.x()/out.required_force.z(), s.plane_z);
  out.margin = (s.b-s.A*out.point.head<2>()).minCoeff();
  out.valid = out.point.allFinite() && std::isfinite(out.margin);
  if (!out.valid) out.error = "non-finite ZMP";
  return out;
}
}  // namespace motion_control_pkg::wbc
