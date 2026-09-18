#pragma once
#include "motion_control_pkg/wbc/model.hpp"

namespace motion_control_pkg::wbc {
struct StaticBalanceOptions {
  std::vector<std::string> support_links;
  std::vector<std::string> unknown_steering;
  double inset{0.02};
  double com_weight{0.01};
};
struct BalanceEvaluation {
  Eigen::Vector3d com;
  Eigen::MatrixXd jacobian;
  double margin{};
};
class StaticBalance {
public:
  StaticBalance(const std::string& urdf_path, StaticBalanceOptions options,
                const std::vector<std::string>& package_dirs = {});
  const CentroidalModel& model() const { return model_; }
  const SupportResult& region() const { return region_; }
  const SteeringBounds& bounds() const { return bounds_; }
  const StaticBalanceOptions& options() const { return options_; }
  StatePolicy statePolicy() const;
  BalanceEvaluation evaluate(JointSnapshot state) const;
private:
  StaticBalanceOptions options_;
  CentroidalModel model_;
  SupportModel support_;
  SteeringBounds bounds_;
  SupportResult region_;
};
}  // namespace motion_control_pkg::wbc
