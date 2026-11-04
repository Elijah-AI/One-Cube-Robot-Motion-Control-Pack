#pragma once

#include "motion_control_pkg/motion/tcp_level_constraint.hpp"
#include "motion_control_pkg/execution/motion_safety.hpp"

namespace motion_control_pkg::motion {

// Adapt the existing active-joint path layout without changing solver indices.
// Scope owns measured monitoring until the action has finished settling/stopping.
class TcpLevelPlan {
public:
  using Q=TcpLevelConstraint::Configuration;
  using Path=TcpLevelConstraint::Path;
  TcpLevelPlan(bool enabled, const pinocchio::Model& model, const Q& start,
               bool left, bool right, bool legs, const Eigen::Isometry3d& lt,
               const Eigen::Isometry3d& rt, double tolerance)
    : start_(start), mask_((left?1U:0U)|(right?2U:0U)|4U) {
    if(!enabled) return;
    constraint_=std::make_shared<TcpLevelConstraint>(model,start,left,right,lt,rt,tolerance);
    std::vector<std::string> names;
    for(bool l:{true,false}) {
      if((l&&!left)||(!l&&!right)) continue;
      for(int j=1;j<=7;++j) names.push_back("Joint"+std::to_string(j)+(l?"_L":"_R"));
    }
    if(legs) for(const char* name:{"leg_joint1","leg_joint2","leg_joint3","body_joint"}) names.emplace_back(name);
    for(const auto& name:names) {
      if(!model.existJointName(name)) {mapping_error_="missing TCP path joint "+name;break;}
      indices_.push_back(model.joints[model.getJointId(name)].idx_q());
    }
  }
  ~TcpLevelPlan() {
    if(safety_ && monitor_) safety_->removeStateConstraint(monitor_);
  }
  TcpLevelPlan(const TcpLevelPlan&)=delete;
  bool initialize(std::string& error) {
    if(!mapping_error_.empty()) {error=mapping_error_;return false;}
    return !constraint_ || constraint_->initialize(error);
  }
  bool target(bool left,const Eigen::Isometry3d& root_tcp,std::string& error) const {
    return !constraint_ || constraint_->checkTarget(left,root_tcp,error);
  }
  bool project(Path& active,std::string& error,const Path& pinned={}) {
    if(!constraint_) return true;
    Path full, full_pinned;
    if(!expand(active,full,error)||!expand(pinned,full_pinned,error) ||
       !constraint_->projectPath(full,full_pinned,error)) return false;
    Path result;
    for(const auto& q:full) {
      Q sample;for(const auto index:indices_) sample.push_back(q[index]);
      result.push_back(std::move(sample));
    }
    active=std::move(result);return true;
  }
  bool validate(const Path& active,std::string& error) {
    if(!constraint_) return true;
    Path full;
    return expand(active,full,error) && constraint_->validatePath(full,error);
  }
  bool startMonitoring(std::string& error) {
    if(!constraint_) return true;
    if(monitor_) {error="TCP level monitor already registered";return false;}
    safety_=execution::motionSafety();
    if(!safety_) {error="TCP level execution requires measured MotionSafety";return false;}
    monitor_=safety_->addStateConstraint(mask_,
      [constraint=constraint_](const auto& names,const auto& q,std::string& e) {
        return constraint->checkMeasured(names,q,e);
      },error);
    return monitor_!=0;
  }
private:
  bool expand(const Path& active,Path& full,std::string& error) const {
    for(const auto& q:active) {
      if(q.size()!=indices_.size()) {error="TCP level active joint layout mismatch";return false;}
      Q merged=start_;
      for(std::size_t i=0;i<q.size();++i) merged[indices_[i]]=q[i];
      full.push_back(std::move(merged));
    }
    return true;
  }
  Q start_;
  std::vector<int> indices_;
  std::string mapping_error_;
  std::shared_ptr<TcpLevelConstraint> constraint_;
  execution::SafetyMask mask_;
  std::shared_ptr<execution::MotionSafety> safety_;
  std::uint64_t monitor_{};
};
}  // namespace motion_control_pkg::motion
