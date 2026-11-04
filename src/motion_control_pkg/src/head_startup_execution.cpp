#include "motion_control_pkg/miracle_head_module.hpp"
#include "motion_control_pkg/execution/ethercat_trajectory.hpp"
#include "motion_control_pkg/motion/head_startup_plan.hpp"
#include <thread>

namespace miracle {
bool MiracleHeadModule::runStartupHome(
    const motion_control_pkg::execution::HeadStartupHomePolicy& policy,
    const std::shared_ptr<motion_control_pkg::execution::MotionSafety>& safety,
    std::string& error) {
  using namespace motion_control_pkg::execution;
  if(!policy.enabled)return true;
  if(!safety) {error="head startup safety monitor unavailable";return false;}
  if(!reserve_body_motion_resource(error))return safety->endHeadStartupHome(false,error);
  struct Release { MiracleHeadModule& module; ~Release(){module.release_body_motion_resource();} } release{*this};
  const auto started=SafetyClock::now();
  const auto after=[&](double seconds) {return started+std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(seconds));};
  MotionSafetyScope scope(kHeadSafety);
  struct RecoveryRelease {
    EthercatBusManager* bus{};
    ~RecoveryRelease(){if(bus)bus->endHeadPitchStartupRecovery();}
  } recovery;
  bool success=false;
  try {
    std::vector<std::string> names;std::vector<double> measured;
    while(!safety->beginHeadStartupHome(names,measured,error)) {
      if(!rclcpp::ok() || safety->latched() || SafetyClock::now()>=after(policy.feedback_wait))
        throw std::runtime_error("head startup preflight: "+error);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::vector<double> full;
    if(!m_galgo->getFullJointState(full))throw std::runtime_error("head startup full state unavailable");
    const auto& model=m_galgo->getModel();
    for(std::size_t j=0;j<names.size();++j) {
      const auto idx=model.joints[model.getJointId(names[j])].idx_q();full.at(idx)=measured[j];
    }
    std::vector<SafetyJoint> joints;
    std::vector<double> start;
    for(const auto& name:{"head_yaw_joint","head_pitch_joint"}) {
      const auto& joint=model.joints[model.getJointId(name)];const auto idx=joint.idx_q();
      joints.push_back({name,model.lowerPositionLimit[idx],model.upperPositionLimit[idx],model.velocityLimit[joint.idx_v()]});
      start.push_back(full.at(idx));
    }
    std::vector<std::vector<double>> path;
    if(!motion_control_pkg::motion::planHeadStartupHome(start,joints,policy,path,error) ||
        !m_galgo->validateHeadStartupPath(full,path,policy.lower,policy.upper,error))
      throw std::runtime_error(error);
    // The SDK must preserve the measured below-URDF hold during enable;
    // otherwise its ordinary clamp creates a step to the URDF lower limit.
    // Arm the pitch-only exception after full path/collision validation.
    if(start[1] < joints[1].lower) {
      if(!m_bus_manager || !m_bus_manager->beginHeadPitchStartupRecovery(
          policy.lower[1],start[1],policy.timeout,error))
        throw std::runtime_error(error);
      recovery.bus=m_bus_manager.get();
      RCLCPP_INFO(rclcpp::get_logger("head_startup_home"),
          "Scoped pitch startup recovery: measured=%.6f lower=%.6f upper_URDF=%.6f rad",
          start[1],policy.lower[1],joints[1].upper);
    }
    // Preflight and collision validation precede the one-shot enable. Hold the
    // measured pose until fresh PDOs confirm operation-enabled; only then may
    // the trajectory's enabled-feedback check and first setpoint run.
    if(!m_bus_manager || !m_bus_manager->enableMotors(m_jointID))
      throw std::runtime_error("head startup initial motor enable failed; see EtherCAT enable diagnostic");
    // Preserve the preflight pose of all other chains throughout recovery,
    // including planning before the first head command is dispatched.
    RCLCPP_INFO(rclcpp::get_logger("head_startup_home"),
        "Startup home [%.6f, %.6f] -> [0,0], command_max_speed=[%.4f, %.4f] rad/s, samples=%zu",
        start[0],start[1],std::min(joints[0].velocity,policy.velocity),
        std::min(joints[1].velocity,policy.velocity),path.size());
    auto completion=precision_policy_;
    completion.completion.position_tolerance=std::min(.002,completion.completion.position_tolerance);
    completion.completion.stable_seconds=std::max(.2,completion.completion.stable_seconds);
    completion.completion.progress_rad=std::min(completion.completion.progress_rad,completion.completion.position_tolerance/10);
    double final_error=0;
    success=executeEthercatTrajectory(path,m_bus_manager,m_jointID,
        [&](const std::vector<double>& q){return setMulJointPos(q,m_jointID);},
        [&](std::string& reason){if(!rclcpp::ok()){reason="shutdown during head startup home";return false;}return true;},
        [](float){},after(policy.timeout),final_error,error,completion);
  } catch(const std::exception& e) {error=e.what();}
  success=scope.finish(success,error);
  return safety->endHeadStartupHome(success,error);
}
}  // namespace miracle
