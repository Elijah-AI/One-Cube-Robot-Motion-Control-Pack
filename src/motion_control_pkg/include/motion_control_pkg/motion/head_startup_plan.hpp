#pragma once
#include "motion_control_pkg/execution/head_startup_home.hpp"
#include "motion_control_pkg/execution/control_period.hpp"
#include "motion_control_pkg/motion/trajectory_derivative_limits.hpp"
#include <ruckig/ruckig.hpp>

namespace motion_control_pkg::motion {
// Both axes use the requested command speed directly. The URDF measured-speed
// limit is separate (0.2 rad/s on this robot, versus 0.1 rad/s for homing).
inline bool planHeadStartupHome(const std::vector<double>& start,
    const std::vector<execution::SafetyJoint>& joints,
    const execution::HeadStartupHomePolicy& policy,
    std::vector<std::vector<double>>& path,std::string& error) {
  policy.validate(joints);
  if(start.size()!=2) {error="invalid head startup start dimensions";return false;}
  ruckig::InputParameter<2> input;
  for(std::size_t j=0;j<2;++j) {
    if(!std::isfinite(start[j]) || start[j]<policy.lower[j] || start[j]>policy.upper[j]) {
      error="measured head start outside configured recovery envelope";return false;
    }
    input.current_position[j]=start[j]; input.target_position[j]=0;
    input.max_velocity[j]=std::min(joints[j].velocity,policy.velocity);
    input.max_acceleration[j]=policy.acceleration;input.max_jerk[j]=policy.jerk;
  }
  ruckig::Ruckig<2> generator;ruckig::Trajectory<2> trajectory;
  if(generator.calculate(input,trajectory)<0) {error="head startup Ruckig planning failed";return false;}
  const double duration=trajectory.get_duration(), dt=execution::kDualArmControlPeriodSeconds;
  if(!std::isfinite(duration) || duration<0 || duration+.2>=policy.timeout) {
    error="head startup trajectory exceeds configured timeout";return false;
  }
  std::vector<std::vector<double>> result;
  const std::size_t count=std::ceil(duration/dt);
  std::array<double,2> q,v,a;
  for(std::size_t i=0;i<=count;++i) {
    trajectory.at_time(std::min(i*dt,duration),q,v,a);
    result.emplace_back(q.begin(),q.end());
  }
  result.front()=start;result.back()={0,0};
  auto previous=start;
  for(const auto& point:result) {
    if(!execution::headHomeCommandInward(start,previous,point)) {error="head startup trajectory leaves inward corridor";return false;}
    previous=point;
  }
  if(!validateCommandDerivatives(result,dt,{input.max_velocity.begin(),input.max_velocity.end()},
      {input.max_acceleration.begin(),input.max_acceleration.end()},
      {input.max_jerk.begin(),input.max_jerk.end()},error))return false;
  path=std::move(result);error.clear();return true;
}
}  // namespace motion_control_pkg::motion
