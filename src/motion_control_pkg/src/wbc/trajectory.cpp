#include "motion_control_pkg/wbc/trajectory.hpp"
#include "motion_control_pkg/wbc/zmp.hpp"
#include "motion_control_pkg/motion/jerk_limited_retiming.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_control_pkg::wbc {
CubicReference::CubicReference(std::vector<std::vector<double>> q, double dt)
  : q_(std::move(q)), dt_(dt) {
  if (q_.size()<2 || q_.size()>50000 || q_[0].empty() || !std::isfinite(dt) || dt<=0)
    throw std::invalid_argument("invalid cubic dimensions or period");
  const auto n=q_.size(), dof=q_[0].size();
  for (const auto& row:q_) {
    if (row.size()!=dof || !std::all_of(row.begin(),row.end(),[](double x){return std::isfinite(x);}))
      throw std::invalid_argument("invalid cubic position knot");
  }
  second_.assign(n,std::vector<double>(dof,0));
  // Thomas solve: M[i-1] + 4 M[i] + M[i+1] = 6 delta2(q)/dt^2,
  // with natural boundaries M[0]=M[n-1]=0.
  std::vector<double> diagonal(n,4);
  for (std::size_t i=1;i+1<n;++i) {
    if (i>1) diagonal[i]-=1/diagonal[i-1];
    for (std::size_t j=0;j<dof;++j) {
      second_[i][j]=6*(q_[i+1][j]-2*q_[i][j]+q_[i-1][j])/(dt*dt);
      if (i>1) second_[i][j]-=second_[i-1][j]/diagonal[i-1];
    }
  }
  for (std::size_t i=n-2;i>0;--i)
    for (std::size_t j=0;j<dof;++j) second_[i][j]=(second_[i][j]-second_[i+1][j])/diagonal[i];
}
ReferenceSample CubicReference::at(double t) const {
  if (!std::isfinite(t)) throw std::invalid_argument("nonfinite reference time");
  t=std::clamp(t,0.0,duration());
  const auto i=std::min(static_cast<std::size_t>(t/dt_),q_.size()-2);
  const double u=(t-i*dt_)/dt_, w=1-u;
  ReferenceSample out;
  for (std::size_t j=0;j<q_[0].size();++j) {
    const double m0=second_[i][j], m1=second_[i+1][j];
    out.positions.push_back(w*q_[i][j]+u*q_[i+1][j]+dt_*dt_/6*((w*w*w-w)*m0+(u*u*u-u)*m1));
    out.velocities.push_back((q_[i+1][j]-q_[i][j])/dt_+dt_/6*((1-3*w*w)*m0+(3*u*u-1)*m1));
    out.accelerations.push_back(w*m0+u*m1);
  }
  return out;
}
std::vector<double> CubicReference::knots() const {
  std::vector<double> out; out.reserve(q_.size());
  for (std::size_t i=0;i<q_.size();++i) out.push_back(i*dt_);
  return out;
}

ReferenceResult prepareReference(const StaticBalance& balance, const JointSnapshot& start,
  const std::vector<std::vector<double>>& positions, const std::vector<double>& velocity,
  const std::vector<double>& acceleration, const std::vector<double>& jerk,
  const ReferenceOptions& options,
  const std::function<bool(const std::vector<double>&, std::string&)>& geometry_check) {
  ReferenceResult out;
  auto fail=[&](const std::string& error) {out.valid=false; out.error=error; out.samples.clear(); out.evaluate={}; return out;};
  try {
    if (!std::isfinite(options.maximum_duration) || options.maximum_duration<=0 ||
        !std::isfinite(options.maximum_zmp_scale) || options.maximum_zmp_scale<1 ||
        !std::isfinite(options.period) || options.period<=0 || options.maximum_samples<2 ||
        options.maximum_samples>50000) return fail("invalid dynamic reference budget");
    const auto& model=balance.model(); const auto& specs=model.joints(); const auto n=specs.size();
    if (start.names.size()!=n || !start.position_valid || !start.velocity_valid || !start.assumed.empty() ||
        positions.size()<2 || positions.size()>options.maximum_samples || velocity.size()!=n ||
        acceleration.size()!=n || jerk.size()!=n) return fail("dynamic reference requires complete explicit q/v and limits");
    model.dynamics(start,std::vector<double>(n,0));
    for (std::size_t j=0;j<n;++j) {
      if (start.names[j]!=specs[j].name || std::abs(start.velocities[j])>.01)
        return fail("dynamic reference requires model joint order and resting start");
      if (!std::isfinite(velocity[j]) || velocity[j]<=0 || !std::isfinite(acceleration[j]) || acceleration[j]<=0 ||
          !std::isfinite(jerk[j]) || jerk[j]<=0) return fail("invalid dynamic reference v/a/jerk limits");
      if (!specs[j].locked && velocity[j]>specs[j].velocity+1e-12)
        return fail("dynamic reference velocity limit exceeds URDF");
    }
    for (const auto& row:positions) {
      if (row.size()!=n) return fail("dynamic reference full q dimension mismatch");
      for (std::size_t j=0;j<n;++j) {
        if (!std::isfinite(row[j]) || (specs[j].locked && std::abs(row[j]-specs[j].lower)>1e-12))
          return fail("nonfinite or changed URDF-locked position");
        // No steering command/rolling support is modeled in this stage.
        if (std::find(balance.options().unknown_steering.begin(),balance.options().unknown_steering.end(),specs[j].name)
            !=balance.options().unknown_steering.end() && std::abs(row[j]-start.positions[j])>1e-12)
          return fail("dynamic reference cannot move steering joints");
      }
    }
    for (std::size_t j=0;j<n;++j) if (std::abs(positions.front()[j]-start.positions[j])>1e-10)
      return fail("dynamic reference does not start at supplied state");
    auto curve=std::make_shared<CubicReference>(positions,options.period);
    const auto evaluate=[curve](double time,int order) {
      const auto s=curve->at(time); return order==0?s.positions:order==1?s.velocities:s.accelerations;
    };
    const auto base=motion::retimeCubicTrajectory(curve->knots(),evaluate,options.period,velocity,acceleration,jerk,options.maximum_duration);
    if (!base.valid) return fail(base.error);
    const motion::retiming_detail::Clock clock{base.source_duration,base.endpoint_window};
    for (int attempt=0;attempt<20;++attempt) {
      const double scale=std::pow(1.5,attempt);
      out.duration=std::ceil(base.duration*scale/options.period)*options.period;
      out.zmp_time_scale=out.duration/base.duration;
      if (out.zmp_time_scale>options.maximum_zmp_scale+1e-12 || out.duration>options.maximum_duration ||
          out.duration/options.period+1>options.maximum_samples) return fail("dynamic reference retiming budget exhausted");
      const double time_scale=out.duration/clock.duration();
      out.evaluate=[curve,clock,time_scale](double t) {
        const auto c=clock.at(std::clamp(t,0.0,clock.duration()*time_scale)/time_scale);
        auto s=curve->at(c[0]);
        for (std::size_t j=0;j<s.positions.size();++j) {
          s.accelerations[j]=(s.accelerations[j]*c[1]*c[1]+s.velocities[j]*c[2])/(time_scale*time_scale);
          s.velocities[j]*=c[1]/time_scale;
        }
        return s;
      };
      out.minimum_zmp_margin=std::numeric_limits<double>::infinity(); out.checked_samples=0; out.samples.clear();
      bool dynamic_ok=true;
      const auto steps=static_cast<std::size_t>(std::llround(out.duration/options.period));
      for (std::size_t i=0;i<=2*steps;++i) {
        const auto s=out.evaluate(i*options.period/2);
        auto state=start; state.positions=s.positions; state.velocities=s.velocities;
        const auto d=model.dynamics(state,s.accelerations);
        if (balance.evaluate(state).margin<0) return fail("reference geometry violates static COM support");
        if (geometry_check && !geometry_check(s.positions,out.error)) return fail(out.error);
        const auto z=predictZmp(d,balance.region());
        if (!z.valid || z.margin<0) dynamic_ok=false;
        if (z.valid) out.minimum_zmp_margin=std::min(out.minimum_zmp_margin,z.margin);
        ++out.checked_samples;
        if (i%2==0) out.samples.push_back(s);
      }
      if (dynamic_ok) {out.valid=true; out.error.clear(); return out;}
    }
    return fail("dynamic reference iteration budget exhausted");
  } catch (const std::exception& error) {return fail(error.what());}
}
}  // namespace motion_control_pkg::wbc
