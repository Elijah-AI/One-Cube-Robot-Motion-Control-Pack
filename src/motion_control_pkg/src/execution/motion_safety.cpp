#include "motion_control_pkg/execution/shutdown_request.hpp"
#include "motion_control_pkg/execution/motion_safety.hpp"
#include "motion_control_pkg/execution/head_startup_home.hpp"
#include "motion_control_pkg/wbc/tracking.hpp"
#include "motion_control_pkg/sdk/drive_control.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <optional>
#include <map>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace motion_control_pkg::execution {
namespace {
double seconds(SafetyClock::time_point t) { return std::chrono::duration<double>(t.time_since_epoch()).count(); }
std::mutex installed_mutex;
std::weak_ptr<MotionSafety> installed;
unsigned index(SafetyMask mask) {
  for (unsigned i=0;i<4;++i) if (mask == (1U<<i)) return i;
  throw std::invalid_argument("invalid motion safety channel");
}
}
struct MotionSafety::Impl {
  struct Channel {
    std::vector<SafetyJoint> joints;
    Read read;
    std::unique_ptr<wbc::PositionTrackingGuard> tracking;
    SafetySample sample;
    SafetyClock::time_point recovery_after{};
    bool healthy{false}, active{false};
    std::string error{"waiting for measured feedback"};
    std::optional<SafetyClock::time_point> stable_since;
    unsigned stable_frames{};
    double peak{}, measured_speed{};
    std::uint64_t commands{};
    SafetyClock::time_point last_command{};
    double max_command_gap_ms{};
    std::vector<double> held_position;
  };
  explicit Impl(SafetyPolicy p) : policy(p) {}
  mutable std::mutex mutex;
  std::condition_variable changed;
  std::array<Channel,4> channels;
  SafetyPolicy policy;
  SafetyMask configured{}, affected{};
  bool fault{false}, confirmed{false}, tracking_stop{false};
  std::uint64_t epoch{1};
  unsigned executions{};
  std::string cause, last_tracking_stop;
  SafetyClock::time_point fault_time{};
  std::atomic<bool> quit{false};
  std::thread worker;
  double max_reader_ms{}, max_transaction_ms{};
  struct Constraint {SafetyMask required; StateConstraint check;};
  std::map<std::uint64_t,Constraint> constraints;
  std::uint64_t next_constraint{1};
  enum class Home { Disabled, Waiting, Running, Done, Failed };
  Home home{Home::Disabled};
  HeadStartupHomePolicy home_policy;
  std::thread::id home_owner;
  SafetyClock::time_point home_deadline{};
  std::vector<double> home_initial, home_command, home_best;
  std::array<bool,2> home_inside{{false,false}};
  bool homeException() const { return home==Home::Waiting || home==Home::Running; }
  bool homeCaller() const { return home==Home::Running && home_owner==std::this_thread::get_id(); }
  bool trackingEnabled(unsigned channel) const {
    return channel<2 ? policy.arm_error_enabled : policy.body_error_enabled;
  }
  SafetyJoint effectiveJoint(unsigned channel, std::size_t j) const {
    auto joint=channels[channel].joints[j];
    if (channel==3 && homeException()) {
      joint.lower=home_policy.lower[j]; joint.upper=home_policy.upper[j];
      // Homing commands have their own lower speed cap. Keep the URDF measured
      // speed limit so normal tracking variation does not trip at command speed.
    }
    return joint;
  }

  bool checkConstraint(const Constraint& constraint, SafetyClock::time_point now, std::string& error) {
    std::vector<std::string> names;
    std::vector<double> positions;
    auto oldest=now, newest=SafetyClock::time_point{};
    for(unsigned i=0;i<4;++i) if(constraint.required&(1U<<i)) {
      const auto& c=channels[i];
      if(!fresh(c,now)) {error="TCP constraint requires fresh ancestor feedback";return false;}
      oldest=std::min(oldest,c.sample.acquired);newest=std::max(newest,c.sample.acquired);
      for(const auto& j:c.joints) names.push_back(j.name);
      positions.insert(positions.end(),c.sample.position.begin(),c.sample.position.end());
    }
    if(newest-oldest>std::chrono::milliseconds(20)) {
      error="TCP constraint feedback skew exceeds 20 ms";return false;
    }
    try {return constraint.check(names,positions,error);}
    catch(const std::exception& e) {error=e.what();return false;}
    catch(...) {error="measured state constraint exception";return false;}
  }

  bool fresh(const Channel& c, SafetyClock::time_point now) const {
    return c.healthy && c.sample.acquired != SafetyClock::time_point{} &&
      c.sample.acquired <= now && seconds(now)-seconds(c.sample.acquired) < policy.max_feedback_age;
  }
  bool stationary(const Channel& c, SafetyClock::time_point now) const {
    return fresh(c,now) && c.stable_since && c.stable_frames >= 5 &&
      c.sample.acquired-*c.stable_since >= std::chrono::milliseconds(100);
  }
  void trip(const std::string& reason, SafetyClock::time_point now, bool recoverable_tracking=false) {
    // A real fault or explicit stop must override a recoverable tracking stop.
    if (fault && (!tracking_stop || recoverable_tracking)) return;
    fault = true; tracking_stop = recoverable_tracking;
    confirmed = false; cause = reason; fault_time = now;
    affected = 0;
    for (unsigned i=0;i<4;++i) {
      auto& c=channels[i];
      if (c.active) affected |= 1U<<i;
      c.stable_since.reset(); c.stable_frames=0;
    }
    // Even an idle stop needs fresh measurements before it can be reset.
    affected = configured;
    changed.notify_all();
  }
  void clearTracking(Channel& c) {
    std::vector<wbc::JointSpec> joints;
    for (std::size_t k=0;k<c.joints.size();++k) {
      auto j=effectiveJoint(static_cast<unsigned>(&c-channels.data()),k);
      if(&c==&channels[3] && homeException()) {j.lower-=1e-6;j.upper+=1e-6;}
      joints.push_back({j.name,j.lower,j.upper,j.velocity,0,false});
    }
    const bool arm=&c<&channels[2];
    c.tracking=std::make_unique<wbc::PositionTrackingGuard>(std::move(joints),
        wbc::TrackingPolicy{arm?policy.arm_error_limit_rad:policy.tracking_error,
            arm?policy.arm_error_hard_limit_rad:policy.hard_error,
            policy.tracking_dwell,policy.max_feedback_age,trackingEnabled(static_cast<unsigned>(&c-channels.data()))});
    c.peak=0;
  }
  void releaseTrackingStop(SafetyClock::time_point now) {
    if (!fault || !tracking_stop || !confirmed || executions ||
        std::any_of(channels.begin(),channels.end(),[](const auto& c){return c.active;})) return;
    for (unsigned i=0;i<4;++i)
      if ((configured&(1U<<i)) && !stationary(channels[i],now)) return;
    last_tracking_stop=cause;
    for (auto& c:channels) if(c.read) {clearTracking(c);c.held_position=c.sample.position;}
    fault=false;tracking_stop=false;confirmed=false;affected=0;cause.clear();++epoch;
    changed.notify_all();
  }
};

MotionSafety::MotionSafety(SafetyPolicy policy) : impl_(std::make_unique<Impl>(policy)) {
  for (double x:{policy.tracking_error,policy.hard_error,policy.tracking_dwell,policy.max_feedback_age,policy.stop_timeout,
      policy.arm_error_limit_rad,policy.arm_error_hard_limit_rad})
    if (!std::isfinite(x)||x<=0) throw std::invalid_argument("invalid motion safety policy");
  if (policy.hard_error<policy.tracking_error || policy.arm_error_hard_limit_rad<policy.arm_error_limit_rad)
    throw std::invalid_argument("invalid tracking thresholds");
}
MotionSafety::~MotionSafety() { stop(); }
void MotionSafety::addChannel(SafetyMask mask, std::vector<SafetyJoint> joints, Read read) {
  auto& p=*impl_; std::lock_guard<std::mutex> lock(p.mutex);
  if (p.worker.joinable() || joints.empty() || !read || (p.configured&mask))
    throw std::invalid_argument("invalid motion safety registration");
  for (const auto& j:joints) if (j.name.empty() || !std::isfinite(j.lower) || !std::isfinite(j.upper) ||
      j.lower>j.upper || !std::isfinite(j.velocity) || j.velocity<=0) throw std::invalid_argument("invalid safety joint bounds");
  auto& c=p.channels[index(mask)]; c.joints=std::move(joints); c.read=std::move(read);
  p.clearTracking(c); p.configured|=mask;
}
void MotionSafety::start() {
  auto& p=*impl_; std::lock_guard<std::mutex> lock(p.mutex);
  if (p.worker.joinable()) return;
  if (!p.configured) throw std::logic_error("motion safety has no channels");
  p.quit=false;
  p.worker=std::thread([this]{
    while (!impl_->quit) {
      const auto next=SafetyClock::now()+std::chrono::milliseconds(10);
      poll(); std::this_thread::sleep_until(next);
    }
  });
}
void MotionSafety::configureHeadStartupHome(const HeadStartupHomePolicy& policy) {
  auto& p=*impl_; std::lock_guard<std::mutex> lock(p.mutex);
  if (p.worker.joinable() || !(p.configured&kHeadSafety) || p.home!=Impl::Home::Disabled)
    throw std::logic_error("head startup home must be configured once before monitoring starts");
  policy.validate(p.channels[3].joints);
  p.home_policy=policy;
  if (policy.enabled) {
    p.home=Impl::Home::Waiting;
    p.home_deadline=SafetyClock::now()+std::chrono::duration_cast<SafetyClock::duration>(
        std::chrono::duration<double>(policy.timeout));
    p.clearTracking(p.channels[3]);
  }
}
bool MotionSafety::beginHeadStartupHome(std::vector<std::string>& names,
    std::vector<double>& positions,std::string& error) {
  auto& p=*impl_;std::lock_guard<std::mutex> lock(p.mutex);
  if (p.fault || p.home!=Impl::Home::Waiting) {error="head startup home unavailable or fault latched";return false;}
  const auto now=SafetyClock::now();
  for (unsigned i=0;i<4;++i) if (p.configured&(1U<<i)) {
    if (p.channels[i].active || !p.stationary(p.channels[i],now)) {
      error="startup home requires fresh stationary feedback: channel="+std::to_string(i)+" "+p.channels[i].error;return false;
    }
  }
  names.clear();positions.clear();
  for (auto& c:p.channels) if(c.read) {
    for(const auto& j:c.joints)names.push_back(j.name);
    positions.insert(positions.end(),c.sample.position.begin(),c.sample.position.end());
    c.held_position=c.sample.position;
  }
  auto& head=p.channels[3];
  p.home_initial=p.home_command=p.home_best=head.sample.position;
  for(std::size_t j=0;j<2;++j) p.home_inside[j]=head.sample.position[j]>=head.joints[j].lower &&
      head.sample.position[j]<=head.joints[j].upper;
  p.home=Impl::Home::Running;p.home_owner=std::this_thread::get_id();error.clear();return true;
}
bool MotionSafety::endHeadStartupHome(bool success,std::string& error) {
  auto& p=*impl_;std::lock_guard<std::mutex> lock(p.mutex);
  if (p.home==Impl::Home::Disabled || p.home==Impl::Home::Done || p.home==Impl::Home::Failed) {
    error="head startup home already closed or disabled";return false;
  }
  if (p.home==Impl::Home::Running && !p.homeCaller()) {error="head startup home owner mismatch";return false;}
  const auto& head=p.channels[3];
  success=success && p.homeCaller() && !p.fault && p.stationary(head,SafetyClock::now()) &&
      head.sample.position.size()==2 && head.stable_since &&
      head.sample.acquired-*head.stable_since>=std::chrono::milliseconds(200);
  if(success) for(std::size_t j=0;j<2;++j)
    success=success && std::abs(head.sample.position[j])<=.002 &&
        head.sample.position[j]>=head.joints[j].lower && head.sample.position[j]<=head.joints[j].upper;
  if (!success) {
    if(error.empty())error="head startup home did not reach measured zero";
    p.trip(error,SafetyClock::now());
  }
  p.home=success?Impl::Home::Done:Impl::Home::Failed;
  p.home_owner={};p.clearTracking(p.channels[3]);
  return success;
}
void MotionSafety::stop() { impl_->quit=true; if (impl_->worker.joinable()) impl_->worker.join(); }
void MotionSafety::poll(SafetyClock::time_point now) {
  auto& p=*impl_;
  for (unsigned i=0;i<4;++i) {
    auto& c=p.channels[i]; if (!c.read) continue;
    SafetySample sample; std::string error; bool valid=false;
    const auto read_started=SafetyClock::now();
    try { valid=c.read(sample,error); }
    catch (const std::exception& e) { error=e.what(); }
    catch (...) { error="feedback reader exception"; }
    // Readers can block briefly. Never use a clock sampled before acquisition.
    const auto observed = std::max(now, SafetyClock::now());
    std::lock_guard<std::mutex> lock(p.mutex);
    p.max_reader_ms=std::max(p.max_reader_ms,1000*(seconds(observed)-seconds(read_started)));
    const auto old=c.sample;
    valid=valid && sample.position.size()==c.joints.size() && sample.velocity.size()==c.joints.size() &&
      sample.acquired!=SafetyClock::time_point{} && sample.acquired>c.recovery_after && sample.acquired<=observed &&
      seconds(observed)-seconds(sample.acquired)<p.policy.max_feedback_age &&
      (sample.sequence>0 || sample.wraps_at_million);
    if (sample.wraps_at_million && sample.sequence>1000000) valid=false;
    if (old.acquired!=SafetyClock::time_point{} &&
        (sample.acquired<old.acquired || (sample.sequence<old.sequence &&
         !(sample.wraps_at_million && old.sequence>900000 && sample.sequence<100000)))) valid=false;
    const bool new_frame=old.acquired==SafetyClock::time_point{} ||
        (sample.sequence!=old.sequence && sample.acquired>old.acquired);
    // An unchanged device frame cannot acquire a new host timestamp.
    if (!new_frame && old.acquired!=SafetyClock::time_point{} && sample.acquired!=old.acquired) valid=false;
    if (!new_frame && old.acquired!=SafetyClock::time_point{} &&
        (sample.position!=old.position || sample.velocity!=old.velocity)) valid=false;
    double speed=0;
    if (valid) for (std::size_t j=0;j<c.joints.size();++j) {
      const auto joint=p.effectiveJoint(i,j); const double q=sample.position[j], v=sample.velocity[j];
      if (!std::isfinite(q)||!std::isfinite(v)||q<joint.lower-1e-6||q>joint.upper+1e-6||std::abs(v)>joint.velocity+1e-6) {
        valid=false;
        std::ostringstream detail;
        detail.precision(9);
        detail << (i==3 && p.homeException()?"head recovery feedback violates bounds/velocity: ":
            "measured state violates URDF: ") << joint.name
            << " position=" << q << " velocity=" << v
            << " lower=" << joint.lower << " upper=" << joint.upper
            << " max_velocity=" << joint.velocity;
        error=detail.str(); break;
      }
      if(i==3 && p.home==Impl::Home::Running) {
        const auto& normal=c.joints[j];
        if (std::abs(q)>std::abs(p.home_best[j])+.002 ||
            (p.home_inside[j] && (q<normal.lower-1e-6 || q>normal.upper+1e-6))) {
          valid=false;error="head startup home moved away from zero or left normal range";break;
        }
        if(std::abs(q)<std::abs(p.home_best[j]))p.home_best[j]=q;
        if(q>=normal.lower+.002 && q<=normal.upper-.002)p.home_inside[j]=true;
      }
      speed=std::max(speed,std::abs(v));
      if (new_frame && old.acquired!=SafetyClock::time_point{})
        speed=std::max(speed,std::abs(q-old.position[j])/(seconds(sample.acquired)-seconds(old.acquired)));
    }
    c.healthy=valid;
    if (!valid) {
      c.error=error.empty()?"invalid, stale or non-advancing device feedback":error;
      c.stable_since.reset(); c.stable_frames=0;
      if (p.tracking_stop || p.home==Impl::Home::Running || std::any_of(p.channels.begin(),p.channels.end(),[](const auto& x){return x.active;}))
        p.trip("channel="+std::to_string(i)+": "+c.error,observed);
      continue;
    }
    c.sample=sample; c.error.clear(); c.measured_speed=speed;
    // Startup homing keeps its independent ancestor-motion guard. Ordinary
    // holding errors use body_error below, with the selected limits and stop policy.
    if (!p.fault && !c.active && i==2 && p.home==Impl::Home::Running &&
        c.held_position.size()==sample.position.size()) {
      for(std::size_t j=0;j<sample.position.size();++j)if(std::abs(sample.position[j]-c.held_position[j])>0.01)
        p.trip("uncommanded joint moved: "+c.joints[j].name,observed);
    }
    if (speed>0.01) { c.stable_since.reset(); c.stable_frames=0; }
    else if (new_frame) {
      if (!c.stable_since) c.stable_since=sample.acquired;
      ++c.stable_frames;
    }
    const bool monitor_hold=p.trackingEnabled(i) &&
        std::any_of(p.channels.begin(),p.channels.end(),[](const auto& x){return x.active;});
    if ((c.active || monitor_hold) && (!p.fault || p.tracking_stop) && !c.tracking->observe(seconds(sample.acquired),seconds(observed),
        sample.position,sample.velocity,sample.controller_command))
      p.trip(std::string(i<2?"arm_error: ":"body_error: ")+c.tracking->error()+" channel="+std::to_string(i)+
          " feedback_sequence="+std::to_string(sample.sequence),observed,
          c.tracking->failure()==wbc::PositionTrackingGuard::Failure::PositionError);
    c.peak=std::max(c.peak,c.tracking->peak());
  }
  std::lock_guard<std::mutex> lock(p.mutex);
  if(p.homeException() && SafetyClock::now()>=p.home_deadline)
    p.trip("head startup home timeout",SafetyClock::now());
  if(!p.fault || p.tracking_stop) for(const auto& item:p.constraints) {
    std::string error;
    if(!p.checkConstraint(item.second,std::max(now,SafetyClock::now()),error)) {
      p.trip("measured path constraint: "+error,SafetyClock::now());break;
    }
  }
  if (p.fault) {
    bool stopped=true;
    for (unsigned i=0;i<4;++i) if (p.affected&(1U<<i)) stopped &= p.stationary(p.channels[i],SafetyClock::now());
    p.confirmed=stopped; p.changed.notify_all();
    p.releaseTrackingStop(SafetyClock::now());
  }
}

std::uint64_t MotionSafety::addStateConstraint(SafetyMask required,StateConstraint check,std::string& error) {
  auto& p=*impl_;std::lock_guard<std::mutex> lock(p.mutex);
  if(p.fault || !check || !required || (required&~p.configured)) {
    error="cannot register measured constraint: fault or missing safety channels";return 0;
  }
  Impl::Constraint constraint{required,std::move(check)};
  if(!p.checkConstraint(constraint,SafetyClock::now(),error)) return 0;
  const auto id=p.next_constraint++;
  p.constraints.emplace(id,std::move(constraint));error.clear();return id;
}
void MotionSafety::removeStateConstraint(std::uint64_t id) {
  std::lock_guard<std::mutex> lock(impl_->mutex);impl_->constraints.erase(id);
}

MotionSafety::CommandPermit::CommandPermit(MotionSafety& owner,SafetyMask mask,std::string& error,bool configuration,
    std::chrono::milliseconds wait,const std::function<bool()>& canceled,bool teaching)
  : owner_(&owner),lock_(owner.impl_->mutex),channels_(mask) {
  auto& p=*owner.impl_; const auto deadline=SafetyClock::now()+wait;
  if(teaching && (!configuration || (mask&~kArmSafety))) {
    error="teaching standstill permit requires arm configuration only";return;
  }
  sdk::DriveFeedbackWindow teach_window(SafetyClock::now(),std::chrono::milliseconds(100),
      kTeachStandstillVelocity,kTeachStandstillDisplacement);
  for (;;) {
    const auto now=SafetyClock::now(); acquired_=now;
    if (shutdownRequested()) {error="orderly shutdown in progress";return;}
    if (canceled && canceled()) {error="control mode change canceled while waiting for standstill";return;}
    if (p.fault) {error=(p.tracking_stop?"position tracking stop in progress: ":"motion fault latched: ")+p.cause; return;}
    if (p.home==Impl::Home::Failed || (p.homeException() &&
        (!p.homeCaller() || mask!=kHeadSafety || configuration))) {
      error="head startup home reserves motion; normal commands unavailable";return;
    }
    if (!mask || (mask&~p.configured)) {error="unregistered motion safety channel"; return;}
    for (unsigned i=0;i<4;++i) if ((p.configured&(1U<<i)) && !p.fresh(p.channels[i],now)) {
      error="related chain feedback unhealthy: channel="+std::to_string(i);return;
    }
    bool waiting=false;
    bool teach_stationary=false;
    if(teaching) {
      std::vector<sdk::DriveAxis> axes;
      for(unsigned i=0;i<4;++i)if(mask&(1U<<i)) {
        const auto& c=p.channels[i];
        for(std::size_t j=0;j<c.sample.position.size();++j)
          axes.push_back({c.sample.position[j],std::max(std::abs(c.sample.velocity[j]),c.measured_speed),
              c.sample.sequence,c.sample.acquired,!c.active,false,!c.active,c.sample.wraps_at_million});
      }
      teach_stationary=teach_window.update(axes,false,false,now);
    }
    for (unsigned i=0;i<4;++i) if (mask&(1U<<i)) {
      auto& c=p.channels[i];
      const bool stationary=teaching?teach_stationary:p.stationary(c,now);
      if (configuration && (c.active || !stationary)) {
        std::ostringstream detail;
        detail << "control mode change requires an idle stationary chain: channel=" << i
            << " active=" << c.active << " stationary_frames=" << c.stable_frames
            << " stationary_ms=" << (c.stable_since ?
                1000*(seconds(c.sample.acquired)-seconds(*c.stable_since)) : 0)
            << " measured_speed_rad_s=" << c.measured_speed;
        if(teaching)detail << " teaching_window=1 max_velocity_rad_s=" << kTeachStandstillVelocity
            << " max_displacement_rad=" << kTeachStandstillDisplacement;
        error=detail.str();
        if(c.active || now>=deadline)return;
        waiting=true;break;
      }
      if (!p.fresh(c,now) || (!c.active && !stationary)) {
        error="motion feedback not qualified: channel="+std::to_string(i)+" "+c.error;
        if (c.active) p.trip(error,now);
        return;
      }
    }
    if(waiting) {p.changed.wait_for(lock_,std::chrono::milliseconds(10));continue;}
    allowed_=true; error.clear();
    return;
  }
}
MotionSafety::CommandPermit MotionSafety::command(SafetyMask mask,std::string& error) { return CommandPermit(*this,mask,error); }
MotionSafety::CommandPermit MotionSafety::configuration(SafetyMask mask,std::string& error) { return CommandPermit(*this,mask,error,true); }
MotionSafety::CommandPermit MotionSafety::configurationWhenStationary(SafetyMask mask,std::string& error,
    const std::function<bool()>& canceled,std::chrono::milliseconds timeout) {
  return CommandPermit(*this,mask,error,true,timeout,canceled);
}
MotionSafety::CommandPermit MotionSafety::teachingConfigurationWhenStationary(SafetyMask mask,std::string& error,
    const std::function<bool()>& canceled,std::chrono::milliseconds timeout) {
  return CommandPermit(*this,mask,error,true,timeout,canceled,true);
}
bool MotionSafety::CommandPermit::validate(SafetyMask channel,const std::vector<double>& q) {
  if (!allowed_ || !(channels_&channel)) return false;
  auto& p=*owner_->impl_;
  const auto& c=p.channels[index(channel)];
  bool valid=q.size()==c.joints.size();
  if (valid && channel==kHeadSafety && p.homeCaller())
    valid=headHomeCommandInward(p.home_initial,p.home_command,q);
  if (valid) for(std::size_t j=0;j<q.size();++j) {
    const auto joint=p.effectiveJoint(index(channel),j);
    const double epsilon=(channel==kHeadSafety && p.homeCaller())?1e-6:1e-8;
    valid &= std::isfinite(q[j]) && q[j]>=joint.lower-epsilon && q[j]<=joint.upper+epsilon;
  }
  if (!valid) failed("joint command violates URDF bounds or dimensions");
  return valid;
}
bool MotionSafety::CommandPermit::committed(SafetyMask channel,const std::vector<double>& q) {
  auto& p=*owner_->impl_;
  if (!allowed_ || p.fault || !(channels_&channel)) return false;
  if (channel==kHeadSafety && p.homeCaller()) p.home_command=q;
  if (!std::any_of(p.channels.begin(),p.channels.end(),[](const auto& x){return x.active;})) {
    for(unsigned i=0;i<4;++i) {
      auto& other=p.channels[i];other.held_position=other.sample.position;
      if (other.read && p.trackingEnabled(i)) {
        p.clearTracking(other);
        if(!other.tracking->command(seconds(SafetyClock::now()),other.held_position)) {
          p.trip(other.tracking->error(),SafetyClock::now());return false;
        }
      }
    }
  }
  auto& c=p.channels[index(channel)];
  if (!c.active) {p.clearTracking(c);c.last_command={};c.max_command_gap_ms=0;}
  const auto now=SafetyClock::now();
  if(c.last_command!=SafetyClock::time_point{})
    c.max_command_gap_ms=std::max(c.max_command_gap_ms,1000*(seconds(now)-seconds(c.last_command)));
  c.last_command=now;++c.commands;
  p.max_transaction_ms=std::max(p.max_transaction_ms,1000*(seconds(now)-seconds(acquired_)));
  c.active=true;
  if (!c.tracking->command(seconds(SafetyClock::now()),q)) {p.trip(c.tracking->error(),SafetyClock::now()); return false;}
  return true;
}
void MotionSafety::CommandPermit::failed(const std::string& reason) {
  if (!allowed_) return;
  auto& p=*owner_->impl_;
  p.max_transaction_ms=std::max(p.max_transaction_ms,1000*(seconds(SafetyClock::now())-seconds(acquired_)));
  for (unsigned i=0;i<4;++i) if (channels_&(1U<<i)) owner_->impl_->channels[i].active=true;
  owner_->impl_->trip(reason,SafetyClock::now());
  allowed_=false;
}
void MotionSafety::requestStop(const std::string& reason) {auto& p=*impl_;std::lock_guard<std::mutex> lock(p.mutex);p.trip(reason,SafetyClock::now());}
bool MotionSafety::setErrorTrackingEnabled(SafetyMask group,bool enabled,double error_rad,
    double hard_error_rad,std::string& detail) {
  if (shutdownRequested()) {detail="orderly shutdown in progress";return false;}
  if (group!=kArmSafety && group!=kBodySafety) {detail="select arm_error or body_error group";return false;}
  auto& p=*impl_;std::lock_guard<std::mutex> lock(p.mutex);
  if (enabled && (!std::isfinite(error_rad) || !std::isfinite(hard_error_rad) ||
      error_rad<=0 || hard_error_rad<error_rad)) {
    detail="supply finite rad limits: 0 < position_error_rad <= hard_error_rad";return false;
  }
  if (p.fault || p.homeException() || p.executions ||
      std::any_of(p.channels.begin(),p.channels.end(),[](const auto& c){return c.active;})) {
    detail="error tracking option requires idle motion resources and no pending stop/fault";return false;
  }
  const bool arm=group==kArmSafety;
  auto& option=arm?p.policy.arm_error_enabled:p.policy.body_error_enabled;
  auto& limit=arm?p.policy.arm_error_limit_rad:p.policy.tracking_error;
  auto& hard=arm?p.policy.arm_error_hard_limit_rad:p.policy.hard_error;
  option=enabled;
  if (enabled) {limit=error_rad;hard=hard_error_rad;}
  for(unsigned i=0;i<4;++i) if((group&(1U<<i)) && p.channels[i].read) p.clearTracking(p.channels[i]);
  std::ostringstream message;
  message<<(arm?"arm_error":"body_error")<<"="<<(enabled?"ON":"OFF")
      <<" position_error_rad="<<limit
      <<" hard_error_rad="<<hard
      <<" dwell_ms="<<1000*p.policy.tracking_dwell
      <<"; position-error stop releases after measured standstill and worker exit; other protection unchanged";
  detail=message.str();
  return true;
}
bool MotionSafety::latched() const {std::lock_guard<std::mutex> lock(impl_->mutex);return impl_->fault;}
bool MotionSafety::workersIdle() const {std::lock_guard<std::mutex> lock(impl_->mutex);return impl_->executions==0;}
bool MotionSafety::validateRecoverySample(SafetyMask mask, const SafetySample& sample,
    std::string& error, bool require_stationary) const {
  auto& p=*impl_; std::lock_guard<std::mutex> lock(p.mutex);
  const auto now=SafetyClock::now();
  if (shutdownRequested() || p.executions || !p.fault || p.home==Impl::Home::Running) {
    error="recovery requires an idle controller with its motion fence retained";return false;
  }
  if (!mask || (mask&(mask-1)) || !(p.configured&mask)) {
    error="invalid recovery channel";return false;
  }
  const auto& c=p.channels[index(mask)];
  if (!sample.sequence || sample.acquired==SafetyClock::time_point{} || sample.acquired>now ||
      std::chrono::duration<double>(now-sample.acquired).count()>=p.policy.max_feedback_age ||
      sample.position.size()!=c.joints.size() || sample.velocity.size()!=c.joints.size()) {
    error="recovery requires fresh measured feedback for every selected joint";return false;
  }
  for (std::size_t j=0;j<c.joints.size();++j) {
    const auto& bound=c.joints[j];
    const double speed_limit=require_stationary?std::min(.01,bound.velocity):bound.velocity;
    if (!std::isfinite(sample.position[j]) || !std::isfinite(sample.velocity[j]) ||
        sample.position[j]<bound.lower || sample.position[j]>bound.upper ||
        std::abs(sample.velocity[j])>speed_limit) {
      std::ostringstream detail;
      detail << "recovery feedback exceeds validation limits: " << bound.name
             << " position=" << sample.position[j] << " velocity_rad_s=" << sample.velocity[j]
             << " lower=" << bound.lower << " upper=" << bound.upper
             << " max_velocity_rad_s=" << speed_limit << " stationary_required=" << require_stationary;
      error=detail.str();return false;
    }
  }
  error.clear();return true;
}
bool MotionSafety::ready(std::string& error) const {
  if (shutdownRequested()) {error="orderly shutdown in progress";return false;}
  auto& p=*impl_; std::lock_guard<std::mutex> lock(p.mutex);
  if (p.fault) {error=(p.tracking_stop?"position tracking stop in progress: ":"motion fault latched: ")+p.cause;return false;}
  if (p.home==Impl::Home::Failed || (p.homeException() && !p.homeCaller())) {
    error="head startup home pending or running; normal motion blocked";return false;
  }
  for (unsigned i=0;i<4;++i) if ((p.configured&(1U<<i)) && !p.fresh(p.channels[i],SafetyClock::now())) {
    error="feedback unhealthy: channel="+std::to_string(i)+" "+p.channels[i].error;return false;
  }
  error.clear();return p.configured!=0;
}
bool MotionSafety::beginRecoveryFeedbackQualification(std::string& error) {
  auto& p=*impl_; std::lock_guard<std::mutex> lock(p.mutex);
  if (shutdownRequested() || !p.fault || p.tracking_stop || p.executions ||
      p.homeException() || !p.configured ||
      std::any_of(p.channels.begin(),p.channels.end(),[](const auto& c){return c.active;})) {
    error="feedback requalification requires idle explicit recovery with its fault fence retained";
    return false;
  }
  const auto after=SafetyClock::now();
  for (auto& c:p.channels) if (c.read) {
    // Disabled/error feedback was not admitted to normal monitoring. Its frame
    // counter may have wrapped or restarted since the last accepted sample.
    // An in-flight read acquired before this boundary cannot seed the new epoch.
    c.recovery_after=after; c.sample={}; c.healthy=false;
    c.stable_since.reset(); c.stable_frames=0; c.measured_speed=0;
    c.error="waiting for post-recovery acquired feedback";
  }
  p.confirmed=false; p.affected=p.configured; p.changed.notify_all();
  error.clear(); return true;
}
bool MotionSafety::reset(std::string& error) {
  if (shutdownRequested()) {error="orderly shutdown in progress";return false;}
  auto& p=*impl_; std::lock_guard<std::mutex> lock(p.mutex);
  if (p.executions) {error="old execution workers still own resources";return false;}
  if (!p.fault) {error="no latched fault";return false;}
  if (!p.confirmed) {
    std::ostringstream detail; detail << "physical standstill not confirmed";
    const auto now=SafetyClock::now();
    for (unsigned i=0;i<4;++i) if ((p.configured&(1U<<i)) && !p.stationary(p.channels[i],now)) {
      const auto& c=p.channels[i];
      detail << "; channel=" << i << " fresh=" << p.fresh(c,now)
             << " sequence=" << c.sample.sequence << " stationary_frames=" << c.stable_frames
             << " measured_speed=" << c.measured_speed << " error=" << c.error;
    }
    error=detail.str(); return false;
  }
  for (unsigned i=0;i<4;++i) if ((p.configured&(1U<<i)) && !p.stationary(p.channels[i],SafetyClock::now())) {
    error="all channels must have fresh fault-free stationary feedback";return false;
  }
  for (auto& c:p.channels) if (c.read) {c.active=false;p.clearTracking(c);}
  p.fault=false;p.tracking_stop=false;p.confirmed=false;p.cause.clear();p.affected=0;++p.epoch;
  // An explicit reset may release a failed startup only after normal URDF
  // feedback has qualified above. It never retries the startup movement.
  if(p.home==Impl::Home::Failed)p.home=Impl::Home::Disabled;
  error="software latch reset; no drive clear, enable or motion requested";return true;
}
void MotionSafety::enterExecution(){std::lock_guard<std::mutex> lock(impl_->mutex);++impl_->executions;}
void MotionSafety::leaveExecution(){
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if(impl_->executions)--impl_->executions;
  impl_->releaseTrackingStop(SafetyClock::now());
}
bool MotionSafety::finish(SafetyMask mask,bool success,std::string& detail) {
  auto& p=*impl_;std::unique_lock<std::mutex> lock(p.mutex);
  bool active=false;for(unsigned i=0;i<4;++i)if(mask&(1U<<i))active|=p.channels[i].active;
  if (!success && active && !p.fault) p.trip(detail.empty()?"motion canceled or failed after dispatch":detail,SafetyClock::now());
  if (p.fault && active) {
    if (p.tracking_stop && detail.find(p.cause)==std::string::npos)
      detail+=(detail.empty()?"":"; ")+p.cause;
    const auto deadline=SafetyClock::now()+std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(p.policy.stop_timeout));
    p.changed.wait_until(lock,deadline,[&]{return p.confirmed;});
    detail += (detail.empty()?"":"; ")+std::string(p.confirmed?"measured standstill confirmed; ":"STOP UNCONFIRMED; ")+
        (p.tracking_stop?"position tracking stop awaits worker exit; no persistent latch":"fault remains latched");
    success=false;
  }
  for(unsigned i=0;i<4;++i)if(mask&(1U<<i)) {
    p.channels[i].active=false;p.channels[i].held_position=p.channels[i].sample.position;
  }
  const bool result=success && !p.fault;
  p.releaseTrackingStop(SafetyClock::now());
  return result;
}
std::string MotionSafety::status() const {
  auto& p=*impl_;std::lock_guard<std::mutex> lock(p.mutex);std::ostringstream out;
  const bool expired=seconds(SafetyClock::now())-seconds(p.fault_time)>p.policy.stop_timeout;
  out<<"epoch="<<p.epoch<<" state="<<(p.fault?(p.tracking_stop?
    (p.confirmed?"ERROR_TRACKING_STANDSTILL_CONFIRMED":(expired?"ERROR_TRACKING_STOP_UNCONFIRMED":"ERROR_TRACKING_STOPPING")):
    (p.confirmed?"FAULT_STANDSTILL_CONFIRMED":(expired?"STOP_UNCONFIRMED":"STOPPING"))):"MONITORING")
    <<" active_workers="<<p.executions<<" stop_policy=freeze_last_command"
    <<" max_reader_ms="<<p.max_reader_ms<<" max_transaction_ms="<<p.max_transaction_ms<<" cause="<<p.cause
    <<" arm_error="<<(p.policy.arm_error_enabled?"ON":"OFF")
    <<" arm_error_limit_rad="<<p.policy.arm_error_limit_rad
    <<" arm_error_hard_limit_rad="<<p.policy.arm_error_hard_limit_rad
    <<" body_error="<<(p.policy.body_error_enabled?"ON":"OFF")
    <<" body_error_limit_rad="<<p.policy.tracking_error
    <<" body_error_hard_limit_rad="<<p.policy.hard_error
    <<" last_error_stop="<<p.last_tracking_stop;
  const char* home_names[]={"DISABLED","WAITING","RUNNING","DONE","FAILED"};
  out<<" head_startup_home="<<home_names[static_cast<unsigned>(p.home)];
  for(unsigned i=0;i<4;++i)if(p.configured&(1U<<i)) {
    const auto& c=p.channels[i];out<<"; ch="<<i<<" active="<<c.active<<" fresh="<<p.fresh(c,SafetyClock::now())
      <<" feedback_age_ms="<<(c.sample.acquired==SafetyClock::time_point{}?-1:(seconds(SafetyClock::now())-seconds(c.sample.acquired))*1000)
      <<" stationary="<<p.stationary(c,SafetyClock::now())<<" commands="<<c.commands
      <<" max_command_gap_ms="<<c.max_command_gap_ms<<" peak_error_rad="<<c.peak<<" error="<<c.error;
  }return out.str();
}
void installMotionSafety(const std::shared_ptr<MotionSafety>& safety){std::lock_guard<std::mutex> lock(installed_mutex);installed=safety;}
std::shared_ptr<MotionSafety> motionSafety(){std::lock_guard<std::mutex> lock(installed_mutex);return installed.lock();}
thread_local MotionSafetyScope* MotionSafetyScope::current_=nullptr;
MotionSafetyScope::MotionSafetyScope(SafetyMask mask):safety_(motionSafety()),channels_(mask),previous_(current_){
  current_=this;if(safety_)safety_->enterExecution();
}
MotionSafetyScope::~MotionSafetyScope(){
  if(safety_){if(!finished_){std::string error="motion worker exited without terminal confirmation";finish(false,error);}safety_->leaveExecution();}
  current_=previous_;
}
bool MotionSafetyScope::finish(bool success,std::string& detail){
  if(finished_)return success && (!safety_||!safety_->latched());
  finished_=true;return !safety_?success:safety_->finish(channels_,success,detail);
}
bool MotionSafetyScope::beforeTerminal(bool success,std::string& detail){return current_?current_->finish(success,detail):success;}
}  // namespace motion_control_pkg::execution
