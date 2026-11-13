#include "motion_control_pkg/sdk/sdk_command_gate.hpp"
#include "motion_control_pkg/execution/motion_safety.hpp"
#include "motion_control_pkg/execution/shutdown_request.hpp"
#include <algorithm>
#include <sstream>

namespace motion_control_pkg::sdk {
namespace {
constexpr double radians=0.017453292519943295;
// The readiness predicate occupies DriveAxis::disabled here: the same window
// still checks advancing frames, position displacement and measured velocity.
bool waitMode(SdkCommandGate& gate, unsigned mask, int mode, int drag,
    bool button_released, DriveClock::time_point after,
    const std::function<bool()>& canceled, std::string& error,
    const std::function<bool(const DualArmDeviceState&,std::string&)>& check={},
    std::chrono::milliseconds timeout=std::chrono::milliseconds(2000),bool teaching=false) {
  const double max_velocity=teaching?execution::kTeachStandstillVelocity:.01;
  const double max_displacement=teaching?execution::kTeachStandstillDisplacement:.001;
  DriveFeedbackWindow window(after,std::chrono::milliseconds(100),max_velocity,max_displacement);
  const auto end=DriveClock::now()+timeout;
  while(DriveClock::now()<end) {
    if(canceled()) {error="mode/drag feedback wait canceled; "+error;return false;}
    DualArmDeviceState state;
    const auto result=gate.readState(state);
    const auto now=DriveClock::now();
    if(!result) {
      error=result.error_message;window.update({},true,false,now);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));continue;
    }
    std::vector<DriveAxis> axes;
    std::ostringstream out;
    out << "expected_state=" << (mode<0?"1|3":std::to_string(mode))
        << " expected_drag=" << (drag<0?"any":std::to_string(drag))
        << " timeout_ms=" << timeout.count() << " max_velocity_rad_s=" << max_velocity
        << " max_displacement_rad=" << max_displacement << ";";
    for (unsigned i=0;i<2;++i) if (mask&(1U<<i)) {
      const auto& a=state.arms[i];
      const bool matches=(mode<0 ? armFeedbackOperational(a) : a.current_state==mode) &&
        (drag<0 || a.drag_space==drag) && (drag!=1 || a.impedance_type==1) &&
        (!button_released || a.tip_di==0) && a.low_speed;
      out << " arm=" << i << " state=" << a.current_state << " drag=" << a.drag_space
          << " impedance=" << a.impedance_type << " button=" << a.tip_di
          << " error=" << a.error_code << " low_speed=" << a.low_speed;
      double max_speed=0.;
      for(double velocity:a.joint_velocity)max_speed=std::max(max_speed,std::abs(velocity*radians));
      out << " max_speed_rad_s=" << max_speed << " frame=" << a.feedback_sequence
          << " age_ms=" << std::chrono::duration<double,std::milli>(now-a.acquired).count();
      for (unsigned j=0;j<7;++j)
        axes.push_back({a.joint_position[j]*radians,a.joint_velocity[j]*radians,
          static_cast<std::uint64_t>(a.feedback_sequence),a.acquired,matches,a.error_code!=0,matches,true});
    }
    error=out.str();
    std::string rejection;
    if(check && !check(state,rejection)) {error="feedback rejected: "+rejection+"; "+error;return false;}
    if(window.update(axes,true,false,now))return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  error="mode/drag/standstill confirmation timed out: "+error;
  return false;
}
}

SdkCommandGate::ModeFeedbackScope::ModeFeedbackScope(SdkCommandGate& owner,unsigned mask,
    int target,std::chrono::milliseconds lifetime,bool mode_lock_held):gate(owner) {
  std::unique_lock<std::mutex> mode_lock(gate.mode_transition_mutex_,std::defer_lock);
  if(!mode_lock_held)mode_lock.lock();
  std::lock_guard<std::mutex> lock(gate.mutex_);
  if(!mask || (mask&~3U) || (target!=1 && target!=3) || gate.mode_feedback_mask_.load())return;
  gate.mode_feedback_target_=target;
  gate.mode_feedback_epoch_=gate.drive_epoch_.load();
  gate.mode_feedback_owner_=std::this_thread::get_id();
  gate.mode_feedback_deadline_ns_=std::chrono::duration_cast<std::chrono::nanoseconds>(
      (DriveClock::now()+lifetime).time_since_epoch()).count();
  gate.mode_feedback_mask_=mask;active=true;
}
SdkCommandGate::ModeFeedbackScope::~ModeFeedbackScope() {
  if(active)gate.mode_feedback_mask_=0;
}
bool SdkCommandGate::modeTransitionBlocksCaller() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_feedback_mask_.load() && mode_feedback_owner_!=std::this_thread::get_id();
}
bool SdkCommandGate::expectedModeTransition(ArmChannel channel,const ArmFeedback& feedback) const {
  const auto arm=static_cast<unsigned>(channel);
  const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(
      DriveClock::now().time_since_epoch()).count();
  return arm<2 && (mode_feedback_mask_.load()&(1U<<arm)) && feedback.error_code==0 &&
      feedback.current_state==100+mode_feedback_target_.load() &&
      now<mode_feedback_deadline_ns_.load() && mode_feedback_epoch_.load()==drive_epoch_.load();
}

DriveControlResult SdkCommandGate::setTeachMode(
    const std::vector<ArmControlModeCommand>& commands, bool enable,
    const std::function<bool()>& externally_canceled) {
  DriveControlResult result;
  unsigned mask=0;
  for (const auto& command:commands) {
    auto i=static_cast<unsigned>(command.channel);
    if (i>=2 || (mask&(1U<<i)) || command.mode!=ArmControlMode::kJointImpedance) {
      result.message="invalid teaching arm/mode";return result;
    }
    mask|=1U<<i;
  }
  if (!mask) {result.message="no teaching arm selected";return result;}
  const auto epoch=drive_epoch_.load();
  const auto canceled=[&] {
    return execution::shutdownRequested() || externally_canceled() || epoch!=drive_epoch_.load();
  };
  const auto safety=execution::motionSafety();
  const std::function<bool(const DualArmDeviceState&,std::string&)> normal_feedback=
      [&](const DualArmDeviceState&,std::string& error) {return !safety || safety->ready(error);};
  const auto timeout=options_.teach_transition_timeout;
  std::optional<ModeFeedbackScope> feedback_scope;
  if (enable) {
    if (teaching_mask_.load()) {result.message="exit current teaching mode before enabling again";return result;}
    if (!waitMode(*this,mask,-1,0,true,DriveClock::now(),canceled,result.message,normal_feedback,timeout,true)) return result;
    if (canceled()) {result.message="teaching transition canceled";return result;}
    // Include bounded stationary waits and the existing SDK settle/readback
    // intervals. Every final mode still needs its own advancing-frame window.
    const auto lifetime=timeout*6+options_.tool_parameter_settle_time+
        options_.tool_readback_retry_interval*options_.tool_readback_attempts+
        options_.impedance_parameter_settle_time+options_.impedance_mode_settle_time;
    feedback_scope.emplace(*this,mask,3,lifetime);
    if(!*feedback_scope) {result.message="another arm mode transition is active";return result;}
    auto mode=configureControlModesImpl(commands,&result.command_issued,canceled,true);
    if (!mode) {
      result.message=(result.command_issued?"teaching configuration incomplete: ":
          "teaching configuration rejected before commands: ")+mode.error_message;
      if(result.command_issued) {
        teaching_mask_.fetch_or(mask);
        if (safety) safety->requestStop(result.message);
      }
      return result;
    }
    if (!waitMode(*this,mask,3,0,true,DriveClock::now(),canceled,result.message,normal_feedback,timeout,true)) {
      teaching_mask_.fetch_or(mask);
      if (safety) safety->requestStop("teaching torque-mode confirmation failed");
      return result;
    }
  }
  std::unique_lock<std::mutex> transition(mode_transition_mutex_,std::try_to_lock);
  if (!transition.owns_lock()) {result.message="another arm mode transition is active";return result;}
  if(modeTransitionBlocksCaller()) {result.message="another arm mode transition is active";return result;}
  // Exit while faulted only disarms teaching and disables servos. Explicit
  // recover_robot owns fault recovery; this switch never clears a fault.
  const bool disable_only=!enable && safety && safety->latched();
  if (!enable && !disable_only &&
      !waitMode(*this,mask,-1,-1,true,DriveClock::now(),canceled,result.message,normal_feedback,timeout,true)) return result;
  const auto failed_phase=[&](const char* phase) {
    result.message=std::string(phase)+": "+result.message;
    if(result.command_issued && safety)safety->requestStop(result.message);
    return result;
  };
  unsigned precommand_resamples=0;
  std::string last_precommand_rejection;
  const auto commit_stage=[&](bool exit_drag_only) {
    DualArmDeviceState held;
    std::string error;
    std::optional<execution::MotionSafety::CommandPermit> permit;
    const auto deadline=DriveClock::now()+timeout;
    const auto remaining=[&] {
      return std::max(std::chrono::milliseconds(0),
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline-DriveClock::now()));
    };
    for(;;) {
      if (safety && !disable_only) {
        permit.emplace(safety->teachingConfigurationWhenStationary(mask,error,canceled,remaining()));
        if (!*permit) {result.message=error;return false;}
      }
      // The monitor may have waited for standstill. Capture the hold only after
      // that wait, while its permit is locked; never restore a pre-wait snapshot.
      if (!readState(held)) {result.message="teaching transition has no valid feedback";return false;}
      bool qualified=true;
      if(!disable_only) for(unsigned i=0;i<2;++i) if(mask&(1U<<i)) {
        const auto& arm=held.arms[i];
        const auto now=DriveClock::now();
        double max_speed=0.;
        for(unsigned j=0;j<7;++j) {
          if(!std::isfinite(arm.joint_position[j]) || !std::isfinite(arm.joint_velocity[j])) {
            result.message="teaching command feedback is non-finite";return false;
          }
          max_speed=std::max(max_speed,std::abs(arm.joint_velocity[j]*radians));
        }
        if(!armFeedbackOperational(arm) || !arm.low_speed || arm.tip_di!=0 ||
            arm.acquired==DriveClock::time_point{} || arm.acquired>now ||
            now-arm.acquired>std::chrono::milliseconds(100) ||
            max_speed>execution::kTeachStandstillVelocity ||
            (enable && (arm.current_state!=3 || arm.drag_space!=0 || arm.impedance_type!=1)) ||
            (!enable && !exit_drag_only && arm.drag_space!=0)) {
          std::ostringstream detail;
          detail << "precommand feedback rejected: arm=" << i << " state=" << arm.current_state
              << " drag=" << arm.drag_space << " impedance=" << arm.impedance_type
              << " button=" << arm.tip_di << " error=" << arm.error_code
              << " low_speed=" << arm.low_speed << " max_speed_rad_s=" << max_speed
              << " frame=" << arm.feedback_sequence
              << " age_ms=" << std::chrono::duration<double,std::milli>(now-arm.acquired).count();
          last_precommand_rejection=detail.str();qualified=false;break;
        }
      }
      if(!qualified) {
        // A fresh SDK frame can change after either stationary window. Release
        // the permit and qualify a NEW full window within this stage's original
        // deadline. Only observations are retried; no SDK batch has begun yet.
        permit.reset();++precommand_resamples;
        const auto left=remaining();
        if(left.count()==0) {
          result.message="teaching precommand confirmation timed out: "+last_precommand_rejection;return false;
        }
        if(!waitMode(*this,mask,enable?3:-1,(enable || !exit_drag_only)?0:-1,true,
            DriveClock::now(),canceled,result.message,normal_feedback,left,true)) {
          result.message+="; "+last_precommand_rejection;return false;
        }
        continue;
      }
      if(permit) for(unsigned i=0;i<2;++i) if(mask&(1U<<i)) {
        std::vector<double> radians_q;
        for(double q:held.arms[i].joint_position)radians_q.push_back(q*radians);
        if(!permit->validate(1U<<i,radians_q)) {result.message="teaching measured pose violates position limits";return false;}
      }
      break;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (canceled() || !beginCommandBatchWithRetry()) {
      result.message="teaching transaction canceled or begin failed";return false;
    }
    teaching_mask_.fetch_or(mask); // A partial transaction must keep normal commands blocked.
    result.command_issued=true;
    for (unsigned i=0;i<2;++i) if (mask&(1U<<i)) {
      const auto arm=static_cast<ArmChannel>(i);
      // Never combine drag-exit and position mode in the same SDK packet.
      // The second stage follows independently acquired drag=0 feedback.
      bool ok=true;
      if(enable || disable_only || exit_drag_only)ok=device_.setDragSpace(arm,enable?1:0);
      if (!enable && !exit_drag_only) {
        ok=(disable_only ? device_.setServoOff(arm) :
          (device_.setJointPosition(arm,held.arms[i].joint_position) && device_.setPositionMode(arm))) && ok;
      }
      if (!ok) {result.message="teaching batch setter failed";if(permit)permit->failed(result.message);return false;}
    }
    result.command_issued=true;
    if (canceled() || !device_.commitCommandBatch()) {
      result.message="teaching batch commit failed";if(permit)permit->failed(result.message);return false;
    }
    return true;
  };
  if(!enable && !disable_only) {
    // Match the SDK ExitDrag protocol: exit drag, confirm its echo on fresh
    // stationary frames, then recapture the hold before selecting position mode.
    if(!commit_stage(true))return failed_phase("teach OFF drag-exit command");
    if(!waitMode(*this,mask,-1,0,true,DriveClock::now(),canceled,result.message,normal_feedback,timeout,true))
      return failed_phase("teach OFF drag-exit confirmation");
    feedback_scope.emplace(*this,mask,1,timeout*3,true);
    if(!*feedback_scope) {
      result.message="another arm mode transition is active";
      return failed_phase("teach OFF position preparation");
    }
  }
  if(!commit_stage(false))return failed_phase(enable?"teach ON drag command":
      disable_only?"teach OFF disable command":"teach OFF position command");
  const std::function<bool(const DualArmDeviceState&,std::string&)> feedback_check=disable_only?
      std::function<bool(const DualArmDeviceState&,std::string&)>{}:normal_feedback;
  result.feedback_confirmed=waitMode(*this,mask,enable?3:(disable_only?0:1),enable?1:0,
      !disable_only,DriveClock::now(),canceled,result.message,feedback_check,
      disable_only?options_.mode_transition_timeout:timeout,!disable_only);
  if (result.feedback_confirmed && !enable) teaching_mask_.fetch_and(~mask);
  if(!result.feedback_confirmed)return failed_phase(enable?"teach ON drag confirmation":
      disable_only?"teach OFF disabled confirmation":"teach OFF position confirmation");
  result.message=(enable?"joint teaching armed; hold the physical tool button to drag; ":
      disable_only?"teaching disarmed; servos disabled; recovery required; ":
      "teaching OFF; drag exit confirmed; holding measured position; ")+result.message;
  if(precommand_resamples)result.message+="; precommand_resamples="+std::to_string(precommand_resamples)+
      "; "+last_precommand_rejection;
  return result;
}

DriveControlResult SdkCommandGate::recoverPositionMode(
    const std::function<bool(const DualArmDeviceState&,std::string&)>& check,
    const std::function<bool()>& externally_canceled,
    const std::function<bool(const DualArmDeviceState&,std::string&)>& settling_check) {
  DriveControlResult result;
  std::unique_lock<std::mutex> transition(mode_transition_mutex_,std::try_to_lock);
  std::unique_lock<std::mutex> clear(drive_clear_mutex_,std::try_to_lock);
  if (!transition.owns_lock() || !clear.owns_lock() || !check) {
    result.message="arm recovery busy or missing validation";return result;
  }
  const auto epoch=drive_epoch_.load();
  const auto canceled=[&] {return execution::shutdownRequested() || externally_canceled() || epoch!=drive_epoch_.load();};
  if (!waitMode(*this,3,0,-1,true,DriveClock::now(),canceled,result.message,check,options_.mode_transition_timeout)) return result;
  DualArmDeviceState held;
  if (!readState(held) || !check(held,result.message) || canceled()) return result;
  ModeFeedbackScope feedback_scope(*this,3,1,options_.mode_transition_timeout*2,true);
  if(!feedback_scope) {result.message="another arm mode transition is active";return result;}
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (canceled() || !beginCommandBatchWithRetry()) {result.message="arm recovery canceled/begin failed";return result;}
    for (unsigned i=0;i<2;++i) {
      const auto arm=static_cast<ArmChannel>(i);
      if (!device_.setDragSpace(arm,0) || !device_.setJointPosition(arm,held.arms[i].joint_position) ||
          !device_.setJointLimits(arm,10,10) || !device_.setPositionMode(arm)) {
        result.message="arm recovery hold/mode setter failed";return result;
      }
    }
    result.command_issued=true;
    if (canceled() || !device_.commitCommandBatch()) {result.message="arm recovery commit failed";return result;}
  }
  const auto held_check=[&](const DualArmDeviceState& state,std::string& error) {
    // Servo enable can produce a short measured-speed transient. Continue to
    // enforce freshness, URDF limits and hold displacement while waitMode owns
    // the unchanged low-speed, advancing-frame stationary window and timeout.
    if (!(settling_check?settling_check:check)(state,error)) return false;
    for(unsigned i=0;i<2;++i) for(unsigned j=0;j<7;++j)
      if(std::abs(state.arms[i].joint_position[j]-held.arms[i].joint_position[j])*radians>.01) {
        error="arm displaced from recovery hold position";return false;
      }
    return true;
  };
  result.feedback_confirmed=waitMode(*this,3,1,0,true,DriveClock::now(),canceled,result.message,held_check,
      options_.mode_transition_timeout);
  if(result.feedback_confirmed) teaching_mask_=0;
  result.message=(result.feedback_confirmed?"arms enabled at measured positions; ":"arm enable unconfirmed; ")+result.message;
  return result;
}
} // namespace motion_control_pkg::sdk
