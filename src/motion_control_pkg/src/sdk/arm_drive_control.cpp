#include "motion_control_pkg/execution/shutdown_request.hpp"
#include "motion_control_pkg/sdk/sdk_command_gate.hpp"

#include <sstream>

namespace motion_control_pkg::sdk {
DriveControlResult SdkCommandGate::controlDrives(bool left, bool right, bool clear_fault, bool recovery_stop,
    const std::function<bool()>& cancel_clear) {
  DriveControlResult result;
  if (execution::shutdownRequested()) { result.message="shutdown in progress"; return result; }
  if (!left && !right) { result.message="no arm selected"; return result; }
  std::unique_lock<std::mutex> clear_lock(drive_clear_mutex_,std::defer_lock);
  if (clear_fault && !clear_lock.try_lock()) {
    result.message="arm clear already in progress"; return result;
  }
  // Mark stop intent before waiting for the SDK mutex. A clearing loop must
  // yield after its current vendor call instead of taking the next transaction.
  auto epoch=clear_fault ? drive_epoch_.load() : drive_epoch_.fetch_add(1)+1;
  const auto canceled=[&] { return epoch!=drive_epoch_.load() || execution::shutdownRequested() ||
      (clear_fault && cancel_clear && cancel_clear()); };
  const auto read=[&](std::vector<DriveAxis>& axes,std::string& detail) {
    DualArmDeviceState state;
    if (!readState(state)) { detail="SDK read failed or frame sequence invalid"; return false; }
    std::ostringstream out;
    for (std::size_t i=0;i<kArmCount;++i) {
      if (!(i==0 ? left : right)) continue;
      const auto& arm=state.arms[i];
      out << (i==0 ? " A" : " B") << ": state=" << arm.current_state
          << " error=" << arm.error_code << " frame=" << arm.feedback_sequence;
      for (std::size_t j=0;j<kArmDof;++j) {
        constexpr double rad=3.14159265358979323846/180.;
        axes.push_back({arm.joint_position[j]*rad,arm.joint_velocity[j]*rad,
          static_cast<std::uint64_t>(arm.feedback_sequence),arm.acquired,
          arm.current_state==0 && arm.low_speed,arm.error_code!=0,
          (arm.current_state==0 || arm.current_state==100) && arm.low_speed,true});
      }
    }
    detail=out.str(); return true;
  };
  if (clear_fault && !waitDriveFeedback(read,canceled,true,true,DriveClock::now(),result.message))
    return result;
  auto issued=DriveClock::now();
  {
    // Same gate as all motion batches. No settling wait holds this mutex.
    std::lock_guard<std::mutex> lock(mutex_);
    if (released_ || execution::shutdownRequested()) { result.message="shutdown in progress"; return result; }
    if (clear_fault) {
      if (canceled()) { result.message="clear superseded by emergency stop"; return result; }
      bool ok=true;
      if (left) ok=device_.clearServoError(ArmChannel::kLeft) && ok;
      if (right) ok=!canceled() && device_.clearServoError(ArmChannel::kRight) && ok;
      result.command_issued=ok;
    } else {
      result.command_issued=device_.emergencyStop(left,right);
    }
    issued=DriveClock::now();
  }
  if (!result.command_issued) { result.message="SDK operation unavailable or invocation failed"; return result; }
  result.feedback_confirmed=waitDriveFeedback(read,canceled,clear_fault,recovery_stop && !clear_fault,issued,result.message);
  if (!clear_fault && recovery_stop && !result.feedback_confirmed && !canceled()) {
    // The vendor's void EStop ignores its internal batch/commit return values.
    // During recovery/rollback, a busy enable transaction can therefore leave
    // the arm enabled. Make one checked servo-off transaction after timeout;
    // never repeat a clear, an enable or a position target here.
    const auto first_failure=result.message;
    bool committed=false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (released_ || canceled()) result.message="servo-off superseded or driver released";
      else if (!beginCommandBatchWithRetry()) result.message="servo-off batch begin failed";
      else {
        bool set=true;
        if(left)set=device_.setServoOff(ArmChannel::kLeft) && set;
        if(right)set=device_.setServoOff(ArmChannel::kRight) && set;
        if(!set)result.message="servo-off setter failed";
        else if(canceled())result.message="servo-off superseded before commit";
        else {
          committed=device_.commitCommandBatch();
          issued=DriveClock::now();
          if(!committed)result.message="servo-off commit failed";
        }
      }
    }
    if(committed)result.feedback_confirmed=waitDriveFeedback(read,canceled,false,true,issued,result.message);
    result.message="EStop confirmation failed: "+first_failure+"; checked servo-off fallback: "+result.message;
  }
  if (clear_fault && result.feedback_confirmed) {
    // A controller error of zero alone does not prove seven servo errors clear.
    // Query each register with checked SDK return status, one transaction per
    // lock acquisition, so emergency stop can interrupt between queries.
    std::ostringstream servo;
    for (std::size_t i=0;i<kArmCount && result.feedback_confirmed;++i) {
      if (!(i==0 ? left : right)) continue;
      for (std::size_t j=0;j<kArmDof;++j) {
        long code=-1; bool read_ok=false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!canceled()) read_ok=device_.readServoError(static_cast<ArmChannel>(i),j,code);
        }
        servo << (i==0 ? " A" : " B") << j+1 << "=" << code;
        if (!read_ok) servo << "(query_failed)";
        if (!read_ok || code!=0 || canceled()) {
          result.feedback_confirmed=false; break;
        }
      }
    }
    if (result.feedback_confirmed)
      result.feedback_confirmed=waitDriveFeedback(read,canceled,true,false,DriveClock::now(),result.message);
    result.message+="; servo readback:"+servo.str();
  }
  if (canceled()) result.feedback_confirmed=false;
  result.message=(result.feedback_confirmed ? (recovery_stop?"confirmed reset-eligible and stationary; ":"confirmed disabled and stationary; ") :
      "command invoked, outcome unconfirmed; ")+result.message+
      "; motion fence retained; no enable or trajectory resume";
  return result;
}
bool SdkCommandGate::readShutdownFeedback(std::vector<DriveAxis>& axes, std::string& detail) {
  DualArmDeviceState state;
  const auto result=readState(state);
  if (!result) { detail=result.error_message; return false; }
  std::ostringstream out;
  for (const auto& arm:state.arms) {
    out << " state=" << arm.current_state << " error=" << arm.error_code
        << " sequence=" << arm.feedback_sequence << " low_speed=" << arm.low_speed;
    for (std::size_t j=0;j<kArmDof;++j) {
      constexpr double rad=3.14159265358979323846/180.;
      axes.push_back({arm.joint_position[j]*rad, arm.joint_velocity[j]*rad,
        static_cast<std::uint64_t>(arm.feedback_sequence),arm.acquired,
        arm.current_state==0 && arm.low_speed,arm.error_code!=0,true,true});
    }
  }
  detail=out.str(); return true;
}

SdkCommandResult SdkCommandGate::disableForShutdown() {
  if (!execution::shutdownRequested()) return fail("no shutdown request");
  ++drive_epoch_;
  std::lock_guard<std::mutex> lock(mutex_);
  if (released_) return fail("driver already released");
  const auto deadline=DriveClock::now()+options_.clear_timeout;
  bool begun=false;
  do {
    if (device_.beginCommandBatch()) { begun=true; break; }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while (DriveClock::now()<deadline);
  if (!begun) return fail("shutdown servo-off batch begin");
  if (!device_.setServoOff(ArmChannel::kLeft) || !device_.setServoOff(ArmChannel::kRight))
    return fail("shutdown servo-off target state");
  if (!device_.commitCommandBatch()) return fail("shutdown servo-off commit");
  return SdkCommandResult::ok();
}
}  // namespace motion_control_pkg::sdk
