#include "motion_control_pkg/sdk/ethercat_drive_maintenance.hpp"
#include "motion_control_pkg/execution/shutdown_request.hpp"
#include <sstream>

namespace motion_control_pkg::sdk {
namespace {
bool disabled(std::uint16_t status) {
  const auto state=status & 0x006fU;
  return state==0x0040U || state==0x0021U || state==0x0023U;
}
bool frameValid(const OnecubeEthercatFeedbackV1& f,DriveClock::time_point now) {
  const auto acquired=DriveClock::time_point(std::chrono::nanoseconds(f.acquired_ns));
  return f.sequence && f.acquired_ns>0 && acquired<=now &&
    now-acquired<=std::chrono::milliseconds(100) &&
    f.status!=0 && f.status!=0xffff && std::isfinite(f.position) && std::isfinite(f.velocity);
}
}

bool EthercatDriveMaintenance::read(std::vector<DriveAxis>& axes,std::string& detail,bool preflight) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!frames_valid_) { detail="no timestamped PDO feedback"; return false; }
  if (!preflight && (!receipt_ || !receipt_->issued || (clearing_ && (reset_failed_ || !receipt_->reset_issued ||
      DriveClock::now()<reset_until_)))) {
    detail=reset_failed_ ? "reset interrupted by invalid/moving feedback" : "waiting for bus write/reset release";
    return false;
  }
  std::ostringstream out;
  for (int id:selected_) {
    if (static_cast<std::size_t>(id)>=frames_.size()) { detail="EtherCAT shutdown/maintenance axis missing"; return false; }
    const auto& f=frames_[id];
    if (!frameValid(f,DriveClock::now())) { detail="invalid/stale EtherCAT PDO frame"; return false; }
    const auto acquired=DriveClock::time_point(std::chrono::nanoseconds(f.acquired_ns));
    if (!preflight && acquired<=write_time_) { detail="waiting for post-command PDO frame"; return false; }
    const bool fault=f.error!=0 || (f.status & 8U)!=0;
    const bool fault_state=(f.status & 0x006fU)==0x0008U;
    axes.push_back({f.position,f.velocity,f.sequence,acquired,disabled(f.status),fault,
                   disabled(f.status) || fault_state,false});
    out << " alias=" << id+1 << " status=0x" << std::hex << f.status
        << " error=0x" << f.error << std::dec << " frame=" << f.sequence
        << " position_rad=" << f.position << " velocity_rad_s=" << f.velocity
        << " disabled=" << disabled(f.status)
        << " age_ms=" << std::chrono::duration<double,std::milli>(DriveClock::now()-acquired).count();
  }
  detail=out.str(); return true;
}

DriveControlResult EthercatDriveMaintenance::control(bool clear_fault) {
  DriveControlResult result;
  if (execution::shutdownRequested()) { result.message="shutdown in progress"; return result; }
  std::unique_lock<std::mutex> clear_lock(clear_mutex_,std::defer_lock);
  if (clear_fault && !clear_lock.try_lock()) {
    result.message="lower-body clear already in progress"; return result;
  }
  std::uint64_t epoch;
  { std::lock_guard<std::mutex> lock(mutex_); epoch=epoch_; }
  const auto canceled=[&] { std::lock_guard<std::mutex> lock(mutex_); return epoch!=epoch_ || execution::shutdownRequested(); };
  if (clear_fault && !waitDriveFeedback(
      [&](auto& axes,auto& detail){return read(axes,detail,true);},
      canceled,true,true,DriveClock::now(),result.message)) return result;
  const auto requested=DriveClock::now();
  auto receipt=std::make_shared<Receipt>();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (execution::shutdownRequested() || (clear_fault && epoch!=epoch_)) {
      result.message="clear superseded by emergency stop"; return result;
    }
    epoch=++epoch_;
    active_=true; clearing_=clear_fault;
    receipt_=receipt; reset_failed_=false;
    reset_until_=requested+std::chrono::milliseconds(200);
    write_time_={};
  }
  result.feedback_confirmed=waitDriveFeedback(
      [&](auto& axes,auto& detail){return read(axes,detail,false);},
      canceled,clear_fault,false,requested,result.message);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (epoch!=epoch_) result.feedback_confirmed=false;
    result.command_issued=clear_fault ? receipt->reset_issued : receipt->issued;
  }
  result.message=(result.feedback_confirmed ? "confirmed disabled and stationary; " :
      "drive operation unconfirmed; ")+result.message+
      "; lower-body disable retained; no enable or trajectory resume";
  return result;
}

EthercatDriveMaintenance::WriteTicket EthercatDriveMaintenance::update(
    const std::vector<OnecubeEthercatFeedbackV1>& frames,bool valid,
    std::vector<DriverSDK::motorTargetStruct>& targets,std::vector<bool>& enable_ready) {
  std::lock_guard<std::mutex> lock(mutex_);
  frames_=frames; frames_valid_=valid;
  if (!active_) return {};
  const auto now=DriveClock::now();
  bool reset=clearing_ && !reset_failed_ && now<reset_until_;
  if (reset) {
    bool safe=valid;
    for (int id:selected_) {
      safe=safe && static_cast<std::size_t>(id)<frames.size();
      if (!safe) break;
      const auto& f=frames[id];
      safe=frameValid(f,now) && std::abs(f.velocity)<=.01 &&
          (disabled(f.status) || (f.status & 0x006fU)==0x0008U);
    }
    if (!safe) { reset_failed_=true; reset=false; }
  }
  for (int id:selected_) {
    if (static_cast<std::size_t>(id)>=targets.size() ||
        static_cast<std::size_t>(id)>=enable_ready.size()) continue;
    auto& target=targets[id];
    target.enabled=reset ? -1 : 0;
    target.vel=target.tor=target.kp=target.kd=0;
    if (valid && static_cast<std::size_t>(id)<frames.size() && frameValid(frames[id],now))
      target.pos=frames[id].position;
    enable_ready[id]=false;
  }
  return {epoch_,reset,receipt_};
}

void EthercatDriveMaintenance::written(WriteTicket ticket,bool success) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ticket.epoch || !ticket.receipt) return;
  if (success) {
    if (!ticket.receipt->issued && ticket.epoch==epoch_) write_time_=DriveClock::now();
    ticket.receipt->issued=true;
    if (ticket.reset) ticket.receipt->reset_issued=true;
  }
  if (ticket.epoch!=epoch_) return;
  if (!success) { if (clearing_) reset_failed_=true; return; }
}

void EthercatDriveMaintenance::requestShutdownDisable() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++epoch_;
  active_=true; clearing_=false; reset_failed_=false;
  receipt_=std::make_shared<Receipt>(); write_time_={};
}
}  // namespace motion_control_pkg::sdk
