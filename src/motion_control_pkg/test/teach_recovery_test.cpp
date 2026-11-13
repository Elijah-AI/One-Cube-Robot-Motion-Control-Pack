#include "motion_control_pkg/sdk/sdk_command_gate.hpp"
#include "motion_control_pkg/sdk/ethercat_drive_maintenance.hpp"
#include "motion_control_pkg/execution/robot_recovery.hpp"
#include "motion_control_pkg/execution/motion_safety.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <future>

using namespace motion_control_pkg::sdk;
using namespace motion_control_pkg::execution;
using namespace std::chrono_literals;
static void require(bool value,const char* why) {if(!value)throw std::runtime_error(why);}
struct Device : DualArmDeviceInterface {
  DualArmDeviceState state{},pending{};
  bool advance=true,echo_drag=true,commit_ok=true;
  std::atomic<int> commits{0},clears{0},joint_writes{0};
  // Some controllers acknowledge drag exit but ignore a position-mode request
  // in that same packet. Require a separately observed drag-off frame/window.
  bool require_drag_exit_ack=false,freeze_drag_exit=false,ignore_position_mode=false;
  unsigned drop_drag_exit_mask=0;
  bool press_button_after_exit=false,move_after_exit=false;
  std::chrono::milliseconds drag_exit_delay{0};
  std::array<DriveClock::time_point,2> drag_exit_ready{},drag_exit_seen{};
  std::array<bool,2> wrote_drag{};
  std::atomic<int> exit_ack_frames{0},ignored_position_modes{0};
  double position_after_drag_exit=0.;
  std::chrono::milliseconds transition_delay{0};
  double enable_velocity_rad_s=0.,enable_displacement_deg=0.;
  bool persistent_enable_velocity=false;
  int stuck_target=0,fault_target=0,invalid_position_target=0,frozen_target=0,wrong_state_target=0;
  std::array<int,2> mode_targets{};
  std::array<DriveClock::time_point,2> mode_ready{};
  std::atomic<int> seen_101{0},seen_103{0};
  std::atomic<std::int64_t> position_change_ns{0};
  std::atomic<double> reported_velocity_rad_s{0.};
  std::function<void(DualArmDeviceState&)> feedback_hook;
  std::array<std::array<double,7>,2> commanded{};
  Device() {
    for(auto& a:state.arms) {
      a.current_state=1;a.low_speed=true;a.feedback_sequence=1;
      a.drag_space=0;a.impedance_type=0;a.tip_di=0;a.joint_position.fill(12.0);
    }
  }
  bool checkCompatibility()override{return true;}
  bool connect(const std::array<unsigned char,4>&)override{return true;}
  bool release()override{return true;}
  void disableLogging()override{}
  bool readState(DualArmDeviceState& out)override{
    const auto now_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(DriveClock::now().time_since_epoch()).count();
    if(position_change_ns.load()>0 && now_ns>=position_change_ns.load())
      for(auto& arm:state.arms)arm.joint_position.fill(33.);
    for(unsigned i=0;i<2;++i) {
      auto& a=state.arms[i];
      if(reported_velocity_rad_s.load()!=0.)
        a.joint_velocity.fill(reported_velocity_rad_s.load()/0.017453292519943295);
      if(drag_exit_ready[i]!=DriveClock::time_point{} && DriveClock::now()>=drag_exit_ready[i]) {
        a.drag_space=0;drag_exit_ready[i]={};drag_exit_seen[i]=DriveClock::now();
        if(position_after_drag_exit!=0.)a.joint_position.fill(position_after_drag_exit);
        if(press_button_after_exit)a.tip_di=1;
        if(move_after_exit)a.joint_velocity.fill(2.);
      }
      if(drag_exit_seen[i]!=DriveClock::time_point{} && a.drag_space==0)++exit_ack_frames;
      const bool transitioning=mode_ready[i]!=DriveClock::time_point{};
      if(transitioning) {
        if(DriveClock::now()>=mode_ready[i] && stuck_target!=mode_targets[i]) {
          a.current_state=mode_targets[i];mode_ready[i]={};
        } else {
          a.current_state=wrong_state_target==mode_targets[i]?102:100+mode_targets[i];
          if(fault_target==mode_targets[i])a.error_code=91;
          if(invalid_position_target==mode_targets[i])a.joint_position[0]=999.;
        }
      }
      if(a.current_state==101)++seen_101;
      if(a.current_state==103)++seen_103;
      if(mode_targets[i]==1 && enable_velocity_rad_s!=0.) {
        const bool moving=a.current_state==101 || persistent_enable_velocity;
        a.joint_velocity.fill(moving?enable_velocity_rad_s/0.017453292519943295:0.);
        a.low_speed=!moving;
        if(enable_displacement_deg!=0.)a.joint_position.fill(12.+enable_displacement_deg);
      }
      if(advance && !(freeze_drag_exit && drag_exit_seen[i]!=DriveClock::time_point{}) &&
          !(transitioning && frozen_target==mode_targets[i]))++a.feedback_sequence;
    }
    out=state;if(feedback_hook)feedback_hook(out);return true;
  }
  bool beginCommandBatch()override{pending=state;wrote_drag.fill(false);return true;}
  bool clearServoError(ArmChannel c)override{++clears;state.arms[static_cast<int>(c)].error_code=0;return true;}
  bool setServoOff(ArmChannel c)override{pending.arms[static_cast<int>(c)].current_state=0;return true;}
  bool setPositionMode(ArmChannel c)override{
    const auto i=static_cast<unsigned>(c);
    if(ignore_position_mode || (require_drag_exit_ack && (wrote_drag[i] || state.arms[i].drag_space!=0 ||
        drag_exit_seen[i]==DriveClock::time_point{} || DriveClock::now()-drag_exit_seen[i]<100ms))) {
      ++ignored_position_modes;return true;
    }
    pending.arms[i].current_state=1;return true;
  }
  bool setTorqueMode(ArmChannel c)override{pending.arms[static_cast<int>(c)].current_state=3;return true;}
  bool setJointLimits(ArmChannel,int,int)override{return true;}
  bool setTool(ArmChannel c,const ToolParameters& t)override{
    auto& a=pending.arms[static_cast<int>(c)];a.tool_kinematics=t.kinematics;a.tool_dynamics=t.dynamics;return true;
  }
  bool setJointImpedanceGains(ArmChannel,const std::array<double,7>&,const std::array<double,7>&)override{return true;}
  bool setCartesianImpedanceGains(ArmChannel,const std::array<double,7>&,const std::array<double,7>&)override{return true;}
  bool setImpedanceType(ArmChannel c,ArmControlMode m)override{pending.arms[static_cast<int>(c)].impedance_type=static_cast<int>(m);return true;}
  bool setDragSpace(ArmChannel c,int space)override{
    const auto i=static_cast<unsigned>(c);wrote_drag[i]=true;
    if(echo_drag && !((drop_drag_exit_mask&(1U<<i)) && space==0))pending.arms[i].drag_space=space;
    return true;
  }
  bool setJointPosition(ArmChannel c,const std::array<double,7>& q)override{++joint_writes;commanded[static_cast<int>(c)]=q;return true;}
  bool commitCommandBatch()override{
    ++commits;
    if(commit_ok) {
      for(unsigned i=0;i<2;++i)if(wrote_drag[i] && pending.arms[i].drag_space==0 && state.arms[i].drag_space!=0) {
        drag_exit_ready[i]=DriveClock::now()+drag_exit_delay;
        pending.arms[i].drag_space=state.arms[i].drag_space;
      }
      for(unsigned i=0;i<2;++i)if(pending.arms[i].current_state!=state.arms[i].current_state) {
        const int target=pending.arms[i].current_state;
        if(transition_delay.count()>0 && (target==1 || target==3)) {
          mode_targets[i]=target;mode_ready[i]=DriveClock::now()+transition_delay;
          pending.arms[i].current_state=100+target;
        } else mode_ready[i]={};
      }
      state=pending;
    }
    return commit_ok;
  }
  bool emergencyStop(bool left,bool right)override{
    for(unsigned i=0;i<2;++i)if(i==0?left:right) {
      state.arms[i].current_state=0;mode_ready[i]={};
    }
    return true;
  }
};
struct HeadBus {
  EthercatDriveMaintenance maintenance{{31,30}};
  std::atomic<bool> quit{false};
  std::atomic<unsigned> wrong_axis{0},resets{0};
  std::thread worker;
  HeadBus():worker([this] {
    std::vector<OnecubeEthercatFeedbackV1> frames(32);
    std::vector<DriverSDK::motorTargetStruct> targets(32);
    std::vector<bool> enabled(32,true);
    for(auto& t:targets)t.enabled=1;
    for(auto& f:frames){f.position=.2;f.status=8;f.error=12;}
    while(!quit) {
      const auto now=DriveClock::now();
      for(auto& f:frames){++f.sequence;f.acquired_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();}
      auto ticket=maintenance.update(frames,true,targets,enabled);
      for(int id=0;id<30;++id)if(targets[id].enabled!=1)++wrong_axis;
      for(int id:{30,31})if(targets[id].enabled==-1){frames[id].error=0;frames[id].status=0x21;++resets;}
      maintenance.written(ticket,true);std::this_thread::sleep_for(4ms);
    }
  }){}
  ~HeadBus(){quit=true;worker.join();}
};
static SdkCommandGateOptions options() {
  SdkCommandGateOptions o;o.tool_parameter_settle_time=0ms;o.impedance_parameter_settle_time=0ms;o.impedance_mode_settle_time=0ms;
  o.teach_transition_timeout=2000ms;return o;
}
static std::vector<ArmControlModeCommand> commands() {
  std::vector<ArmControlModeCommand> out(2);
  for(unsigned i=0;i<2;++i){auto& c=out[i];c.channel=static_cast<ArmChannel>(i);c.mode=ArmControlMode::kJointImpedance;
    c.tool.dynamics[0]=1.;c.impedance.stiffness.fill(5.);c.impedance.damping.fill(.3);}
  return out;
}
// Exercise the same safety reader as production, with real monitor polling and
// delayed vendor state transitions instead of instantly echoing final modes.
struct SafetyRig {
  Device device;
  SdkCommandGate gate;
  std::shared_ptr<MotionSafety> monitor=std::make_shared<MotionSafety>();
  std::atomic<std::int64_t> force_moving_until_ns{0};
  SafetyRig():gate(device,ActiveArmConfiguration::kDualArm,[] {
    auto o=options();o.mode_transition_timeout=400ms;o.teach_transition_timeout=600ms;return o;
  }()){}
  void start() {
    for(unsigned i=0;i<2;++i)monitor->addChannel(1U<<i,
      std::vector<SafetyJoint>(7,{"fake_arm_joint",-2.,2.,2.}),
      [this,i](SafetySample& sample,std::string& error) {
        DualArmDeviceState state;
        if(!gate.readState(state))return false;
        const auto& a=state.arms[i];
        if(!armFeedbackOperational(a) && !gate.expectedModeTransition(static_cast<ArmChannel>(i),a)) {
          error="unexpected arm state="+std::to_string(a.current_state)+" error="+std::to_string(a.error_code);return false;
        }
        sample.sequence=a.feedback_sequence;sample.acquired=a.acquired;
        for(unsigned j=0;j<7;++j) {
          sample.position.push_back(a.joint_position[j]*0.017453292519943295);
          const auto now_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(DriveClock::now().time_since_epoch()).count();
          sample.velocity.push_back(now_ns<force_moving_until_ns.load()?.04:a.joint_velocity[j]*0.017453292519943295);
        }
        return true;
      });
    installMotionSafety(monitor);monitor->start();
    const auto end=DriveClock::now()+1s;std::string error;
    while(DriveClock::now()<end) {
      {auto permit=monitor->configuration(3,error);if(permit)return;}
      std::this_thread::sleep_for(5ms);
    }
    throw std::runtime_error("fake monitor failed to settle: "+error);
  }
  ~SafetyRig(){monitor->stop();installMotionSafety(nullptr);}
};
int main()try {
  int cases=0;const auto no=[] {return false;};
  {
    // Even after the fixed 1 s mode-settle delay, this needs more than the old
    // 2 s feedback deadline. Exercise the production 5 s default end to end.
    Device device;device.transition_delay=3500ms;
    SdkCommandGate gate(device,ActiveArmConfiguration::kDualArm);
    const auto r=gate.setTeachMode(commands(),true,no);
    require(r.success(),("5 s teaching transition failed: "+r.message).c_str());++cases;
  }
  {
    // Both the SDK window and the independent configuration permit must use
    // the new teaching velocity. Ordinary command admission stays strict.
    SafetyRig rig;rig.start();rig.device.reported_velocity_rad_s=.025;
    auto r=rig.gate.setTeachMode(commands(),true,no);
    require(r.success(),("teaching rejected .025 rad/s: "+r.message).c_str());
    r=rig.gate.setTeachMode(commands(),false,no);
    require(r.success(),("teaching OFF retained old velocity limit: "+r.message).c_str());
    std::string error;
    {auto normal=rig.monitor->command(kArmSafety,error);
      require(!normal,"teaching limits leaked into ordinary command admission");}
    require(!rig.gate.configureControlModes(commands()),"normal mode configuration used teaching limits");
    ++cases;
  }
  {
    SafetyRig rig;rig.start();
    rig.force_moving_until_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
        (DriveClock::now()+5s).time_since_epoch()).count();
    auto r=rig.gate.setTeachMode(commands(),true,no);
    require(!r.success() && !r.command_issued,"teaching accepted monitor velocity above .03");++cases;
  }
  {
    SafetyRig rig;rig.device.require_drag_exit_ack=true;rig.device.drag_exit_delay=120ms;
    rig.device.transition_delay=140ms;rig.device.position_after_drag_exit=25.;rig.start();
    auto r=rig.gate.setTeachMode(commands(),true,no);require(r.success(),r.message.c_str());
    r=rig.gate.setTeachMode(commands(),false,no);
    require(r.success(),("separate drag-exit acknowledgement regression: "+r.message).c_str());
    require(rig.device.ignored_position_modes==0 && rig.device.exit_ack_frames>=5 &&
        rig.device.commanded[0][0]==25. && rig.device.commanded[1][0]==25. && !rig.monitor->latched(),
        "exit did not acknowledge drag-off before refreshing hold and selecting position mode");++cases;
  }
  for(unsigned selected=0;selected<2;++selected) {
    SafetyRig rig;rig.device.require_drag_exit_ack=true;rig.device.drag_exit_delay=120ms;rig.start();
    const std::vector<ArmControlModeCommand> one{commands()[selected]};
    auto r=rig.gate.setTeachMode(one,true,no);require(r.success(),r.message.c_str());
    r=rig.gate.setTeachMode(one,false,no);require(r.success(),r.message.c_str());
    const auto& other=rig.device.state.arms[1-selected];
    require(rig.device.joint_writes==1 && rig.device.commanded[selected][0]==12. &&
        other.current_state==1 && other.drag_space==0 && rig.gate.teachingMask()==0,
        "single-arm staged exit affected the other arm");++cases;
  }
  for(int bad=0;bad<5;++bad) {
    SafetyRig rig;rig.device.require_drag_exit_ack=true;rig.device.drag_exit_delay=30ms;
    if(bad==0)rig.device.drop_drag_exit_mask=3;
    if(bad==1)rig.device.freeze_drag_exit=true;
    if(bad==2)rig.device.press_button_after_exit=true;
    if(bad==3)rig.device.move_after_exit=true;
    if(bad==4)rig.device.drop_drag_exit_mask=2; // One arm acknowledges, the other does not.
    rig.start();auto r=rig.gate.setTeachMode(commands(),true,no);require(r.success(),r.message.c_str());
    r=rig.gate.setTeachMode(commands(),false,no);
    require(!r.success() && r.command_issued && !r.feedback_confirmed && rig.gate.teachingMask()==3 &&
        rig.device.joint_writes==0 && rig.device.ignored_position_modes==0 && rig.monitor->latched(),
        "unconfirmed drag exit advanced to hold/mode commands or lost its fence");
    require(r.message.find("teach OFF drag-exit confirmation")!=std::string::npos &&
        r.message.find("expected_drag=0")!=std::string::npos && r.message.find("disabled/standstill")==std::string::npos,
        "drag-exit error did not identify its expected state and failed phase");++cases;
  }
  {
    SafetyRig rig;rig.device.drag_exit_delay=30ms;rig.start();
    auto r=rig.gate.setTeachMode(commands(),true,no);require(r.success(),r.message.c_str());
    r=rig.gate.setTeachMode(commands(),false,[&]{return rig.device.exit_ack_frames.load()>=4;});
    require(!r.success() && r.command_issued && rig.device.joint_writes==0 && rig.gate.teachingMask()==3 &&
        rig.monitor->latched(),"canceled drag exit still sent hold/mode commands or released fence");++cases;
  }
  {
    SafetyRig rig;rig.device.ignore_position_mode=true;rig.start();
    auto r=rig.gate.setTeachMode(commands(),true,no);require(r.success(),r.message.c_str());
    r=rig.gate.setTeachMode(commands(),false,no);
    require(!r.success() && r.command_issued && rig.monitor->latched() && rig.gate.teachingMask()==3 &&
        r.message.find("teach OFF position confirmation")!=std::string::npos &&
        r.message.find("expected_state=1")!=std::string::npos && r.message.find("state=3 drag=0")!=std::string::npos &&
        r.message.find("disabled/standstill")==std::string::npos,
        "position-mode refusal was hidden or mislabeled as disable failure");++cases;
  }
  {
    SafetyRig rig;rig.start();auto r=rig.gate.setTeachMode(commands(),true,no);require(r.success(),r.message.c_str());
    rig.monitor->requestStop("explicit faulted exit test");
    r=rig.gate.setTeachMode(commands(),false,no);require(r.success(),r.message.c_str());
    require(rig.device.joint_writes==0 && rig.device.state.arms[0].current_state==0 &&
        rig.device.state.arms[1].current_state==0 && rig.gate.teachingMask()==0 && rig.monitor->latched(),
        "faulted OFF sent hold targets or implicitly recovered the fault");++cases;
  }
  {
    Device d;SdkCommandGate gate(d,ActiveArmConfiguration::kDualArm,options());
    require(gate.teachingMask()==0 && static_cast<bool>(gate.sendJointPosition(ArmChannel::kLeft,{})),
      "default-OFF teaching changed ordinary trajectory behavior");
    auto r=gate.setTeachMode(commands(),true,no);require(r.success(),r.message.c_str());
    require(gate.teachingMask()==3 && d.clears==0,"teaching must not auto-clear faults");++cases;
    auto sent=gate.sendJointPosition(ArmChannel::kLeft,{});require(!sent,"trajectory admitted while teaching");++cases;
    d.state.arms[0].joint_position.fill(32.);
    r=gate.setTeachMode(commands(),false,no);require(r.success(),r.message.c_str());
    require(gate.teachingMask()==0 && d.commanded[0][0]==32. && d.state.arms[0].current_state==1,
      "teach exit restored stale command instead of measured pose");++cases;
  }
  for(int bad=0;bad<4;++bad) {
    Device d;SdkCommandGate gate(d,ActiveArmConfiguration::kDualArm,options());
    if(bad==0)d.advance=false;
    if(bad==1)d.state.arms[0].tip_di=1;
    if(bad==2)d.state.arms[0].joint_velocity[2]=20.;
    if(bad==3)d.state.arms[0].error_code=7;
    const auto r=gate.setTeachMode(commands(),true,no);
    require(!r.success() && d.commits==0,"unsafe teach preflight changed controller");++cases;
  }
  for(int scenario=0;scenario<4;++scenario) {
    Device d;auto o=options();o.teach_transition_timeout=600ms;
    SdkCommandGate gate(d,ActiveArmConfiguration::kDualArm,o);
    auto r=gate.setTeachMode(commands(),true,no);require(r.success(),r.message.c_str());
    const int commits_before=d.commits.load();
    unsigned reads_since_cancel=0,injected=0;
    DriveClock::time_point rejected_at{},first_requalified{};
    // A window polls cancellation before each read; its final held snapshot is
    // a second read. Inject there to reproduce feedback changing after a valid
    // stationary window, without issuing any extra controller commands.
    d.feedback_hook=[&](DualArmDeviceState& state) {
      ++reads_since_cancel;
      if(reads_since_cancel>1 && (!injected || scenario==2)) {
        ++injected;rejected_at=DriveClock::now();
        if(scenario==1)state.arms[0].joint_velocity[0]=.04/0.017453292519943295;
        else state.arms[0].low_speed=false;
      } else if(injected && first_requalified==DriveClock::time_point{}) {
        first_requalified=DriveClock::now();
      }
    };
    const auto began=DriveClock::now();
    r=gate.setTeachMode(commands(),false,[&] {
      reads_since_cancel=0;return scenario==3 && injected>0;
    });
    const auto elapsed=DriveClock::now()-began;
    require(injected>0,"post-window feedback transient was not exercised");
    if(scenario<2) {
      require(r.success(),("post-window transient rejected before deadline: "+r.message).c_str());
      require(d.commits==commits_before+2 && d.joint_writes==2 && gate.teachingMask()==0 &&
          first_requalified>=rejected_at && elapsed>=300ms,
          "transient retry duplicated commands or skipped a fresh stationary window");
    } else {
      require(!r.success() && !r.command_issued && d.commits==commits_before &&
          d.joint_writes==0 && gate.teachingMask()==3,
          "persistent/canceled post-window feedback issued a command or lost teaching fence");
      if(scenario==2)require(elapsed>=600ms && elapsed<1500ms &&
          r.message.find("low_speed=0")!=std::string::npos,
          "post-window retry was unbounded or omitted rejection feedback");
    }
    ++cases;
  }
  {
    Device d;d.echo_drag=false;SdkCommandGate gate(d,ActiveArmConfiguration::kDualArm,options());
    auto r=gate.setTeachMode(commands(),true,no);require(!r.success() && r.command_issued && gate.teachingMask()==3,
      "missing drag echo treated as success or unfenced");++cases;
  }
  {
    Device d;SdkCommandGate gate(d,ActiveArmConfiguration::kDualArm,options());auto c=commands();c[1].channel=c[0].channel;
    require(!gate.setTeachMode(c,true,no).success() && d.commits==0,"duplicate arm changed hardware");++cases;
    require(!gate.setTeachMode(commands(),true,[]{return true;}).success() && d.commits==0,"canceled teaching wrote hardware");++cases;
  }
  {
    Device d;for(auto& a:d.state.arms){a.current_state=0;a.drag_space=1;}
    SdkCommandGate gate(d,ActiveArmConfiguration::kDualArm,options());
    auto r=gate.recoverPositionMode([](const auto&,auto&){return true;},no);
    require(r.success() && d.commanded[0][0]==12. && d.state.arms[0].drag_space==0,
      "recovery did not enable at measured position and disarm drag");++cases;
  }
  {
    Device d;for(auto& a:d.state.arms){a.current_state=0;a.drag_space=0;}
    SdkCommandGate gate(d,ActiveArmConfiguration::kDualArm,options());
    auto r=gate.recoverPositionMode([](const auto&,auto& error){error="URDF rejected";return false;},no);
    require(!r.success() && d.commits==0,"invalid recovery geometry enabled arm");++cases;
  }
  {
    Device d;for(auto& a:d.state.arms){a.current_state=100;a.error_code=12;}
    SdkCommandGate gate(d,ActiveArmConfiguration::kDualArm,options());
    auto r=gate.controlDrives(true,true,false,true);
    require(r.success() && d.clears==0,"faulted stationary arms cannot reach explicit reset phase");++cases;
  }
  {
    HeadBus bus;
    require(!bus.maintenance.releaseAfterConfirmedClear(),"head override released without confirmed clear");
    auto r=bus.maintenance.control(false,true);require(r.success(),r.message.c_str());
    r=bus.maintenance.control(true);require(r.success(),r.message.c_str());
    require(bus.resets>0 && bus.wrong_axis==0,"head clear missed head or touched non-head drives");
    require(bus.maintenance.releaseAfterConfirmedClear(),"confirmed head clear cannot recover");
    require(!bus.maintenance.releaseAfterConfirmedClear(),"head release is repeatable without another clear");++cases;
  }
  {
    Device d;SdkCommandGate gate(d,ActiveArmConfiguration::kDualArm,options());
    auto r=gate.controlDrives(true,true,true,false,[]{return true;});
    require(!r.success() && d.clears==0,"canceled recovery still cleared arm faults");++cases;
  }
  for(int scenario=0;scenario<6;++scenario) {
    std::array<int,3> stops{},clears{};int enables=0,resets=0;bool canceled=false;
    RobotRecoveryHooks h;h.canceled=[&]{return canceled;};
    for(unsigned i=0;i<3;++i)h.drives[i]=[&,i](bool clear){
      (clear?clears[i]:stops[i])++;
      return DriveControlResult{true,!(clear && scenario==1 && i==1),"checked"};
    };
    h.enable_arms=[&]{++enables;if(scenario==3)canceled=true;return DriveControlResult{true,true,"arm"};};
    h.enable_body=[&]{++enables;return DriveControlResult{true,scenario!=2,"body"};};
    h.reset_latch=[&](auto&){++resets;return scenario!=5;};
    const auto r=recoverRobot(h,scenario!=4);
    require(clears==std::array<int,3>{1,1,1},"not every body group was cleared");
    if(scenario==0) require(r.success && r.motion_ready && enables==2 && resets==1,"full recovery failed");
    if(scenario==1) require(!r.success && enables==0 && r.head_cleared,"partial clear enabled drives/skipped head");
    if(scenario==2 || scenario==3)require(!r.success && resets==0 && stops==std::array<int,3>{2,2,2},"failed enable did not disable all groups");
    if(scenario==4)require(r.success && !r.motion_ready && enables==0 && resets==0,"clear-only enabled/reset robot");
    if(scenario==5)require(!r.success && !r.motion_ready && !r.drives_enabled && resets==1 &&
      stops==std::array<int,3>{2,2,2},"failed stationary qualification did not disable all groups");
    ++cases;
  }
  {
    MotionSafety monitor;monitor.addChannel(kHeadSafety,{{"waist_test",0,175.*3.141592653589793/180.,1}},
      [](auto&,auto&){return false;});
    SafetySample sample;sample.sequence=1;sample.acquired=SafetyClock::now();sample.position={0};sample.velocity={0};std::string error;
    require(!monitor.validateRecoverySample(kHeadSafety,sample,error),"unfenced recovery accepted");
    monitor.requestStop("test");require(monitor.validateRecoverySample(kHeadSafety,sample,error),"valid recovery rejected");
    sample.position={3.141592653589793};require(!monitor.validateRecoverySample(kHeadSafety,sample,error),"180 degrees exceeded 175 degree limit");
    sample.position={0};sample.acquired-=1s;require(!monitor.validateRecoverySample(kHeadSafety,sample,error),"stale recovery accepted");
    auto permit=monitor.command(kHeadSafety,error);require(!permit,"recovery validation released ordinary motion fence");++cases;
  }
  {
    SafetyRig rig;rig.device.transition_delay=140ms;rig.start();
    auto r=rig.gate.setTeachMode(commands(),true,no);require(r.success(),r.message.c_str());
    const int commits=rig.device.commits.load();
    r=rig.gate.setTeachMode(commands(),true,no);
    require(!r.success() && !r.command_issued && rig.device.commits==commits && !rig.monitor->latched(),
      "duplicate ON changed hardware or latched a fault");++cases;
    r=rig.gate.setTeachMode(commands(),false,no);require(r.success(),r.message.c_str());
    require(rig.device.seen_101>0 && rig.device.seen_103>0 && !rig.monitor->latched() && rig.gate.teachingMask()==0,
      "normal 103/101 transitions failed or remained fenced");
    ArmFeedback transition;transition.current_state=101;
    require(!rig.gate.expectedModeTransition(ArmChannel::kLeft,transition),"transition scope leaked after OFF");++cases;
  }
  for(int bad=0;bad<5;++bad) {
    SafetyRig rig;rig.device.transition_delay=140ms;
    if(bad==0)rig.device.stuck_target=1;
    if(bad==1)rig.device.fault_target=1;
    if(bad==2)rig.device.invalid_position_target=1;
    if(bad==3)rig.device.frozen_target=1;
    if(bad==4)rig.device.wrong_state_target=1;
    rig.start();auto r=rig.gate.setTeachMode(commands(),true,no);require(r.success(),r.message.c_str());
    r=rig.gate.setTeachMode(commands(),false,no);
    require(!r.success() && r.command_issued && rig.gate.teachingMask()==3 && rig.monitor->latched(),
      "stuck/faulted/out-of-bounds/stale/wrong transition escaped protection");
    ArmFeedback transition;transition.current_state=101;
    require(!rig.gate.expectedModeTransition(ArmChannel::kLeft,transition),"failed transition scope leaked");++cases;
  }
  {
    SafetyRig rig;rig.device.transition_delay=140ms;rig.start();
    auto r=rig.gate.setTeachMode(commands(),true,no);require(r.success(),r.message.c_str());
    const auto now=DriveClock::now();
    // SDK preflight can settle before the independent monitor. Change the
    // measured pose during that monitor wait, then require the new hold.
    rig.force_moving_until_ns=std::chrono::duration_cast<std::chrono::nanoseconds>((now+250ms).time_since_epoch()).count();
    rig.device.position_change_ns=std::chrono::duration_cast<std::chrono::nanoseconds>((now+150ms).time_since_epoch()).count();
    r=rig.gate.setTeachMode(commands(),false,no);require(r.success(),r.message.c_str());
    require(rig.device.commanded[0][0]==33. && rig.device.commanded[1][0]==33.,
      "teach OFF sent the stale snapshot captured before the stationary wait");++cases;
  }
  {
    SafetyRig rig;rig.device.transition_delay=200ms;rig.start();auto one=commands();one.resize(1);
    auto future=std::async(std::launch::async,[&]{return rig.gate.setTeachMode(one,true,no);});
    const auto end=DriveClock::now()+1s;
    while(rig.device.seen_103==0 && DriveClock::now()<end)std::this_thread::sleep_for(2ms);
    require(rig.device.seen_103>0,"delayed torque transition was not observed");
    ArmFeedback transition;transition.current_state=103;
    require(!rig.gate.expectedModeTransition(ArmChannel::kRight,transition),"transition allowance reached unselected arm");
    transition.error_code=7;
    require(!rig.gate.expectedModeTransition(ArmChannel::kLeft,transition),"error accepted during expected transition");
    require(!rig.gate.sendJointPosition(ArmChannel::kRight,{}) && rig.device.joint_writes==0,
      "ordinary joint writes admitted during teaching transition");
    require(!rig.gate.configureControlModes(commands()),"foreign mode change admitted during teaching transition");
    const auto exit=rig.gate.setTeachMode(one,false,no);
    require(!exit.success() && !exit.command_issued,
      "foreign OFF sent drag-exit during another caller's mode confirmation");++cases;
    const auto r=future.get();require(r.success(),r.message.c_str());++cases;
  }
  {
    MotionSafety monitor;std::uint64_t sequence=0;
    monitor.addChannel(kLeftSafety,{{"wait_joint",-1.,1.,1.}},[&](SafetySample& sample,std::string&) {
      sample.position={0};sample.velocity={0};sample.sequence=++sequence;sample.acquired=SafetyClock::now();return true;
    });
    monitor.poll();std::string error;
    {auto early=monitor.configuration(kLeftSafety,error);require(!early,"unsettled monitor unexpectedly admitted configuration");}
    monitor.start();
    {auto settled=monitor.configurationWhenStationary(kLeftSafety,error,no,400ms);
      require(static_cast<bool>(settled),error.c_str());}
    {auto canceled=monitor.configurationWhenStationary(kLeftSafety,error,[]{return true;},400ms);
      require(!canceled && error.find("canceled")!=std::string::npos,"standstill wait ignored cancellation");}
    monitor.stop();require(!monitor.latched(),"waiting/rejection latched a fault without a command");++cases;
  }
  {
    SafetyRig rig;rig.device.transition_delay=140ms;rig.start();rig.monitor->requestStop("explicit test recovery");
    auto r=rig.gate.controlDrives(true,true,false,true);require(r.success(),r.message.c_str());
    const auto validate=[&](const DualArmDeviceState& state,std::string& error) {
      for(unsigned i=0;i<2;++i) {
        const auto& a=state.arms[i];
        if(a.error_code || (a.current_state!=0 && a.current_state!=1 &&
          !rig.gate.expectedModeTransition(static_cast<ArmChannel>(i),a)))return false;
        SafetySample sample;sample.sequence=a.feedback_sequence;sample.acquired=a.acquired;
        for(unsigned j=0;j<7;++j) {
          sample.position.push_back(a.joint_position[j]*0.017453292519943295);
          sample.velocity.push_back(a.joint_velocity[j]*0.017453292519943295);
        }
        if(!rig.monitor->validateRecoverySample(1U<<i,sample,error))return false;
      }
      return true;
    };
    r=rig.gate.recoverPositionMode(validate,no);require(r.success(),r.message.c_str());
    require(rig.device.seen_101>0 && rig.monitor->latched(),"recovery mishandled transition or reset latch implicitly");++cases;
  }
  {
    MotionSafety monitor;bool enabled=true;std::uint64_t sequence=482060;SafetySample latest;
    monitor.addChannel(kLeftSafety,{{"recovery_joint",-1.,1.,1.}},[&](SafetySample& sample,std::string& error) {
      if(!enabled) {error="arm disabled";return false;}
      sample.position={.2};sample.velocity={0};sample.sequence=sequence++;
      sample.acquired=SafetyClock::now();sample.wraps_at_million=true;latest=sample;return true;
    });
    std::string error;
    require(!monitor.beginRecoveryFeedbackQualification(error),"unfenced feedback requalification accepted");
    monitor.poll();monitor.requestStop("explicit recovery");enabled=false;monitor.poll();
    enabled=true;sequence=143303;monitor.poll();
    require(monitor.validateRecoverySample(kLeftSafety,latest,error),"fresh recovery hold rejected");
    require(!monitor.reset(error),"missed wrap was accepted without explicit requalification");
    monitor.enterExecution();
    require(!monitor.beginRecoveryFeedbackQualification(error),"active worker requalified feedback");
    monitor.leaveExecution();
    require(monitor.beginRecoveryFeedbackQualification(error),error.c_str());
    require(monitor.latched() && !monitor.reset(error),"requalification released fence before new samples");
    {auto permit=monitor.command(kLeftSafety,error);require(!permit,"requalification admitted commands");}
    for(int i=0;i<6;++i) {monitor.poll();if(i<5)std::this_thread::sleep_for(25ms);}
    require(monitor.reset(error),error.c_str());require(monitor.ready(error),error.c_str());
    std::cout << "PASS recovery after missed frame wrap requires new stationary window\n";++cases;
  }
  // Requalification is never permission to accept disabled, stale, frozen,
  // moving, out-of-range or reversing feedback.
  for(int bad=0;bad<6;++bad) {
    MotionSafety monitor;std::uint64_t sequence=143303;
    const auto old_acquisition=SafetyClock::now();
    monitor.addChannel(kLeftSafety,{{"recovery_joint",-1.,1.,1.}},[&](SafetySample& sample,std::string& error) {
      if(bad==0) {error="arm disabled";return false;}
      sample.position={bad==4?2.:.2};sample.velocity={bad==3?.02:0.};
      sample.sequence=sequence;sample.wraps_at_million=true;
      if(bad==5)--sequence;else if(bad!=2)++sequence;
      sample.acquired=bad==1?old_acquisition:SafetyClock::now();return true;
    });
    monitor.requestStop("explicit recovery");std::string error;
    require(monitor.beginRecoveryFeedbackQualification(error),error.c_str());
    for(int i=0;i<6;++i) {monitor.poll();if(i<5)std::this_thread::sleep_for(25ms);}
    require(!monitor.reset(error) && monitor.latched(),"invalid recovery feedback released fence");
    require(error.find("channel=0")!=std::string::npos && error.find("stationary_frames=")!=std::string::npos,
      "standstill failure lacks channel diagnostics");++cases;
  }
  {
    MotionSafety monitor;std::promise<void> acquired,release;
    auto released=release.get_future();
    monitor.addChannel(kLeftSafety,{{"recovery_joint",-1.,1.,1.}},[&](SafetySample& sample,std::string&) {
      sample.position={.2};sample.velocity={0};sample.sequence=482060;
      sample.acquired=SafetyClock::now();sample.wraps_at_million=true;
      acquired.set_value();released.wait();return true;
    });
    monitor.requestStop("explicit recovery");
    auto pending=std::async(std::launch::async,[&]{monitor.poll();});acquired.get_future().wait();
    std::string error;const bool began=monitor.beginRecoveryFeedbackQualification(error);
    release.set_value();pending.get();require(began,error.c_str());
    require(!monitor.reset(error) && error.find("fresh=0 sequence=0")!=std::string::npos,
      "in-flight pre-recovery acquisition seeded new frame history");++cases;
  }
  for(int scenario=0;scenario<8;++scenario) {
    SafetyRig rig;rig.device.transition_delay=140ms;rig.device.enable_velocity_rad_s=.02;
    if(scenario==2)rig.device.persistent_enable_velocity=true;
    if(scenario==3)rig.device.enable_velocity_rad_s=3.;
    if(scenario==4)rig.device.fault_target=1;
    if(scenario==5)rig.device.invalid_position_target=1;
    if(scenario==6)rig.device.enable_displacement_deg=1.;
    if(scenario==7)rig.device.frozen_target=1;
    rig.start();rig.monitor->requestStop("explicit recovery settling test");
    auto r=rig.gate.controlDrives(true,true,false,true);require(r.success(),r.message.c_str());
    const auto validate=[&](const DualArmDeviceState& state,std::string& error,bool stationary) {
      for(unsigned i=0;i<2;++i) {
        const auto& a=state.arms[i];
        if(a.error_code || (a.current_state!=0 && a.current_state!=1 &&
          !rig.gate.expectedModeTransition(static_cast<ArmChannel>(i),a)))return false;
        SafetySample sample;sample.sequence=a.feedback_sequence;sample.acquired=a.acquired;
        for(unsigned j=0;j<7;++j) {
          sample.position.push_back(a.joint_position[j]*0.017453292519943295);
          sample.velocity.push_back(a.joint_velocity[j]*0.017453292519943295);
        }
        if(!rig.monitor->validateRecoverySample(1U<<i,sample,error,stationary))return false;
      }
      return true;
    };
    const auto stationary=[&](const auto& state,auto& error){return validate(state,error,true);};
    const auto settling=[&](const auto& state,auto& error){return validate(state,error,false);};
    // Scenario zero reproduces the old strict callback: a short enable transient
    // fails immediately. Only scenario one can settle and succeed with the fix.
    r=scenario==0?rig.gate.recoverPositionMode(stationary,no):
      rig.gate.recoverPositionMode(stationary,no,settling);
    require(r.command_issued && r.success()==(scenario==1),"recovery settling validation result incorrect");
    require(rig.monitor->latched(),"arm enable released whole-robot software fence");++cases;
  }
  {
    MotionSafety monitor;monitor.addChannel(kLeftSafety,{{"recovery_joint",-1.,1.,1.}},
      [](auto&,auto&){return false;});monitor.requestStop("explicit recovery");
    SafetySample sample;sample.position={.2};sample.velocity={.02};sample.sequence=1;
    sample.acquired=SafetyClock::now();std::string error;
    require(!monitor.validateRecoverySample(kLeftSafety,sample,error),"moving pre-enable hold accepted");
    require(monitor.validateRecoverySample(kLeftSafety,sample,error,false),"bounded settling sample rejected");
    sample.velocity={2.};
    require(!monitor.validateRecoverySample(kLeftSafety,sample,error,false),"URDF overspeed accepted during settling");
    require(error.find("velocity_rad_s=")!=std::string::npos,"recovery rejection lacks measured velocity");++cases;
  }
  std::cout << "teach/recovery cases=" << cases << " PASS\n";return 0;
} catch(const std::exception& e) {std::cerr << e.what()<<'\n';return 1;}
