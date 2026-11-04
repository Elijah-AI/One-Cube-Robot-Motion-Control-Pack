#include "motion_control_pkg/execution/shutdown_request.hpp"
#include "motion_control_pkg/sdk/sdk_command_gate.hpp"
#include "motion_control_pkg/sdk/ethercat_drive_maintenance.hpp"
#include <iostream>
#include <stdexcept>
using namespace motion_control_pkg;
using namespace sdk;
static void check(bool b,const char* m){if(!b)throw std::runtime_error(m);}
struct Device: DualArmDeviceInterface {
  int batches=0,commits=0,offs=0,motions=0,releases=0,reads=0;
  bool off_ok=true;
  bool checkCompatibility() override{return true;}
  bool connect(const std::array<unsigned char,4>&) override{return true;}
  bool release() override{++releases;return true;}
  void disableLogging() override{}
  bool readState(DualArmDeviceState& s) override{++reads;for(auto& a:s.arms){a.feedback_sequence=reads;a.current_state=0;a.low_speed=true;}return true;}
  bool beginCommandBatch() override{++batches;return true;}
  bool commitCommandBatch() override{++commits;return true;}
  bool clearServoError(ArmChannel) override{++motions;return true;}
  bool setServoOff(ArmChannel) override{++offs;return off_ok;}
  bool setPositionMode(ArmChannel) override{++motions;return true;}
  bool setTorqueMode(ArmChannel) override{++motions;return true;}
  bool setJointLimits(ArmChannel,int,int) override{return true;}
  bool setTool(ArmChannel,const ToolParameters&) override{return true;}
  bool setJointImpedanceGains(ArmChannel,const std::array<double,7>&,const std::array<double,7>&) override{return true;}
  bool setCartesianImpedanceGains(ArmChannel,const std::array<double,7>&,const std::array<double,7>&) override{return true;}
  bool setImpedanceType(ArmChannel,ArmControlMode) override{return true;}
  bool setJointPosition(ArmChannel,const std::array<double,7>&) override{++motions;return true;}
};
int main() try {
  Device d;SdkCommandGate gate(d,ActiveArmConfiguration::kDualArm);
  check(!gate.disableForShutdown(),"servo-off available without shutdown");
  execution::installShutdownSignalHandlers();
  raise(SIGINT);raise(SIGINT);raise(SIGTERM);
  check(execution::shutdownRequested(),"signal latch failed");
  check(!gate.connect({0,0,0,0}),"new connection accepted");
  check(!gate.initializePositionMode(),"initialization accepted");
  ArmControlModeCommand command;command.channel=ArmChannel::kLeft;
  check(!gate.configureControlModes({command}),"mode accepted");
  std::array<double,7> joints{};
  check(!gate.sendJointPosition(ArmChannel::kLeft,joints),"position accepted");
  check(!gate.controlDrives(true,true,true).command_issued,"clear accepted");
  check(d.motions==0 && d.batches==0,"normal command reached driver");
  check(static_cast<bool>(gate.disableForShutdown()),"servo-off failed");
  check(d.offs==2 && d.commits==1,"servo-off not one two-arm transaction");
  std::vector<DriveAxis> axes;std::string detail;
  check(gate.readShutdownFeedback(axes,detail) && axes.size()==14,"passive arm feedback unavailable");
  check(static_cast<bool>(gate.release()) && static_cast<bool>(gate.release()) && d.releases==1,"release not idempotent");
  const int reads=d.reads;axes.clear();
  check(!gate.readShutdownFeedback(axes,detail) && reads==d.reads,"read after release reached driver");
  Device bad;bad.off_ok=false;SdkCommandGate failed(bad,ActiveArmConfiguration::kDualArm);
  check(!failed.disableForShutdown() && !bad.commits,"partly formed disable batch committed");

  EthercatDriveMaintenance mailbox({29,28,27,26,31,30});
  std::vector<OnecubeEthercatFeedbackV1> frames(32);
  std::vector<DriverSDK::motorTargetStruct> targets(32);
  std::vector<bool> ready(32,true);
  for(auto& t:targets)t.enabled=1;
  const auto refresh=[&]{for(auto& f:frames){f.status=0x21;f.sequence++;f.acquired_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(DriveClock::now().time_since_epoch()).count();}};
  refresh();mailbox.update(frames,true,targets,ready);
  axes.clear();check(mailbox.readShutdownFeedback(axes,detail,false) && axes.size()==6,"six-axis passive read");
  check(!mailbox.control(true).command_issued,"clear admitted during shutdown");
  mailbox.requestShutdownDisable();
  auto ticket=mailbox.update(frames,true,targets,ready);
  for(int i=0;i<32;++i)check(targets[i].enabled==(i>=26?0:1),"wrong shutdown motor mapping");
  mailbox.written(ticket,false);axes.clear();
  check(!mailbox.readShutdownFeedback(axes,detail,true),"failed PDO write confirmed");
  mailbox.written(ticket,true);axes.clear();
  check(!mailbox.readShutdownFeedback(axes,detail,true),"pre-command PDO confirmed");
  refresh();mailbox.update(frames,true,targets,ready);axes.clear();
  check(mailbox.readShutdownFeedback(axes,detail,true) && axes.size()==6,"post-write PDO unavailable");
  mailbox.update(frames,false,targets,ready);axes.clear();
  check(!mailbox.readShutdownFeedback(axes,detail,true),"invalid bus accepted");
  std::cout<<"PASS signal idempotence, SDK command fence/release, six-axis disable/write/PDO guards; no hardware\n";
} catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
