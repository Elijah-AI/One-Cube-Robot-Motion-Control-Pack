#include "motion_control_pkg/sdk/sdk_command_gate.hpp"
#include "motion_control_pkg/sdk/ethercat_drive_maintenance.hpp"
#include <atomic>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace motion_control_pkg::sdk;
using namespace std::chrono_literals;
static void check(bool ok,const char* reason) { if (!ok) throw std::runtime_error(reason); }

struct ArmDevice : DualArmDeviceInterface {
  DualArmDeviceState state;
  std::atomic<int> reads{0}, clears{0}, modes{0}, batches{0}, stops{0};
  bool advance{true}, stop_ok{true}, clear_ok{true};
  bool ignore_stop=false,off_ok=true,commit_ok=true;
  std::array<bool,2> pending_off{{false,false}};
  std::atomic<int> servo_off{0};
  bool servo_read_ok{true};
  long servo_error{0};
  bool selected_left{},selected_right{};
  ArmDevice() { for (auto& a:state.arms) { a.feedback_sequence=1; a.low_speed=true; } }
  bool checkCompatibility() override { return true; }
  bool connect(const std::array<unsigned char,4>&) override { return true; }
  bool release() override { return true; }
  void disableLogging() override {}
  bool readState(DualArmDeviceState& out) override {
    ++reads;
    if (advance) for (auto& a:state.arms) ++a.feedback_sequence;
    out=state; return true;
  }
  bool emergencyStop(bool left,bool right) override {
    ++stops; selected_left=left; selected_right=right;
    for (int i=0;i<2;++i) if (!ignore_stop && (i==0 ? left : right)) state.arms[i].current_state=0;
    return stop_ok;
  }
  bool readServoError(ArmChannel,std::size_t,long& error) override {
    error=servo_error; return servo_read_ok;
  }
  bool beginCommandBatch() override { ++batches;pending_off.fill(false);return true; }
  bool setServoOff(ArmChannel channel) override {
    ++servo_off;pending_off[static_cast<unsigned>(channel)]=true;return off_ok;
  }
  bool clearServoError(ArmChannel channel) override {
    ++clears;
    if (clear_ok) { auto& a=state.arms[static_cast<int>(channel)]; a.error_code=0; a.current_state=0; }
    return true;
  }
  bool setPositionMode(ArmChannel) override { ++modes; return true; }
  bool setTorqueMode(ArmChannel) override { ++modes; return true; }
  bool setJointLimits(ArmChannel,int,int) override { return true; }
  bool setTool(ArmChannel,const ToolParameters&) override { return true; }
  bool setJointImpedanceGains(ArmChannel,const std::array<double,7>&,const std::array<double,7>&) override {return true;}
  bool setCartesianImpedanceGains(ArmChannel,const std::array<double,7>&,const std::array<double,7>&) override {return true;}
  bool setImpedanceType(ArmChannel,ArmControlMode) override {return true;}
  bool setJointPosition(ArmChannel,const std::array<double,7>&) override { ++modes; return true; }
  bool commitCommandBatch() override {
    ++batches;
    if(commit_ok)for(unsigned i=0;i<2;++i)if(pending_off[i])state.arms[i].current_state=0;
    return commit_ok;
  }
};

struct Bus {
  EthercatDriveMaintenance maintenance;
  std::atomic<bool> quit{false};
  std::atomic<int> resets{0},disables{0},unsafe{0},writes{0};
  bool fault,valid,write_ok;
  std::thread worker;
  Bus(bool f=false,bool v=true,bool w=true) : fault(f),valid(v),write_ok(w),worker([this]{run();}) {}
  ~Bus() { quit=true; worker.join(); }
  void run() {
    std::vector<OnecubeEthercatFeedbackV1> frames(32);
    std::vector<DriverSDK::motorTargetStruct> targets(32);
    std::vector<bool> ready(32,true);
    for (auto& f:frames) { f.position=1.; f.status=fault?8:0x37; f.error=fault?12:0; }
    for (auto& t:targets) t.enabled=1;
    while (!quit) {
      const auto now=DriveClock::now().time_since_epoch();
      for (auto& f:frames) {
        ++f.sequence; f.acquired_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
      }
      auto ticket=maintenance.update(frames,valid,targets,ready);
      if (targets[30].enabled!=1 || targets[31].enabled!=1) ++unsafe;
      for (int id:EthercatDriveMaintenance::ids) {
        if (ticket.epoch && targets[id].enabled==1) ++unsafe;
        if (targets[id].enabled==-1) ++resets;
        if (targets[id].enabled==0) ++disables;
        if (write_ok && targets[id].enabled!=1) {
          frames[id].status=0x21;
          if (targets[id].enabled==-1) frames[id].error=0;
        }
      }
      maintenance.written(ticket,write_ok); ++writes;
      std::this_thread::sleep_for(4ms);
    }
  }
};

static void until(const std::function<bool()>& done) {
  const auto deadline=DriveClock::now()+2s;
  while (!done() && DriveClock::now()<deadline) std::this_thread::sleep_for(1ms);
  check(done(),"test synchronization timeout");
}

int main() try {
  int cases=0;
  const auto base=DriveClock::now();
  const DriveAxis axis{0,0,1,base,true,false,true,false};
  {
    DriveFeedbackWindow teaching(base,100ms,
        motion_control_pkg::execution::kTeachStandstillVelocity,
        motion_control_pkg::execution::kTeachStandstillDisplacement);
    DriveFeedbackWindow ordinary(base);
    bool confirmed=false;
    for(int i=0;i<=5;++i) {
      auto a=axis;a.sequence+=i;a.acquired=base+i*20ms;
      a.velocity=.03;a.position=i*.01;
      confirmed=teaching.update({a},true,false,a.acquired);
      check(!ordinary.update({a},true,false,a.acquired),"teaching limits leaked into normal drive window");
      if(i<5)check(!confirmed,"teaching accepted before 100 ms");
    }
    check(confirmed,"teaching rejected .03 rad/s or .05 rad window boundary");++cases;
  }
  for(int bad=0;bad<4;++bad) {
    DriveFeedbackWindow teaching(base,100ms,.03,.05);
    for(int i=0;i<12;++i) {
      auto a=axis;a.sequence+=i;a.acquired=base+i*20ms;
      if(bad==0)a.velocity=.030001;
      if(bad==1)a.position=i*.0101;
      if(bad==2)a.disabled=false;
      if(bad==3){a.sequence=axis.sequence;a.acquired=base;}
      check(!teaching.update({a},true,false,base+i*20ms),"teaching accepted excess speed/displacement or unqualified frames");
    }
    ++cases;
  }
  {
    DriveFeedbackWindow window(base);
    std::vector<DriveAxis> axes{axis};
    for(int i=0;i<30;++i) check(!window.update(axes,false,false,base+1ms),"repeated frame accepted");
    ++cases;
  }
  for (int bad=0;bad<7;++bad) {
    DriveFeedbackWindow window(base);
    for (int i=0;i<12;++i) {
      auto a=axis; a.sequence+=i; a.acquired=base+i*20ms;
      if(bad==0) a.velocity=.1;
      if(bad==1) a.position=std::numeric_limits<double>::quiet_NaN();
      if(bad==2) a.disabled=false;
      if(bad==3) a.fault=true;
      if(bad==4) a.position=i*.002;  // moving despite zero reported velocity
      if(bad==5) a.acquired-=200ms;
      if(bad==6) a.acquired+=20ms;
      check(!window.update({a},true,false,base+i*20ms),"invalid feedback accepted");
    }
    ++cases;
  }
  {
    DriveFeedbackWindow window(base);
    bool confirmed=false;
    for (int i=0;i<7;++i) {
      auto a=axis; a.sequence+=i; a.acquired=base+i*20ms;
      confirmed=window.update({a},true,false,a.acquired);
    }
    check(confirmed,"fresh disabled feedback not accepted"); ++cases;
  }
  for (int selection=0;selection<3;++selection) {
    ArmDevice device;
    SdkCommandGate gate(device,ActiveArmConfiguration::kDualArm);
    const bool left=selection!=1,right=selection!=0;
    auto r=gate.controlDrives(left,right,false);
    check(r.success() && device.selected_left==left && device.selected_right==right,"arm routing/stop failed");
    check(device.modes==0 && device.batches==0 && device.clears==0,"stop nested/changed modes");
    ++cases;
  }
  for (int selection=0;selection<3;++selection) {
    ArmDevice device;device.ignore_stop=true;
    for(auto& a:device.state.arms)a.current_state=1;
    SdkCommandGate gate(device,ActiveArmConfiguration::kDualArm);
    const bool left=selection!=1,right=selection!=0;
    const auto r=gate.controlDrives(left,right,false,true);
    check(r.success() && device.stops==1 && device.servo_off==int(left)+int(right),
      "recovery stop fallback did not confirm selected arms disabled");
    for(unsigned i=0;i<2;++i)check(device.state.arms[i].current_state==((i==0?left:right)?0:1),
      "recovery stop fallback changed unselected arm");
    check(device.modes==0 && device.clears==0 && r.message.find("checked servo-off fallback")!=std::string::npos,
      "fallback sent a position/enable/clear command or hid its diagnostic");++cases;
  }
  for (int bad=0;bad<3;++bad) {
    ArmDevice device;device.ignore_stop=true;
    for(auto& a:device.state.arms)a.current_state=1;
    if(bad==0)device.off_ok=false;
    if(bad==1)device.commit_ok=false;
    if(bad==2)device.advance=false;
    SdkCommandGate gate(device,ActiveArmConfiguration::kDualArm);
    const auto r=gate.controlDrives(true,true,false,true);
    check(r.command_issued && !r.feedback_confirmed && device.modes==0 && device.clears==0,
      "failed or unconfirmed fallback claimed success");++cases;
  }
  {
    ArmDevice device; device.state.arms[0].current_state=100; device.state.arms[0].error_code=13;
    SdkCommandGate gate(device,ActiveArmConfiguration::kDualArm);
    check(gate.controlDrives(true,false,true).success(),"arm clear failed");
    check(device.clears==1 && device.modes==0 && device.batches==0,"clear sent motion/empty batch");
    ++cases;
  }
  {
    ArmDevice device; device.advance=false;
    SdkCommandGate gate(device,ActiveArmConfiguration::kDualArm);
    auto r=gate.controlDrives(true,true,false);
    check(r.command_issued && !r.feedback_confirmed,"stale arm frame acknowledged"); ++cases;
  }
  {
    ArmDevice device; device.state.arms[0].current_state=1;
    SdkCommandGate gate(device,ActiveArmConfiguration::kDualArm);
    check(!gate.controlDrives(true,false,true).command_issued && device.clears==0,"enabled arm cleared"); ++cases;
  }
  {
    ArmDevice device; device.clear_ok=false; device.state.arms[0].error_code=13;
    SdkCommandGate gate(device,ActiveArmConfiguration::kDualArm);
    auto r=gate.controlDrives(true,false,true);
    check(r.command_issued && !r.feedback_confirmed,"uncleared arm error acknowledged"); ++cases;
  }
  {
    ArmDevice device; SdkCommandGate gate(device,ActiveArmConfiguration::kDualArm);
    auto clear=std::async(std::launch::async,[&]{return gate.controlDrives(true,false,true);});
    until([&]{return device.reads>=3;});
    check(gate.controlDrives(true,true,false).success(),"stop could not interrupt clear preflight");
    auto r=clear.get();
    check(!r.command_issued && !r.feedback_confirmed && device.clears==0,"clear issued after stop"); ++cases;
  }
  {
    ArmDevice device; SdkCommandGate gate(device,ActiveArmConfiguration::kDualArm);
    auto clear=std::async(std::launch::async,[&]{return gate.controlDrives(true,false,true);});
    until([&]{return device.clears>0;});
    check(gate.controlDrives(true,true,false).success(),"stop blocked by clear readback");
    check(!clear.get().feedback_confirmed,"interrupted clear reported success"); ++cases;
  }
  {
    ArmDevice device; device.stop_ok=false;
    SdkCommandGate gate(device,ActiveArmConfiguration::kDualArm);
    check(!gate.controlDrives(true,true,false).command_issued,"failed SDK stop accepted"); ++cases;
  }
  for (bool read_ok:{false,true}) {
    ArmDevice device; device.servo_read_ok=read_ok; device.servo_error=read_ok ? 9 : 0;
    SdkCommandGate gate(device,ActiveArmConfiguration::kDualArm);
    auto r=gate.controlDrives(true,false,true);
    check(r.command_issued && !r.feedback_confirmed,"servo error/read failure hidden by controller state"); ++cases;
  }
  {
    Bus bus;
    check(bus.maintenance.control(false).success(),"leg stop not confirmed");
    check(bus.unsafe==0 && bus.resets==0 && bus.disables>0,"leg stop changed head/enabled/reset"); ++cases;
  }
  {
    Bus bus(true);
    check(bus.maintenance.control(true).success(),"leg clear not confirmed");
    check(bus.unsafe==0 && bus.resets>0 && bus.disables>0,"leg reset/disable sequence wrong"); ++cases;
  }
  {
    Bus bus(true,false);
    check(!bus.maintenance.control(true).command_issued && bus.resets==0,"cleared legs without PDO timestamps"); ++cases;
  }
  {
    Bus bus(false,false);
    auto r=bus.maintenance.control(false);
    check(r.command_issued && !r.feedback_confirmed,"leg stop with no feedback falsely confirmed"); ++cases;
  }
  {
    Bus bus(false,true,false);
    check(!bus.maintenance.control(false).success(),"failed bus write accepted"); ++cases;
  }
  {
    Bus bus(true);
    auto clear=std::async(std::launch::async,[&]{return bus.maintenance.control(true);});
    until([&]{return bus.resets>0;});
    check(bus.maintenance.control(false).success(),"leg stop failed during reset pulse");
    const auto interrupted=clear.get();
    check(interrupted.command_issued && !interrupted.feedback_confirmed,
          "preempted leg clear lost write evidence or reported success");
    const int count=bus.resets;
    std::this_thread::sleep_for(40ms);
    check(bus.resets==count && bus.unsafe==0,"reset pulse continued after emergency stop"); ++cases;
  }
  std::cout << cases << " drive control cases passed (fake devices only)\n";
  return 0;
} catch (const std::exception& error) {
  std::cerr << error.what() << '\n'; return 1;
}
