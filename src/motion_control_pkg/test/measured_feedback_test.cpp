#include "motion_control_pkg/wbc/measured_feedback.hpp"
#include "motion_control_pkg/execution/whole_body_completion.hpp"
#include <atomic>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

using motion_control_pkg::wbc::MeasuredFeedbackBuilder;
using namespace motion_control_pkg::execution;
static constexpr int64_t kStamp = 1000000000LL;
static void check(bool ok, const char* reason) {if (!ok) throw std::runtime_error(reason);}

struct Fixture {
  const std::array<uint16_t,6> ids{{0,1,2,3,4,5}};
  motion_control_pkg::sdk::DualArmDeviceState arms;
  std::vector<DriverSDK::motorActualStruct> motors{6};
  Fixture() {
    for (auto& a:arms.arms) {a.feedback_sequence=100;a.current_state=1;a.low_speed=true;}
    for (auto& m:motors) {m.statusWord=0x0037;m.errorCode=0;}
  }
  bool read(MeasuredFeedbackBuilder& b, bool snapshot, std::string& error,
      uint64_t sequence=500, int64_t stamp=kStamp, int64_t now=kStamp+1000000) const {
    sensor_msgs::msg::JointState out;
    return snapshot ? b.buildSnapshot(arms,motors,sequence,stamp,now,out,error) :
                      b.build(arms,motors,sequence,stamp,now,out,error);
  }
};

int main() try {
  unsigned cases=0;
  const auto run=[&](const char* name,auto test) {test();++cases;std::cout<<"PASS "<<name<<'\n';};
  run("post-enable and first-tick snapshot may reuse a fresh PDO",[] {
    Fixture f;MeasuredFeedbackBuilder b(f.ids);std::string e;
    check(f.read(b,true,e),"post-enable snapshot failed");
    check(f.read(b,true,e,500,kStamp,kStamp+1100000),"first tick rejected the same fresh frame");
  });
  run("execution/head snapshots do not consume publisher advancement",[] {
    Fixture f;MeasuredFeedbackBuilder b(f.ids);std::string e;
    check(f.read(b,false,e),"publisher baseline failed");
    check(f.read(b,true,e),"snapshot rejected publisher's frame");
    ++f.arms.arms[0].feedback_sequence;++f.arms.arms[1].feedback_sequence;
    check(f.read(b,true,e,501,kStamp+2000000,kStamp+3000000),"new snapshot failed");
    check(f.read(b,false,e,501,kStamp+2000000,kStamp+3000000),"snapshot consumed publisher frame");
    check(!f.read(b,false,e,501,kStamp+2000000,kStamp+3500000),"publisher published a repeated PDO");
  });
  run("out-of-order snapshot copies cannot poison publisher history",[] {
    Fixture old;Fixture newer;MeasuredFeedbackBuilder b(old.ids);std::string e;
    for (auto& a:newer.arms.arms) ++a.feedback_sequence;
    check(newer.read(b,false,e,501,kStamp+2000000,kStamp+3000000),"publisher baseline failed");
    check(old.read(b,true,e,500,kStamp,kStamp+3000000),"fresh older copy was rejected");
    for (auto& a:newer.arms.arms) ++a.feedback_sequence;
    check(newer.read(b,false,e,502,kStamp+4000000,kStamp+5000000),"snapshot changed publisher history");
  });
  run("concurrent snapshot readers are independent of publisher state",[] {
    Fixture f;MeasuredFeedbackBuilder b(f.ids);std::atomic<bool> ok{true};
    const auto reader=[&] {for(int i=0;i<300;++i) {std::string e;if(!f.read(b,true,e))ok=false;}};
    std::thread a(reader),c(reader);
    Fixture pub;std::string e;
    for(int i=0;i<20;++i) {
      for(auto& arm:pub.arms.arms)++arm.feedback_sequence;
      const auto stamp=kStamp+i*2000000LL;
      if(!pub.read(b,false,e,500+i,stamp,stamp+1000000))ok=false;
    }
    a.join();c.join();check(ok,"concurrent readers interfered");
  });
  run("repeated snapshots preserve acquisition time and expire",[] {
    Fixture f;MeasuredFeedbackBuilder b(f.ids);std::string e;sensor_msgs::msg::JointState out;
    for(auto age:{1000000LL,50000000LL,99000000LL}) {
      check(b.buildSnapshot(f.arms,f.motors,500,kStamp,kStamp+age,out,e),"fresh snapshot rejected");
      check(out.header.stamp.sec==1 && out.header.stamp.nanosec==0,"repeated read refreshed acquisition time");
    }
    check(!f.read(b,true,e,500,kStamp,kStamp+101000000),"frozen snapshot stayed fresh");
  });
  run("invalid acquisition clocks remain rejected",[] {
    Fixture f;MeasuredFeedbackBuilder b(f.ids);std::string e;
    check(!f.read(b,true,e,500,0,kStamp),"zero timestamp accepted");
    check(!f.read(b,true,e,500,kStamp,kStamp-1),"future timestamp accepted");
  });
  run("missing PDO and invalid arm counters remain rejected",[] {
    Fixture f;MeasuredFeedbackBuilder b(f.ids);std::string e;
    check(!f.read(b,true,e,0),"zero PDO sequence accepted");
    f.arms.arms[0].feedback_sequence=-1;check(!f.read(b,true,e),"missing arm counter accepted");
    f.arms.arms[0].feedback_sequence=1000001;check(!f.read(b,true,e),"invalid arm counter accepted");
  });
  run("disabled/faulted arm feedback remains rejected",[] {
    Fixture f;MeasuredFeedbackBuilder b(f.ids);std::string e;
    for(int state:{0,100}) {f.arms.arms[0].current_state=state;check(!f.read(b,true,e),"invalid arm state accepted");}
    f.arms.arms[0].current_state=1;f.arms.arms[0].error_code=1;
    check(!f.read(b,true,e),"arm fault accepted");
  });
  run("body/head motor faults and missing motors remain rejected",[] {
    Fixture f;MeasuredFeedbackBuilder b(f.ids);std::string e;
    f.motors[0].statusWord=0x003f;check(!f.read(b,true,e),"drive fault bit accepted");
    f.motors[0].statusWord=0x0037;f.motors[0].errorCode=0x8611;
    check(!f.read(b,true,e),"drive error accepted");
    f.motors[0].errorCode=0;f.motors.resize(5);check(!f.read(b,true,e),"missing motor accepted");
  });
  run("non-finite arm and motor measurements remain rejected",[] {
    Fixture f;MeasuredFeedbackBuilder b(f.ids);std::string e;
    const double nan=std::numeric_limits<double>::quiet_NaN();
    for(auto member:{&motion_control_pkg::sdk::ArmFeedback::joint_position,
                     &motion_control_pkg::sdk::ArmFeedback::joint_velocity,
                     &motion_control_pkg::sdk::ArmFeedback::joint_effort}) {
      (f.arms.arms[0].*member)[0]=nan;check(!f.read(b,true,e),"non-finite arm value accepted");
      (f.arms.arms[0].*member)[0]=0;
    }
    for(auto member:{&DriverSDK::motorActualStruct::pos,&DriverSDK::motorActualStruct::vel,&DriverSDK::motorActualStruct::tor}) {
      f.motors[0].*member=nan;check(!f.read(b,true,e),"non-finite motor value accepted");f.motors[0].*member=0;
    }
  });
  run("publisher still rejects reversed sequence/time and allows vendor wrap",[] {
    Fixture f;MeasuredFeedbackBuilder b(f.ids);std::string e;
    check(f.read(b,false,e),"baseline failed");
    check(!f.read(b,false,e,499,kStamp+1000000,kStamp+2000000),"PDO reversal accepted");
    check(!f.read(b,false,e,501,kStamp-1,kStamp+2000000),"timestamp reversal accepted");
    f.arms.arms[0].feedback_sequence=99;
    check(!f.read(b,false,e,501,kStamp+1000000,kStamp+2000000),"arm reversal accepted");
    Fixture wrap;MeasuredFeedbackBuilder w(wrap.ids);
    for(auto& a:wrap.arms.arms)a.feedback_sequence=999999;
    check(wrap.read(w,false,e),"wrap baseline failed");
    for(auto& a:wrap.arms.arms)a.feedback_sequence=0;
    check(wrap.read(w,false,e,501,kStamp+2000000,kStamp+3000000),"valid vendor wrap rejected");
  });
  const auto start=JointCompletionGuard::Time(std::chrono::seconds(1));
  run("leg completion cannot settle or stay fresh from repeated PDOs",[&] {
    JointCompletionGuard guard({0},start,start+std::chrono::seconds(1));std::string e;
    for(int ms=0;ms<=90;ms+=5)
      check(guard.update(500,{0},{0},start+std::chrono::milliseconds(ms),e)==JointCompletionGuard::Status::Waiting,"repeated PDO completed leg motion");
    check(guard.update(500,{0},{0},start+std::chrono::milliseconds(101),e)==JointCompletionGuard::Status::Failed,"frozen PDO stayed valid for completion");
  });
  run("arm completion cannot settle from repeated controller frames",[&] {
    Fixture f;ArmCompletionGuard guard(std::vector<double>(7,0),start,start+std::chrono::seconds(1));std::string e;
    for(int ms=0;ms<=90;ms+=5)
      check(guard.update(f.arms.arms[0],start+std::chrono::milliseconds(ms),e)==JointCompletionGuard::Status::Waiting,"repeated arm frame completed motion");
    check(guard.update(f.arms.arms[0],start+std::chrono::milliseconds(101),e)==JointCompletionGuard::Status::Failed,"frozen arm stayed valid for completion");
  });
  run("whole-body completion requires advancing arm and leg frames",[&] {
    Fixture f;WholeBodyCompletionFeedback measured;measured.arms=f.arms;measured.leg_sequence=500;
    WholeBodyCompletionGuard guard(std::vector<double>(7,0),{},std::vector<double>(4,0),start,start+std::chrono::seconds(1));std::string e;
    for(int ms=0;ms<=90;ms+=5)
      check(guard.update(measured,start+std::chrono::milliseconds(ms),e)==JointCompletionGuard::Status::Waiting,"repeated whole-body frames completed motion");
    check(guard.update(measured,start+std::chrono::milliseconds(101),e)==JointCompletionGuard::Status::Failed,"frozen whole-body frames accepted");
  });
  run("precision completion still succeeds with real advancing frames",[&] {
    JointCompletionOptions options{.002,.01,.2,0,.0002};
    JointCompletionGuard guard({0},start,start+std::chrono::seconds(1),false,options);std::string e;
    for(int i=0;i<5;++i) {
      const auto status=guard.update(500+i,{0},{0},start+std::chrono::milliseconds(50*i),e);
      check(status==(i==4?JointCompletionGuard::Status::Settled:JointCompletionGuard::Status::Waiting),"advancing precision settling changed");
    }
  });
  std::cout<<"PASS measured feedback snapshot cases="<<cases<<"; hardware_access=false\n";
  return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
