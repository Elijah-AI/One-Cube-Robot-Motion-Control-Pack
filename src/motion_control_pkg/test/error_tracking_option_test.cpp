#include "motion_control_pkg/execution/motion_safety.hpp"
#include "motion_control_pkg/wbc/tracking.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

using namespace motion_control_pkg;
using namespace execution;
using namespace std::chrono_literals;
static void check(bool value,const std::string& message) {
  if(!value) throw std::runtime_error(message);
}
struct Fixture {
  MotionSafety safety;
  std::array<double,4> q{},v{};
  std::array<bool,4> invalid{},frozen{};
  std::array<SafetyClock::time_point,4> stamps{};
  std::array<std::uint64_t,4> sequences{};
  std::string detail;
  Fixture():safety([]{SafetyPolicy p;p.stop_timeout=.03;return p;}()) {
    for(unsigned i=0;i<4;++i) safety.addChannel(1U<<i,
        {{i<2?(i==0?"arm_L":"arm_R"):(i==2?"leg":"head"),-3,3,5}},
        [this,i](SafetySample& sample,std::string& error) {
          if(invalid[i]){error="device fault";return false;}
          if(!frozen[i]){stamps[i]=SafetyClock::now();++sequences[i];}
          sample={{q[i]},{v[i]},stamps[i],sequences[i],false};return true;
        });
  }
  void pollFor(int ms=140) {
    const auto end=SafetyClock::now()+std::chrono::milliseconds(ms);
    do{safety.poll();std::this_thread::sleep_for(10ms);}while(SafetyClock::now()<end);
    safety.poll();
  }
  bool enable(double limit=.02,double hard=.08,SafetyMask group=kArmSafety) {
    return safety.setErrorTrackingEnabled(group,true,limit,hard,detail);
  }
  bool send(SafetyMask mask,double target) {
    auto permit=safety.command(mask,detail);
    if(!permit)return false;
    for(unsigned i=0;i<4;++i)if(mask&(1U<<i)) {
      const double value=i>=2 && (mask&3U)?q[i]:target;
      if(!permit.validate(1U<<i,{value}) || !permit.committed(1U<<i,{value}))return false;
    }
    return true;
  }
  void expectRecovered(SafetyMask mask) {
    pollFor();
    check(!safety.ready(detail),"old execution lost its stop fence");
    check(!safety.finish(mask,false,detail),"stopped action reported success");
    check(detail.find("no persistent latch")!=std::string::npos,detail);
    check(!safety.ready(detail),"worker exit was not required");
    safety.leaveExecution();
    check(safety.ready(detail),detail);
    check(!safety.latched(),"tracking stop remained latched after standstill/worker exit");
    check(send(mask,0),"next motion required a manual reset");
  }
};
int main() try {
  int cases=0;
  const auto pass=[&](const char* name){std::cout<<"PASS "<<name<<'\n';++cases;};
  {
    Fixture f;f.pollFor();check(f.safety.status().find("arm_error=OFF")!=std::string::npos,"arm default not OFF");
    check(f.safety.status().find("body_error=OFF")!=std::string::npos,"body default not OFF");
    check(f.send(kLeftSafety|kRightSafety,.2),f.detail);f.pollFor();
    check(f.safety.ready(f.detail),"default-off arm error stopped motion");
    check(!f.safety.latched(),"default-off error latched");
    pass("default OFF for both arms, including hard position errors");
  }
  {
    Fixture f;f.pollFor();check(f.send(kBodySafety,.2),f.detail);f.pollFor();
    check(f.safety.ready(f.detail),"default-off body error stopped motion");
    pass("body_error defaults OFF for legs, waist and head, including hard errors");
  }
  for(auto group:{kArmSafety,kBodySafety}) {
    Fixture f;f.pollFor();
    for(auto limits:std::array<std::array<double,2>,4>{{{{0,.08}},{{-.01,.08}},{{.03,.02}},{{NAN,.08}}}})
      check(!f.enable(limits[0],limits[1],group),"invalid group rad limits accepted");
    check(!f.enable(.02,std::numeric_limits<double>::infinity(),group),"infinite group limit accepted");
    check(!f.safety.setErrorTrackingEnabled(kLegSafety,true,.02,.08,f.detail),"partial group accepted");
    check(f.enable(.05,.2,group),f.detail);
    check(f.send(group,.04),f.detail);f.pollFor();
    check(!f.safety.latched(),"selected group limit ignored");
    check(!f.safety.setErrorTrackingEnabled(group,false,0,0,f.detail),"active group switch accepted");
    check(!f.safety.setErrorTrackingEnabled(group==kArmSafety?kBodySafety:kArmSafety,
        true,.02,.08,f.detail),"other group changed during motion");
  }
  pass("both groups validate limits atomically and reject all switches during execution");
  {
    Fixture f;f.pollFor();
    for(auto limits:std::array<std::array<double,2>,4>{{{{0,.08}},{{-.01,.08}},{{.03,.02}},{{NAN,.08}}}})
      check(!f.enable(limits[0],limits[1]),"invalid rad limits accepted");
    check(!f.enable(.02,std::numeric_limits<double>::infinity()),"infinite hard limit accepted");
    check(f.enable(.05,.2),f.detail);
    check(f.send(kLeftSafety,.04),f.detail);f.pollFor();
    check(!f.safety.latched(),"supplied arm threshold ignored");
    check(!f.safety.setErrorTrackingEnabled(kArmSafety,false,0,0,f.detail),"active tracking switch accepted");
    pass("explicit finite rad limits, atomic validation and idle-only switching");
  }
  for(auto mask:{kLeftSafety,kRightSafety,kLeftSafety|kRightSafety,kLeftSafety|kRightSafety|kLegSafety}) {
    Fixture f;f.pollFor();check(f.enable(),f.detail);f.safety.enterExecution();
    check(f.send(mask,.03),f.detail);f.pollFor(100);
    check(!f.safety.ready(f.detail) && f.detail.find("tracking stop")!=std::string::npos,"sustained arm error not fenced");
    check(!f.send(mask,0),"old stream continued after tracking stop");
    f.expectRecovered(mask);
  }
  pass("left/right/dual/coupled arm streams stop on sustained error and accept a new task without reset");
  {
    Fixture f;f.pollFor();check(f.enable(.01,.04),f.detail);f.safety.enterExecution();
    check(f.send(kRightSafety,.05),f.detail);f.safety.poll();
    check(f.safety.status().find("hard position tracking error")!=std::string::npos,"supplied hard rad limit ignored");
    f.expectRecovered(kRightSafety);pass("hard arm error also uses a recoverable stop");
  }
  {
    Fixture f;f.pollFor();check(f.enable(.01,.1),f.detail);f.safety.enterExecution();
    check(f.send(kLegSafety,0),f.detail);f.q[0]=.02;f.pollFor(110);
    check(f.safety.status().find("tracking stop")!=std::string::npos ||
        f.safety.status().find("ERROR_TRACKING_")!=std::string::npos,"idle arm hold reference did not use the configured limit");
    f.expectRecovered(kLegSafety);pass("uncommanded arm holds use the same configured rad thresholds");
  }
  {
    Fixture f;f.pollFor();check(f.enable(),f.detail);f.safety.enterExecution();f.safety.enterExecution();
    check(f.send(kLeftSafety,.1),f.detail);f.safety.poll();f.pollFor();
    check(!f.safety.finish(kLeftSafety,false,f.detail),"tracking failure returned success");
    f.safety.leaveExecution();check(!f.safety.ready(f.detail),"another old worker did not retain the fence");
    f.safety.leaveExecution();check(f.safety.ready(f.detail),f.detail);
    pass("all old workers must exit before tracking stop releases");
  }
  {
    Fixture f;f.pollFor();check(f.enable(),f.detail);f.safety.enterExecution();
    check(f.send(kLeftSafety,.1),f.detail);f.v[0]=.02;f.safety.poll();
    check(!f.safety.finish(kLeftSafety,false,f.detail),"unconfirmed stop returned success");
    check(f.detail.find("STOP UNCONFIRMED")!=std::string::npos,"moving stop was confirmed");
    f.safety.leaveExecution();check(!f.safety.ready(f.detail),"unconfirmed stop admitted a task");
    f.v[0]=0;f.pollFor();check(f.safety.ready(f.detail),f.detail);
    pass("worker exit alone cannot release an unconfirmed moving stop");
  }
  for(auto mask:{kLegSafety,kHeadSafety,kBodySafety}) for(double target:{.03,.1}) {
    Fixture f;f.pollFor();check(f.enable(.02,.08,kBodySafety),f.detail);
    f.safety.enterExecution();check(f.send(mask,target),f.detail);f.pollFor(110);
    check(f.safety.status().find("body_error: ")!=std::string::npos,"body stop has no group diagnostic");
    check(!f.send(mask,0),"old body stream continued after error");
    f.expectRecovered(mask);
  }
  pass("leg/head/combined body streams stop on sustained and hard errors, then release like arms");
  {
    Fixture f;f.pollFor();check(f.enable(),f.detail);check(f.enable(.02,.08,kBodySafety),f.detail);
    f.safety.enterExecution();
    {
      auto p=f.safety.command(kLeftSafety|kLegSafety,f.detail);check(bool(p),f.detail);
      check(p.validate(kLeftSafety,{.1}) && p.committed(kLeftSafety,{.1}),"arm dispatch failed");
      check(p.validate(kLegSafety,{.1}) && p.committed(kLegSafety,{.1}),"leg dispatch failed");
    }
    f.safety.poll();f.pollFor();
    check(!f.safety.finish(kLeftSafety|kLegSafety,false,f.detail),"coupled error succeeded");
    f.safety.leaveExecution();check(!f.safety.latched() && f.safety.ready(f.detail),f.detail);
    pass("simultaneous arm and body position errors use the same measured stop and release policy");
  }
  for(unsigned chain=0;chain<4;++chain) for(int fault=0;fault<4;++fault) {
    Fixture f;f.pollFor();f.safety.enterExecution();check(f.send(1U<<chain,0),f.detail);
    if(fault==0)f.invalid[chain]=true;
    if(fault==1)f.q[chain]=4;
    if(fault==2)f.v[chain]=6;
    if(fault==3){f.frozen[chain]=true;std::this_thread::sleep_for(110ms);}
    f.safety.poll();check(f.safety.latched(),"disabled tracking masked real feedback fault");
  }
  pass("driver fault, URDF position, measured overspeed and stale PDO stay protected when tracking is OFF");
  for(auto mask:{kLeftSafety,kLegSafety,kHeadSafety}) for(int fault=0;fault<3;++fault) {
    Fixture f;f.pollFor();check(f.enable(),f.detail);check(f.enable(.02,.08,kBodySafety),f.detail);
    f.safety.enterExecution();check(f.send(mask,.1),f.detail);f.safety.poll();
    if(fault==0){f.invalid[0]=true;f.safety.poll();f.invalid[0]=false;}
    if(fault==1)f.safety.requestStop("operator stop");
    if(fault==2){auto permit=f.safety.command(kRightSafety,f.detail);check(!permit,"pending stop permitted another command");f.invalid[2]=true;}
    f.pollFor();f.invalid[2]=false;f.pollFor();
    check(!f.safety.finish(mask,false,f.detail),"escalated fault succeeded");f.safety.leaveExecution();
    check(f.safety.latched() && !f.safety.ready(f.detail),"real fault was auto-cleared with tracking stop");
  }
  pass("real faults and explicit stops override a recoverable tracking stop");
  for(auto group:{kArmSafety,kBodySafety}) {
    Fixture f;f.pollFor();check(f.enable(.01,.04,group),f.detail);
    const auto other=group==kArmSafety?kBodySafety:kArmSafety;
    check(f.send(other,.2),f.detail);f.pollFor();
    check(!f.safety.latched(),"one group's switch changed the other group's thresholds");
  }
  pass("arm_error and body_error enable independently");
  for(auto group:{kArmSafety,kBodySafety}) {
    Fixture f;f.pollFor();check(f.enable(.05,.2,group),f.detail);
    check(f.enable(.03,.1,group==kArmSafety?kBodySafety:kArmSafety),f.detail);
    check(f.safety.setErrorTrackingEnabled(group,false,0,0,f.detail),f.detail);
    check(f.send(group,.2),f.detail);f.pollFor();check(!f.safety.latched(),"group OFF did not disable error thresholds");
  }
  pass("disabling either group leaves the other group's configured protection unchanged");
  for(unsigned idle=2;idle<4;++idle) for(bool enabled:{false,true}) {
    Fixture f;f.pollFor();if(enabled)check(f.enable(.02,.08,kBodySafety),f.detail);
    f.safety.enterExecution();check(f.send(kLeftSafety,0),f.detail);
    f.q[idle]=.015;f.pollFor(110);check(!f.safety.latched(),"fixed 0.01-rad hold guard bypassed body_error");
    f.q[idle]=.03;f.pollFor(110);
    if(enabled)f.expectRecovered(kLeftSafety);
    else check(f.safety.ready(f.detail),"disabled body hold error still stopped another chain");
  }
  pass("uncommanded leg/waist/head holds follow body_error and its selected limits");
  for(unsigned chain:{2u,3u}) {
    Fixture f;f.pollFor();check(f.enable(.01,.04,kBodySafety),f.detail);f.safety.enterExecution();
    const auto mask=1U<<chain;check(f.send(mask,.1),f.detail);f.v[chain]=.02;f.safety.poll();
    check(!f.safety.finish(mask,false,f.detail),"moving body stop succeeded");f.safety.leaveExecution();
    check(!f.safety.ready(f.detail),"moving body stop released after worker exit");
    check(!f.safety.setErrorTrackingEnabled(kBodySafety,false,0,0,f.detail),"pending body stop allowed disabling");
    f.v[chain]=0;f.pollFor();check(f.safety.ready(f.detail),f.detail);
  }
  pass("body stop requires measured standstill and rejects toggles while pending");
  for(auto group:{kArmSafety,kBodySafety}) {
    Fixture f;f.pollFor();f.safety.requestStop("operator stop");
    check(!f.safety.setErrorTrackingEnabled(group,true,.02,.08,f.detail),"faulted switch accepted");
  }
  pass("neither switch clears an existing fault");
  for(const auto& names:std::vector<std::vector<std::string>>{
      {"leg_joint1","leg_joint2","leg_joint3","body_joint"},{"head_yaw_joint","head_pitch_joint"}}) {
    for(std::size_t axis=0;axis<names.size();++axis) {
      MotionSafety safety;std::vector<SafetyJoint> joints;
      std::vector<double> actual(names.size(),0);std::uint64_t sequence=0;std::string detail;
      for(const auto& name:names)joints.push_back({name,-3,3,5});
      const auto mask=names.size()==4?kLegSafety:kHeadSafety;
      safety.addChannel(mask,joints,[&](SafetySample& sample,std::string&){
        sample={actual,actual,SafetyClock::now(),++sequence,false};return true;});
      const auto settle=[&]{for(int i=0;i<15;++i){safety.poll();std::this_thread::sleep_for(10ms);}};
      settle();check(safety.setErrorTrackingEnabled(kBodySafety,true,.02,.08,detail),detail);
      safety.enterExecution();auto target=actual;target[axis]=.1;
      {auto permit=safety.command(mask,detail);check(bool(permit),detail);
       check(permit.validate(mask,target) && permit.committed(mask,target),detail);}
      safety.poll();check(safety.status().find("hard position tracking error: "+names[axis])!=std::string::npos,
          "body_error missed joint "+names[axis]);
      settle();check(!safety.finish(mask,false,detail),"body joint error succeeded");safety.leaveExecution();
      check(safety.ready(detail),detail);
    }
  }
  pass("body_error covers each of three leg joints, waist, head yaw and head pitch");
  {
    Fixture f;f.pollFor();check(f.enable(),f.detail);
    check(f.safety.setErrorTrackingEnabled(kArmSafety,false,0,0,f.detail),f.detail);
    check(f.send(kLeftSafety,.15),f.detail);f.pollFor();check(!f.safety.latched(),"OFF did not disable thresholds");
    pass("explicit OFF disables both arm error thresholds again");
  }
  {
    wbc::PositionTrackingGuard g({{"arm",-3,3,5,0,false}},wbc::TrackingPolicy{.02,.08,.08,.1,false});
    check(g.command(1,{.2}) && g.observe(1.01,1.02,{0},{0}),"OFF rejected a position error");
    check(!g.observe(1.02,1.2,{0},{0}) && g.failure()==wbc::PositionTrackingGuard::Failure::InvalidData,
        "OFF suppressed feedback age validation");
    pass("tracking guard reports failure classes and preserves feedback validation");
  }
  std::cout<<"PASS error_tracking_option groups="<<cases<<"; no hardware or motion goals\n";
  return 0;
} catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
