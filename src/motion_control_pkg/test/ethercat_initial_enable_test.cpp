#include "motion_control_pkg/sdk/ethercat_initial_enable.hpp"
#include "motion_control_pkg/execution/ethercat_command_safety.hpp"
#include "motion_control_pkg/execution/head_startup_home.hpp"
#include <future>
#include <iostream>
#include <stdexcept>

using namespace motion_control_pkg;
using namespace std::chrono_literals;
void require(bool ok,const std::string& reason) {if(!ok)throw std::runtime_error(reason);}
struct Sim {
  sdk::EthercatInitialEnable guard;
  sdk::EthercatEnableSample frame;
  std::vector<std::uint16_t> ids{31,30};
  sdk::EnableClock::time_point now=sdk::EnableClock::now();
  unsigned ticks{},writes{},disables{};
  int failure{};
  bool blocked{};
  std::vector<DriverSDK::motorTargetStruct> last;
  std::string error;
  Sim() {
    frame.motors.resize(32);frame.acquired=now;frame.sequence=1;
    frame.motors[31].pos=.027f;frame.motors[30].pos=-.387f;
    for(auto id:ids){frame.motors[id].statusWord=0x0231;frame.motors[id].errorCode=0;}
  }
  struct Timing {
    Sim& s;
    auto now()const{return s.now;}
    void wait() {
      s.now+=4ms;++s.ticks;
      if(s.failure==1)return; // Frozen PDO, even if the caller keeps reading.
      ++s.frame.sequence;s.frame.acquired=s.now;
      if(s.ticks>4 && s.failure!=2)for(auto id:s.ids)s.frame.motors[id].statusWord=0x0237;
      if(s.ticks==8) {
        if(s.failure==3)s.frame.motors[31].statusWord=0x0231;
        if(s.failure==4)s.frame.motors[31].errorCode=0x7121;
        if(s.failure==5)s.blocked=true;
        if(s.failure==6)s.frame.motors[31].pos+=.02f;
        if(s.failure==7)--s.frame.sequence;
      }
    }
  };
  bool run() {
    return guard.run(ids,1.,[&](auto& f,std::string&){f=frame;return true;},
      [&](const auto& targets,std::string&){
        ++writes;last=targets;
        require(targets[0].pos==.027f && targets[1].pos==-.387f,"hold target chased feedback");
        for(auto& t:targets)require(t.enabled==1 && t.vel==0 && t.tor==0,"unexpected enable command");
        return true;
      },[&]{++disables;},[&](std::string& e){if(blocked){e="operator stop";return false;}return true;},error,Timing{*this});
  }
};
int main()try {
  Sim normal;require(normal.run(),normal.error);require(normal.ticks>=155 && !normal.disables,"enabled confirmation too short");
  auto writes=normal.writes;require(!normal.run() && normal.writes==writes,"success allowed a second initial enable");
  std::cout<<"PASS disabled to enabled with fixed hold and 600 ms fresh confirmation\n";
  for(int mode=1;mode<=7;++mode) {
    Sim s;s.failure=mode;require(!s.run(),"unsafe handshake succeeded mode="+std::to_string(mode));
    require(s.disables==1,"failed enable was not released");
    const auto sent=s.writes;s.failure=0;s.blocked=false;
    require(!s.run() && s.writes==sent,"failed handshake automatically retried");
  }
  std::cout<<"PASS stale PDO, timeout, lost enable, drive fault, stop, displacement, sequence failure; no retry\n";
  for(int mode=0;mode<4;++mode) {
    Sim s;
    if(mode==0)s.frame.motors[31].vel=.02;
    if(mode==1)s.frame.motors[31].errorCode=1;
    if(mode==2)s.frame.acquired=s.now-100ms;
    if(mode==3)s.blocked=true;
    require(!s.run() && s.writes==0 && s.disables==0,"invalid preflight wrote targets");
  }
  std::cout<<"PASS preflight rejects moving, faulted, stale and fenced states before enable\n";
  auto safety=std::make_shared<execution::MotionSafety>();
  std::vector<execution::SafetyJoint> joints{{"head_yaw_joint",-2.92,2.92,.2},{"head_pitch_joint",-.62,.41,.2}};
  std::uint64_t seq=0;
  safety->addChannel(execution::kHeadSafety,joints,[&](auto& f,std::string&){f={{.027,-.387},{0,0},execution::SafetyClock::now(),++seq,false};return true;});
  execution::HeadStartupHomePolicy home;home.enabled=true;home.lower={-2.92,-.7};home.upper={2.92,.41};
  safety->configureHeadStartupHome(home);execution::installMotionSafety(safety);
  for(int i=0;i<8;++i){safety->poll();std::this_thread::sleep_for(20ms);}
  std::vector<std::string> names;std::vector<double> positions;std::string error;
  require(safety->beginHeadStartupHome(names,positions,error),error);
  auto targets=normal.last;
  {
    execution::EthercatCommandSafety permit(normal.ids,targets);
    require(bool(permit),permit.error());require(permit.committed(),"startup owner could not commit initial hold");
  }
  auto alien=std::async(std::launch::async,[&]{execution::EthercatCommandSafety p(normal.ids,targets);return bool(p);});
  require(!alien.get(),"another thread enabled the head during startup");
  safety->requestStop("test stop");
  {execution::EthercatCommandSafety p(normal.ids,targets);require(!p,"latched stop allowed enable write");}
  execution::installMotionSafety(nullptr);
  std::cout<<"PASS startup owner permit, foreign-thread denial and stop fence\n";
  return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
