#include "motion_control_pkg/execution/head_startup_home.hpp"
#include "motion_control_pkg/motion/head_startup_plan.hpp"
#include <array>
#include <future>
#include <iostream>
#include <thread>
using namespace motion_control_pkg;
using namespace execution;
using namespace std::chrono_literals;
void require(bool ok,const std::string& message){if(!ok)throw std::runtime_error(message);}
struct Fixture {
  MotionSafety safety;
  std::vector<SafetyJoint> joints{{"head_yaw_joint",-2.92,2.92,.2},{"head_pitch_joint",-.62,.41,.2}};
  HeadStartupHomePolicy policy;
  std::vector<double> q{0.,-.7};
  std::array<double,3> other_q{};
  std::uint64_t sequence{0};
  bool bad_head{false},frozen{false};
  double pitch_velocity{0};
  SafetyClock::time_point acquired{};
  std::string error;
  Fixture(bool enabled=true):safety(SafetyPolicy{.015,.08,.08,.1,.01}) {
    for(unsigned i=0;i<3;++i)safety.addChannel(1U<<i,{{"other",-1,1,1}},
      [this,i](SafetySample& sample,std::string&){sample={{other_q[i]},{0},SafetyClock::now(),1,false};
        sample.sequence=std::chrono::duration_cast<std::chrono::nanoseconds>(sample.acquired.time_since_epoch()).count();return true;});
    safety.addChannel(kHeadSafety,joints,[this](SafetySample& sample,std::string& reason){
      if(bad_head){reason="head PDO fault";return false;}
      if(!frozen){++sequence;acquired=SafetyClock::now();}
      sample={q,{0,pitch_velocity},acquired,sequence,false};return true;});
    policy.enabled=enabled;policy.lower={-2.92,-.7};policy.upper={2.92,.41};
    safety.configureHeadStartupHome(policy);
  }
  void settle(int ms=140){for(int i=0;i<=ms/20;++i){safety.poll();std::this_thread::sleep_for(20ms);}}
  bool begin(){std::vector<std::string> names;std::vector<double> values;return safety.beginHeadStartupHome(names,values,error);}
  bool command(const std::vector<double>& target){auto permit=safety.command(kHeadSafety,error);return permit&&permit.validate(kHeadSafety,target)&&permit.committed(kHeadSafety,target);}
};
int main()try {
  int cases=0;
  auto pass=[&](const char* label){++cases;std::cout<<"PASS "<<label<<'\n';};
  {
    Fixture f(false);f.settle();require(!f.safety.ready(f.error),"default-off accepted head outside URDF");
    require(!f.begin(),"disabled startup began");pass("default disabled preserves normal URDF bounds");
  }
  {
    Fixture f;f.settle();require(!f.safety.ready(f.error),"normal admission accepted while waiting");
    for(auto group:{kArmSafety,kBodySafety})
      require(!f.safety.setErrorTrackingEnabled(group,true,.02,.08,f.error),"tracking option changed during startup reservation");
    {auto p=f.safety.command(kHeadSafety,f.error);require(!p,"ordinary command accepted before startup owner");}
    require(f.begin(),f.error);
    auto alien=std::async(std::launch::async,[&]{std::string e;auto p=f.safety.command(kHeadSafety,e);return bool(p);});
    require(!alien.get(),"foreign thread got recovery permission");
    {auto p=f.safety.command(kLegSafety,f.error);require(!p,"head owner granted leg command");}
    require(f.command(f.q),f.error);require(f.command({0,-.69}),f.error);
    require(!f.command({0,-.695}),"outward reversal accepted");
    require(f.safety.latched(),"bad command failed without latching");pass("startup reservation, owner and head-only inward command gates");
  }
  {
    Fixture f;f.settle();require(f.begin(),f.error);require(f.command(f.q),f.error);
    f.other_q[2]=.02;f.safety.poll();
    require(f.safety.latched(),"body_error OFF suppressed startup ancestor-motion guard");
    pass("startup ancestor-motion guard remains independent of body_error");
  }
  {
    Fixture f;f.q={0,-.701};f.settle();require(!f.begin(),"outside configured recovery envelope accepted");
    pass("beyond recovery envelope refused before dispatch");
  }
  {
    Fixture f;f.settle();require(f.begin(),f.error);require(f.command(f.q),f.error);
    f.q[1]=-.704;f.safety.poll();require(f.safety.latched(),"outward measured movement accepted");
    pass("outward feedback trips recovery");
  }
  {
    Fixture f;f.settle();require(f.begin(),f.error);require(f.command({0,-.61}),f.error);
    f.q={0,-.61};f.safety.poll();require(!f.safety.latched(),"valid inward position rejected");
    f.q[1]=-.621;f.safety.poll();require(f.safety.latched(),"normal range re-exit accepted");
    pass("normal head bounds restored on measured re-entry");
  }
  for(int failure=0;failure<4;++failure) {
    Fixture f;f.settle();require(f.begin(),f.error);require(f.command(f.q),f.error);
    if(failure==0){f.bad_head=true;f.safety.poll();}
    if(failure==1){f.frozen=true;std::this_thread::sleep_for(110ms);f.safety.poll();}
    if(failure==2)f.safety.requestStop("operator stop");
    if(failure==3){f.pitch_velocity=.2002;f.safety.poll();}
    require(!f.command({0,-.69}) && f.safety.latched(),"fault/stop allowed more commands");
    require(!f.safety.endHeadStartupHome(false,f.error),"failed recovery succeeded");
    require(!f.begin(),"failed startup retried");
  }
  pass("PDO fault, stale feedback, operator stop and measured overspeed revoke commands without retry");
  {
    Fixture f;f.settle();require(f.begin(),f.error);require(f.command(f.q),f.error);
    f.pitch_velocity=.11;f.safety.poll();
    require(!f.safety.latched(),"command speed incorrectly used as measured overspeed threshold");
    pass("homing retains URDF measured-speed guard above the command cap");
  }
  {
    Fixture f;f.settle();require(f.begin(),f.error);require(f.command({0,0}),f.error);
    f.q={0,0};f.settle(260);
    require(f.safety.finish(kHeadSafety,true,f.error),f.error);
    require(f.safety.endHeadStartupHome(true,f.error),f.error);
    require(f.safety.ready(f.error),f.error);require(!f.begin(),"completed startup retried");
    require(!f.command({0,-.7}),"ordinary motion retained widened bounds");
    pass("measured zero closes one-shot exception and restores normal limits");
  }
  {
    Fixture f;std::vector<std::vector<double>> path;std::string error;
    for(const auto& q:std::vector<std::vector<double>>{{0,-.7},{2.9,.4},{-2.9,-.7},{0,0}}) {
      require(motion::planHeadStartupHome(q,f.joints,f.policy,path,error),error);
      require(path.front()==q && path.back()==std::vector<double>{0,0},"planner endpoint changed");
      for(std::size_t i=1;i<path.size();++i)for(std::size_t j=0;j<2;++j)
        require(std::abs(path[i][j]-path[i-1][j])/.004<=
            f.policy.velocity+1e-6,
            "startup command exceeds requested speed");
    }
    const auto before=path;
    require(!motion::planHeadStartupHome({0,-.9},f.joints,f.policy,path,error) && path==before,"invalid start changed path");
    auto invalid=f.policy;invalid.lower[0]=-3;
    bool rejected=false;try{invalid.validate(f.joints);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"yaw bound extension accepted");
    for(int boundary=0;boundary<2;++boundary) {
      invalid=f.policy;
      if(boundary==0)invalid.lower[1]=-.701;
      else invalid.upper[1]=.42;
      rejected=false;
      try{invalid.validate(f.joints);}catch(const std::invalid_argument&){rejected=true;}
      require(rejected,"pitch startup envelope exceeded -0.7 or URDF upper bound");
    }
    for(double speed:{.10001,0.,-1.}) {
      invalid=f.policy;invalid.velocity=speed;rejected=false;
      try{invalid.validate(f.joints);}catch(const std::invalid_argument&){rejected=true;}
      require(rejected,"invalid homing command speed accepted");
    }
    auto slower_joints=f.joints;slower_joints[1].velocity=.05;
    rejected=false;try{f.policy.validate(slower_joints);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"homing command speed above URDF accepted");
    // Exercise an insufficient trajectory budget with a valid timing policy;
    // the production feedback wait is now 30 s, longer than this test budget.
    f.policy.feedback_wait=5;
    f.policy.timeout=6;
    require(!motion::planHeadStartupHome({0,-.7},f.joints,f.policy,path,error),"insufficient timeout accepted");
    pass("real Ruckig recovery paths obey velocity, acceleration, jerk and endpoints");
  }
  {
    Fixture f;std::vector<std::vector<double>> path;std::string error;
    for(std::size_t axis=0;axis<2;++axis) {
      std::vector<double> start(2,0.);start[axis]=axis==0?1.:-.7;
      require(motion::planHeadStartupHome(start,f.joints,f.policy,path,error),error);
      double peak=0.;
      for(std::size_t i=1;i<path.size();++i)
        peak=std::max(peak,std::abs(path[i][axis]-path[i-1][axis])/.004);
      require(peak>.0999 && peak<=.100001,"axis does not reach the requested 0.1 rad/s command cap");
      std::cout<<"INFO axis="<<axis<<" peak_command_rad_s="<<peak<<'\n';
    }
    pass("yaw and pitch both reach 0.1 rad/s without hidden axis scaling");
  }
  std::cout<<"PASS head_startup_home scenarios="<<cases<<'\n';return 0;
} catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
