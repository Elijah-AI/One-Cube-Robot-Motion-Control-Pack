#include "motion_control_pkg/execution/orderly_shutdown.hpp"
#include <iostream>
#include <stdexcept>
using namespace motion_control_pkg;
using namespace std::chrono_literals;
static void check(bool b,const char* m) {if(!b) throw std::runtime_error(m);}
struct Fixture {
  sdk::DriveClock::time_point now=sdk::DriveClock::now()+1s;
  bool idle=true,read_ok=true,disable_ok=true,release_ok=true,disabled=false;
  bool stale=false,repeated=false,drift=false,missing=false,remain_enabled=false,throws=false;
  double velocity=0;
  int reads=0,disables=0,releases=0;
  execution::OrderlyShutdown shutdown;
  Fixture():shutdown({[this]{return idle;},
    [this](auto& axes,auto& detail,bool){
      ++reads;detail="fake device feedback";
      if(throws) throw std::runtime_error("read failed");
      for(int j=0;j<(missing?19:20);++j)
        axes.push_back({drift?reads*.002:0.,velocity,static_cast<uint64_t>(repeated?1:reads),
          stale?now-200ms:now,disabled&&!remain_enabled,false,true,false});
      return read_ok;},
    [this](auto&){++disables;disabled=true;return disable_ok;},
    [this](auto&){++releases;return release_ok;}},20,now) {}
  void tick(){shutdown.tick(now);now+=20ms;}
  void run(int count=800){for(int i=0;i<count;++i)tick();}
};
int main() try {
  int cases=0;
  { Fixture f;f.run();check(f.shutdown.complete() && f.disables==1 && f.releases==1,"normal shutdown order/count");++cases; }
  // Stop proof cannot be replaced by a command return, stale cache or idle flag.
  for(int fault=0;fault<7;++fault) {
    Fixture f;
    if(fault==0)f.velocity=.02;
    if(fault==1)f.stale=true;
    if(fault==2)f.repeated=true;
    if(fault==3)f.drift=true;
    if(fault==4)f.missing=true;
    if(fault==5)f.idle=false;
    if(fault==6)f.read_ok=false;
    f.run();
    check(f.shutdown.phase()==execution::OrderlyShutdown::Phase::FailedHold && !f.disables && !f.releases,"unsafe stop advanced");
    f.velocity=0;f.stale=f.repeated=f.drift=f.missing=false;f.idle=f.read_ok=true;f.run();
    check(!f.releases && !f.disables,"failure implicitly retried/resumed");++cases;
  }
  { Fixture f;f.disable_ok=false;f.run();check(f.disables==1 && !f.releases && !f.shutdown.complete(),"failed disable released");++cases; }
  { Fixture f;f.remain_enabled=true;f.run();check(f.disables==1 && !f.releases && !f.shutdown.complete(),"submission mistaken for disabled");++cases; }
  { Fixture f;f.release_ok=false;f.run();check(f.releases==1 && !f.shutdown.complete(),"failed release reported complete");++cases; }
  { Fixture f;f.throws=true;f.run();check(!f.disables && !f.releases && !f.shutdown.complete(),"exception unwound drivers");++cases; }
  { Fixture f;
    while(!f.disables)f.tick();
    for(int i=0;i<20;++i)f.tick();
    check(!f.releases,"500 ms disabled dwell skipped");
    f.read_ok=false;f.tick();f.read_ok=true;
    for(int i=0;i<20;++i)f.tick();
    check(!f.releases,"read failure did not restart dwell");
    f.run();check(f.shutdown.complete(),"fresh feedback did not qualify");++cases;
  }
  { Fixture f;
    while(!f.disables)f.tick();
    f.drift=true;f.run();check(!f.releases,"drift after disable accepted despite zero velocity");++cases;
  }
  { Fixture f;
    while(f.shutdown.phase()!=execution::OrderlyShutdown::Phase::Releasing) f.tick();
    f.read_ok=false;f.run();check(!f.releases && !f.shutdown.complete(),"release used an old confirmation");++cases;
  }
  {
    int sequence=0,disables=0,releases=0;
    bool disabled=false;
    const auto start=sdk::DriveClock::now();
    execution::OrderlyShutdown shutdown({[]{return true;},
      [&](auto& axes,auto&,bool) {
        axes.push_back({0,0,static_cast<std::uint64_t>(++sequence),sdk::DriveClock::now(),disabled,false,true,false});return true;
      },[&](auto&){++disables;disabled=true;return true;},[&](auto&){++releases;return true;}},1,start);
    while(!shutdown.complete() && sdk::DriveClock::now()-start<2s) {
      shutdown.tick(sdk::DriveClock::now());std::this_thread::sleep_for(10ms);
    }
    check(shutdown.complete() && disables==1 && releases==1,"read-time acquisition rejected as future");++cases;
  }
  std::cout<<"PASS orderly_shutdown cases="<<cases<<"; no hardware\n";
} catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
