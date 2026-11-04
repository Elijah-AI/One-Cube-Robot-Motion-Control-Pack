#include "motion_control_pkg/sdk/ethercat_startup_readiness.hpp"
#include <iostream>
#include <stdexcept>

void check(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
int main() {
  using namespace motion_control_pkg::sdk;
  const std::int64_t now=10000000000LL;
  OnecubeEthercatBusStateV1 bus;
  bus.sequence=100;bus.acquired_ns=now;bus.ready_since_ns=now-kEthercatStartupStableNs;
  bus.expected_slaves=bus.responding_slaves=6;
  bus.domain_count=bus.complete_domains=6;bus.link_up=1;bus.al_states=8;
  std::vector<OnecubeEthercatFeedbackV1> frames(32);
  for (int id:{29,28,27,26,31,30}) {
    frames[id].sequence=100;frames[id].acquired_ns=now;frames[id].status=0x0231;
  }
  check(ethercatStartupReady(bus,frames,now),"OP stable, disabled drives should pass communication gate");
  for (unsigned al:{0u,1u,2u,4u,0x0cu}) {
    auto bad=bus;bad.al_states=al;
    check(!ethercatStartupReady(bad,frames,now),"SAFEOP/mixed AL must not release startup");
  }
  auto bad=bus;bad.ready_since_ns=now-1999000000LL;
  check(!ethercatStartupReady(bad,frames,now),"2 s stabilization required");
  bad=bus;bad.ready_since_ns=0;
  check(!ethercatStartupReady(bad,frames,now),"OP loss must restart stabilization");
  bad=bus;bad.complete_domains=5;
  check(!ethercatStartupReady(bad,frames,now),"partial PDO must block");
  bad=bus;bad.responding_slaves=5;
  check(!ethercatStartupReady(bad,frames,now),"missing slave must block");
  bad=bus;bad.expected_slaves=bad.responding_slaves=5;
  check(!ethercatStartupReady(bad,frames,now),"wrong robot topology must block");
  bad=bus;bad.link_up=0;
  check(!ethercatStartupReady(bad,frames,now),"link loss must block");
  bad=bus;bad.sequence=0;
  check(!ethercatStartupReady(bad,frames,now),"empty SDK cache must block");
  check(!ethercatStartupReady(bus,frames,now+20000001LL),"repeated reads must expire");
  bad=bus;bad.acquired_ns=now+1;
  check(!ethercatStartupReady(bad,frames,now),"future bus timestamp must block");
  for (int id:{29,28,27,26,31,30}) {
    auto stale=frames;stale[id].acquired_ns=now-20000001LL;
    check(!ethercatStartupReady(bus,stale,now),"every head/waist/leg PDO must be fresh");
    stale=frames;stale[id].status=0xffff;
    check(!ethercatStartupReady(bus,stale,now),"invalid axis status must block");
  }
  frames[30].status=0x0218;frames[30].error=0xff32;
  check(ethercatStartupReady(bus,frames,now),"bus OP must not be confused with fault-clear authorization");
  std::cout<<"PASS: startup OP, complete PDO, stabilization, stale/missing feedback gates\n";
}
