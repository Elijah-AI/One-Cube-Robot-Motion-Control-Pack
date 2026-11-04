#include "motion_control_pkg/wbc/tracking.hpp"
#include <iostream>
#include <limits>

using namespace motion_control_pkg::wbc;
static void check(bool ok,const char* reason) {if(!ok)throw std::runtime_error(reason);}
static PositionTrackingGuard guard() {return PositionTrackingGuard({{"Joint5_L",-3.,3.,2.4,0,false}});}
int main() try {
  {
    auto g=guard();check(g.command(1.,{1.5}),"command");
    check(g.observe(1.01,1.011,{1.527571},{-.24},{1.527571}),"dwell started too early");
    check(g.observe(1.08,1.081,{1.527571},{-.24},{1.527571}),"dwell shortened");
    check(!g.observe(1.091,1.092,{1.527571},{-.24},{1.527571}),"controller echo masked sent-target error");
    for(const auto* field:{"sustained position tracking error: Joint5_L","measured_rad=1.527571",
        "command_rad=1.5","error_rad=0.027571","limit_rad=0.02","dwell_ms=81",
        "feedback_age_ms=1","reference_age_ms=91","controller_error_rad=0"})
      check(g.error().find(field)!=std::string::npos,field);
    const auto error=g.error();check(!g.command(1.1,{1.517571}),"fault not latched");
    check(g.error()==error,"fault detail overwritten");
  }
  {
    auto g=guard();check(g.command(2.,{0.}),"command");check(g.command(2.04,{.2}),"future command");
    check(g.observe(2.03,2.05,{0.},{0.}),"future command used for older measurement");
    check(!g.observe(2.041,2.05,{0.},{0.}),"hard threshold relaxed");
    check(g.error().find("hard position tracking error:")!=std::string::npos,"hard error type");
    check(g.error().find("command_velocity_rad_s=5")!=std::string::npos,"command derivative missing");
  }
  {
    auto g=guard();check(g.command(3.,{0.}),"command");
    check(g.observe(3.01,3.011,{.025},{0.}),"dwell start");
    check(g.observe(3.01,3.12,{.025},{0.}),"duplicate feedback advanced dwell");
    check(g.observe(3.05,3.051,{.019},{0.}),"below threshold rejected");
    check(g.observe(3.06,3.061,{.025},{0.}),"dwell not reset");
    check(g.observe(3.13,3.131,{.025},{0.}),"dwell reset not respected");
    check(!g.observe(3.141,3.142,{.025},{0.},{std::numeric_limits<double>::quiet_NaN()}),"threshold failed without valid echo");
    check(g.error().find("controller_command_rad=")==std::string::npos,"invalid diagnostic echo printed");
  }
  {
    auto g=guard();check(g.command(6.,{1.49433112}),"command");
    check(g.observe(6.01,6.011,{1.51105004},{-.449584735},{1.5096652}),"reported lag rejected");
    check(g.observe(6.10,6.101,{1.51105004},{-.449584735},{1.5096652}),"reported lag exceeded new threshold");
    check(g.observe(6.11,6.111,{1.51333112},{0.}),"below threshold rejected");
    check(g.observe(6.12,6.121,{1.51443112},{0.}),"new dwell started too early");
    check(g.observe(6.19,6.191,{1.51443112},{0.}),"new dwell shortened");
    check(!g.observe(6.201,6.202,{1.51443112},{0.}),"new threshold not enforced");
    check(g.error().find("limit_rad=0.02")!=std::string::npos,"new threshold diagnostic missing");
  }
  {
    auto g=guard();check(g.command(7.,{0.}),"command");
    check(g.observe(7.01,7.011,{.02},{0.}),"threshold equality rejected");
    check(g.observe(7.10,7.101,{.02},{0.}),"threshold equality accumulated dwell");
  }
  {
    auto g=guard();check(g.command(4.,{0.}),"command");
    check(!g.observe(4.01,4.17,{0.},{0.}),"stale feedback accepted");
  }
  {
    auto g=guard();check(g.command(5.,{0.}),"command");
    check(!g.observe(5.01,5.011,{0.},{2.5}),"URDF velocity relaxed");
  }
  std::cout << "PASS tracking diagnostics preserve dwell, hard error, history, replay, stale and URDF protection\n";
  return 0;
} catch(const std::exception& e) {std::cerr << e.what() << '\n';return 1;}
