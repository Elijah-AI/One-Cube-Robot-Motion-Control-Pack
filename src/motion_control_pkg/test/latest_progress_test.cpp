#include "motion_control_pkg/execution/latest_progress.hpp"
#include "motion_control_pkg/execution/preplanned_dispatch.hpp"
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace motion_control_pkg::execution;
using namespace std::chrono_literals;
using Clock=std::chrono::steady_clock;
static void require(bool value,const std::string& message) {
  if(!value)throw std::runtime_error(message);
}
struct FakeState {Clock::time_point now=Clock::time_point(1s);unsigned wakes=0;bool late=false;};
struct FakeTiming {
  std::shared_ptr<FakeState> state;
  auto now()const{return state->now;}
  void sleep_until(Clock::time_point t)const {
    state->now=std::max(state->now,t);
    if(state->late && state->wakes==1300)state->now+=4500us;
    ++state->wakes;
  }
};
struct ReleaseOnExit {
  std::promise<void>& promise;bool done=false;
  void release(){if(!done){done=true;promise.set_value();}}
  ~ReleaseOnExit(){release();}
};
int main()try {
  unsigned cases=0;
  {
    bool rejected=false;
    try{LatestProgressPublisher p([](float){},0ms);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"zero timer period accepted");
    rejected=false;try{LatestProgressPublisher p({});}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"empty publisher accepted");
    ++cases;std::cout<<"PASS invalid configuration rejected before execution\n";
  }
  {
    std::promise<void> entered,release,latest;
    auto ready=entered.get_future();auto published=latest.get_future();auto gate=release.get_future().share();
    std::atomic<unsigned> calls{0};std::atomic<bool> terminal{false},late_publication{false};
    const auto producer=std::this_thread::get_id();std::thread::id publishing_thread;
    LatestProgressPublisher progress([&](float value){
      if(terminal)late_publication=true;
      publishing_thread=std::this_thread::get_id();
      const auto count=++calls;
      if(count==1){entered.set_value();gate.wait();}
      if(count==2 && value==95.0F)latest.set_value();
    },1ms);
    ReleaseOnExit release_guard{release};
    progress.update(0);
    require(ready.wait_for(2s)==std::future_status::ready,"publisher timer never ran");
    // Publication is blocked while the ACTUAL dispatcher runs 1500 fake 4ms
    // ticks. The producer can complete without releasing the publication gate.
    auto clock=std::make_shared<FakeState>();auto cancel=std::make_shared<CancellationToken>();
    std::string error;unsigned sent=0;
    require(dispatchPreplannedSamples(1500,clock->now,cancel,[&](std::size_t i){
      ++sent;clock->now+=50us;progress.update(95.0F*(i+1)/1500);return true;
    },error,FakeTiming{clock}),error);
    require(sent==1500 && !cancel->requested(),"observer blocked or canceled motor samples");
    require(calls==1,"blocked publisher was called recursively");
    release_guard.release();
    require(published.wait_for(2s)==std::future_status::ready,"latest update was not published after coalescing");
    progress.stop();terminal=true;
    require(publishing_thread!=producer,"ROS publication ran on the producer");
    require(calls==2,"progress queue/backlog was drained instead of latest-only delivery");
    progress.update(50);std::this_thread::sleep_for(5ms);progress.stop();
    require(calls==2 && !late_publication,"feedback escaped stop/terminal boundary");
    ++cases;std::cout<<"PASS blocked consumer cannot delay 1500 motor ticks; bounded latest-only delivery; no post-terminal publication\n";
  }
  {
    std::promise<void> entered,release;auto ready=entered.get_future();auto gate=release.get_future().share();
    LatestProgressPublisher progress([&](float){entered.set_value();gate.wait();},1ms);
    ReleaseOnExit release_guard{release};progress.update(1);
    require(ready.wait_for(2s)==std::future_status::ready,"in-flight callback missing");
    auto stopped=std::async(std::launch::async,[&]{progress.stop();});
    require(stopped.wait_for(20ms)==std::future_status::timeout,"stop failed to drain in-flight callback");
    release_guard.release();require(stopped.wait_for(2s)==std::future_status::ready,"stop did not join callback");stopped.get();
    ++cases;std::cout<<"PASS close joins in-flight publication before result/lifetime release\n";
  }
  {
    LatestProgressPublisher progress([](float){throw std::runtime_error("injected publish error");},1ms);
    progress.update(1);const auto end=Clock::now()+2s;
    while(!progress.failed() && Clock::now()<end)std::this_thread::sleep_for(1ms);
    require(progress.failed() && progress.error().find("injected publish error")!=std::string::npos,
        "consumer exception was lost or escaped its worker");progress.stop();
    ++cases;std::cout<<"PASS publication failure is reported to the motion owner\n";
  }
  {
    std::atomic<unsigned> calls{0};LatestProgressPublisher progress([&](float){++calls;},2ms);
    std::this_thread::sleep_for(15ms);require(calls==0,"idle timer published invented progress");
    progress.update(100);const auto end=Clock::now()+2s;
    while(calls==0 && Clock::now()<end)std::this_thread::sleep_for(1ms);
    progress.stop();require(calls==1,"unchanged progress was republished without an update");
    ++cases;std::cout<<"PASS idle timer and shutdown do not manufacture or replay progress\n";
  }
  for(bool before:{true,false}) {
    auto clock=std::make_shared<FakeState>();clock->late=before;
    auto cancel=std::make_shared<CancellationToken>();std::string error;unsigned sent=0;
    const bool ok=dispatchPreplannedSamples(1500,clock->now,cancel,[&](std::size_t i){
      ++sent;clock->now+=(i==1300 && !before)?4500us:50us;return true;
    },error,FakeTiming{clock});
    require(!ok && cancel->requested(),"deadline protection was relaxed");
    require(error.find("at sample 1300")!=std::string::npos &&
        error.find(before?"phase=before_sample":"phase=after_sample")!=std::string::npos &&
        error.find("late_us=4500")!=std::string::npos,"deadline phase/latency not surfaced: "+error);
    require(sent==(before?1300u:1301u),"deadline violation sent extra points");
    ++cases;
  }
  std::cout<<"PASS before/after deadline diagnostics and cancellation, with unchanged 4ms threshold\n";
  std::cout<<"PASS latest_progress cases="<<cases<<"; no hardware or motion goals\n";
  return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
