#include "motion_control_pkg/execution/ethercat_trajectory.hpp"
#include "motion_control_pkg/execution/whole_body_completion.hpp"
#include "motion_control_pkg/motion/trajectory_derivative_limits.hpp"
#include "com_interfaces/action/move_joints.hpp"
#include "com_interfaces/action/move_pose.hpp"
#include "com_interfaces/precision_arrival_goal.hpp"
#include <ruckig/ruckig.hpp>
#include <iostream>
#include <stdexcept>
#include <array>
using namespace motion_control_pkg;
using namespace execution;
using Time=JointCompletionGuard::Time;
using Status=JointCompletionGuard::Status;
void require(bool b,const std::string& msg){if(!b)throw std::runtime_error(msg);}
struct Sim {
 Time time{Time{}+std::chrono::seconds(10)};
 std::vector<double> actual,command,velocity;
 std::uint64_t seq{1};
 bool frozen{false},blocked{false};
 double lag{.08},bias{0};
 unsigned sent{0};
 void advance(Time to) {
  double dt=std::chrono::duration<double>(to-time).count();
  if(dt>0){
   for(std::size_t j=0;j<actual.size();++j){
    double next=command[j]-bias+(actual[j]-(command[j]-bias))*std::exp(-dt/lag);
    velocity[j]=(next-actual[j])/dt;actual[j]=next;
   }
   time=to;if(!frozen)++seq;
  }
 }
};
struct Timing {
 Sim* s;
 Time now()const{return s->time;}
 void sleep_until(Time t)const{s->advance(t);}
};
int main()try{
 unsigned passed=0;auto pass=[&](const char* n){std::cout<<"PASS "<<n<<'\n';++passed;};
 PrecisionPolicy policy;policy.validate();
 std::string error;
 auto t=Time{}+std::chrono::seconds(1);
 require(!com_interfaces::action::MoveJoints::Goal{}.precision_arrival &&
         !com_interfaces::action::MovePose::Goal{}.precision_arrival,
         "precision arrival must default OFF in both generated Actions");
 for (std::uint8_t part=0;part<=8;++part) for (bool pose : {false,true}) {
   require(com_interfaces::validatePrecisionArrivalGoal(false,part,pose,0,error),
           "disabled option changed ordinary admission");
   require(com_interfaces::validatePrecisionArrivalGoal(true,part,pose,0,error)==
           (pose ? part>=4 && part<=7 : part==3 || part==4),
           "unsupported precision chain accepted or valid chain rejected");
   require(!com_interfaces::validatePrecisionArrivalGoal(true,part,pose,1,error),
           "precision impedance request accepted");
 }
 pass("Action defaults OFF; only supported position-mode chains may opt in");
 const auto ordinary_policy=actionPrecisionPolicy(policy,false);
 require(!ordinary_policy.approach_enabled && ordinary_policy.completion.position_tolerance==.02 &&
         ordinary_policy.completion.stable_seconds==.1 && ordinary_policy.completion.stall_seconds==0,
         "ordinary action retained precision approach or settling criteria");
 auto custom=policy;custom.completion.position_tolerance=.001;custom.completion.stable_seconds=.3;
 custom.approach_distance=.005;custom.validate();
 const auto selected=actionPrecisionPolicy(custom,true);
 require(selected.completion.position_tolerance==.001 && selected.completion.stable_seconds==.3 &&
         selected.approach_enabled==custom.approach_enabled,"opt-in lost configured precision");
 actionPrecisionPolicy(custom,false).validate();
 require(custom.completion.position_tolerance==.001 && custom.approach_enabled,
         "per-Goal selection mutated the configured chain policy");
 pass("per-Goal policy honors custom opt-in and prevents precision leaking to the next ordinary Goal");
 for (bool enabled : {false,true}) {
   const auto active=actionPrecisionPolicy(policy,enabled);
   WholeBodyCompletionGuard whole(std::vector<double>(7,0),{},{0,0,0,0},t,
       t+std::chrono::seconds(3),active.completion);
   WholeBodyCompletionFeedback measured;
   measured.arms.arms[0].current_state=1;measured.arms.arms[0].low_speed=true;
   measured.leg_positions[3]=.015;
   Status state{};
   for(int i=0;i<=30;++i) {
     measured.arms.arms[0].feedback_sequence=i+1;measured.leg_sequence=i+1;
     state=whole.update(measured,t+std::chrono::milliseconds(i*10),error);
   }
   require(state==(enabled?Status::Waiting:Status::Settled),
           "WBC per-Action position tolerance was not applied");
   WholeBodyCompletionGuard task_gate(std::vector<double>(7,0),{},{0,0,0,0},t,
       t+std::chrono::seconds(3),active.completion);
   measured.leg_positions[3]=0;
   for(int i=0;i<=30;++i) {
     measured.arms.arms[0].feedback_sequence=i+1;measured.leg_sequence=i+1;
     state=task_gate.update(measured,t+std::chrono::milliseconds(i*10),error,false);
   }
   require(state==Status::Waiting,"precision OFF bypassed TCP/task completion checks");
   WholeBodyCompletionGuard frozen(std::vector<double>(7,0),{},{0,0,0,0},t,
       t+std::chrono::seconds(3),active.completion);
   measured.arms.arms[0].feedback_sequence=1;measured.leg_sequence=1;
   for(int i=0;i<=10;++i) state=frozen.update(measured,t+std::chrono::milliseconds(i*10),error);
   require(state==Status::Failed,"precision option bypassed advancing-frame checks");
 }
 pass("WBC 15 mrad residual passes only ordinary mode; TCP and stale-feedback fences remain active");
 for (double residual : {.019999,.02,.020001}) {
   JointCompletionGuard ordinary({0},t,t+std::chrono::seconds(2),false,ordinary_policy.completion);
   Status state{};
   for(int i=0;i<=15;++i)
     state=ordinary.update(i+1,{residual},{0},t+std::chrono::milliseconds(i*10),error);
   require(state==(residual<=.02?Status::Settled:Status::Waiting),
           "ordinary 0.02 rad boundary is inconsistent");
 }
 pass("ordinary arrival accepts the inclusive 0.02 rad boundary and rejects larger residuals");
 for (bool enabled : {false,true}) {
   unsigned planned=0,slow_segments=0;
   motion::PrecisionApproach path;
   auto plan=[&](const auto& from,const auto& to,bool slow,auto& q,auto& v,auto& acc,std::string&) {
     ++planned;slow_segments+=slow;
     q={from,to};v.assign(2,std::vector<double>(from.size(),0));acc=v;return true;
   };
   require(motion::planPrecisionApproach({0},{.2},actionPrecisionPolicy(policy,enabled),
       plan,path,error),error);
   require(planned==(enabled?2U:1U) && slow_segments==(enabled?1U:0U) &&
           (path.gate!=motion::kNoApproachGate)==enabled,
           "MoveJ OFF still inserted a slow suffix or ON lost it");
   require(path.positions.front()==std::vector<double>{0} && path.positions.back()==std::vector<double>{.2},
           "Action option changed the original endpoint");
   if (enabled) {
     require(motion::planPrecisionApproach({0},{.2},actionPrecisionPolicy(policy,false),
         plan,path,error),error);
     require(path.gate==motion::kNoApproachGate && path.positions.size()==2 && slow_segments==1,
             "an earlier precision Goal leaked its approach gate into the next ordinary Goal");
   }
 }
 pass("ordinary MoveJ uses a single normal segment; opt-in retains one precision suffix");
 // Legacy arms keep .01 rad / 100 ms. New precision is opt-in to the guard.
 JointCompletionGuard legacy({0},t,t+std::chrono::seconds(2));
 JointCompletionGuard strict({0},t,t+std::chrono::seconds(2),false,policy.completion);
 Status a{},b{};
 for(int i=0;i<=30;++i){auto now=t+std::chrono::milliseconds(i*10);a=legacy.update(i+1,{.005},{0},now,error);b=strict.update(i+1,{.005},{0},now,error);}
 require(a==Status::Settled&&b==Status::Waiting,"precision must not change legacy arm tolerance");
 pass("legacy arm unchanged; 0.005 rad rejected by precise joint guard");
 JointCompletionGuard precise({0},t,t+std::chrono::seconds(2),false,policy.completion);
 for(int i=0;i<=20;++i){b=precise.update(i+1,{.0019},{0},t+std::chrono::milliseconds(i*10),error);if(i<20)require(b==Status::Waiting,"premature success");}
 require(b==Status::Settled,"200ms fresh frames must settle");pass("0.002 rad and 200 ms measured settling");
 JointCompletionGuard stale({0},t,t+std::chrono::seconds(2),false,policy.completion);
 for(int i=0;i<=10;++i)b=stale.update(1,{0},{0},t+std::chrono::milliseconds(i*10),error);
 require(b==Status::Failed&&error.find("stopped advancing")!=std::string::npos,"cached frames must fail");
 JointCompletionGuard backward({0},t,t+std::chrono::seconds(2),false,policy.completion);
 backward.update(20,{0},{0},t,error);require(backward.update(19,{0},{0},t+std::chrono::milliseconds(10),error)==Status::Failed,"reversed feedback accepted");
 JointCompletionGuard nonfinite({0},t,t+std::chrono::seconds(2),false,policy.completion);
 require(nonfinite.update(1,{NAN},{0},t,error)==Status::Failed,"NaN accepted");pass("stale reversed and invalid feedback fail closed");
 JointCompletionGuard stalled({0},t,t+std::chrono::seconds(5),false,policy.completion);
 for(int i=0;i<=150;++i)b=stalled.update(i+1,{.005},{0},t+std::chrono::milliseconds(i*10),error);
 require(b==Status::Failed&&error.find("stalled")!=std::string::npos&&error.find("worst_joint_index=0")!=std::string::npos,"static residual must diagnose without retry");pass("bounded no-progress failure includes joint target actual tolerance");
 auto invalid=policy;invalid.completion.position_tolerance=NAN;bool threw=false;try{invalid.validate();}catch(...){threw=true;}require(threw,"NaN config accepted");
 invalid=policy;invalid.speed_scale=2;threw=false;try{invalid.validate();}catch(...){threw=true;}require(threw,"unsafe scale accepted");pass("policy validation");
 // Real Ruckig, including head URDF limits 0.08/0.05 rad/s.
 for(const std::vector<double> limits: {std::vector<double>{1.31,1.31,1.31,.61},std::vector<double>{.08,.05}}){
  const auto n=limits.size();std::vector<double> start(n,0),target(n,.2);target[0]=-.4;
  auto plan=[&](const auto& from,const auto& to,bool slow,auto& p,auto& v,auto& a,std::string& reason){
   ruckig::InputParameter<ruckig::DynamicDOFs> in(n);ruckig::Trajectory<ruckig::DynamicDOFs> traj(n);ruckig::Ruckig<ruckig::DynamicDOFs> gen(n);
   in.current_position=from;in.target_position=to;
   for(std::size_t j=0;j<n;++j){in.max_velocity[j]=slow?std::min(limits[j]*policy.speed_scale,policy.velocity_cap):limits[j];in.max_acceleration[j]=slow?policy.acceleration_cap:5;in.max_jerk[j]=slow?policy.jerk_cap:10;}
   if(gen.calculate(in,traj)<0){reason="ruckig failed";return false;}
   const std::size_t count=std::ceil(traj.get_duration()/.004);std::vector<double> q(n),qd(n),qdd(n);
   for(std::size_t i=0;i<=count;++i){traj.at_time(std::min(i*.004,traj.get_duration()),q,qd,qdd);p.push_back(q);v.push_back(qd);a.push_back(qdd);}
   p.front()=from;p.back()=to;v.front().assign(n,0);v.back().assign(n,0);a.front().assign(n,0);a.back().assign(n,0);return true;
  };
  motion::PrecisionApproach path;
  require(motion::planPrecisionApproach(start,target,policy,plan,path,error),error);
  require(path.gate>0&&path.gate<path.positions.size()-1,"missing gate");
  require(path.positions.front()==start&&path.positions.back()==target,"endpoint altered");
  for(std::size_t i=0;i<path.positions.size();++i)for(std::size_t j=0;j<n;++j){
   require(path.positions[i][j]>=std::min(start[j],target[j])-1e-9&&path.positions[i][j]<=std::max(start[j],target[j])+1e-9,"overshoot");
   if(i>path.gate)require(std::abs(path.velocities[i][j])<=std::min(limits[j]*policy.speed_scale,policy.velocity_cap)+1e-8,"tail speed bound");
  }
  require(motion::validateCommandDerivatives(path.positions,.004,limits,std::vector<double>(n,5),std::vector<double>(n,10),error),error);
  Sim sim;sim.actual=start;sim.command=start;sim.velocity.assign(n,0);Timing clock{&sim};
  auto read=[&](JointCompletionFeedback& f,std::string&){f.position=sim.actual;f.velocity=sim.velocity;f.sequence=sim.seq;f.acquired=sim.time;return true;};
  auto check=[&](std::string& reason){if(sim.blocked){reason="safety latched";return false;}return true;};
  auto send=[&](const auto& q){sim.command=q;++sim.sent;return true;};
  double final_error;
  require(executePrecisionTrajectory(path.positions,send,read,check,[](float){},sim.time+std::chrono::seconds(120),final_error,error,policy,path.gate,clock),error);
  require(final_error<=.002&&sim.sent==path.positions.size(),"settling did not reach target or resent commands");
  require(sim.command==target,"original goal changed");
  // No commands when insufficient budget, canceled or safety-latched.
  sim.actual=start;sim.command=start;sim.velocity.assign(n,0);sim.sent=0;
  require(!executePrecisionTrajectory(path.positions,send,read,check,[](float){},sim.time+std::chrono::milliseconds(50),final_error,error,policy,path.gate,clock)&&sim.sent==0,"budget must reject before sending");
  sim.blocked=true;
  require(!executePrecisionTrajectory(path.positions,send,read,check,[](float){},sim.time+std::chrono::seconds(120),final_error,error,policy,path.gate,clock)&&sim.sent==0,"latched safety sent commands");
  sim.blocked=false;
  unsigned checks=0;auto cancel=[&](std::string& reason){if(++checks>path.gate+4){reason="canceled during handover";return false;}return true;};
  require(!executePrecisionTrajectory(path.positions,send,read,cancel,[](float){},sim.time+std::chrono::seconds(120),final_error,error,policy,path.gate,clock),"cancel accepted");
  require(sim.sent==path.gate+1,"cancellation must prevent slow suffix");
  sim.actual=start;sim.command=start;sim.velocity.assign(n,0);sim.sent=0;sim.frozen=true;
  require(!executePrecisionTrajectory(path.positions,send,read,check,[](float){},sim.time+std::chrono::seconds(120),final_error,error,policy,path.gate,clock),"frozen handover accepted");
  require(sim.sent==path.gate+1&&error.find("stopped advancing")!=std::string::npos,"stale handover dispatched suffix");
  sim.actual=start;sim.command=start;sim.velocity.assign(n,0);sim.sent=0;sim.frozen=false;
  auto reject=[&](const auto& q){if(sim.sent==4)return false;return send(q);};
  require(!executePrecisionTrajectory(path.positions,reject,read,check,[](float){},sim.time+std::chrono::seconds(120),final_error,error,policy,path.gate,clock)&&sim.sent==4,"rejected command did not stop dispatch");
  pass(n==4?"real Ruckig legs/waist plus lagging feedback, exact endpoint, safety and cancel":"real Ruckig head URDF speeds plus lagging feedback, exact endpoint, safety and cancel");
  // Short moves are entirely slow; zero moves remain measured no-ops.
  motion::PrecisionApproach shortpath;
  require(motion::planPrecisionApproach(start,std::vector<double>(n,.01),policy,plan,shortpath,error)&&shortpath.gate==motion::kNoApproachGate,"short move gate");
  require(motion::planPrecisionApproach(start,start,policy,plan,shortpath,error)&&shortpath.positions.back()==start,"no-op plan");
 }
 pass("short and zero moves preserve target without extra approach");
 // Permanent 5 mrad residual fails; no synthetic offset and no second movement.
 Sim residual;residual.actual={.005};residual.command={0};residual.velocity={0};residual.bias=-.005;
 double e;unsigned sends=0;
 auto read=[&](JointCompletionFeedback& f,std::string&){f={residual.actual,residual.velocity,residual.seq,residual.time};return true;};
 require(!waitForJointCompletion(std::vector<double>{0},residual.time+std::chrono::seconds(5),policy,read,[](std::string&){return true;},e,error,Timing{&residual}),"static bias falsely succeeded");
 require(error.find("stalled")!=std::string::npos&&residual.command[0]==0&&sends==0,"static error caused target offset");pass("static residual fails without automatic offset or movement");
 for (bool enabled : {false,true}) {
   Sim sample;sample.actual={.005};sample.command={.005};sample.velocity={0};
   auto read_sample=[&](JointCompletionFeedback& f,std::string&){f={sample.actual,sample.velocity,sample.seq,sample.time};return true;};
   const bool ok=waitForJointCompletion(std::vector<double>{0},sample.time+std::chrono::seconds(5),
       actionPrecisionPolicy(policy,enabled),read_sample,[](std::string&){return true;},e,error,Timing{&sample});
   require(ok==!enabled,"HEAD/LEG per-Goal measured settling selection failed");
   require(sample.sent==0 && sample.command==std::vector<double>{.005},"settling changed commands");
 }
 pass("same HEAD/LEG residual settles with OFF and fails bounded precision ON without offset or replay");
 std::cout<<"PASS precision_arrival scenarios="<<passed<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
