#include "motion_control_pkg/miracle_m_algo.hpp"
#include "motion_control_pkg/motion/scoped_collision_scene.hpp"
#include "motion_control_pkg/motion/planning_worker_pool.hpp"
#include <rcutils/logging.h>
#include <sys/resource.h>
#include <atomic>
#include <future>
#include <iomanip>
#include <iostream>
#include <numeric>

using Path = std::vector<std::vector<double>>;
using Pose = Eigen::Isometry3d;
using Clock = std::chrono::steady_clock;
static void require(bool ok, const std::string& message) { if (!ok) throw std::runtime_error(message); }
static double seconds(Clock::time_point t) { return std::chrono::duration<double>(Clock::now()-t).count(); }
static double cpu() { timespec t; clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&t); return t.tv_sec+t.tv_nsec*1e-9; }
static long rss() { rusage r{}; getrusage(RUSAGE_SELF,&r); return r.ru_maxrss; }
static void poolTest() {
  for(unsigned count:{2U,4U}) {
    motion_control_pkg::motion::PlanningWorkerPool pool(count);
    std::mutex mutex; std::condition_variable cv; unsigned arrived=0;
    std::vector<std::thread::id> ids(count);
    pool.run([&](std::size_t i) {
        ids[i]=std::this_thread::get_id();
        std::unique_lock<std::mutex> lock(mutex); ++arrived; cv.notify_all();
        require(cv.wait_for(lock,std::chrono::seconds(2),[&]{return arrived==count;}),"workers did not overlap");
    });
    require(ids[0]!=ids[1],"workers share thread");
    std::atomic<bool> other_finished{false}; bool threw=false;
    try { pool.run([&](std::size_t i) {
        if(i==0) throw std::runtime_error("injected worker failure");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));other_finished=true;
    }); } catch(const std::runtime_error&) {threw=true;}
    require(threw && other_finished,"exception escaped before other job drained");
    for(int repeat=0;repeat<20;++repeat)pool.run([&](std::size_t i){require(ids[i]==std::this_thread::get_id(),"thread recreated");});
    std::atomic<unsigned> ran{0};pool.run([&](std::size_t i){require(i<2,"idle IK worker executed");++ran;},2);
    require(ran==2,"wrong active worker count");
  }
    std::cout<<"PASS 2/4 persistent workers overlap, drain exceptions, run two IK jobs, and remain reusable\n";
}
struct Fixture {
    MiracleMAlgo algo;
    std::vector<double> neutral,start;
    std::array<Pose,2> initial,targets;
    Fixture(const std::string& root):algo(root+"/src/onecube_robot_description/urdf/onecube_robot_description.urdf",root+"/src") {
        require(algo.getFullJointState(neutral),"neutral");
        require(algo.updateChainJointsAngle("left_arm",{.8045046960995934,-.7106088931426268,.5295298256393933,-.9450260159378988,-.37056347598195566,-.16073897211859947,-.34476481302261197})==0,"left state");
        require(algo.updateChainJointsAngle("right_arm",{-.5737369649852467,-.8629368873651962,-.5753896267099474,-.8682243487403323,.7008482975591221,.034796401984532814,.5726308588998467})==0,"right state");
        require(algo.getFullJointState(start),"start");
        for(unsigned arm=0;arm<2;++arm) {
            require(algo.getFramePoseAtConfiguration("base_link",arm?"gripper_base_r_link":"gripper_base_l_link",start,initial[arm]),"initial FK");
            targets[arm]=initial[arm]; targets[arm].translation().x()+=.02;targets[arm].translation().z()+=.04;
        }
        algo.setCollisionCheckMode(0);
    }
    int index(const std::string& name) {const auto& m=algo.getModel(); return m.joints[m.getJointId(name)].idx_q();}
};
static void regression(Fixture& f) {
    auto& a=f.algo;
    std::array<Path,2> reference,serial,parallel;
    std::array<Pose,2> starts,serial_starts;double elapsed=0;
    for(unsigned arm=0;arm<2;++arm)
        require(a.planSingleArmCartesianIkWaypoints(arm==0,f.start,f.targets[arm],.002,.02,reference[arm],starts[arm]),"reference IK");
    a.configurePlanningWorkers(1);
    require(a.planDualArmCartesianIkWaypoints(f.start,f.targets,.002,.02,serial,serial_starts,elapsed),"serial pair IK");
    require(serial==reference,"serial wrapper changed numerical output");
    a.configurePlanningWorkers(2);
    {
        // This reproduces the production outer scene lock; a worker entering
        // a public locking API would hang and trip CTest's timeout.
        motion_control_pkg::motion::ScopedCollisionScene scene(a,0,f.start);
        require(a.planDualArmCartesianIkWaypoints(f.start,f.targets,.002,.02,parallel,serial_starts,elapsed),"parallel pair IK under scene lock");
    }
    require(parallel==reference,"parallel IK differs from serial");
    for(unsigned arm=0;arm<2;++arm)require((starts[arm].matrix()-serial_starts[arm].matrix()).norm()==0,"start pose changed");
    require(!a.planDualArmCartesianIkWaypoints(f.start,f.targets,.002,.02,parallel,serial_starts,elapsed,[]{return true;}),"cancellation accepted IK");
    require(parallel[0].empty() && parallel[1].empty(),"partial IK returned after cancellation");
    require(!a.planDualArmCartesianIkWaypoints(f.start,f.targets,.002,.02,parallel,serial_starts,elapsed,
        []()->bool {throw std::runtime_error("injected IK worker exception");}),"IK worker exception did not fail closed");
    auto invalid_target=f.targets;invalid_target[1].translation().x()=std::numeric_limits<double>::quiet_NaN();
    require(!a.planDualArmCartesianIkWaypoints(f.start,invalid_target,.002,.02,parallel,serial_starts,elapsed),"invalid target accepted");
    require(a.planDualArmCartesianIkWaypoints(f.start,f.targets,.002,.02,parallel,serial_starts,elapsed),"IK not reusable after failure");
    require(parallel==reference,"IK changed after failure");
    std::cout<<"PASS dual MoveL IK exact serial equality, outer scene lock, cancellation and failure recovery\n";

    Path samples;
    for(const auto& q:reference[0])for(int repeat=0;repeat<8;++repeat)samples.push_back(q);
    require(samples.size()>=64,"FK fixture too short");
    double pe[2]{},re[2]{};std::string error[2];
    for(unsigned mode=0;mode<2;++mode) {
        a.configurePlanningWorkers(mode+1);
        require(a.validateSingleArmCartesianTrajectory(true,f.start,starts[0],f.targets[0],samples,.001,.01,pe[mode],re[mode],error[mode]),"valid FK path: "+error[mode]);
    }
    require(pe[0]==pe[1] && re[0]==re[1],"FK error metrics changed");
    auto backwards=samples;backwards[samples.size()/2]=samples.front();
    for(unsigned mode=0;mode<2;++mode) {
        a.configurePlanningWorkers(mode+1);
        require(!a.validateSingleArmCartesianTrajectory(true,f.start,starts[0],f.targets[0],backwards,.001,.01,pe[mode],re[mode],error[mode]),"regression across split accepted");
    }
    require(error[0]==error[1] && error[0].find("regressed")!=std::string::npos,"ordered FK diagnostic differs");
    auto wrong_endpoint=samples;wrong_endpoint.back()=samples[samples.size()/2];
    require(!a.validateSingleArmCartesianTrajectory(true,f.start,starts[0],f.targets[0],wrong_endpoint,.001,.01,pe[0],re[0],error[0]),"bad endpoint accepted");
    Path full(129,f.start);for(unsigned i=0;i<full.size();++i)full[i][f.index("head_yaw_joint")]=.001*i;
    std::vector<Pose> poses;
    require(a.getFramePosesAtConfigurations("base_link","gripper_base_l_link",full,poses),"batch FK");
    for(unsigned i=0;i<full.size();++i) {Pose p;require(a.getFramePoseAtConfiguration("base_link","gripper_base_l_link",full[i],p),"reference FK");require((p.matrix()-poses[i].matrix()).norm()==0,"batch FK differs");}
    full[64].pop_back();require(!a.getFramePosesAtConfigurations("base_link","gripper_base_l_link",full,poses),"bad FK dimension accepted");
    std::cout<<"PASS batch FK exact equality, progress regression across split, endpoint and input rejection\n";

    Path checks(129,f.neutral);std::size_t invalid=0;
    a.setCollisionCheckMode(1);
    for(unsigned workers:{1U,2U,4U}) {
        a.configurePlanningWorkers(workers);
        require(a.validateFullConfigurations(checks,invalid) && invalid==checks.size(),"neutral collision fixture");
        auto bad=checks;bad[64][f.index("body_joint")]=10;bad[12].pop_back();
        require(!a.validateFullConfigurations(bad,invalid) && invalid==12,"first invalid ordering");
        bad=checks;bad[70][0]=std::numeric_limits<double>::quiet_NaN();
        require(!a.validateFullConfigurations(bad,invalid) && invalid==70,"nonfinite collision input");
    }
    require(!a.validateFullConfigurations(checks,invalid,[]{return true;}),"collision cancellation");
    require(!a.validateFullConfigurations(checks,invalid,[]()->bool {throw std::runtime_error("injected collision worker exception");}),"collision worker exception did not fail closed");
    // Exercise geometry replacement with the SAME name and object/pair counts.
    // A count-only cache invalidation would miss the moved obstacle.
    Pose box=Pose::Identity();box.translation().x()=20;
    a.addExternalCollisionBox("parallel_test_obstacle",Eigen::Vector3d(10,10,10),box);
    a.setCollisionCheckMode(2);
    require(a.validateFullConfigurations(checks,invalid),"far obstacle");
    a.removeExternalCollisionObject("parallel_test_obstacle");
    box.translation().x()=0;
    a.addExternalCollisionBox("parallel_test_obstacle",Eigen::Vector3d(10,10,10),box);
    require(!a.validateFullConfigurations(checks,invalid) && invalid==0,"moved obstacle cache stale");
    require(!a.isFullQValid(a.toEigenConfig(f.neutral)),"serial obstacle reference");
    a.setCollisionCheckMode(1);require(a.validateFullConfigurations(checks,invalid),"self-only mode not propagated");
    a.setCollisionCheckMode(0);require(a.validateFullConfigurations(checks,invalid),"disabled mode not propagated");
    a.removeExternalCollisionObject("parallel_test_obstacle");
    a.setCollisionCheckMode(2);require(a.validateFullConfigurations(checks,invalid),"removed obstacle cache stale");
    std::vector<double> after;require(a.getFullJointState(after) && after==f.start,"planning changed measured model state");
    // Simultaneous callers serialize at the coordinator, never interleave pool jobs.
    auto caller=[&]{std::size_t i;return a.validateFullConfigurations(checks,i);};
    auto f1=std::async(std::launch::async,caller),f2=std::async(std::launch::async,caller);
    require(f1.get() && f2.get(),"concurrent callers failed");
    bool rejected=false;try {a.configurePlanningWorkers(3);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"unbounded worker configuration");
    std::cout<<"PASS collision modes, same-size scene replacement, first-invalid ordering, cancellation, concurrent callers and unchanged measured state\n";
}

static void comparisons(Fixture& f) {
    auto& a=f.algo;
    constexpr unsigned trials=35;
    std::array<std::array<Path,2>,trials> ik_reference;
    std::array<std::vector<Pose>,trials> fk_reference;
    std::array<bool,trials> collision_reference{};
    std::array<std::size_t,trials> invalid_reference{};
    for(unsigned workers:{1U,2U,4U}) {
        a.configurePlanningWorkers(workers);
        for(unsigned t=0;t<trials;++t) {
            auto targets=f.targets;
            for(unsigned arm=0;arm<2;++arm) {
                targets[arm].translation().x()+=.0004*(t%12);
                targets[arm].translation().z()+=.0003*(t%9);
            }
            a.setCollisionCheckMode(0);
            std::array<Path,2> paths;std::array<Pose,2> starts;double elapsed=0;
            require(a.planDualArmCartesianIkWaypoints(f.start,targets,.002,.02,paths,starts,elapsed),"comparison IK");
            if(workers==1)ik_reference[t]=std::move(paths);
            else require(paths==ik_reference[t],"comparison IK output changed");
            Path fk(127+t%3,f.start);
            for(unsigned i=0;i<fk.size();++i) {
                fk[i][f.index("Joint1_L")]+=.02*std::sin((i+t)*.1);
                fk[i][f.index("head_yaw_joint")]=.1*std::sin((i+t)*.03);
            }
            std::vector<Pose> poses;
            require(a.getFramePosesAtConfigurations("body_link",t%2?"gripper_base_l_link":"gripper_base_r_link",fk,poses),"comparison FK");
            if(workers==1)fk_reference[t]=std::move(poses);
            else for(unsigned i=0;i<poses.size();++i)
                require((poses[i].matrix()-fk_reference[t][i].matrix()).norm()==0,"comparison FK output changed");
            Path collision(127+t%3,f.neutral);
            for(unsigned i=0;i<collision.size();++i)collision[i][f.index("head_yaw_joint")]=.2*std::sin((i+t)*.03);
            if(t%7==0)collision[64][f.index("body_joint")]=10;
            a.setCollisionCheckMode(t%3);
            std::size_t invalid=0;
            const bool valid=a.validateFullConfigurations(collision,invalid);
            if(workers==1) {collision_reference[t]=valid;invalid_reference[t]=invalid;}
            else require(valid==collision_reference[t] && invalid==invalid_reference[t],"comparison collision result changed");
        }
    }
    std::cout<<"PASS 210 serial-parallel comparisons: 105 each at 2 and 4 workers; max numerical difference=0\n";
}

static void benchmark(Fixture& f) {
    auto& a=f.algo;const long baseline=rss();
    Path fk(12000,f.start),collisions(12000,f.neutral);
    for(unsigned i=0;i<fk.size();++i)fk[i][f.index("Joint1_L")]+=.01*std::sin(i*.01);
    for(const std::string stage:{"ik","fk","collision"})for(unsigned workers:{1U,2U,4U}) {
        const auto setup=Clock::now();a.configurePlanningWorkers(workers);a.setCollisionCheckMode(stage=="collision"?1:0);
        const double setup_s=seconds(setup);
        const auto run=[&] {
            if(stage=="ik") for(int job=0;job<24;++job) {
                std::array<Path,2> paths;std::array<Pose,2> starts;double elapsed;
                require(a.planDualArmCartesianIkWaypoints(f.start,f.targets,.002,.02,paths,starts,elapsed),"benchmark IK");
            } else if(stage=="fk") {
                std::vector<Pose> poses;require(a.getFramePosesAtConfigurations("base_link","gripper_base_l_link",fk,poses),"benchmark FK");
            } else {std::size_t invalid;require(a.validateFullConfigurations(collisions,invalid),"benchmark collision");}
        };
        run();std::vector<double> walls,cpus,jitter_us;
        std::atomic<bool> probe_done{false};std::size_t missed=0;
        // A synthetic host scheduling probe only. This is NOT a measurement of
        // the ROS/SDK/EtherCAT control loop, and sends no robot commands.
        std::thread probe([&] {
            auto next=Clock::now()+std::chrono::milliseconds(4);
            while(!probe_done.load()) {
                std::this_thread::sleep_until(next);
                const auto now=Clock::now();
                jitter_us.push_back(std::chrono::duration<double,std::micro>(now-next).count());
                next+=std::chrono::milliseconds(4);
                while(next<=now) {++missed;next+=std::chrono::milliseconds(4);}
            }
        });
        try {
            for(int trial=0;trial<7;++trial) {
                const auto t=Clock::now();const auto c=cpu();run();
                walls.push_back(seconds(t));cpus.push_back(cpu()-c);
            }
        } catch (...) {probe_done=true;probe.join();throw;}
        probe_done=true;probe.join();
        for(unsigned i=0;i<walls.size();++i)std::cout<<std::setprecision(12)<<"TRIAL,"<<stage<<','<<workers<<','<<i<<','<<walls[i]<<','<<cpus[i]<<'\n';
        std::sort(walls.begin(),walls.end());std::sort(cpus.begin(),cpus.end());
        std::cout<<std::setprecision(9)<<"BENCH,"<<stage<<','<<workers<<','<<walls[3]<<','<<cpus[3]<<','<<rss()<<','<<setup_s<<",baseline_rss_kib="<<baseline<<'\n';
        std::sort(jitter_us.begin(),jitter_us.end());
        if(!jitter_us.empty())std::cout<<"SYNTHETIC_4MS,"<<stage<<','<<workers<<','<<jitter_us.size()<<','
            <<jitter_us[std::min(jitter_us.size()-1,jitter_us.size()*99/100)]<<','<<jitter_us.back()<<','<<missed<<'\n';
    }
}
int main(int argc,char**argv) try {
    require(argc>=2,"expected repository path [--benchmark]");
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL);
    require(rcutils_logging_set_logger_level("MiracleMAlgo",RCUTILS_LOG_SEVERITY_FATAL)==RCUTILS_RET_OK,"logger setup");
    poolTest();Fixture f(argv[1]);regression(f);
    if(argc==3 && std::string(argv[2])=="--benchmark") { comparisons(f); benchmark(f); }
    std::cout<<"PASS parallel planning regression; no vendor SDK or dispatch linked\n";return 0;
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
