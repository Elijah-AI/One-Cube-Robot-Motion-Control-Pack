#include "motion_control_pkg/miracle_m_algo.hpp"
#include "motion_control_pkg/motion/ik_recovery_transport.hpp"
#include <rcutils/logging.h>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <unistd.h>

using namespace motion_control_pkg::motion;
using Pose=Eigen::Isometry3d;
static void check(bool value,const std::string& error){if(!value)throw std::runtime_error(error);}
static std::vector<double> values(const Json::Value& array){std::vector<double> q;for(const auto& x:array)q.push_back(x.asDouble());return q;}
static Pose pose(const Json::Value& array){check(array.size()==16,"pose dimension");Pose p;for(int i=0;i<16;++i)p.matrix().data()[i]=array[i].asDouble();return p;}
static double seconds(IkClock::time_point begin){return std::chrono::duration<double>(IkClock::now()-begin).count();}
// A ROS process can already have hundreds of open DDS/device descriptors.
// Exercise spawning with the low descriptors occupied, including the broker's
// fixed descriptor number, without leaking these descriptors into the worker.
struct BusyDescriptors {
  std::vector<int> descriptors;
  BusyDescriptors() {
    while(true) {
      const int fd=open("/dev/null",O_RDONLY|O_CLOEXEC);
      check(fd>=0,"open descriptor fixture");
      if(fd>=198){close(fd);break;}
      descriptors.push_back(fd);
    }
  }
  ~BusyDescriptors(){for(int fd:descriptors)close(fd);}
};
static void topology() {
  cpu_set_t mask;CPU_ZERO(&mask);check(sched_getaffinity(0,sizeof(mask),&mask)==0,"CPU affinity");
  std::vector<int> cpus;for(int i=0;i<CPU_SETSIZE;++i)if(CPU_ISSET(i,&mask))cpus.push_back(i);
  if(cpus.size()>=6) {
    const auto root=std::filesystem::temp_directory_path()/("ik_topology_test_"+std::to_string(getpid()));
    for(int cpu:cpus){const auto dir=root/("cpu"+std::to_string(cpu))/"topology";std::filesystem::create_directories(dir);std::ofstream file(dir/"thread_siblings_list");file<<(cpu==cpus[0]||cpu==cpus[1]?std::to_string(cpus[0])+","+std::to_string(cpus[1]):std::to_string(cpu));}
    const auto selected=selectIkRecoveryCpus(4,{},root.string());
    check(selected==std::vector<int>(cpus.begin()+2,cpus.begin()+6),"hybrid topology must reserve P-core sibling pairs");
    std::filesystem::remove_all(root);
  }
  bool rejected=false;try{selectIkRecoveryCpus(2,{cpus.front(),cpus.front()});}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"duplicate CPU IDs rejected");
  std::cout<<"PASS hybrid E-core selection and explicit affinity validation\n";
}

int main(int argc,char** argv) try {
  check(argc==2,"workspace argument required");topology();
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL);
  const std::string root=argv[1];
  Json::Value fixtures;std::ifstream input(root+"/src/motion_control_pkg/test/fixtures/ik_recovery_failures.json");input>>fixtures;check(fixtures.size()==4,"failure fixtures");
  auto fixture=[&](const std::string& name){for(const auto& x:fixtures)if(x["name"]==name)return x;throw std::runtime_error("missing fixture");};
  const auto arm=fixture("left_arm_pose__failure_0"),dual=fixture("dual_arm_pose__failure_0");
  auto start=values(arm["start_q"]),known=values(arm["known_target_q"]);const auto goal=pose(arm["left_goal_column_major"]);
  MiracleMAlgo algo(root+"/src/onecube_robot_description/urdf/onecube_robot_description.urdf",root+"/src");algo.setCollisionCheckMode(1);
  std::vector<double> out;
  check(!algo.solveSingleArmCartesianIk(true,start,goal,out),"baseline fixture must fail without recovery");
  check(!algo.solveDualArmCartesianIk(values(dual["start_q"]),pose(dual["left_goal_column_major"]),pose(dual["right_goal_column_major"]),out),"baseline dual fixture must fail");
  IkRecoveryOptions options;{BusyDescriptors busy;algo.configureIkRecovery(options);}
  check(algo.ikRecoveryStats().cpus.size()==std::min<std::size_t>(8,selectIkRecoveryCpus(8,{}).size()),"worker count");
  check(algo.solveSingleArmCartesianIk(true,known,goal,out),"primary success");
  check(algo.ikRecoveryStats().requests==0,"successful primary IK must not dispatch recovery");
  check(algo.solveSingleArmCartesianIk(true,start,goal,out),"failed primary single arm recovered");
  auto stats=algo.ikRecoveryStats();check(stats.requests==1&&stats.successes==1&&stats.attempted<=24&&stats.attempted>=1,"24-seed recovery counters");
  std::vector<double> full;check(algo.mergeSingleArmJoints(true,start,out,full),"merge recovery result");
  Pose actual;check(algo.getFramePoseAtConfiguration("base_link","gripper_base_l_link",full,actual),"recovered FK");
  check((actual.translation()-goal.translation()).norm()<.000051&&Eigen::Quaterniond(actual.rotation()).angularDistance(Eigen::Quaterniond(goal.rotation()))<.000051,"recovered endpoint");
  check(algo.isFullQValid(algo.toEigenConfig(full)),"recovered collision check");
  for(int i=0;i<algo.getModel().nq;++i) {
    bool active=false;for(int j=1;j<=7;++j)active=active||i==algo.getModel().joints[algo.getModel().getJointId("Joint"+std::to_string(j)+"_L")].idx_q();
    if(!active)check(full[i]==start[i],"single-arm frozen joint");
  }
  check(algo.solveDualArmCartesianIk(values(dual["start_q"]),pose(dual["left_goal_column_major"]),pose(dual["right_goal_column_major"]),out),"dual arm recovery");
  std::cout<<"PASS primary-success bypass, real single/dual failure recovery, 24 seeds, FK and frozen joints\n";

  const auto before=algo.ikRecoveryStats().requests;
  {IkPlanningStopScope cancel([]{return true;});check(!algo.solveSingleArmCartesianIk(true,start,goal,out),"pre-cancel rejected");}
  auto invalid=start;invalid[0]=std::numeric_limits<double>::quiet_NaN();check(!algo.solveSingleArmCartesianIk(true,invalid,goal,out),"invalid input rejected");
  {IkRecoverySuppressionScope continuation;check(!algo.solveSingleArmCartesianIk(true,start,goal,out),"continuation must not receive distant retry");}
  check(algo.ikRecoveryStats().requests==before,"cancel/invalid/continuation must not dispatch");
  Pose obstacle=Pose::Identity();obstacle.translation()<<20,20,20;
  algo.addExternalCollisionBox("recovery_scene",Eigen::Vector3d(.1,.1,.1),obstacle);algo.setCollisionCheckMode(2);
  check(algo.solveSingleArmCartesianIk(true,start,goal,out),"far scene snapshot recovery");
  algo.removeExternalCollisionObject("recovery_scene");obstacle.translation()=goal.translation();
  algo.addExternalCollisionBox("recovery_scene",Eigen::Vector3d(.2,.2,.2),obstacle);
  check(!algo.solveSingleArmCartesianIk(true,start,goal,out),"same-name moved obstacle must reject endpoint");
  algo.removeExternalCollisionObject("recovery_scene");
  check(algo.solveSingleArmCartesianIk(true,start,goal,out),"pool reuses updated scene after removal");
  std::cout<<"PASS cancellation/input gates, continuation exclusion, scene replacement and reuse\n";

  // Exact octree payload roundtrip, including collision-pair names and mode.
  auto cloud=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for(int i=0;i<5;++i)cloud->push_back(pcl::PointXYZ(20.f+.01f*i,20.f,20.f));
  algo.addExternalCollisionPointcloud("recovery_cloud",cloud);
  const auto scene=IkRecoveryAccess::scene(algo);IkRecoveryAccess::applyScene(algo,scene);
  auto roundtrip=IkRecoveryAccess::scene(algo);roundtrip.removeMember("revision");auto expected=scene;expected.removeMember("revision");
  check(ik_transport::encode(roundtrip)==ik_transport::encode(expected),"octree scene roundtrip");
  check(algo.solveSingleArmCartesianIk(true,start,goal,out),"pointcloud scene passed through workers");algo.removeExternalCollisionObject("recovery_cloud");algo.setCollisionCheckMode(1);

  const auto whole=fixture("whole_left__failure_0");Eigen::VectorXd result;
  const auto whole_start=algo.toEigenConfig(values(whole["start_q"]));const auto whole_goal=pose(whole["left_goal_column_major"]);
  check(algo.solveWholeBodyIK(whole_start,whole_goal,std::nullopt,result,{1.,1.,1.,.2},0.,.000001,1,true),"whole-body failed primary recovery");
  const auto waist=fixture("whole_left_waist__failure_0");const auto waist_start=algo.toEigenConfig(values(waist["start_q"]));
  check(algo.solveWholeBodyIK(waist_start,pose(waist["left_goal_column_major"]),std::nullopt,result,{1.,1.,1.,.2},0.,.000001,1,true,true),"waist-only recovery");
  for(const auto name:{"leg_joint1","leg_joint2","leg_joint3"}){auto j=algo.getModel().joints[algo.getModel().getJointId(name)].idx_q();check(result[j]==waist_start[j],"waist-only frozen leg");}
  std::cout<<"PASS octree snapshot, whole-body and waist-only recovery\n";

  Pose unreachable=whole_goal;unreachable.translation().x()+=10.;
  std::atomic<bool> cancel{false};std::thread trigger([&]{std::this_thread::sleep_for(std::chrono::milliseconds(80));cancel=true;});
  const auto begin=IkClock::now();bool cancelled_result;
  {IkPlanningStopScope context([&]{return cancel.load();});cancelled_result=algo.solveWholeBodyIK(whole_start,unreachable,std::nullopt,result,{1.,1.,1.,.2},0.,.000001,1,true);}
  trigger.join();check(!cancelled_result&&seconds(begin)<1.2,"in-flight cancellation latency");
  check(algo.solveSingleArmCartesianIk(true,start,goal,out),"pool reusable after cancellation");
  const auto exhausted=IkClock::now();
  check(!algo.solveWholeBodyIK(whole_start,unreachable,std::nullopt,result,{1.,1.,1.,.2},0.,.000001,1,true),"unreachable target fails");
  check(seconds(exhausted)<3.0,"bounded recovery overall budget");stats=algo.ikRecoveryStats();check(stats.attempted<=24&&stats.timed_out>0,"timeout accounting");
  std::cout<<"PASS active cancellation, pool reuse, unreachable bound and timeout counters\n";

  std::ifstream children("/proc/self/task/"+std::to_string(getpid())+"/children");pid_t broker=-1;children>>broker;
  check(broker>0&&std::filesystem::read_symlink("/proc/"+std::to_string(broker)+"/exe").filename()=="ik_recovery_worker","owned broker identity");
  kill(broker,SIGKILL);
  check(!algo.solveSingleArmCartesianIk(true,start,goal,out),"broker crash must fail closed");
  check(algo.solveSingleArmCartesianIk(true,known,goal,out),"primary path survives recovery broker failure");
  options.enabled=false;algo.configureIkRecovery(options);check(algo.ikRecoveryStats().status=="disabled","disable cleanup");
  std::cout<<"PASS broker crash isolation, unaffected primary path and cleanup\n";
  return 0;
}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
