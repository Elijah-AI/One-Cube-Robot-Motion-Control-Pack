#include "motion_control_pkg/miracle_m_algo.hpp"
#include "motion_control_pkg/motion/joint_space_control.hpp"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>

static void check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
int main(int argc, char** argv) try {
  check(argc == 2, "workspace argument required");
  const std::string root = argv[1];
  const auto urdf = root + "/src/onecube_robot_description/urdf/onecube_robot_description.urdf";
  MiracleMAlgo algo(urdf, root + "/src");
  algo.configurePlanningWorkers(2);
  const auto original_lower = algo.getModel().lowerPositionLimit;
  const auto original_upper = algo.getModel().upperPositionLimit;
  motion_control_pkg::wbc::PositionLimitOverrides limits;
  const std::array<double,7> lower{-178,-120,-178,-145,-178,-60,-90};
  const std::array<double,7> upper{178,120,178,60,178,60,90};
  constexpr double rad = 0.017453292519943295769;
  for (const char side : {'L','R'}) for (int j=0;j<7;++j)
    limits["Joint"+std::to_string(j+1)+"_"+side]={lower[j]*rad,upper[j]*rad};
  auto bad=limits;bad["Joint7_R"].second=std::numeric_limits<double>::quiet_NaN();
  bool rejected=false;
  try {algo.setSdkArmPositionLimits(bad);}catch(const std::invalid_argument&){rejected=true;}
  check(rejected && algo.getModel().lowerPositionLimit==original_lower &&
        algo.getModel().upperPositionLimit==original_upper,"invalid update must be atomic");
  bad=limits;bad.erase("Joint7_R");bad["body_joint"]={-3.2,3.2};rejected=false;
  try {algo.setSdkArmPositionLimits(bad);}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"SDK override must not accept body/head joints");
  algo.setSdkArmPositionLimits(limits);
  for (pinocchio::JointIndex j=1;j<algo.getModel().joints.size();++j) {
    const auto index=algo.getModel().joints[j].idx_q();const auto name=algo.getModel().names[j];
    const auto entry=limits.find(name);
    check(algo.getModel().lowerPositionLimit[index] == (entry==limits.end()?original_lower[index]:entry->second.first),"lower limit source");
    check(algo.getModel().upperPositionLimit[index] == (entry==limits.end()?original_upper[index]:entry->second.second),"upper limit source");
  }
  std::vector<double> lo(7),hi(7);check(algo.getChainJointsLimt(0,lo,hi),"chain limits");
  const auto effective=motion_control_pkg::motion::intersectPositionLimits(lo,hi,lo,hi);
  check(effective.valid && std::abs(effective.upper[0]-178*rad)<1e-12,"execution limit must retain SDK 178 degrees");
  std::vector<double> q;check(algo.getFullJointState(q),"full state");
  const auto j1=algo.getModel().joints[algo.getModel().getJointId("Joint1_L")].idx_q();
  algo.setCollisionCheckMode(0);q[j1]=177*rad;
  std::size_t invalid=0;
  check(algo.validateFullConfigurations({q},invalid),"177 degrees must pass SDK arm position validation");
  auto beyond=q;beyond[j1]=178.001*rad;
  check(!algo.validateFullConfigurations({beyond},invalid),"SDK upper bound still enforced");
  const auto payload=IkRecoveryAccess::sdkArmPositionLimits(algo);
  MiracleMAlgo worker_model(urdf,root+"/src");
  IkRecoveryAccess::applySdkArmPositionLimits(worker_model,payload);
  check(IkRecoveryAccess::signature(algo)==IkRecoveryAccess::signature(worker_model),"recovery model signature must include identical SDK limits");

  motion_control_pkg::motion::IkRecoveryOptions options;
  options.workers=2;options.worker_executable=(std::filesystem::read_symlink("/proc/self/exe").parent_path()/"ik_recovery_worker").string();
  algo.configureIkRecovery(options);
  check(algo.ikRecoveryStats().status=="ready","real broker startup must accept SDK model signature");
  rejected=false;try{algo.setSdkArmPositionLimits(limits);}catch(const std::logic_error&){rejected=true;}
  check(rejected,"live recovery model limits cannot be mutated");

  motion_control_pkg::wbc::StaticBalanceOptions balance;
  balance.support_links={"base_front_wheel_1_pitch_link","base_rear_wheel_1_pitch_link","base_front_wheel_2_link","base_rear_wheel_2_link"};
  balance.unknown_steering={"base_front_wheel_1_yaw_joint","base_rear_wheel_1_yaw_joint"};
  balance.inset=0.;balance.steering_uncertainty_margin_enabled=false;
  algo.configureStaticBalance(balance);
  auto snapshot=algo.staticBalance()->model().neutralSnapshot();
  for(std::size_t i=0;i<snapshot.names.size();++i)
    if(snapshot.names[i]=="Joint1_L")snapshot.positions[i]=177*rad;
  check(algo.staticBalance()->model().evaluate(snapshot).com.allFinite(),"WBC COM model must accept SDK-valid arm positions outside URDF");
  for(std::size_t i=0;i<snapshot.names.size();++i)
    if(snapshot.names[i]=="Joint1_L")snapshot.positions[i]=178.001*rad;
  rejected=false;try{algo.staticBalance()->model().evaluate(snapshot);}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"WBC COM model still rejects SDK limit violations");
  std::cout<<"PASS SDK arm position limits: atomic validation, arms-only override, 177 degree acceptance, out-of-range rejection, real recovery broker, WBC propagation\n";
  return 0;
} catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
