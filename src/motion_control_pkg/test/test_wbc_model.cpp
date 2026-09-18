#include "motion_control_pkg/wbc/model.hpp"
#include "motion_control_pkg/wbc/static_balance.hpp"
#include "motion_control_pkg/wbc/zmp.hpp"
#include "motion_control_pkg/wbc/trajectory.hpp"
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace wbc = motion_control_pkg::wbc;
namespace {
const std::vector<std::string> wheels{
  "base_front_wheel_1_pitch_link", "base_rear_wheel_1_pitch_link",
  "base_front_wheel_2_link", "base_rear_wheel_2_link"};
std::size_t index(const wbc::JointSnapshot& s, const std::string& name) {
  return std::distance(s.names.begin(), std::find(s.names.begin(), s.names.end(), name));
}
std::vector<wbc::JointSpec> stateSpecs() {
  return {{"a", -1, 1, 2, 3, false}, {"b", -1, 1, 2, 3, false},
          {"locked", 0.2, 0.2, 0, 0, true}};
}
}

TEST(WbcState, MissingJointsAreInvalidNotZero) {
  wbc::WholeBodyState state(stateSpecs());
  std::string error;
  ASSERT_TRUE(state.update({"a"}, {0.4}, {0.1}, 10, error));
  auto s = state.snapshot(10.01, 0.1, 0.02);
  EXPECT_FALSE(s.position_valid);
  EXPECT_EQ(s.missing, std::vector<std::string>{"b"});
  EXPECT_TRUE(std::isnan(s.positions[1]));
  EXPECT_DOUBLE_EQ(s.positions[2], 0.2);
}

TEST(WbcState, LiveClockIsReadInsideSnapshotAndFutureStillRejected) {
  wbc::WholeBodyState state(stateSpecs()); std::string error;
  ASSERT_TRUE(state.update({"a","b"},{0,0},{0,0},10.01,error));
  // An earlier clock sample is deliberately stale/future; a live clock is
  // sampled by the snapshot instead, without relaxing future validation.
  EXPECT_FALSE(state.snapshot(10,.15,.06).position_valid);
  int calls=0;
  const auto live=state.snapshotNow([&]{++calls; return 10.02;},.15,.06);
  EXPECT_TRUE(live.position_valid); EXPECT_TRUE(live.velocity_valid); EXPECT_EQ(calls,1);
  EXPECT_FALSE(state.snapshotNow([]{return 10.;},.15,.06).position_valid);
  EXPECT_THROW(state.snapshotNow({},.15,.06),std::invalid_argument);
}

TEST(WbcReference, CubicInterpolationAndAnalyticDerivatives) {
  wbc::CubicReference curve({{0,.2},{.1,.3},{.05,.6},{.2,.1}},.2);
  EXPECT_NEAR(curve.at(.4).positions[0],.05,1e-14);
  const double h=1e-6;
  for (double t:{.1,.2,.29,.4,.55}) {
    auto s=curve.at(t), lo=curve.at(t-h), hi=curve.at(t+h);
    for (std::size_t j=0;j<2;++j) {
      EXPECT_NEAR(s.velocities[j],(hi.positions[j]-lo.positions[j])/(2*h),1e-7);
      EXPECT_NEAR(s.accelerations[j],(hi.velocities[j]-lo.velocities[j])/(2*h),2e-4);
    }
  }
  EXPECT_THROW(wbc::CubicReference({{0},{1,2}},.1),std::invalid_argument);
  EXPECT_THROW(curve.at(std::numeric_limits<double>::quiet_NaN()),std::invalid_argument);
}

TEST(WbcReference, PreparedReferenceAndFailureAtomicity) {
  wbc::StaticBalance balance(WBC_TEST_URDF,{wheels,{"base_front_wheel_1_yaw_joint","base_rear_wheel_1_yaw_joint"},.02,.01});
  auto start=balance.model().neutralSnapshot(); auto end=start.positions;
  end[index(start,"leg_joint1")]+=.03;
  const std::vector<std::vector<double>> path{start.positions,end};
  std::vector<double> v(start.names.size(),.2), a(start.names.size(),1), j(start.names.size(),5);
  for (std::size_t k=0;k<v.size();++k) if (!balance.model().joints()[k].locked)
    v[k]=std::min(v[k],balance.model().joints()[k].velocity);
  auto result=wbc::prepareReference(balance,start,path,v,a,j);
  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_GT(result.minimum_zmp_margin,0);
  EXPECT_EQ(result.checked_samples,2*result.samples.size()-1);
  for (const auto& s:{result.samples.front(),result.samples.back()}) {
    for (auto value:s.velocities) EXPECT_NEAR(value,0,1e-12);
    for (auto value:s.accelerations) EXPECT_NEAR(value,0,1e-12);
  }
  auto mid=result.evaluate(result.duration*.37), lo=result.evaluate(result.duration*.37-1e-5), hi=result.evaluate(result.duration*.37+1e-5);
  const auto leg=index(start,"leg_joint1");
  EXPECT_NEAR(mid.velocities[leg],(hi.positions[leg]-lo.positions[leg])/2e-5,1e-6);
  EXPECT_NEAR(mid.accelerations[leg],(hi.velocities[leg]-lo.velocities[leg])/2e-5,1e-6);
  auto reject=wbc::prepareReference(balance,start,path,v,a,j,{},[](const auto&,std::string& error){error="injected geometry rejection";return false;});
  EXPECT_FALSE(reject.valid); EXPECT_TRUE(reject.samples.empty()); EXPECT_FALSE(reject.evaluate);
  auto assumed=start; assumed.assumed={"base_front_wheel_1_yaw_joint"};
  EXPECT_FALSE(wbc::prepareReference(balance,assumed,path,v,a,j).valid);
  auto moving=start; moving.velocities[leg]=.02;
  EXPECT_FALSE(wbc::prepareReference(balance,moving,path,v,a,j).valid);
  wbc::ReferenceOptions tiny; tiny.maximum_duration=.001;
  EXPECT_FALSE(wbc::prepareReference(balance,start,path,v,a,j,tiny).valid);
  auto steering=end; steering[index(start,"base_front_wheel_1_yaw_joint")]=.01;
  EXPECT_FALSE(wbc::prepareReference(balance,start,{start.positions,steering},v,a,j).valid);
}

TEST(WbcReference, DynamicFailureRetimesButBudgetNeverFallsBack) {
  // Tight (yet nonempty) support policy and fast small arm movement: geometry
  // remains valid but transient angular momentum must not escape the region.
  wbc::StaticBalance balance(WBC_TEST_URDF,{wheels,{"base_front_wheel_1_yaw_joint","base_rear_wheel_1_yaw_joint"},.075,.01});
  auto start=balance.model().neutralSnapshot(); auto end=start.positions;
  end[index(start,"Joint4_L")]=.02;
  std::vector<double> v(start.names.size(),1), a(start.names.size(),1000), j(start.names.size(),1000000);
  for (std::size_t k=0;k<v.size();++k) if (!balance.model().joints()[k].locked)
    v[k]=balance.model().joints()[k].velocity;
  wbc::ReferenceOptions no_scale; no_scale.maximum_zmp_scale=1;
  auto rejected=wbc::prepareReference(balance,start,{start.positions,end},v,a,j,no_scale);
  EXPECT_FALSE(rejected.valid); EXPECT_TRUE(rejected.samples.empty()); EXPECT_FALSE(rejected.evaluate);
  EXPECT_NE(rejected.error.find("budget"),std::string::npos) << rejected.error;
  wbc::ReferenceOptions larger_budget; larger_budget.maximum_zmp_scale=100;
  auto accepted=wbc::prepareReference(balance,start,{start.positions,end},v,a,j,larger_budget);
  ASSERT_TRUE(accepted.valid) << accepted.error;
  EXPECT_GT(accepted.zmp_time_scale,1);
  EXPECT_GE(accepted.minimum_zmp_margin,0);
}
TEST(WbcState, PacketOrderAndVelocityValidity) {
  wbc::WholeBodyState state(stateSpecs()); std::string error;
  ASSERT_TRUE(state.update({"b", "a"}, {-0.2, 0.4}, {}, 10, error));
  auto s = state.snapshot(10.01, 0.1, 0.02);
  EXPECT_TRUE(s.position_valid); EXPECT_FALSE(s.velocity_valid);
  EXPECT_DOUBLE_EQ(s.positions[0], 0.4);
  ASSERT_TRUE(state.update({"b", "a"}, {-0.1, 0.5}, {0.2, 0.3}, 10.02, error));
  s = state.snapshot(10.03, 0.1, 0.02);
  EXPECT_TRUE(s.velocity_valid); EXPECT_DOUBLE_EQ(s.velocities[0], 0.3);
}
TEST(WbcState, StaleFutureAndSkewRejected) {
  wbc::WholeBodyState state(stateSpecs()); std::string error;
  ASSERT_TRUE(state.update({"a"}, {0}, {0}, 10, error));
  ASSERT_TRUE(state.update({"b"}, {0}, {0}, 10.05, error));
  EXPECT_FALSE(state.snapshot(10.06, 0.1, 0.02).position_valid);
  EXPECT_FALSE(state.snapshot(11, 0.1, 1).position_valid);
  EXPECT_FALSE(state.snapshot(9.9, 0.1, 1).position_valid);
  EXPECT_TRUE(state.snapshot(10.06, 0.1, 0.1).position_valid);
}
TEST(WbcState, InvalidPacketDoesNotPartiallyUpdate) {
  wbc::WholeBodyState state(stateSpecs()); std::string error;
  ASSERT_TRUE(state.update({"a", "b"}, {0, 0}, {}, 10, error));
  EXPECT_FALSE(state.update({"a", "b"}, {0.5, 2}, {}, 11, error));
  EXPECT_DOUBLE_EQ(state.snapshot(10, 0.1, 0.1).positions[0], 0);
  EXPECT_FALSE(state.update({"a", "a"}, {0, 0}, {}, 11, error));
  EXPECT_FALSE(state.update({"unknown"}, {0}, {}, 11, error));
  EXPECT_FALSE(state.update({"a"}, {0.5}, {}, 10, error));
  EXPECT_FALSE(state.update({"a"}, {0}, {0, 0}, 11, error));
  EXPECT_FALSE(state.update({"a"}, {std::numeric_limits<double>::quiet_NaN()}, {}, 11, error));
  EXPECT_FALSE(state.update({"locked"}, {0.3}, {}, 11, error));
  EXPECT_FALSE(state.update({"locked"}, {0.2}, {0.1}, 11, error));
  state.clear(); EXPECT_FALSE(state.snapshot(1, 0.1, 0.1).position_valid);
  ASSERT_TRUE(state.update({"a", "b"}, {0, 0}, {}, 1, error));
}
TEST(WbcState, PolicyValidation) {
  wbc::WholeBodyState state(stateSpecs());
  EXPECT_THROW(state.snapshot(0, -1, 0.1), std::invalid_argument);
  EXPECT_THROW(state.snapshot(0, 1, -0.1), std::invalid_argument);
}

TEST(WbcZmp, StaticLimitAndNonzeroPlane) {
  wbc::CentroidalModel model(WBC_TEST_URDF); wbc::SupportModel support(WBC_TEST_URDF,wheels);
  const auto state=model.neutralSnapshot();
  const auto d=model.dynamics(state,std::vector<double>(state.names.size(),0));
  const auto s=support.evaluate(model.evaluate(state),.02);
  const auto z=wbc::predictZmp(d,s);
  ASSERT_TRUE(z.valid); EXPECT_NEAR(z.point.x(),d.com.x(),1e-12);
  EXPECT_NEAR(z.point.y(),d.com.y(),1e-12); EXPECT_DOUBLE_EQ(z.point.z(),s.plane_z);
  EXPECT_NEAR(z.required_force.z(),model.mass()*9.81,1e-9);
  EXPECT_NEAR(z.margin,s.signed_margin,1e-12);
}

TEST(WbcZmp, MomentumRateAndLIPMSigns) {
  wbc::CentroidalModel model(WBC_TEST_URDF); wbc::SupportModel support(WBC_TEST_URDF,wheels);
  const auto state=model.neutralSnapshot(); auto d=model.dynamics(state,std::vector<double>(state.names.size(),0));
  const auto s=support.evaluate(model.evaluate(state),0);
  d.acceleration=Eigen::Vector3d(1,-.5,0); d.angular_momentum_rate=Eigen::Vector3d(2,3,0);
  const auto z=wbc::predictZmp(d,s); ASSERT_TRUE(z.valid);
  EXPECT_NEAR(z.point.x(),d.com.x()-(d.com.z()-s.plane_z)/9.81-3/(d.mass*9.81),1e-12);
  EXPECT_NEAR(z.point.y(),d.com.y()+.5*(d.com.z()-s.plane_z)/9.81+2/(d.mass*9.81),1e-12);
  d.acceleration.z()=-9.81; EXPECT_FALSE(wbc::predictZmp(d,s).valid);
  d.acceleration.z()=-10; EXPECT_FALSE(wbc::predictZmp(d,s).valid);
}

TEST(WbcZmp, CompleteStateAndExplicitSteeringRequired) {
  wbc::CentroidalModel model(WBC_TEST_URDF); auto s=model.neutralSnapshot();
  std::vector<double> a(s.names.size(),0);
  s.velocity_valid=false; EXPECT_THROW(model.dynamics(s,a),std::invalid_argument);
  s.velocity_valid=true; s.assumed={"base_front_wheel_1_yaw_joint"};
  EXPECT_THROW(model.dynamics(s,a),std::invalid_argument);
  s.assumed.clear(); EXPECT_THROW(model.dynamics(s,{}),std::invalid_argument);
  a[0]=std::numeric_limits<double>::quiet_NaN(); EXPECT_THROW(model.dynamics(s,a),std::invalid_argument);
  a[0]=0; a[index(s,"base_front_wheel_1_pitch_joint")]=.1;
  EXPECT_THROW(model.dynamics(s,a),std::invalid_argument);
}

TEST(WbcZmp, MomentumDerivativeMatchesFiniteDifference) {
  wbc::CentroidalModel model(WBC_TEST_URDF); auto s=model.neutralSnapshot();
  std::vector<double> a(s.names.size(),0);
  for (std::size_t i=0; i<s.names.size(); ++i) if (!model.joints()[i].locked) {
    s.velocities[i]=.1*std::sin(double(i)); a[i]=.2*std::cos(double(i));
  }
  const auto d=model.dynamics(s,a); auto plus=s, minus=s; const double dt=1e-5;
  for (std::size_t i=0;i<s.names.size();++i) {
    plus.positions[i]+=dt*s.velocities[i]+.5*dt*dt*a[i];
    minus.positions[i]+=-dt*s.velocities[i]+.5*dt*dt*a[i];
    plus.velocities[i]+=dt*a[i]; minus.velocities[i]-=dt*a[i];
  }
  const auto dp=model.dynamics(plus,a), dm=model.dynamics(minus,a);
  EXPECT_LT(((dp.angular_momentum-dm.angular_momentum)/(2*dt)-d.angular_momentum_rate).norm(),1e-6);
  EXPECT_LT(((dp.velocity-dm.velocity)/(2*dt)-d.acceleration).norm(),1e-6);
}

TEST(WbcState, ExplicitAssumptionAndTimestampedVelocity) {
  wbc::StatePolicy policy; policy.assumed_positions["b"] = 0;
  policy.differentiate_joints = {"a"};
  wbc::WholeBodyState state(stateSpecs(), policy); std::string error;
  ASSERT_TRUE(state.update({"a"}, {0}, {}, 1, error));
  auto s = state.snapshot(1, .1, .02);
  EXPECT_TRUE(s.position_valid); EXPECT_FALSE(s.velocity_valid);
  EXPECT_EQ(s.assumed, std::vector<std::string>{"b"});
  ASSERT_TRUE(state.update({"a"}, {.01}, {}, 1.01, error));
  s = state.snapshot(1.01, .1, .02);
  EXPECT_TRUE(s.velocity_valid); EXPECT_NEAR(s.velocities[0], 1, 1e-10);
  EXPECT_EQ(s.estimated_velocity, std::vector<std::string>{"a"});
  ASSERT_TRUE(state.update({"a"}, {.02}, {}, 1.02, error));
  EXPECT_NEAR(state.snapshot(1.02,.1,.02).velocities[0], 1, 1e-10);
  ASSERT_TRUE(state.update({"a"}, {.5}, {}, 1.03, error));
  EXPECT_FALSE(state.snapshot(1.03,.1,.02).velocity_valid);
  ASSERT_TRUE(state.update({"a"}, {.5}, {}, 2, error));
  EXPECT_FALSE(state.snapshot(2,.1,.02).velocity_valid);
  EXPECT_FALSE(state.snapshot(3,.1,.02).position_valid);
}

TEST(WbcBalance, SteeringBoundsAndRobustRegion) {
  wbc::StaticBalanceOptions options; options.support_links = wheels;
  options.unknown_steering = {"base_front_wheel_1_yaw_joint", "base_rear_wheel_1_yaw_joint"};
  wbc::StaticBalance balance(WBC_TEST_URDF, options);
  auto state = balance.model().neutralSnapshot();
  const auto nominal = balance.model().evaluate(state);
  const auto linear = balance.evaluate(state);
  for (std::size_t i = 0; i < state.names.size(); ++i) {
    if (balance.model().joints()[i].locked) continue;
    auto plus = state, minus = state;
    plus.positions[i] += 1e-6; minus.positions[i] -= 1e-6;
    EXPECT_LT(((balance.evaluate(plus).com-balance.evaluate(minus).com)/2e-6-linear.jacobian.col(i)).norm(), 1e-6);
  }
  EXPECT_GT(balance.evaluate(state).margin, 0);
  std::cout << "steering COM bound=" << balance.bounds().com_xy
            << " support bound=" << balance.bounds().support_xy
            << " robust neutral margin=" << balance.evaluate(state).margin << '\n';
  wbc::SupportModel support(WBC_TEST_URDF, wheels);
  const auto neutral_support = support.evaluate(nominal, 0);
  for (int k = 0; k < 61; ++k) {
    state.positions[index(state, options.unknown_steering[0])] = -3.14+6.28*k/60;
    state.positions[index(state, options.unknown_steering[1])] = 3.14-6.28*k/60;
    const auto actual = balance.model().evaluate(state);
    EXPECT_LE((actual.com-nominal.com).head<2>().norm(), balance.bounds().com_xy+1e-9);
    const auto footprint = support.evaluate(actual, options.inset);
    ASSERT_TRUE(footprint.geometry_valid);
    for (const auto& contact : footprint.contact_candidates)
      EXPECT_LE((contact.second-neutral_support.contact_candidates.at(contact.first)).head<2>().norm(),
                balance.bounds().support_xy+1e-9);
    EXPECT_GE(footprint.signed_margin+1e-9, balance.evaluate(state).margin);
  }
  options.unknown_steering.clear();
  EXPECT_THROW(wbc::StaticBalance(WBC_TEST_URDF, options), std::invalid_argument);
}

TEST(WbcModel, MassAndComMatchIndependentUrdfLinkSumIncludingRoot) {
  wbc::CentroidalModel model(WBC_TEST_URDF);
  const auto urdf = urdf::parseURDFFile(WBC_TEST_URDF);
  ASSERT_TRUE(urdf);
  for (int sample = 0; sample < 5; ++sample) {
    auto state = model.neutralSnapshot();
    for (std::size_t i = 0; i < model.joints().size(); ++i) {
      const auto& j = model.joints()[i];
      if (sample && !j.locked) state.positions[i] = j.lower +
        (j.upper - j.lower) * (0.5 + 0.2 * std::sin(double(i + sample)));
    }
    const auto result = model.evaluate(state);
    Eigen::Vector3d weighted = Eigen::Vector3d::Zero(); double mass = 0;
    for (const auto& link : urdf->links_) {
      const auto& in = link.second->inertial; if (!in) continue;
      const Eigen::Vector3d local(in->origin.position.x, in->origin.position.y, in->origin.position.z);
      weighted += in->mass * (result.link_poses.at(link.first) * local);
      mass += in->mass;
    }
    EXPECT_NEAR(result.mass, mass, 1e-9);
    EXPECT_LT((result.com - weighted / mass).norm(), 1e-10);
    EXPECT_GT(result.mass, urdf->getRoot()->inertial->mass);
    EXPECT_TRUE(result.synthetic);
  }
}
TEST(WbcModel, JacobianMatchesFiniteDifferenceAndVelocity) {
  wbc::CentroidalModel model(WBC_TEST_URDF);
  auto state = model.neutralSnapshot();
  for (std::size_t i = 0; i < model.joints().size(); ++i) {
    const auto& j = model.joints()[i];
    if (!j.locked) {
      state.positions[i] = 0.5 * (j.lower + j.upper);
      state.velocities[i] = 0.05 * std::sin(double(i));
    }
  }
  const auto result = model.evaluate(state); const double h = 1e-6;
  for (std::size_t i = 0; i < model.joints().size(); ++i) {
    if (model.joints()[i].locked) continue;
    auto plus = state, minus = state;
    plus.positions[i] += h; minus.positions[i] -= h;
    const Eigen::Vector3d numeric = (model.evaluate(plus).com - model.evaluate(minus).com) / (2*h);
    EXPECT_LT((numeric - result.com_jacobian.col(i)).norm(), 1e-7) << model.joints()[i].name;
  }
  const Eigen::Map<const Eigen::VectorXd> v(state.velocities.data(), state.velocities.size());
  EXPECT_LT((result.velocity - result.com_jacobian * v).norm(), 1e-10);
}
TEST(WbcModel, MappingUsesNamesAndRejectsIncompleteSnapshots) {
  wbc::CentroidalModel model(WBC_TEST_URDF);
  auto state = model.neutralSnapshot();
  state.positions[index(state, "Joint2_L")] = 0.2;
  const auto expected = model.evaluate(state).com;
  std::reverse(state.names.begin(), state.names.end());
  std::reverse(state.positions.begin(), state.positions.end());
  std::reverse(state.velocities.begin(), state.velocities.end());
  EXPECT_LT((expected - model.evaluate(state).com).norm(), 1e-12);
  state.position_valid = false; EXPECT_THROW(model.evaluate(state), std::invalid_argument);
  state = model.neutralSnapshot(); state.names[0] = "unknown";
  EXPECT_THROW(model.evaluate(state), std::invalid_argument);
  state = model.neutralSnapshot(); state.positions[0] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(model.evaluate(state), std::invalid_argument);
}
TEST(WbcModel, WholeBodyIncludesUnrequestedArmAndHead) {
  wbc::CentroidalModel model(WBC_TEST_URDF); auto state = model.neutralSnapshot();
  const auto neutral = model.evaluate(state).com;
  state.positions[index(state, "Joint2_R")] = 0.5;
  EXPECT_GT((model.evaluate(state).com - neutral).norm(), 1e-5);
  state = model.neutralSnapshot(); state.positions[index(state, "head_yaw_joint")] = 0.5;
  EXPECT_GT((model.evaluate(state).com - neutral).norm(), 1e-6);
}

class SupportTest : public testing::Test {
protected:
  wbc::CentroidalModel model{WBC_TEST_URDF};
  wbc::SupportModel support{WBC_TEST_URDF, wheels};
};
TEST_F(SupportTest, GeometryFromUrdfAndHalfspaceMargin) {
  const auto com = model.evaluate(model.neutralSnapshot());
  const auto raw = support.evaluate(com, 0.0), inset = support.evaluate(com, 0.02);
  ASSERT_TRUE(raw.geometry_valid) << raw.error;
  ASSERT_TRUE(inset.geometry_valid) << inset.error;
  EXPECT_FALSE(raw.contact_observed);
  EXPECT_EQ(raw.polygon.size(), 4U);
  EXPECT_NEAR(raw.plane_z, -0.05, 2e-6);
  EXPECT_NEAR(raw.signed_margin - inset.signed_margin, 0.02, 1e-10);
  for (const auto& p : raw.polygon) EXPECT_LE((raw.A * p - raw.b).maxCoeff(), 1e-10);
  auto outside = com; outside.com.x() = 10;
  EXPECT_LT(support.evaluate(outside, 0).signed_margin, 0);
  EXPECT_FALSE(support.evaluate(com, 10).geometry_valid);
}
TEST_F(SupportTest, CasterContactFollowsUrdfKinematics) {
  auto state = model.neutralSnapshot();
  const auto initial_model = model.evaluate(state);
  const auto initial = support.evaluate(initial_model, 0);
  state.positions[index(state, "base_front_wheel_1_yaw_joint")] = 1.0;
  const auto turned_model = model.evaluate(state);
  const auto turned = support.evaluate(turned_model, 0);
  ASSERT_TRUE(turned.geometry_valid) << turned.error;
  const Eigen::Vector3d expected = turned_model.link_poses.at(wheels[0]) *
    (initial_model.link_poses.at(wheels[0]).inverse() * initial.contact_candidates.at(wheels[0]));
  // This URDF places contact almost under the steering pivot. Do not invent
  // a nonzero caster trail: verify its actual FK, with mesh-bottom tolerance.
  EXPECT_LT((turned.contact_candidates.at(wheels[0]) - expected).norm(), 2e-5);
  EXPECT_GT((turned_model.link_poses.at(wheels[0]).linear() -
              initial_model.link_poses.at(wheels[0]).linear()).norm(), 0.5);
}
TEST_F(SupportTest, ContactGeometryUsesSuppliedFrameTransform) {
  auto com = model.evaluate(model.neutralSnapshot());
  const auto initial = support.evaluate(com, 0);
  com.link_poses.at(wheels[0]).translation().x() += 0.04;
  const auto shifted = support.evaluate(com, 0);
  ASSERT_TRUE(shifted.geometry_valid);
  EXPECT_NEAR(shifted.contact_candidates.at(wheels[0]).x() -
    initial.contact_candidates.at(wheels[0]).x(), 0.04, 1e-10);
}
TEST_F(SupportTest, NoncoplanarAndMissingFramesDoNotProduceValidPolygon) {
  auto com = model.evaluate(model.neutralSnapshot());
  com.link_poses.at(wheels[0]).translation().z() += 0.05;
  EXPECT_FALSE(support.evaluate(com, 0).geometry_valid);
  com.link_poses.erase(wheels[0]); EXPECT_THROW(support.evaluate(com, 0), std::invalid_argument);
}
TEST(WbcSupport, InvalidGeometryRolesRejected) {
  EXPECT_THROW(wbc::SupportModel(WBC_TEST_URDF, {"a", "b", "c"}), std::invalid_argument);
  EXPECT_THROW(wbc::SupportModel(WBC_TEST_URDF, {wheels[0], wheels[0], wheels[1]}), std::invalid_argument);
  EXPECT_THROW(wbc::SupportModel(WBC_TEST_URDF, {wheels[0], wheels[1]}), std::invalid_argument);
}
