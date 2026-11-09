#include "motion_control_pkg/motion/rigid_grasp_constraint.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <Eigen/Cholesky>
#include <algorithm>
#include <array>
#include <map>
#include <cmath>

namespace motion_control_pkg::motion {
namespace {
using Q=RigidGraspConstraint::Q;
using Path=RigidGraspConstraint::Path;
double distance(const Q& a,const Q& b) {
  double d=0;for(std::size_t j=0;j<a.size();++j) d=std::max(d,std::abs(a[j]-b[j]));return d;
}
Eigen::Vector3d logRotation(const Eigen::Matrix3d& r) {
  const Eigen::AngleAxisd aa(r);return aa.angle()*aa.axis();
}
double angle(const Eigen::Vector3d& a,const Eigen::Vector3d& b) {
  return std::atan2(a.cross(b).norm(),a.dot(b));
}
bool isPose(const Eigen::Isometry3d& p) {
  return p.matrix().allFinite() &&
    (p.rotation().transpose()*p.rotation()).isApprox(Eigen::Matrix3d::Identity(),1e-8) &&
    std::abs(p.rotation().determinant()-1)<1e-8;
}
}
struct RigidGraspConstraint::Impl {
  pinocchio::Model model;
  pinocchio::Data data;
  Q reference;
  std::array<Eigen::Isometry3d,2> tools;
  std::array<pinocchio::FrameIndex,2> frames{};
  Eigen::Isometry3d relative=Eigen::Isometry3d::Identity();
  Eigen::Vector3d local_up=Eigen::Vector3d::UnitZ();
  std::vector<int> arms;
  std::map<std::string,int> required;
  RigidGraspOptions options;
  double reach{0};
  bool initialized{false};
  std::string setup_error;
  Impl(const pinocchio::Model& m,Q q,const Eigen::Isometry3d& l,const Eigen::Isometry3d& r,RigidGraspOptions o)
    :model(m),data(model),reference(std::move(q)),tools{l,r},options(o) {
    if(!model.existFrame("base_link") || model.frames[model.getFrameId("base_link")].parentJoint!=0) {
      setup_error="rigid grasp requires a fixed base_link";return;
    }
    for(int side=0;side<2;++side) {
      const std::string frame=side==0?"gripper_base_l_link":"gripper_base_r_link";
      if(!model.existFrame(frame)||!isPose(tools[side])) {setup_error="invalid calibrated suction TCP";return;}
      frames[side]=model.getFrameId(frame);
      double length=tools[side].translation().norm()+model.frames[frames[side]].placement.translation().norm();
      for(auto j=model.frames[frames[side]].parentJoint;j!=0;j=model.parents[j]) {
        if(model.joints[j].nq()!=1 || model.joints[j].shortname().find("JointModelR")!=0) {
          setup_error="rigid grasp certificate requires scalar revolute ancestors";return;
        }
        required.emplace(model.names[j],model.joints[j].idx_q());
        length+=model.jointPlacements[j].translation().norm();
      }
      reach=std::max(reach,length);
      for(int j=1;j<=7;++j) {
        const auto name="Joint"+std::to_string(j)+(side==0?"_L":"_R");
        if(!model.existJointName(name)) {setup_error="rigid grasp missing joint "+name;return;}
        arms.push_back(model.joints[model.getJointId(name)].idx_q());
      }
    }
  }
  bool configuration(const Q& q,std::string& e) const {
    if(q.size()!=static_cast<std::size_t>(model.nq)) {e="rigid grasp configuration size mismatch";return false;}
    for(std::size_t j=0;j<q.size();++j) if(!std::isfinite(q[j]) ||
      q[j]<model.lowerPositionLimit[j]-1e-8 || q[j]>model.upperPositionLimit[j]+1e-8) {
      e="rigid grasp configuration violates URDF limits";return false;
    }
    return true;
  }
  std::array<Eigen::Isometry3d,2> poses(const Q& q) {
    pinocchio::framesForwardKinematics(model,data,Eigen::Map<const Eigen::VectorXd>(q.data(),q.size()));
    std::array<Eigen::Isometry3d,2> p;
    for(int i=0;i<2;++i) {
      p[i].setIdentity();p[i].linear()=data.oMf[frames[i]].rotation();
      p[i].translation()=data.oMf[frames[i]].translation();p[i]=p[i]*tools[i];
    }return p;
  }
  // Dimensionless maximum violation; every component has an explicit unit.
  double errorAt(const std::array<Eigen::Isometry3d,2>& p) const {
    const Eigen::Isometry3d rel=p[0].inverse()*p[1];
    return std::max({(rel.translation()-relative.translation()).norm()/options.position_tolerance_m,
      logRotation(relative.rotation().transpose()*rel.rotation()).norm()/options.orientation_tolerance_rad,
      std::abs(p[0].translation().z()-p[1].translation().z())/options.position_tolerance_m,
      angle(p[0].rotation().col(2),-Eigen::Vector3d::UnitZ())/options.orientation_tolerance_rad,
      angle(p[1].rotation().col(2),-Eigen::Vector3d::UnitZ())/options.orientation_tolerance_rad,
      angle(p[0].rotation()*local_up,Eigen::Vector3d::UnitZ())/options.orientation_tolerance_rad});
  }
  bool check(const Q& q,std::string& e) {
    if(!initialized) {e="rigid grasp not initialized";return false;}
    if(!configuration(q,e)) return false;
    const double v=errorAt(poses(q));
    if(!std::isfinite(v)||v>1.) {e="rigid grasp relative pose / common height / downward normal violation, normalized="+std::to_string(v);return false;}
    e.clear();return true;
  }
  Eigen::Matrix<double,9,1> residual(const Q& q) {
    const auto p=poses(q);const Eigen::Isometry3d rel=p[0].inverse()*p[1];
    Eigen::Matrix<double,9,1> e;
    e.head<3>()=rel.translation()-relative.translation();
    e.segment<3>(3)=.2*logRotation(relative.rotation().transpose()*rel.rotation());
    e.tail<3>()=.2*(p[0].rotation()*local_up-Eigen::Vector3d::UnitZ());return e;
  }
  bool project(Q& q,std::string& error) {
    // Jointly solve BOTH arms; the lower-body path is immutable. Differencing
    // the full calibrated FK includes the actual rotated/translated tool frames.
    for(int iteration=0;iteration<100;++iteration) {
      const auto e=residual(q);
      if(e.norm()<1e-9) return check(q,error);
      Eigen::Matrix<double,9,14> jac;
      constexpr double eps=1e-6;
      for(int j=0;j<14;++j) {Q trial=q;trial[arms[j]]+=eps;jac.col(j)=(residual(trial)-e)/eps;}
      Eigen::Matrix<double,14,1> dq;
      for(int active_set=0;active_set<=14;++active_set) {
        dq=-jac.transpose()*(jac*jac.transpose()+1e-9*Eigen::Matrix<double,9,9>::Identity()).ldlt().solve(e);
        bool blocked=false;
        for(int j=0;j<14;++j) if((q[arms[j]]<=model.lowerPositionLimit[arms[j]]+1e-7&&dq[j]<0)||
                                 (q[arms[j]]>=model.upperPositionLimit[arms[j]]-1e-7&&dq[j]>0)) {
          jac.col(j).setZero();blocked=true;
        }
        if(!blocked) break;
      }
      if(!dq.allFinite()) break;
      if(dq.cwiseAbs().maxCoeff()>.04) dq*=.04/dq.cwiseAbs().maxCoeff();
      bool accepted=false;
      for(double scale=1;scale>=1./256;scale*=.5) {
        Q trial=q;
        for(int j=0;j<14;++j) trial[arms[j]]=std::clamp(q[arms[j]]+scale*dq[j],
          model.lowerPositionLimit[arms[j]],model.upperPositionLimit[arms[j]]);
        if(residual(trial).norm()<e.norm()) {q=std::move(trial);accepted=true;break;}
      }
      if(!accepted) break;
    }
    error="coupled rigid-grasp projection failed; residual="+std::to_string(residual(q).norm());return false;
  }
  bool edge(const Q& a,const Q& b,double ea,double eb,unsigned depth,std::size_t& budget,std::string& e) {
    if(++budget>2000000) {e="rigid grasp interpolation certificate budget exceeded";return false;}
    double rotation=0;for(const auto& n:required) rotation+=std::abs(a[n.second]-b[n.second]);
    // Conservative revolute-chain Lipschitz bound: TCP position <= reach*dq;
    // relative position includes rotation of the left coordinate frame.
    const double bound=std::max(4*reach*rotation/options.position_tolerance_m,
                               2*rotation/options.orientation_tolerance_rad);
    if(std::max(ea,eb)+.5*bound<=1) return true;
    if(depth==18) {e="rigid grasp interpolation could not be certified";return false;}
    Q mid=a;for(std::size_t j=0;j<a.size();++j) mid[j]=.5*(a[j]+b[j]);
    if(!check(mid,e)) return false;
    const double em=errorAt(poses(mid));
    return edge(a,mid,ea,em,depth+1,budget,e)&&edge(mid,b,em,eb,depth+1,budget,e);
  }
};
RigidGraspConstraint::RigidGraspConstraint(const pinocchio::Model& m,Q q,
  const Eigen::Isometry3d& l,const Eigen::Isometry3d& r,RigidGraspOptions o)
  :impl_(std::make_unique<Impl>(m,std::move(q),l,r,o)) {}
RigidGraspConstraint::~RigidGraspConstraint()=default;
bool RigidGraspConstraint::initialize(std::string& e) {
  auto& p=*impl_;p.initialized=false;
  if(!p.setup_error.empty()) {e=p.setup_error;return false;}
  if(!std::isfinite(p.options.position_tolerance_m)||p.options.position_tolerance_m<1e-5||p.options.position_tolerance_m>.01||
     !std::isfinite(p.options.orientation_tolerance_rad)||p.options.orientation_tolerance_rad<1e-4||p.options.orientation_tolerance_rad>.05) {
    e="invalid rigid-grasp position/orientation tolerances";return false;
  }
  if(!p.configuration(p.reference,e)) return false;
  const auto start=p.poses(p.reference);
  p.relative=start[0].inverse()*start[1];p.local_up=start[0].rotation().transpose()*Eigen::Vector3d::UnitZ();
  if(p.relative.translation().norm()<.02) {e="rigid grasp suction points must be distinct (>= 20 mm)";return false;}
  p.initialized=true;
  if(!p.check(p.reference,e)) {p.initialized=false;e="rigid grasp start: "+e;return false;}
  return true;
}
bool RigidGraspConstraint::check(const Q& q,std::string& e) {return impl_->check(q,e);}
bool RigidGraspConstraint::checkTargets(const Eigen::Isometry3d& l,const Eigen::Isometry3d& r,std::string& e) const {
  if(!impl_->initialized||!isPose(l)||!isPose(r)||impl_->errorAt({l,r})>1) {
    e="rigid grasp targets must preserve initial left_T_right, common height and downward suction normals";return false;
  }e.clear();return true;
}
bool RigidGraspConstraint::validatePath(const Path& path,std::string& e) {
  if(path.empty()) {e="empty rigid-grasp path";return false;}
  double previous=0;std::size_t budget=0;
  for(std::size_t i=0;i<path.size();++i) {
    if(!check(path[i],e)) {e+=" at sample "+std::to_string(i);return false;}
    const double error=impl_->errorAt(impl_->poses(path[i]));
    if(i&&!impl_->edge(path[i-1],path[i],previous,error,0,budget,e)) return false;
    previous=error;
  }e.clear();return true;
}
bool RigidGraspConstraint::projectPath(Path& path,const Path& pinned,std::string& e) {
  if(path.empty()||!check(path.front(),e)||!check(path.back(),e)) {e="rigid grasp endpoints: "+e;return false;}
  for(const auto& q:pinned) if(!check(q,e)) return false;
  for(const auto& q:path) if(!impl_->configuration(q,e)) return false;
  if(validatePath(path,e)) {
    if(path.size()==2&&distance(path.front(),path.back())>1e-10) {
      Q mid=path.front();for(std::size_t j=0;j<mid.size();++j) mid[j]=.5*(mid[j]+path.back()[j]);
      path.insert(path.begin()+1,std::move(mid));
    }return true;
  }
  Path result{path.front()};
  for(std::size_t i=1;i<path.size();++i) {
    const auto count=std::max<std::size_t>(2,std::ceil(distance(path[i-1],path[i])/.025));
    if(result.size()+count>10000) {e="rigid grasp projection budget exceeded";return false;}
    for(std::size_t k=1;k<=count;++k) {
      Q q=path[i-1];const double alpha=double(k)/count;
      for(std::size_t j=0;j<q.size();++j) q[j]+=alpha*(path[i][j]-q[j]);
      const bool anchor=k==count&&(i+1==path.size()||std::any_of(pinned.begin(),pinned.end(),
        [&](const Q& p){return distance(p,path[i])<1e-10;}));
      if(anchor) q=path[i];else if(!impl_->project(q,e)) return false;
      if(distance(result.back(),q)>.2) {e="rigid grasp changed IK branch";return false;}
      result.push_back(std::move(q));
    }
  }
  if(!validatePath(result,e)) return false;
  path=std::move(result);return true;
}
bool RigidGraspConstraint::compensateLegPath(Path& path,std::string& e) {
  if(path.empty()||!check(path.front(),e)) {e="rigid LEG start: "+e;return false;}
  for(const auto& q:path) if(!impl_->configuration(q,e)) return false;
  Path result{path.front()};
  for(std::size_t i=1;i<path.size();++i) {
    const auto count=std::max<std::size_t>(1,std::ceil(distance(path[i-1],path[i])/.025));
    if(result.size()+count>10000) {e="rigid LEG compensation budget exceeded";return false;}
    for(std::size_t k=1;k<=count;++k) {
      Q q=path[i-1];const double alpha=double(k)/count;
      for(std::size_t j=0;j<q.size();++j) q[j]+=alpha*(path[i][j]-q[j]);
      if(k==count) q=path[i];
      for(int index:impl_->arms) q[index]=result.back()[index];
      if(!impl_->project(q,e)) return false;
      if(distance(result.back(),q)>.2) {e="rigid LEG compensation changed IK branch";return false;}
      result.push_back(std::move(q));
    }
  }
  if(result.size()==2) {
    // TOPPRA's clamped spline needs an interior point even for pure waist yaw.
    Q mid=result.front();for(std::size_t j=0;j<mid.size();++j) mid[j]=.5*(mid[j]+result.back()[j]);
    if(!impl_->project(mid,e)) return false;
    result.insert(result.begin()+1,std::move(mid));
  }
  if(!validatePath(result,e)) return false;
  path=std::move(result);e.clear();return true;
}
bool RigidGraspConstraint::checkMeasured(const std::vector<std::string>& names,const Q& values,std::string& e) {
  if(names.size()!=values.size()) {e="rigid grasp measured layout mismatch";return false;}
  std::map<std::string,double> measured;
  for(std::size_t j=0;j<names.size();++j) if(!std::isfinite(values[j])||!measured.emplace(names[j],values[j]).second) {
    e="invalid/duplicate rigid grasp feedback";return false;
  }
  Q q=impl_->reference;
  for(const auto& n:impl_->required) {
    const auto it=measured.find(n.first);if(it==measured.end()) {e="missing rigid grasp feedback "+n.first;return false;}
    q[n.second]=it->second;
  }return check(q,e);
}
}
