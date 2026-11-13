#include "motion_control_pkg/motion/tcp_level_constraint.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <Eigen/Cholesky>
#include <algorithm>
#include <array>
#include <cmath>
#include <map>

namespace motion_control_pkg::motion {
namespace {
using Q = TcpLevelConstraint::Configuration;
using Path = TcpLevelConstraint::Path;
double angle(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return std::atan2(a.cross(b).norm(), a.dot(b));
}
double maxDifference(const Q& a, const Q& b) {
  double d = 0;
  for (std::size_t i = 0; i < a.size(); ++i) d = std::max(d, std::abs(a[i]-b[i]));
  return d;
}
}

struct TcpLevelConstraint::Impl {
  // Own the model so a registered measured-state callback cannot outlive it.
  pinocchio::Model model;
  pinocchio::Data data;
  Q reference;
  double tolerance;
  bool initialized{false};
  struct Arm {
    bool left;
    pinocchio::FrameIndex frame;
    Eigen::Vector3d tool_axis, normal;
    std::vector<int> qi, vi;
  };
  std::vector<Arm> arms;
  std::map<std::string, int> required;
  std::string setup_error;

  Impl(const pinocchio::Model& m, Q q, bool left, bool right,
       const Eigen::Isometry3d& lt, const Eigen::Isometry3d& rt, double tol)
    : model(m), data(model), reference(std::move(q)), tolerance(tol) {
    if ((!left && !right) || !model.existFrame("base_link") ||
        model.frames[model.getFrameId("base_link")].parentJoint != 0) {
      setup_error = "TCP level requires a fixed base_link model and at least one arm";
      return;
    }
    for (bool l : {true, false}) {
      if ((l && !left) || (!l && !right)) continue;
      const std::string frame = l ? "gripper_base_l_link" : "gripper_base_r_link";
      const auto& tool = l ? lt : rt;
      if (!model.existFrame(frame) || !tool.matrix().allFinite() ||
          !(tool.rotation().transpose()*tool.rotation()).isApprox(Eigen::Matrix3d::Identity(),1e-8) ||
          std::abs(tool.rotation().determinant()-1)>1e-8) {
        setup_error = "invalid TCP frame or tool rotation"; return;
      }
      Arm arm{l, model.getFrameId(frame), tool.rotation().col(2), Eigen::Vector3d::UnitZ(), {}, {}};
      for (int j = 1; j <= 7; ++j) {
        const auto name = "Joint"+std::to_string(j)+(l ? "_L" : "_R");
        if (!model.existJointName(name)) {setup_error = "missing joint "+name; return;}
        const auto& joint = model.joints[model.getJointId(name)];
        if (joint.nq()!=1 || joint.nv()!=1) {setup_error = "TCP level needs scalar joints"; return;}
        arm.qi.push_back(joint.idx_q()); arm.vi.push_back(joint.idx_v());
      }
      // All ancestors (including the four leg/waist joints) must be measured.
      for (auto j = model.frames[arm.frame].parentJoint; j != 0; j = model.parents[j]) {
        if (model.joints[j].nq()!=1 || model.joints[j].nv()!=1) {
          setup_error = "unsupported TCP ancestor joint"; return;
        }
        required.emplace(model.names[j], model.joints[j].idx_q());
      }
      arms.push_back(std::move(arm));
    }
  }
  bool configuration(const Q& q, std::string& error) const {
    if (q.size()!=static_cast<std::size_t>(model.nq)) {error="TCP level configuration size mismatch"; return false;}
    for (std::size_t i=0; i<q.size(); ++i) {
      if (!std::isfinite(q[i]) || q[i]<model.lowerPositionLimit[i]-1e-8 ||
          q[i]>model.upperPositionLimit[i]+1e-8) {
        error="TCP level configuration violates URDF bounds"; return false;
      }
    }
    return true;
  }
  double errorAt(const Q& q) {
    pinocchio::framesForwardKinematics(model, data, Eigen::Map<const Eigen::VectorXd>(q.data(),q.size()));
    double e=0;
    for (const auto& arm:arms)
      e=std::max(e,angle(data.oMf[arm.frame].rotation()*arm.tool_axis,arm.normal));
    return e;
  }
  bool check(const Q& q, std::string& error) {
    if (!initialized) {error="TCP level constraint not initialized"; return false;}
    if (!configuration(q,error)) return false;
    const double e=errorAt(q);
    if (!std::isfinite(e) || e>tolerance) {
      error="TCP level tilt "+std::to_string(e)+" rad exceeds "+std::to_string(tolerance)+" rad";
      return false;
    }
    error.clear(); return true;
  }
  bool edge(const Q& a, const Q& b, double ea, double eb, unsigned depth, std::string& error) {
    double rotation_bound=0;
    for (const auto& item:required) rotation_bound+=std::abs(a[item.second]-b[item.second]);
    // Unit joint axes give a conservative Lipschitz bound on TCP rotation.
    if (std::max(ea,eb)+0.5*rotation_bound<=tolerance) return true;
    if (depth==16) {error="TCP level interpolation could not be certified"; return false;}
    Q mid=a;
    for (std::size_t i=0;i<a.size();++i) mid[i]=0.5*(a[i]+b[i]);
    if (!check(mid,error)) return false;
    const double em=errorAt(mid);
    return edge(a,mid,ea,em,depth+1,error) && edge(mid,b,em,eb,depth+1,error);
  }
  bool project(Q& q, std::string& error) {
    for (const auto& arm:arms) {
      bool done=false;
      for (int iteration=0; iteration<80; ++iteration) {
        const Eigen::Map<const Eigen::VectorXd> eigen_q(q.data(),q.size());
        pinocchio::framesForwardKinematics(model,data,eigen_q);
        const Eigen::Vector3d n=data.oMf[arm.frame].rotation()*arm.tool_axis;
        const double before=angle(n,arm.normal);
        if (before<1e-7) {done=true; break;}
        Eigen::Matrix<double,6,Eigen::Dynamic> jac(6,model.nv); jac.setZero();
        pinocchio::computeFrameJacobian(model,data,eigen_q,arm.frame,pinocchio::LOCAL_WORLD_ALIGNED,jac);
        Eigen::Matrix<double,3,7> j;
        for (int k=0;k<7;++k) j.col(k)=jac.col(arm.vi[k]).tail<3>().cross(n);
        Eigen::Matrix<double,7,1> dq=j.transpose()*
            (j*j.transpose()+1e-6*Eigen::Matrix3d::Identity()).ldlt().solve(arm.normal-n);
        if (!dq.allFinite()) break;
        if (dq.cwiseAbs().maxCoeff()>0.05) dq*=0.05/dq.cwiseAbs().maxCoeff();
        bool accepted=false;
        for (double scale=1;scale>=1.0/128;scale*=0.5) {
          Q trial=q;
          for(int k=0;k<7;++k) trial[arm.qi[k]]=std::clamp(q[arm.qi[k]]+scale*dq[k],
              model.lowerPositionLimit[arm.qi[k]],model.upperPositionLimit[arm.qi[k]]);
          pinocchio::framesForwardKinematics(model,data,Eigen::Map<const Eigen::VectorXd>(trial.data(),trial.size()));
          if(angle(data.oMf[arm.frame].rotation()*arm.tool_axis,arm.normal)<before) {
            q=std::move(trial); accepted=true; break;
          }
        }
        if(!accepted) break;
      }
      if (!done) {error="TCP level projection failed for "+std::string(arm.left?"left":"right")+" arm";return false;}
    }
    return check(q,error);
  }
};

TcpLevelConstraint::TcpLevelConstraint(const pinocchio::Model& model, Configuration reference,
    bool left, bool right, const Eigen::Isometry3d& lt, const Eigen::Isometry3d& rt, double tolerance)
  : impl_(std::make_unique<Impl>(model,std::move(reference),left,right,lt,rt,tolerance)) {}
TcpLevelConstraint::~TcpLevelConstraint()=default;

bool TcpLevelConstraint::initialize(std::string& error) {
  auto& p=*impl_;
  if (!p.setup_error.empty()) {error=p.setup_error;return false;}
  if (!std::isfinite(p.tolerance)||p.tolerance<1e-4||p.tolerance>0.1) {
    error="invalid TCP level tolerance";return false;
  }
  if(!p.configuration(p.reference,error)) return false;
  pinocchio::framesForwardKinematics(p.model,p.data,Eigen::Map<const Eigen::VectorXd>(p.reference.data(),p.reference.size()));
  for (auto& arm:p.arms) {
    const double z=(p.data.oMf[arm.frame].rotation()*arm.tool_axis).z();
    arm.normal=(z>=0?1.0:-1.0)*Eigen::Vector3d::UnitZ();
  }
  p.initialized=true;
  if(!p.check(p.reference,error)) {p.initialized=false;error="TCP level start: "+error;return false;}
  return true;
}
bool TcpLevelConstraint::check(const Q& q,std::string& error) {return impl_->check(q,error);}
bool TcpLevelConstraint::projectConfiguration(Q& q,std::string& error) {
  if(!impl_->initialized || !impl_->configuration(q,error)) return false;
  Q candidate=q;
  if(!impl_->project(candidate,error)) return false;
  q=std::move(candidate);return true;
}
bool TcpLevelConstraint::checkTarget(bool left,const Eigen::Isometry3d& tcp,std::string& error) const {
  const auto& p=*impl_;
  if(!p.initialized || !tcp.matrix().allFinite() ||
      !(tcp.rotation().transpose()*tcp.rotation()).isApprox(Eigen::Matrix3d::Identity(),1e-8) ||
      std::abs(tcp.rotation().determinant()-1)>1e-8) {error="invalid TCP level target";return false;}
  for(const auto& arm:p.arms) if(arm.left==left) {
    if(angle(tcp.rotation().col(2),arm.normal)>p.tolerance) {
      error="TCP level target tilts or flips the tool normal";return false;
    }
    error.clear();return true;
  }
  error="TCP level target arm is not selected";return false;
}
bool TcpLevelConstraint::validatePath(const Path& path,std::string& error) {
  if(path.empty()) {error="TCP level path is empty";return false;}
  double previous=0;
  for(std::size_t i=0;i<path.size();++i) {
    if(!check(path[i],error)) {error+=" at sample "+std::to_string(i);return false;}
    const double e=impl_->errorAt(path[i]);
    if(i && !impl_->edge(path[i-1],path[i],previous,e,0,error)) {
      error+=" between samples "+std::to_string(i-1)+" and "+std::to_string(i);return false;
    }
    previous=e;
  }
  error.clear();return true;
}
bool TcpLevelConstraint::projectPath(Path& path,const Path& pinned,std::string& error) {
  if(path.empty() || !check(path.front(),error) || !check(path.back(),error)) {
    error="TCP level endpoints: "+error;return false;
  }
  for(const auto& q:pinned) if(!check(q,error)) {error="TCP level via: "+error;return false;}
  for(const auto& q:path) if(!impl_->configuration(q,error)) return false;
  if(validatePath(path,error)) {
    // Force path-preserving TOPPRA for a moving constrained two-point path.
    // Time-synchronized Ruckig can otherwise follow a different joint curve.
    if(path.size()==2 && maxDifference(path.front(),path.back())>1e-10) {
      Q mid=path.front();
      for(std::size_t j=0;j<mid.size();++j) mid[j]=0.5*(mid[j]+path.back()[j]);
      path.insert(path.begin()+1,std::move(mid));
    }
    return true;
  }
  Path result{path.front()};
  for(std::size_t i=1;i<path.size();++i) {
    const auto count=std::max<std::size_t>(2,static_cast<std::size_t>(std::ceil(maxDifference(path[i-1],path[i])/0.025)));
    if(result.size()+count>10000) {error="TCP level path exceeds projection budget";return false;}
    for(std::size_t k=1;k<=count;++k) {
      Q q=path[i-1];
      const double alpha=static_cast<double>(k)/count;
      for(std::size_t j=0;j<q.size();++j) q[j]+=alpha*(path[i][j]-q[j]);
      const bool anchor=k==count && (i+1==path.size() ||
        std::any_of(pinned.begin(),pinned.end(),[&](const Q& p){return maxDifference(p,path[i])<1e-10;}));
      if(anchor) q=path[i];
      else if(!impl_->project(q,error)) return false;
      if(maxDifference(result.back(),q)>0.2) {error="TCP level projection changed IK branch";return false;}
      result.push_back(std::move(q));
    }
  }
  if(!validatePath(result,error)) return false;
  path=std::move(result);return true;
}
bool TcpLevelConstraint::checkMeasured(const std::vector<std::string>& names,const Q& positions,
                                      std::string& error) {
  if(names.size()!=positions.size()) {error="TCP level measured state size mismatch";return false;}
  Q q=impl_->reference;
  std::map<std::string,double> measured;
  for(std::size_t i=0;i<names.size();++i)
    if(!std::isfinite(positions[i]) || !measured.emplace(names[i],positions[i]).second) {
      error="TCP level invalid or duplicate measured joint";return false;
    }
  for(const auto& item:impl_->required) {
    auto found=measured.find(item.first);
    if(found==measured.end()) {error="TCP level missing measured joint "+item.first;return false;}
    q[item.second]=found->second;
  }
  return check(q,error);
}
}  // namespace motion_control_pkg::motion
