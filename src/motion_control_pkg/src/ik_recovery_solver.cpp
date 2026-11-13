#include "motion_control_pkg/miracle_m_algo.hpp"
#include "motion_control_pkg/motion/ik_seed_sampler.hpp"
#include "motion_control_pkg/motion/ik_recovery_transport.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <sstream>

using namespace motion_control_pkg::motion;
namespace {
Json::Value vectorJson(const Eigen::VectorXd& q){Json::Value j(Json::arrayValue);for(double x:q)j.append(x);return j;}
Eigen::VectorXd vectorFrom(const Json::Value& j){Eigen::VectorXd q(j.size());for(Json::ArrayIndex i=0;i<j.size();++i)q[i]=j[i].asDouble();return q;}
Json::Value poseJson(const Eigen::Isometry3d& p){Json::Value j(Json::arrayValue);for(int i=0;i<16;++i)j.append(p.matrix().data()[i]);return j;}
Eigen::Isometry3d poseFrom(const Json::Value& j){if(j.size()!=16)throw std::invalid_argument("invalid pose payload");Eigen::Isometry3d p;for(int i=0;i<16;++i)p.matrix().data()[i]=j[i].asDouble();if(!p.matrix().allFinite())throw std::invalid_argument("nonfinite pose");return p;}
std::string hex(const std::string& bytes){constexpr char digits[]="0123456789abcdef";std::string out;out.reserve(bytes.size()*2);for(unsigned char c:bytes){out+=digits[c>>4];out+=digits[c&15];}return out;}
std::string unhex(const std::string& text){if(text.size()%2)throw std::invalid_argument("invalid octree encoding");std::string out;out.reserve(text.size()/2);auto value=[](char c){if(c>='0'&&c<='9')return c-'0';if(c>='a'&&c<='f')return c-'a'+10;throw std::invalid_argument("invalid octree digit");};for(std::size_t i=0;i<text.size();i+=2)out+=static_cast<char>((value(text[i])<<4)|value(text[i+1]));return out;}
bool poseValid(const std::optional<Eigen::Isometry3d>& pose){return !pose||(pose->matrix().allFinite()&&(pose->linear().transpose()*pose->linear()-Eigen::Matrix3d::Identity()).norm()<1e-6&&std::abs(pose->linear().determinant()-1)<1e-6);}
}

std::string IkRecoveryAccess::signature(const MiracleMAlgo& a) {
  Json::Value signature;signature["nq"]=a.model_.nq;
  for(pinocchio::JointIndex i=0;i<a.model_.joints.size();++i){Json::Value joint;joint["name"]=a.model_.names[i];joint["q"]=a.model_.joints[i].idx_q();joint["nq"]=a.model_.joints[i].nq();joint["v"]=a.model_.joints[i].idx_v();joint["parent"]=Json::UInt64(a.model_.parents[i]);joint["placement"]=poseJson(Eigen::Isometry3d(a.model_.jointPlacements[i].toHomogeneousMatrix()));signature["joints"].append(joint);}
  signature["lower"]=vectorJson(a.model_.lowerPositionLimit);signature["upper"]=vectorJson(a.model_.upperPositionLimit);
  for(const auto& frame:a.model_.frames){Json::Value f;f["name"]=frame.name;f["joint"]=Json::UInt64(frame.parentJoint);f["placement"]=poseJson(Eigen::Isometry3d(frame.placement.toHomogeneousMatrix()));signature["frames"].append(f);}
  return ik_transport::encode(signature);
}

Json::Value IkRecoveryAccess::sdkArmPositionLimits(const MiracleMAlgo& a) {
  Json::Value limits;
  for (const auto& item : a.sdk_arm_position_limits_) {
    limits[item.first].append(item.second.first);
    limits[item.first].append(item.second.second);
  }
  return limits;
}

void IkRecoveryAccess::applySdkArmPositionLimits(MiracleMAlgo& a, const Json::Value& limits) {
  if (!limits.isObject() || limits.size() != 14U)
    throw std::invalid_argument("invalid SDK arm position-limit payload");
  motion_control_pkg::wbc::PositionLimitOverrides parsed;
  for (const auto& name : limits.getMemberNames()) {
    const auto& pair = limits[name];
    if (!pair.isArray() || pair.size() != 2U || !pair[0].isNumeric() || !pair[1].isNumeric())
      throw std::invalid_argument("invalid SDK arm position-limit pair");
    parsed[name] = {pair[0].asDouble(), pair[1].asDouble()};
  }
  a.setSdkArmPositionLimits(parsed);
}

Json::Value IkRecoveryAccess::scene(const MiracleMAlgo& a) {
  Json::Value scene;scene["revision"]=Json::UInt64(a.geometry_revision_);scene["mode"]=a.collision_check_mode_;
  scene["external"]=Json::Value(Json::arrayValue);scene["pairs"]=Json::Value(Json::arrayValue);
  for(const auto& name:a.external_objects_) {
    const auto& object=a.geom_model_.geometryObjects.at(a.geom_model_.getGeometryId(name));
    Json::Value j;j["name"]=name;j["pose"]=poseJson(Eigen::Isometry3d(object.placement.toHomogeneousMatrix()));
    if(const auto* box=dynamic_cast<const coal::Box*>(object.geometry.get())){j["kind"]="box";j["size"]=vectorJson(2.0*box->halfSide);}
    else if(const auto* tree=dynamic_cast<const coal::OcTree*>(object.geometry.get())){j["kind"]="octree";std::ostringstream out(std::ios::binary);if(!tree->getTree()->write(out))throw std::runtime_error("octree snapshot failed");j["data"]=hex(out.str());}
    else throw std::runtime_error("unsupported external geometry in IK recovery: "+name);
    scene["external"].append(j);
  }
  for(const auto& pair:a.geom_model_.collisionPairs){Json::Value p;p.append(a.geom_model_.geometryObjects[pair.first].name);p.append(a.geom_model_.geometryObjects[pair.second].name);scene["pairs"].append(p);}
  scene["balance"]=Json::Value();
  if(a.static_balance_) {
    const auto& o=a.static_balance_->options();auto& b=scene["balance"];
    b["inset"]=o.inset;b["com_weight"]=o.com_weight;b["uncertainty"]=o.steering_uncertainty_margin_enabled;
    b["support_links"]=Json::Value(Json::arrayValue);for(const auto& s:o.support_links)b["support_links"].append(s);
    b["unknown_steering"]=Json::Value(Json::arrayValue);for(const auto& s:o.unknown_steering)b["unknown_steering"].append(s);
  }
  return scene;
}
void IkRecoveryAccess::applyScene(MiracleMAlgo& a,const Json::Value& scene) {
  auto old=a.external_objects_;for(const auto& name:old)a.geom_model_.removeGeometryObject(name);a.external_objects_.clear();
  for(const auto& j:scene["external"]) {
    std::shared_ptr<coal::CollisionGeometry> geometry;
    if(j["kind"]=="box"){const auto size=vectorFrom(j["size"]);if(size.size()!=3||(size.array()<=0).any())throw std::invalid_argument("invalid box");geometry=std::make_shared<coal::Box>(size[0],size[1],size[2]);}
    else if(j["kind"]=="octree"){
      std::istringstream input(unhex(j["data"].asString()),std::ios::binary);
      std::unique_ptr<octomap::AbstractOcTree> parsed(octomap::AbstractOcTree::read(input));
      auto* tree=dynamic_cast<octomap::OcTree*>(parsed.get());if(!tree)throw std::runtime_error("invalid octree snapshot");
      parsed.release();geometry=std::make_shared<coal::OcTree>(std::shared_ptr<const octomap::OcTree>(tree));
    }else throw std::invalid_argument("unsupported snapshot geometry");
    const auto pose=poseFrom(j["pose"]);const auto name=j["name"].asString();
    a.geom_model_.addGeometryObject(pinocchio::GeometryObject(name,0,pinocchio::SE3(pose.rotation(),pose.translation()),geometry));a.external_objects_.push_back(name);
  }
  a.geom_model_.removeAllCollisionPairs();
  for(const auto& p:scene["pairs"]){const auto first=p[0].asString(),second=p[1].asString();if(!a.geom_model_.existGeometryName(first)||!a.geom_model_.existGeometryName(second))throw std::runtime_error("snapshot geometry/model mismatch");a.geom_model_.addCollisionPair(pinocchio::CollisionPair(a.geom_model_.getGeometryId(first),a.geom_model_.getGeometryId(second)));}
  a.geom_data_=std::make_shared<pinocchio::GeometryData>(a.geom_model_);++a.geometry_revision_;a.setCollisionCheckMode(scene["mode"].asUInt());
  const auto& b=scene["balance"];
  if(b.isNull())a.static_balance_.reset();
  else {
    motion_control_pkg::wbc::StaticBalanceOptions o;o.inset=b["inset"].asDouble();o.com_weight=b["com_weight"].asDouble();o.steering_uncertainty_margin_enabled=b["uncertainty"].asBool();
    for(const auto& x:b["support_links"])o.support_links.push_back(x.asString());
    for(const auto& x:b["unknown_steering"])o.unknown_steering.push_back(x.asString());
    if(!a.static_balance_||ik_transport::encode(IkRecoveryAccess::scene(a)["balance"])!=ik_transport::encode(b))a.configureStaticBalance(o);
  }
}
Json::Value IkRecoveryAccess::evaluate(MiracleMAlgo& a,const Json::Value& request,const Json::Value& seed,const IkStop& stop) {
  IkSingleSeedScope single;IkPlanningStopScope stopped(stop);
  Eigen::VectorXd q=vectorFrom(seed),out;Json::Value result;bool ok=false;
  if(q.size()!=a.model_.nq||!q.allFinite()||stop())return result;
  if(request["whole"].asBool()) {
    std::optional<Eigen::Isometry3d> left,right;if(!request["left"].isNull())left=poseFrom(request["left"]);if(!request["right"].isNull())right=poseFrom(request["right"]);
    std::array<double,4> weights;for(unsigned i=0;i<4;++i)weights[i]=request["weights"][i].asDouble();
    ok=a.solveWholeBodyIKOnce(q,left,right,out,weights,ik_transport::after(request["seed_timeout_s"].asDouble()),request["collision"].asBool(),request["waist"].asBool());
  }else if(request["mode"].asInt()==2)ok=a.solveDualIKKernel(q,poseFrom(request["left"]),poseFrom(request["right"]),out);
  else {MiracleMAlgo::SingleArmIkWorkspace workspace;ok=a.solveIKKernel(q,poseFrom(request["left"]),out,request["mode"].asInt(),workspace,*a.data_,false,stop);}
  result["solved"]=ok&&!stop();if(result["solved"].asBool())result["q"]=vectorJson(out);return result;
}

void MiracleMAlgo::configureIkRecovery(const IkRecoveryOptions& options,const std::function<bool()>& stop) {
  std::lock_guard<std::recursive_mutex> lock(pinocchio_data_mutex_);
  ik_recovery_.reset();ik_recovery_stop_=stop;
  if(options.enabled)ik_recovery_=std::make_shared<IkRecoveryClient>(options,urdf_path_,mesh_dir_,
      IkRecoveryAccess::signature(*this),IkRecoveryAccess::sdkArmPositionLimits(*this));
}
IkRecoveryStats MiracleMAlgo::ikRecoveryStats() const {return ik_recovery_?ik_recovery_->stats():IkRecoveryStats{};}

bool MiracleMAlgo::recoverIk(const Eigen::VectorXd& reference,const std::optional<Eigen::Isometry3d>& left,
    const std::optional<Eigen::Isometry3d>& right,Eigen::VectorXd& result,int mode,bool whole,
    const std::array<double,4>& weights,bool collision,bool waist,pinocchio::Data& data,const PlanningStop& stop) {
  if(!ik_recovery_||ik_single_seed_kernel||ik_recovery_suppressed)return false;
  const auto recovery_deadline=ik_transport::after(ik_recovery_->options().request_timeout_s);
  auto cancelled=[&]{return IkClock::now()>=recovery_deadline||ikPlanningStopped()||(stop&&stop())||(ik_recovery_stop_&&ik_recovery_stop_());};
  if(cancelled()||reference.size()!=model_.nq||!reference.allFinite()||!poseValid(left)||!poseValid(right))return false;
  namespace seeds=motion_control_pkg::motion::ik_seeds;
  try {
    seeds::Request sample;sample.reference={reference.data(),reference.data()+reference.size()};
    sample.lower={model_.lowerPositionLimit.data(),model_.lowerPositionLimit.data()+model_.nq};sample.upper={model_.upperPositionLimit.data(),model_.upperPositionLimit.data()+model_.nq};
    auto append=[&](const auto& indices){sample.active.insert(sample.active.end(),indices.begin(),indices.end());};
    if(whole){if(left)append(left_arm_q_indices_);if(right)append(right_arm_q_indices_);if(waist)sample.active.push_back(leg_q_indices_.at(3));else append(leg_q_indices_);}
    else if(mode==0)append(left_arm_q_indices_);else if(mode==1)append(right_arm_q_indices_);else if(mode==2){append(left_arm_q_indices_);append(right_arm_q_indices_);}else if(mode==3)append(head_q_indices_);else if(mode==4)append(leg_q_indices_);else return false;
    sample.arms={{left_arm_q_indices_.at(5),left_arm_q_indices_.at(6)},{right_arm_q_indices_.at(5),right_arm_q_indices_.at(6)}};
    pinocchio::GeometryData geometry(geom_model_);geometry.activeCollisionPairs=geom_data_->activeCollisionPairs;
    auto feasible=[&](const seeds::Configuration& values){Eigen::Map<const Eigen::VectorXd> q(values.data(),values.size());return isFullQValidWithData(q,data,geometry)&&(!whole||isStaticBalanceValid(q));};
    const auto population=seeds::generate(sample,ik_recovery_->options().seeds,{},feasible,cancelled);
    if(cancelled()||population.seeds.empty())return false;
    Json::Value request;request["whole"]=whole;request["mode"]=mode;request["collision"]=collision;request["waist"]=waist;
    if(left)request["left"]=poseJson(*left);
    if(right)request["right"]=poseJson(*right);
    for(double w:weights)request["weights"].append(w);
    request["scene"]=IkRecoveryAccess::scene(*this);request["seeds"]=Json::Value(Json::arrayValue);
    for(const auto& seed:population.seeds){Json::Value j(Json::arrayValue);for(double x:seed.q)j.append(x);request["seeds"].append(j);}
    Eigen::VectorXd accepted;
    const auto validate=[&](const Json::Value& json) {
      auto q=vectorFrom(json);if(q.size()!=reference.size()||!q.allFinite()||cancelled())return false;
      for(int j=0;j<q.size();++j)if(q[j]<model_.lowerPositionLimit[j]||q[j]>model_.upperPositionLimit[j]||
          (std::find(sample.active.begin(),sample.active.end(),j)==sample.active.end()&&std::abs(q[j]-reference[j])>1e-9))return false;
      const seeds::Configuration values(q.data(),q.data()+q.size());for(auto arm:sample.arms)if(!seeds::coupledArmValid(values,arm))return false;
      pinocchio::forwardKinematics(model_,data,q);pinocchio::updateFramePlacements(model_,data);
      const double tolerance=whole?1.41e-4:mode==3?.00101:mode==4?.00051:.000051;
      const auto matches=[&](pinocchio::FrameIndex frame,const Eigen::Isometry3d& goal){const auto& pose=data.oMf[frame];return (pose.translation()-goal.translation()).norm()<=tolerance&&Eigen::Quaterniond(pose.rotation()).angularDistance(Eigen::Quaterniond(goal.rotation()))<=tolerance;};
      bool match=false;
      if(whole)match=(!left||matches(left_ee_frame_id_,*left))&&(!right||matches(right_ee_frame_id_,*right));
      else if(mode==2)match=matches(left_ee_frame_id_,*left)&&matches(right_ee_frame_id_,*right);
      else match=left&&matches(mode==0?left_ee_frame_id_:mode==1?right_ee_frame_id_:mode==3?head_ee_frame_id_:leg_ee_frame_id_,*left);
      if(!match||!feasible(values)||cancelled())return false;
      accepted=std::move(q);return true;
    };
    request["remaining_s"]=std::chrono::duration<double>(recovery_deadline-IkClock::now()).count();
    const bool ok=ik_recovery_->solve(std::move(request),validate,cancelled);const auto stats=ik_recovery_->stats();
    RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),"IK recovery: mode=%d whole=%d generated=%zu attempted=%u timeouts=%u workers=%zu seconds=%.6f status=%s",mode,whole,population.seeds.size(),stats.attempted,stats.timed_out,stats.cpus.size(),stats.elapsed_s,stats.status.c_str());
    if(ok&&!cancelled()){result=std::move(accepted);return true;}
  }catch(const std::exception& error){RCLCPP_WARN(rclcpp::get_logger("MiracleMAlgo"),"IK recovery rejected: %s",error.what());}
  return false;
}

bool MiracleMAlgo::solveIKWithData(const Eigen::VectorXd& q,const Eigen::Isometry3d& goal,Eigen::VectorXd& out,int mode,
    SingleArmIkWorkspace& workspace,pinocchio::Data& data,bool continuous,const PlanningStop& stop) {
  if(q.size()!=model_.nq||!q.allFinite()||!goal.matrix().allFinite()||ikPlanningStopped()||(stop&&stop()))return false;
  if(solveIKKernel(q,goal,out,mode,workspace,data,continuous,stop))return true;
  if(continuous)return false; // Cartesian continuation retains its original branch/refinement contract.
  return recoverIk(q,goal,std::nullopt,out,mode,false,{1.,1.,1.,.2},false,false,data,stop);
}
bool MiracleMAlgo::solveDualIK(const Eigen::VectorXd& q,const Eigen::Isometry3d& left,const Eigen::Isometry3d& right,Eigen::VectorXd& out) {
  std::lock_guard<std::recursive_mutex> lock(pinocchio_data_mutex_);
  if(q.size()!=model_.nq||!q.allFinite()||!left.matrix().allFinite()||!right.matrix().allFinite()||ikPlanningStopped())return false;
  if(solveDualIKKernel(q,left,right,out))return true;
  return recoverIk(q,left,right,out,2,false,{1.,1.,1.,.2},false,false,*data_);
}
bool MiracleMAlgo::solveWholeBodyIK(const Eigen::VectorXd& q,const std::optional<Eigen::Isometry3d>& left,
    const std::optional<Eigen::Isometry3d>& right,Eigen::VectorXd& out,const std::array<double,4>& weights,
    double pitch,double timeout,std::size_t attempts,bool collision,bool waist,bool continuous) {
  std::lock_guard<std::recursive_mutex> lock(pinocchio_data_mutex_);
  if(ikPlanningStopped()||q.size()!=model_.nq||!q.allFinite()||!poseValid(left)||!poseValid(right)||(!left&&!right)||!std::isfinite(timeout)||timeout<=0)return false;
  if(!std::all_of(weights.begin(),weights.end(),[](double w){return std::isfinite(w)&&w>=0.;}))return false;
  if(solveWholeBodyIKPrimary(q,left,right,out,weights,pitch,timeout,attempts,collision,waist))return true;
  if(continuous||attempts==0)return false;
  return recoverIk(q,left,right,out,2,true,weights,collision,waist,*data_);
}
