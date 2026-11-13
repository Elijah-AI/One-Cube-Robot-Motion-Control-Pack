#include "motion_control_pkg/motion/ik_recovery_transport.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <set>
#include <sstream>
#include <thread>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>

extern char** environ;
namespace motion_control_pkg::motion {
using namespace ik_transport;
std::vector<int> selectIkRecoveryCpus(std::size_t workers,const std::vector<int>& requested,const std::string& sys_root) {
  cpu_set_t mask;CPU_ZERO(&mask);
  if(sched_getaffinity(0,sizeof(mask),&mask)!=0)throw std::runtime_error("IK recovery: cannot read CPU affinity");
  std::vector<int> allowed,efficient,physical;std::set<std::string> siblings;bool has_smt=false;
  for(int cpu=0;cpu<CPU_SETSIZE;++cpu)if(CPU_ISSET(cpu,&mask)) {
    allowed.push_back(cpu);std::ifstream f(sys_root+"/cpu"+std::to_string(cpu)+"/topology/thread_siblings_list");
    std::string group;std::getline(f,group);
    if(group.empty())group=std::to_string(cpu);
    const bool smt=group.find(',')!=std::string::npos||group.find('-')!=std::string::npos;
    has_smt=has_smt||smt;if(!smt)efficient.push_back(cpu);
    if(siblings.insert(group).second)physical.push_back(cpu);
  }
  if(!requested.empty()) {
    std::set<int> unique;
    for(int cpu:requested)if(cpu<0||cpu>=CPU_SETSIZE||!CPU_ISSET(cpu,&mask)||!unique.insert(cpu).second)
      throw std::invalid_argument("IK recovery: CPU list contains unavailable or duplicate CPUs");
    if(requested.size()!=workers)throw std::invalid_argument("IK recovery: CPU list must match worker count");
    return requested;
  }
  // i7-1355U exposes two SMT P cores and eight single-thread E cores. On a
  // homogeneous/virtual machine use allowed physical cores, never claim E cores.
  auto selected=has_smt&&!efficient.empty()?efficient:physical;
  if(selected.empty())selected=allowed;
  if(selected.empty()||workers==0)throw std::invalid_argument("IK recovery: empty CPU selection");
  selected.resize(std::min(workers,selected.size()));return selected;
}

struct IkRecoveryClient::Impl {
  IkRecoveryOptions options;std::string urdf,meshes,signature;
  Json::Value arm_limits;
  mutable std::timed_mutex mutex;
  IkRecoveryStats stats;int fd{-1};pid_t pid{-1};std::uint64_t serial{0};
  ~Impl(){closeProcess();}
  void closeProcess() {
    if(fd>=0){close(fd);fd=-1;}
    if(pid<=0)return;
    int status=0;
    const auto orderly=after(.1);
    while(waitpid(pid,&status,WNOHANG)==0&&IkClock::now()<orderly)std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if(waitpid(pid,&status,WNOHANG)==0) {
      kill(-pid,SIGTERM);
      const auto end=after(.2);
      while(waitpid(pid,&status,WNOHANG)==0&&IkClock::now()<end)std::this_thread::sleep_for(std::chrono::milliseconds(5));
      if(waitpid(pid,&status,WNOHANG)==0){kill(-pid,SIGKILL);while(waitpid(pid,&status,0)<0&&errno==EINTR){}}
    }
    pid=-1;
  }
  void start() {
    closeProcess();int pair[2];
    if(socketpair(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0,pair)!=0)throw std::runtime_error("IK recovery socketpair failed");
    posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
    constexpr int child_fd=198;
    posix_spawn_file_actions_adddup2(&actions,pair[1],child_fd);
    if(pair[0]!=child_fd)posix_spawn_file_actions_addclose(&actions,pair[0]);
    if(pair[1]!=child_fd)posix_spawn_file_actions_addclose(&actions,pair[1]);
    posix_spawn_file_actions_addopen(&actions,STDOUT_FILENO,"/dev/null",O_WRONLY,0);
    posix_spawnattr_t attr;posix_spawnattr_init(&attr);
    posix_spawnattr_setflags(&attr,POSIX_SPAWN_SETPGROUP);posix_spawnattr_setpgroup(&attr,0);
    std::ostringstream cpu;for(auto c:stats.cpus){if(cpu.tellp()>0)cpu<<',';cpu<<c;}
    std::vector<std::string> args{options.worker_executable,"--fd",std::to_string(child_fd),"--urdf",urdf,"--meshes",meshes,"--cpus",cpu.str()};
    if (!arm_limits.isNull()) {args.push_back("--arm-limits");args.push_back(encode(arm_limits));}
    std::vector<char*> argv;for(auto& a:args)argv.push_back(a.data());argv.push_back(nullptr);
    const int error=posix_spawn(&pid,args[0].c_str(),&actions,&attr,argv.data(),environ);
    posix_spawn_file_actions_destroy(&actions);posix_spawnattr_destroy(&attr);close(pair[1]);
    if(error){close(pair[0]);pid=-1;throw std::runtime_error("IK recovery broker spawn failed: "+std::to_string(error));}
    fd=pair[0];Json::Value hello;
    if(!receiveMessage(fd,hello,after(30))||hello["protocol"].asUInt()!=kProtocol||
       hello["signature"].asString()!=signature||hello["workers"].asUInt()!=stats.cpus.size()) {
      closeProcess();throw std::runtime_error("IK recovery broker startup/model mismatch: "+encode(hello));
    }
    stats.status="ready";
  }
};

IkRecoveryClient::IkRecoveryClient(IkRecoveryOptions options,std::string urdf,std::string meshes,std::string signature,Json::Value arm_limits)
  :impl_(std::make_unique<Impl>()) {
  if(options.seeds<1||options.seeds>96||options.workers<1||options.workers>8||
     !std::isfinite(options.seed_timeout_s)||options.seed_timeout_s<=0||options.seed_timeout_s>.5||
     !std::isfinite(options.request_timeout_s)||options.request_timeout_s<=0||options.request_timeout_s>30)
    throw std::invalid_argument("IK recovery options outside supported bounds");
  if(options.worker_executable.empty())options.worker_executable=(std::filesystem::read_symlink("/proc/self/exe").parent_path()/"ik_recovery_worker").string();
  impl_->options=std::move(options);impl_->urdf=std::move(urdf);impl_->meshes=std::move(meshes);impl_->signature=std::move(signature);
  impl_->arm_limits=std::move(arm_limits);
  impl_->stats.cpus=selectIkRecoveryCpus(impl_->options.workers,impl_->options.cpus);
  impl_->start();
}
IkRecoveryClient::~IkRecoveryClient()=default;
const IkRecoveryOptions& IkRecoveryClient::options() const {return impl_->options;}
IkRecoveryStats IkRecoveryClient::stats() const {std::lock_guard<std::timed_mutex> lock(impl_->mutex);return impl_->stats;}
bool IkRecoveryClient::solve(Json::Value request,const std::function<bool(const Json::Value&)>& accept,const IkStop& stop) {
  auto& p=*impl_;const auto begin=IkClock::now();
  const double budget=request.isMember("remaining_s")?std::min(p.options.request_timeout_s,request["remaining_s"].asDouble()):p.options.request_timeout_s;
  if(!std::isfinite(budget)||budget<=0)return false;
  const auto deadline=after(budget);
  std::unique_lock<std::timed_mutex> lock(p.mutex,std::defer_lock);
  while(!lock.try_lock_for(std::chrono::milliseconds(5)))if(IkClock::now()>=deadline||(stop&&stop()))return false;
  if(stop&&stop())return false;
  ++p.stats.requests;p.stats.attempted=p.stats.timed_out=0;
  bool accepted=false,cancelled=false;
  try {
    // A broken broker never causes an unbounded cold start inside an action.
    // Reconfiguration/restart explicitly restores it; normal IK remains usable.
    if(p.fd<0)throw std::runtime_error("broker unavailable");
    request["id"]=Json::UInt64(++p.serial);request["op"]="solve";
    request["seed_timeout_s"]=p.options.seed_timeout_s;
    request["request_timeout_s"]=std::chrono::duration<double>(deadline-IkClock::now()).count();
    if(!sendMessage(p.fd,request,deadline,stop))throw std::runtime_error("request send failed");
    auto read_deadline=deadline;
    while(true) {
      // Receiving a frame is not interrupted halfway: read at most a small
      // bounded frame; stop is polled by the outer loop and broker every 5 ms.
      if(!cancelled&&((stop&&stop())||IkClock::now()>=deadline)) {
        cancelled=true;Json::Value cancel;cancel["op"]="cancel";cancel["id"]=Json::UInt64(p.serial);
        if(!sendMessage(p.fd,cancel,after(.1)))throw std::runtime_error("cancel send failed");
        read_deadline=after(.8);
      }
      pollfd poller{p.fd,POLLIN,0};const int rc=poll(&poller,1,5);
      if(rc<0&&errno==EINTR)continue;
      if(rc<0||(rc>0&&!(poller.revents&POLLIN))||IkClock::now()>=read_deadline)throw std::runtime_error("broker disconnected or timed out");
      if(!rc)continue;
      Json::Value response;if(!receiveMessage(p.fd,response,read_deadline))throw std::runtime_error("invalid broker response");
      if(response["id"].asUInt64()!=p.serial)throw std::runtime_error("stale IK recovery response");
      if(response["op"]=="candidate") {
        const bool ok=!cancelled&&IkClock::now()<deadline&&!(stop&&stop())&&accept(response["q"]);
        Json::Value ack;ack["op"]="accept";ack["id"]=Json::UInt64(p.serial);ack["accepted"]=ok;
        if(!sendMessage(p.fd,ack,read_deadline))throw std::runtime_error("candidate acknowledgement failed");
        accepted=accepted||ok;
      } else if(response["op"]=="done") {
        p.stats.attempted=response["attempted"].asUInt();p.stats.timed_out=response["timeouts"].asUInt();
        accepted=accepted&&response["success"].asBool()&&!cancelled&&!(stop&&stop());
        p.stats.status=accepted?"success":cancelled?"cancelled":"exhausted";break;
      } else throw std::runtime_error("unexpected broker response");
    }
  } catch(const std::exception& error) {p.stats.status=error.what();p.closeProcess();accepted=false;}
  p.stats.elapsed_s=std::chrono::duration<double>(IkClock::now()-begin).count();
  if(accepted)++p.stats.successes;else if(cancelled)++p.stats.cancelled;else ++p.stats.failures;
  return accepted;
}
} // namespace motion_control_pkg::motion
