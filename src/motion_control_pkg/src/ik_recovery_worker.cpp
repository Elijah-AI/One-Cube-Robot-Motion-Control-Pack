#include "motion_control_pkg/miracle_m_algo.hpp"
#include "motion_control_pkg/motion/ik_recovery_transport.hpp"
#include <rcutils/logging.h>
#include <atomic>
#include <deque>
#include <filesystem>
#include <sstream>
#include <thread>
#include <sched.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/wait.h>

using namespace motion_control_pkg::motion;
using namespace motion_control_pkg::motion::ik_transport;
namespace {
struct Worker {pid_t pid{-1};int fd{-1};int cpu{-1};bool busy{false};std::uint64_t request_id{0};IkClock::time_point deadline;};
struct Pool {
  MiracleMAlgo& algo;int control;std::vector<Worker> workers;
  std::atomic<std::uint64_t>* active;
  Pool(MiracleMAlgo& a,int c,const std::vector<int>& cpus):algo(a),control(c) {
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free,"shared cancellation must be lock free");
    auto* memory=mmap(nullptr,sizeof(*active),PROT_READ|PROT_WRITE,MAP_SHARED|MAP_ANONYMOUS,-1,0);
    if(memory==MAP_FAILED)throw std::runtime_error("IK cancel mmap failed");
    active=new(memory)std::atomic<std::uint64_t>(0);
    for(int cpu:cpus){Worker w;w.cpu=cpu;workers.push_back(w);}
    try {for(std::size_t i=0;i<workers.size();++i)spawn(i);}catch(...){shutdown();throw;}
  }
  void stopWorker(Worker& w) {
    if(w.pid>0){kill(w.pid,SIGKILL);while(waitpid(w.pid,nullptr,0)<0&&errno==EINTR){}w.pid=-1;}
    if(w.fd>=0){close(w.fd);w.fd=-1;}w.busy=false;
  }
  void shutdown(){for(auto& w:workers)stopWorker(w);if(active){munmap(active,sizeof(*active));active=nullptr;}}
  ~Pool(){shutdown();}
  void spawn(std::size_t index) {
    int pair[2];if(socketpair(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0,pair))throw std::runtime_error("worker socketpair failed");
    auto& w=workers[index];const pid_t parent=getpid();const pid_t pid=fork();
    if(pid<0){close(pair[0]);close(pair[1]);throw std::runtime_error("worker fork failed");}
    if(pid==0) {
      close(pair[0]);close(control);for(auto& other:workers)if(other.fd>=0)close(other.fd);
      prctl(PR_SET_PDEATHSIG,SIGKILL);if(getppid()!=parent)_exit(2);
      cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(w.cpu,&mask);
      const int affinity_error=sched_setaffinity(0,sizeof(mask),&mask)?errno:0;
      Json::Value ready;ready["ready"]=affinity_error==0;ready["cpu"]=sched_getcpu();ready["affinity_errno"]=affinity_error;
      if(!sendMessage(pair[1],ready,after(2)))_exit(4);
      if(affinity_error)_exit(3);
      std::uint64_t previous_id=0;Json::Value request;
      while(true) {
        Json::Value job;if(!receiveMessage(pair[1],job,IkClock::time_point::max()))_exit(0);
        const auto id=job["id"].asUInt64();Json::Value result;result["id"]=Json::UInt64(id);result["seed"]=job["seed"];
        try {
          if(previous_id!=id) {request=job["request"];IkRecoveryAccess::applyScene(algo,request["scene"]);previous_id=id;}
          const auto begin=IkClock::now();const auto deadline=after(request["seed_timeout_s"].asDouble());
          auto stop=[&,id,deadline]{return active->load(std::memory_order_acquire)!=id||IkClock::now()>=deadline;};
          auto outcome=IkRecoveryAccess::evaluate(algo,request,request["seeds"][job["seed"].asUInt()],stop);
          result["solver_s"]=std::chrono::duration<double>(IkClock::now()-begin).count();
          result["timeout"]=IkClock::now()>=deadline;
          result["solved"]=outcome["solved"].asBool()&&!stop();
          if(result["solved"].asBool())result["q"]=outcome["q"];
        }catch(const std::exception& e){result["solved"]=false;result["error"]=e.what();}
        if(!sendMessage(pair[1],result,after(1)))_exit(0);
      }
    }
    close(pair[1]);w.pid=pid;w.fd=pair[0];w.busy=false;w.request_id=0;
    Json::Value ready;if(!receiveMessage(w.fd,ready,after(2))||!ready["ready"].asBool()||ready["cpu"].asInt()!=w.cpu) {
      int status=0;const auto exited=waitpid(w.pid,&status,WNOHANG);
      if(exited==w.pid)w.pid=-1;
      throw std::runtime_error("worker CPU affinity/startup failed: requested="+std::to_string(w.cpu)+" ready="+encode(ready)+" wait="+std::to_string(exited)+" status="+std::to_string(status));
    }
  }
  void solve(const Json::Value& request) {
    const auto id=request["id"].asUInt64();const auto count=request["seeds"].size();
    if(!id||count<1||count>96)throw std::runtime_error("invalid recovery request");
    const double seed_budget=request["seed_timeout_s"].asDouble(),total_budget=request["request_timeout_s"].asDouble();
    if(!(seed_budget>0&&seed_budget<=.5&&total_budget>0&&total_budget<=30))throw std::runtime_error("invalid recovery budget");
    const auto deadline=after(total_budget);active->store(id,std::memory_order_release);
    unsigned next=0,timeouts=0;bool stopping=false,success=false,awaiting=false;
    std::deque<Json::Value> candidates;
    while(true) {
      if(IkClock::now()>=deadline&&!success)stopping=true;
      if(stopping){active->store(0,std::memory_order_release);candidates.clear();awaiting=false;}
      // Prefer candidate validation before dispatching more work.
      if(!stopping&&!awaiting&&!candidates.empty()) {
        Json::Value candidate=candidates.front();candidates.pop_front();candidate["op"]="candidate";
        if(!sendMessage(control,candidate,after(.2)))throw std::runtime_error("candidate delivery failed");
        awaiting=true;
      }
      if(!stopping&&!awaiting)for(auto& w:workers)if(!w.busy&&next<count) {
        Json::Value job;job["id"]=Json::UInt64(id);job["seed"]=next++;
        if(w.request_id!=id){job["request"]=request;w.request_id=id;}
        if(!sendMessage(w.fd,job,deadline))throw std::runtime_error("seed dispatch failed");
        w.busy=true;w.deadline=after(seed_budget+.15);
      }
      bool busy=false;for(const auto& w:workers)busy=busy||w.busy;
      if(!busy&&(stopping||(!awaiting&&candidates.empty()&&next==count)))break;
      std::vector<pollfd> fds{{control,POLLIN,0}};for(auto& w:workers)fds.push_back({w.fd,POLLIN,0});
      const int rc=poll(fds.data(),fds.size(),5);if(rc<0&&errno==EINTR)continue;if(rc<0)throw std::runtime_error("broker poll failed");
      if(fds[0].revents) {
        Json::Value command;if(!receiveMessage(control,command,after(.2)))throw std::runtime_error("controller disconnected");
        if(command["id"].asUInt64()!=id)throw std::runtime_error("request id mismatch");
        if(command["op"]=="cancel")stopping=true;
        else if(command["op"]=="accept") {
          if(command["accepted"].asBool()&&!stopping){success=true;stopping=true;}
          awaiting=false;
        } else throw std::runtime_error("unexpected control message");
      }
      for(std::size_t i=0;i<workers.size();++i) {
        auto& w=workers[i];if(!w.busy)continue;
        bool broken=false;
        if(fds[i+1].revents) {
          Json::Value result;
          if(!receiveMessage(w.fd,result,after(.05))||result["id"].asUInt64()!=id)broken=true;
          else {
            w.busy=false;timeouts+=result["timeout"].asBool();
            if(result["solved"].asBool()&&!stopping)candidates.push_back(std::move(result));
          }
        }
        if(broken||(w.busy&&IkClock::now()>=w.deadline)) {++timeouts;stopWorker(w);spawn(i);}
      }
    }
    active->store(0,std::memory_order_release);
    Json::Value done;done["op"]="done";done["id"]=Json::UInt64(id);done["success"]=success;done["attempted"]=next;done["timeouts"]=timeouts;
    if(!sendMessage(control,done,after(.2)))throw std::runtime_error("completion delivery failed");
  }
};
} // namespace

int main(int argc,char** argv) try {
  int fd=-1;std::string urdf,meshes,cpu_list,arm_limits;
  for(int i=1;i+1<argc;i+=2) {const std::string key=argv[i];if(key=="--fd")fd=std::stoi(argv[i+1]);else if(key=="--urdf")urdf=argv[i+1];else if(key=="--meshes")meshes=argv[i+1];else if(key=="--cpus")cpu_list=argv[i+1];else if(key=="--arm-limits")arm_limits=argv[i+1];else throw std::runtime_error("invalid broker argument");}
  if(fd<0||urdf.empty()||meshes.empty())return 2;
  const auto parent=getppid();prctl(PR_SET_PDEATHSIG,SIGTERM);if(getppid()!=parent)return 2;
  for(const char* name:{"OMP_NUM_THREADS","OPENBLAS_NUM_THREADS","MKL_NUM_THREADS"})setenv(name,"1",1);
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL);
  std::vector<int> cpus;std::stringstream stream(cpu_list);std::string part;while(std::getline(stream,part,','))cpus.push_back(std::stoi(part));
  cpus=selectIkRecoveryCpus(cpus.size(),cpus);
  cpu_set_t mask;CPU_ZERO(&mask);for(int cpu:cpus)CPU_SET(cpu,&mask);if(sched_setaffinity(0,sizeof(mask),&mask))return 3;
  // No rclcpp::init, DDS, hardware SDK or planning thread pool in this process.
  MiracleMAlgo algo(urdf,meshes);
  if (!arm_limits.empty()) {
    Json::Value limits;std::istringstream input(arm_limits);input>>limits;
    IkRecoveryAccess::applySdkArmPositionLimits(algo,limits);
  }
  if(std::distance(std::filesystem::directory_iterator("/proc/self/task"),std::filesystem::directory_iterator{})!=1)
    throw std::runtime_error("broker must remain single threaded before fork");
  Pool pool(algo,fd,cpus);
  Json::Value hello;hello["protocol"]=kProtocol;hello["workers"]=static_cast<unsigned>(cpus.size());hello["signature"]=IkRecoveryAccess::signature(algo);
  if(!sendMessage(fd,hello,after(2)))return 4;
  while(true) {
    Json::Value request;if(!receiveMessage(fd,request,IkClock::time_point::max()))break;
    if(request["op"]=="solve")pool.solve(request); // Ignore a late cancel/ack after its completed request.
  }
  return 0;
}catch(const std::exception& error){std::cerr<<"IK recovery broker: "<<error.what()<<'\n';return 1;}
