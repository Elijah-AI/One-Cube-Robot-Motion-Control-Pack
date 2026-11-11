#pragma once
#include "motion_control_pkg/motion/ik_recovery.hpp"
#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdexcept>

namespace motion_control_pkg::motion::ik_transport {
constexpr std::uint32_t kProtocol=1, kMaxMessage=32*1024*1024;
inline bool transfer(int fd, void* buffer, std::size_t bytes, bool writing,
                     IkClock::time_point deadline,const IkStop& stop={}) {
  auto* p=static_cast<char*>(buffer);
  while(bytes) {
    if(IkClock::now()>=deadline || (stop && stop()))return false;
    pollfd poller{fd,static_cast<short>(writing?POLLOUT:POLLIN),0};
    const auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-IkClock::now()).count();
    // Idle persistent workers sleep in poll; only an action with a cancellation
    // callback needs a short polling interval. Socket activity wakes immediately.
    const int wait_ms=stop?5:static_cast<int>(std::clamp<long long>(remaining,1,60000));
    const int rc=poll(&poller,1,wait_ms);
    if(rc<0 && errno==EINTR)continue;
    if(rc<0 || (rc>0 && !(poller.revents&(writing?POLLOUT:POLLIN))))return false;
    if(!rc)continue;
    const auto n=writing?send(fd,p,bytes,MSG_NOSIGNAL|MSG_DONTWAIT):recv(fd,p,bytes,MSG_DONTWAIT);
    if(n<0 && (errno==EINTR||errno==EAGAIN))continue;
    if(n<=0)return false;
    p+=n;bytes-=static_cast<std::size_t>(n);
  }
  return true;
}
inline std::string encode(const Json::Value& message) { Json::StreamWriterBuilder w;w["indentation"]="";return Json::writeString(w,message); }
inline bool sendMessage(int fd,const Json::Value& message,IkClock::time_point deadline,const IkStop& stop={}) {
  auto data=encode(message);if(data.size()>kMaxMessage)return false;
  std::uint32_t size=static_cast<std::uint32_t>(data.size());
  return transfer(fd,&size,sizeof(size),true,deadline,stop)&&transfer(fd,data.data(),size,true,deadline,stop);
}
inline bool receiveMessage(int fd,Json::Value& message,IkClock::time_point deadline,const IkStop& stop={}) {
  std::uint32_t size=0;
  if(!transfer(fd,&size,sizeof(size),false,deadline,stop)||size>kMaxMessage||!size)return false;
  std::string data(size,'\0');if(!transfer(fd,data.data(),size,false,deadline,stop))return false;
  Json::CharReaderBuilder b;std::string error;auto reader=std::unique_ptr<Json::CharReader>(b.newCharReader());
  return reader->parse(data.data(),data.data()+size,&message,&error);
}
inline IkClock::time_point after(double seconds) {return IkClock::now()+std::chrono::duration_cast<IkClock::duration>(std::chrono::duration<double>(seconds));}
} // namespace motion_control_pkg::motion::ik_transport
