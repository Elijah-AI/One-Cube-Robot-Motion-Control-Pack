#include "motion_control_pkg/execution/shutdown_request.hpp"
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
int main(int argc,char** argv) {
  using namespace motion_control_pkg::execution;
  using namespace std::chrono_literals;
  const bool hold=argc>1 && std::strcmp(argv[1],"--hold")==0;
  installShutdownSignalHandlers();
  rclcpp::init(argc,argv,rclcpp::InitOptions{},rclcpp::SignalHandlerOptions::None);
  auto node=std::make_shared<rclcpp::Node>("offline_shutdown_signal_fixture");
  std::atomic<int> frames{0};
  auto timer=node->create_wall_timer(5ms,[&]{++frames;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin([&]{executor.spin();});
  if (!hold) raise(SIGINT);
  const auto deadline=std::chrono::steady_clock::now()+20s;
  while(!shutdownRequested() && std::chrono::steady_clock::now()<deadline) std::this_thread::sleep_for(5ms);
  const int before=frames;
  // --hold runs longer than launch's default 5+5 s escalation deadline. This
  // fixture alone exits the simulated failed hold for the offline assertion.
  const auto end=std::chrono::steady_clock::now()+(hold?12s:1s);
  while(std::chrono::steady_clock::now()<end && rclcpp::ok()) std::this_thread::sleep_for(5ms);
  raise(SIGINT);raise(SIGTERM);
  const bool ok=shutdownRequested() && rclcpp::ok() && frames-before>(hold?1000:100);
  executor.cancel();spin.join();rclcpp::shutdown();
  if (!ok) {std::cerr<<"FAIL signal killed ROS/feedback\n";return 1;}
  std::cout<<"PASS_SIGNAL_CONTEXT_ALIVE frames_after_signal="<<frames-before<<" no hardware\n";
  return 0;
}
