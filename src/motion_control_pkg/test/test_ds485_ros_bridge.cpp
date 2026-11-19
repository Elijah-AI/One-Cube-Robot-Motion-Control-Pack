#include "ds485_driver/marvin_rs485_port.h"
#include "fake_servo_port.h"
#include "motion_control_pkg/gripper/ds485_ros_bridge.hpp"
#include <atomic>
#include <future>
#include <iostream>
#include <map>
using B = motion_control_pkg::gripper::Ds485RosBridge;
using E = motion_control_pkg::gripper::Ds485Executor;
using C = ds485_interfaces::msg::GripperCommand;
using R = ds485_interfaces::msg::GripperResult;
using Stop = ds485_interfaces::msg::GripperStop;
#define CHECK(x)                                                                                   \
  do {                                                                                             \
    if (!(x))                                                                                      \
      throw std::runtime_error(std::string(__func__) + ":" + std::to_string(__LINE__) + " " #x);   \
  } while (0)
// No SDK library is linked into this test. These spies also catch accidental Connect/Release
// ownership.
static int attaches = 0, releases = 0;
bool MarvinLink::connected_ = false;
bool MarvinLink::owner_ = false;
bool MarvinLink::Connect(const std::string &, std::string *) {
  throw std::runtime_error("unexpected SDK Connect");
}
void MarvinLink::Attach() {
  ++attaches;
  connected_ = true;
  owner_ = false;
}
void MarvinLink::Release() {
  CHECK(!owner_);
  ++releases;
  connected_ = false;
}
bool MarvinLink::IsConnected() {
  return connected_;
}
MarvinRs485Port::MarvinRs485Port(char arm, int channel)
    : arm_(arm), channel_(channel), sent_packets_(0), received_packets_(0), dropped_packets_(0) {}
bool MarvinRs485Port::Send(const uint8_t *, int) {
  return false;
}
bool MarvinRs485Port::SendBefore(const uint8_t* data, int size, double deadline) {
  return NowMs() < deadline && Send(data, size);
}
int MarvinRs485Port::Receive(uint8_t *, int, int) {
  return 0;
}
void MarvinRs485Port::Clear() {}
void MarvinRs485Port::SleepMs(int) {}
std::string MarvinRs485Port::Name() const {
  return "SDK-spy";
}
std::string MarvinRs485Port::LastError() const {
  return "offline";
}

class Port : public FakeServoPort {
public:
  std::atomic<int> moves{0}, holds{0};
  bool Send(const uint8_t *p, int n) override {
    bool ok = FakeServoPort::Send(p, n);
    if (n >= 7 && p[4] == DS485_CMD_WRITE) {
      if (p[5] == DS485_REG_MOVE_SPEED)
        ++moves;
      if (p[5] == DS485_REG_SWITCH)
        ++holds;
    }
    return ok;
  }
};
template <class F> void wait(F f) {
  auto end = std::chrono::steady_clock::now() + std::chrono::seconds(6);
  while (!f()) {
    CHECK(std::chrono::steady_clock::now() < end);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  auto node = std::make_shared<rclcpp::Node>("ds485_bridge_test", "/test_motion");
  auto client_node = std::make_shared<rclcpp::Node>("ds485_test_client");
  ds485_driver::DriverConfig config;
  for (auto &c : config.sides) {
    c.gripper.close_angle = 1000;
    c.gripper.blocked_samples = 100000;
    c.reply_timeout_ms = 10;
  }
  Port *ports[2]{};
  B bridge(node, config, [&](unsigned i, uint8_t) {
    auto p = std::make_unique<Port>();
    ports[i] = p.get();
    p->servo.torque_switch = 2;
    return p;
  });
  std::atomic<bool> admitted{true};
  bridge.setAdmission([&](unsigned, const std::function<void()> &f, std::string &error) {
    if (!admitted) {
      error = "test maintenance";
      return false;
    }
    f();
    return true;
  });
  bridge.start();
  std::mutex workers_mutex;
  std::vector<std::thread> workers;
  auto server = rclcpp_action::create_server<B::Action>(
      node, "/Control/control_gripper",
      [&](const auto &, const std::shared_ptr<const B::Action::Goal> goal) {
        std::string e;
        bool busy =
            (goal->gripper_id != 1 && bridge.busy(0)) || (goal->gripper_id != 0 && bridge.busy(1));
        return bridge.validGoal(*goal, e) && !busy ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                                                   : rclcpp_action::GoalResponse::REJECT;
      },
      [](const auto &) { return rclcpp_action::CancelResponse::ACCEPT; },
      [&](const auto &h) {
        std::lock_guard<std::mutex> lock(workers_mutex);
        workers.emplace_back([&, h] { bridge.runAction(h); });
      });
  auto action = rclcpp_action::create_client<B::Action>(client_node, "/Control/control_gripper");
  auto pub = client_node->create_publisher<C>("/ds485/gripper/command", 10);
  auto stop = client_node->create_publisher<Stop>("/ds485/gripper/stop", 10);
  std::mutex results_mutex;
  std::map<std::pair<int, int>, R> results;
  auto sub = client_node->create_subscription<R>("/ds485/gripper/result", 10, [&](R::SharedPtr m) {
    std::lock_guard<std::mutex> lock(results_mutex);
    results[{m->id, m->side}] = *m;
  });
  auto result = [&](int id, int side) {
    wait([&] {
      std::lock_guard<std::mutex> lock(results_mutex);
      return results.count({id, side});
    });
    std::lock_guard<std::mutex> lock(results_mutex);
    return results.at({id, side});
  };
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.add_node(client_node);
  std::thread spin([&] { exec.spin(); });
  auto goal = [](uint8_t side, uint8_t pos) {
    B::Action::Goal g;
    g.task_id = "offline_shared_gripper";
    g.gripper_id = side;
    g.position = {pos, pos};
    g.speed = {1, 1};
    g.max_torque = {100, 100};
    g.com_port = {2, 2};
    g.timeout_ms = 3000;
    g.block_until_complete = true;
    return g;
  };
  auto send = [&](B::Action::Goal g) {
    auto f = action->async_send_goal(g);
    CHECK(f.wait_for(std::chrono::seconds(4)) == std::future_status::ready);
    return f.get();
  };
  try {
    CHECK(action->wait_for_action_server(std::chrono::seconds(4)));
    wait([&] {
      return pub->get_subscription_count() > 0 && stop->get_subscription_count() > 0 &&
             sub->get_publisher_count() > 0;
    });
    C c;
    c.id = 10;
    c.side = 0;
    c.position = 0;
    c.speed = 1;
    c.torque = 100;
    pub->publish(c);
    CHECK(result(10, 0).success);
    auto h = send(goal(2, 0));
    CHECK(h);
    auto f = action->async_get_result(h);
    CHECK(f.wait_for(std::chrono::seconds(4)) == std::future_status::ready);
    auto done = f.get();
    CHECK(done.code == rclcpp_action::ResultCode::SUCCEEDED);
    CHECK(done.result->success);
    CHECK(done.result->final_position.size() == 2);
    auto bad = goal(0, 0);
    bad.com_port[0] = 3;
    CHECK(!send(bad));
    h = send(goal(0, 255));
    CHECK(h);
    wait([&] { return ports[0]->moves >= 3; });
    c.id = 11;
    c.position = 255;
    pub->publish(c);
    CHECK(result(11, 0).result == R::RESULT_FAILED);
    auto cancel = action->async_cancel_goal(h);
    CHECK(cancel.wait_for(std::chrono::seconds(4)) == std::future_status::ready);
    f = action->async_get_result(h);
    CHECK(f.wait_for(std::chrono::seconds(4)) == std::future_status::ready);
    CHECK(f.get().code == rclcpp_action::ResultCode::CANCELED);
    CHECK(ports[0]->holds > 0);
    c.id = 12;
    c.side = 1;
    c.position = 255;
    pub->publish(c);
    wait([&] { return ports[1]->moves >= 2; });
    CHECK(!send(goal(1, 0)));
    Stop s;
    s.side = 2;
    stop->publish(s);
    CHECK(result(12, 1).result == R::RESULT_CANCELED);
    admitted = false;
    c.id = 13;
    c.side = 0;
    pub->publish(c);
    CHECK(result(13, 0).result == R::RESULT_FAILED);
    c.id = 14;
    c.side = 3;
    pub->publish(c);
    CHECK(result(14, 3).result == R::RESULT_FAILED);
    std::cout << "DS485 ROS topic + Action integration: PASS\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    exit_code = 1;
  }
  bridge.shutdown();
  {
    std::lock_guard<std::mutex> lock(workers_mutex);
    for (auto &w : workers)
      if (w.joinable())
        w.join();
  }
  exec.cancel();
  spin.join();
  try {
    auto sdk_node = std::make_shared<rclcpp::Node>("ds485_sdk_ownership_test");
    sdk_node->declare_parameter("unused", 0);
    config.sides[0].enabled = config.sides[1].enabled = false;
    {
      B owner(sdk_node, config);
      owner.start();
      owner.shutdown();
    }
    CHECK(attaches == 1);
    CHECK(releases == 1);
    std::cout << "DS485 SDK Attach ownership: PASS (Connect never called)\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
