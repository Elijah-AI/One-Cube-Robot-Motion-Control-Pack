#include "ds485_driver/gripper_controller.h"
#include "fake_servo_port.h"
#include <atomic>
#include <iostream>
#include <stdexcept>
using E = ds485_driver::GripperController;
#define CHECK(x)                                                                                   \
  do {                                                                                             \
    if (!(x))                                                                                      \
      throw std::runtime_error(std::string(__func__) + ":" + std::to_string(__LINE__) + " " #x);   \
  } while (0)
class Port : public FakeServoPort {
public:
  std::atomic<int> moves{0}, holds{0};
  std::atomic<bool> fail_hold{false}, lose_move_ack{false};
  bool Send(const uint8_t *p, int n) override {
    if (n >= 7 && p[4] == DS485_CMD_WRITE && p[5] == DS485_REG_SWITCH) {
      ++holds;
      servo.no_reply = false;
      servo.reject_switch_write = fail_hold;
    }
    const bool move = n >= 7 && p[4] == DS485_CMD_WRITE && p[5] == DS485_REG_MOVE_SPEED;
    if (move && lose_move_ack)
      servo.no_reply = true;
    bool ok = FakeServoPort::Send(p, n);
    if (move) {
      ++moves;
      servo.no_reply = false;
    }
    return ok;
  }
};
template <class F> void wait(F f) {
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (!f()) {
    CHECK(std::chrono::steady_clock::now() < until);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
struct Fixture {
  Port *ports[2]{};
  std::unique_ptr<E> e;
  std::atomic<int> results{0};
  std::atomic<int> samples{0};
  Fixture(bool bad_right = false, bool explicit_speed = false) {
    std::array<E::Config, 2> c;
    for (auto &v : c) {
      v.gripper.open_angle = 0;
      v.gripper.close_angle = 1000;
      v.gripper.blocked_samples = 100000;
      v.reply_timeout_ms = 10;
    }
    if (bad_right)
      c[1].gripper.close_angle = 0;
    if (explicit_speed)
      c[0].gripper.max_speed = 100;
    e = std::make_unique<E>(
        c,
        [this, explicit_speed](unsigned i, uint8_t) {
          auto p = std::make_unique<Port>();
          ports[i] = p.get();
          p->servo.torque_switch = 2;
          if (i == 0 && explicit_speed)
            p->servo.hide_speed_limit = true;
          return p;
        },
        [this](const auto &, const auto &) { ++results; }, E::StateCallback{}, 2, 0, std::function<bool()>{},
        [this](unsigned, const std::string &message) {
          if (message.find("temperature_C=") != std::string::npos &&
              message.find("voltage_mV=") != std::string::npos &&
              message.find("current_mA=") != std::string::npos &&
              message.find("pwm=") != std::string::npos)
            ++samples;
        });
    e->start();
    wait([&] { return e->state(0).online && e->state(1).online; });
  }
  ~Fixture() {
    if (e)
      e->shutdown();
  }
  E::Command command(unsigned side = 0, bool action = false) {
    E::Command c;
    c.side = side;
    c.action = action;
    c.position = 255;
    c.speed = 1;
    c.torque = 100;
    c.timeout_ms = 3000;
    return c;
  }
  E::Handle submit(E::Command c) {
    std::vector<E::Handle> t;
    std::string error;
    CHECK(e->submit({c}, t, error));
    return t[0];
  }
  E::Outcome done(E::Handle t) {
    wait([&] { return e->outcome(t).done; });
    return e->outcome(t);
  }
};
void test_arrival_and_dual_atomicity() {
  Fixture f;
  auto a = f.command(0, true), b = f.command(1, true);
  a.position = 0;
  b.position = 0;
  std::vector<E::Handle> t;
  std::string err;
  CHECK(f.e->submit({a, b}, t, err));
  CHECK(f.done(t[0]).result == E::Arrived);
  CHECK(f.done(t[1]).result == E::Arrived);
  CHECK(f.results == 0);
  f.e->shutdown();
  Fixture invalid(true);
  CHECK(!invalid.e->submit({a, b}, t, err));
  invalid.e->shutdown();
  CHECK(invalid.ports[0]->moves == 0);
}
void test_action_topic_exclusion() {
  Fixture f;
  auto action = f.submit(f.command(0, true));
  wait([&] { return f.ports[0]->moves > 0; });
  std::vector<E::Handle> t;
  std::string err;
  CHECK(!f.e->submit({f.command(0)}, t, err));
  CHECK(!f.e->submit({f.command(0), f.command(1)}, t, err));
  CHECK(!f.e->busy(1));
  auto right = f.submit(f.command(1));
  CHECK(f.e->busy(1));
  CHECK(!f.e->submit({f.command(1, true)}, t, err));
  f.e->cancel({action, right});
  CHECK(f.done(action).result == E::Canceled);
  CHECK(f.done(right).result == E::Canceled);
  f.e->shutdown();
  CHECK(f.ports[0]->holds > 0);
}
void test_preemption_and_old_cancel() {
  Fixture f;
  auto old = f.submit(f.command());
  wait([&] { return f.ports[0]->moves > 0; });
  auto newer = f.submit(f.command());
  CHECK(f.done(old).result == E::Preempted);
  wait([&] { return f.ports[0]->moves >= 2; });
  f.e->cancel({old});
  CHECK(!f.e->outcome(newer).done);
  f.e->cancel({newer});
  CHECK(f.done(newer).result == E::Canceled);
  f.e->shutdown();
  CHECK(f.results == 2);
}
void test_timeout_and_stop() {
  Fixture f;
  auto c = f.command();
  c.timeout_ms = 15;
  auto t = f.submit(c);
  CHECK(f.done(t).result == E::Timeout);
  CHECK(f.ports[0]->holds > 0);
  auto a = f.submit(f.command());
  auto b = f.submit(f.command(1));
  f.e->stop(2);
  CHECK(f.done(a).result == E::Canceled);
  CHECK(f.done(b).result == E::Canceled);
  wait([&] { return !f.e->busy(0) && !f.e->busy(1); });
  f.e->shutdown();
}
void test_failure_and_shutdown() {
  {
    Fixture f(false, true);
    auto t = f.submit(f.command());
    CHECK(f.done(t).result == E::Failed);
    CHECK(f.ports[0]->moves == 0);
    f.e->shutdown();
  }
  {
    Fixture f;
    f.ports[0]->lose_move_ack = true;
    auto t = f.submit(f.command());
    CHECK(f.done(t).result == E::Failed);
    CHECK(f.ports[0]->holds > 0);
    f.e->shutdown();
  }
  {
    Fixture f;
    auto t = f.submit(f.command());
    wait([&] { return f.ports[0]->moves > 0; });
    f.ports[0]->fail_hold = true;
    f.e->cancel({t});
    CHECK(f.done(t).result == E::Failed);
    std::vector<E::Handle> tickets;
    std::string error;
    CHECK(!f.e->submit({f.command()}, tickets, error));
    CHECK(f.e->busy(0));
    f.e->shutdown();
  }
  {
    Fixture f;
    auto t = f.submit(f.command());
    wait([&] { return f.ports[0]->moves > 0; });
    f.e->shutdown();
    CHECK(f.e->outcome(t).result == E::Canceled);
    CHECK(f.ports[0]->holds > 0);
    std::vector<E::Handle> tickets;
    std::string error;
    CHECK(!f.e->submit({f.command()}, tickets, error));
  }
  {
    Fixture f;
    f.ports[0]->servo.fault_bits = 0x10;
    auto t = f.submit(f.command());
    CHECK(!f.done(t).success());
    f.e->shutdown();
    CHECK(f.ports[0]->moves == 0);
    CHECK(f.ports[0]->holds == 0);
  }
  {
    Fixture f;
    f.ports[0]->servo.fault_bits = 0x10;
    f.e->stop(0);
    wait([&] { return f.e->state(0).servo.fault; });
    f.e->shutdown();
    CHECK(f.ports[0]->holds == 0);
  }
}
void test_recovery_needs_new_command() {
  Fixture f;
  auto &servo = f.ports[0]->servo;
  servo.fault_bits = 0x04;
  auto command = f.command();
  command.position = 0;
  CHECK(f.done(f.submit(command)).result == E::Fault);
  CHECK(servo.motion_writes == 1 && f.ports[0]->holds == 0);
  CHECK(f.samples == 1);
  servo.resolved_causes = 0x04;
  CHECK(f.done(f.submit(command)).result == E::Arrived);
  CHECK(servo.motion_writes == 2 && servo.fault_bits == 0);
  CHECK(f.samples == 1);
}
int main() {
  try {
    test_arrival_and_dual_atomicity();
    test_action_topic_exclusion();
    test_preemption_and_old_cancel();
    test_timeout_and_stop();
    test_failure_and_shutdown();
    test_recovery_needs_new_command();
    std::cout << "DS485 executor: 6 groups PASS\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
