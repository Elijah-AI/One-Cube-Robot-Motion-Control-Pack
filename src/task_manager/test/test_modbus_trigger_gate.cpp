#include "task_manager/modbus_trigger_gate.hpp"
#include <atomic>
#include <thread>
#include <vector>
#include <stdexcept>
int main() {
  task_mgr::ModbusTriggerGate gate;
  auto check=[](bool ok){if(!ok) throw std::runtime_error("trigger gate regression");};
  uint32_t address=0;
  check(!gate.submit(40010));
  gate.set_idle(true);
  std::atomic<int> accepted{0};
  std::vector<std::thread> clients;
  for(int i=0;i<32;++i) clients.emplace_back([&]{if(gate.submit(40010)) ++accepted;});
  for(auto& t:clients)t.join();
  check(accepted==1 && gate.take(address) && address==40010);
  check(!gate.take(address) && !gate.submit(40011));
  gate.set_idle(false);
  for(int i=0;i<20;++i) check(!gate.submit(40010));
  // Completion (success or failure) does not replay any rejected write.
  gate.set_idle(true);
  check(!gate.take(address));
  check(gate.submit(40011));
  // Safety transition discards an idle request before executor dispatch.
  gate.set_idle(false);
  check(!gate.take(address) && !gate.submit(40010));
  gate.set_idle(true);
  check(gate.submit(40010) && gate.take(address));
}
