#ifndef TASK_MANAGER__MODBUS_SLAVE_HPP_
#define TASK_MANAGER__MODBUS_SLAVE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <cstdint>
#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <modbus/modbus.h>
#include <mutex>
#include <unordered_map>
#include <functional>
#include <thread>

namespace task_modbus
{
using RegAddr = uint32_t;
using RegValue = uint16_t;

class ModbusTcpSlave
{
public:
  ModbusTcpSlave(const std::string& ip, uint16_t port, rclcpp::Logger logger);
  ~ModbusTcpSlave();

  void start();
  void stop();

  RegValue get_reg(RegAddr addr);
  void set_reg(RegAddr addr, RegValue val);

  using WriteRegCb = std::function<void(RegAddr addr, RegValue old_val, RegValue new_val)>;
  void register_write_callback(WriteRegCb cb);
  RegAddr get_regaddr();
  void set_regaddr(RegAddr addr);

private:
  std::string listen_ip_;
  uint16_t port_;
  rclcpp::Logger logger_;
  std::unordered_map<RegAddr, RegValue> holding_regs_;
  std::atomic<bool> run_flag_{false};
  std::thread server_thread_;
  WriteRegCb write_cb_;
  RegAddr real_reg_addr_;
  modbus_t* ctx_ = nullptr;
  int server_fd_ = -1;
  std::mutex reg_mutex_;  // 用于保护 holding_regs_ 的访问
  void server_loop();
  void handle_modbus_request(uint8_t* query, int query_len);
};
}

#endif