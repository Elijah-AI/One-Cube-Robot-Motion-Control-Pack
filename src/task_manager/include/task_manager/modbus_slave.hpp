// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Author: gene.wang
// Date: 2026
//
// Description:
// Modbus TCP 从站（Slave）封装头文件。
// 实现基于 libmodbus 的 Modbus TCP 服务器，支持保持寄存器的读写操作，
// 并提供寄存器写入回调注册机制，用于与 PLC 等主站设备通信。

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

namespace task_modbus
{
using RegAddr = uint32_t;   ///< 寄存器地址类型
using RegValue = uint16_t;  ///< 寄存器值类型

// ==================== Modbus 协议常量 ====================
constexpr RegAddr MODBUS_PROTOCOL_ADDR_OFFSET = 40001;  ///< Modbus 协议地址偏移（逻辑地址转实际地址）
constexpr int MODBUS_TCP_HEADER_LEN = 12;                ///< Modbus TCP 最小报文头长度
constexpr int MODBUS_MAX_REGISTERS = 65535;               ///< 最大寄存器数量
constexpr int MODBUS_LISTEN_BACKLOG = 5;                  ///< 监听 socket 最大挂起连接数
constexpr int SERVER_POLL_INTERVAL_MS = 100;              ///< 服务器轮询间隔（毫秒）

/// @brief Modbus TCP 从站类
/// 提供 Modbus TCP 服务器功能，支持保持寄存器的读写操作，
/// 以及写入事件的回调通知机制。
class ModbusTcpSlave
{
public:
  /// @brief 构造函数
  /// @param ip 监听 IP 地址
  /// @param port 监听端口号
  /// @param logger ROS2 日志记录器
  ModbusTcpSlave(const std::string& ip, uint16_t port, rclcpp::Logger logger);

  /// @brief 析构函数，自动停止服务并释放资源
  ~ModbusTcpSlave();

  /// @brief 启动 Modbus TCP 服务（创建监听线程）
  void start();

  /// @brief 停止 Modbus TCP 服务
  void stop();

  /// @brief 读取保持寄存器值
  /// @param addr 寄存器地址
  /// @return 寄存器当前值，未找到时返回 0
  RegValue get_reg(RegAddr addr);

  /// @brief 写入保持寄存器值
  /// @param addr 寄存器地址
  /// @param val 要写入的值
  void set_reg(RegAddr addr, RegValue val);

  /// @brief 寄存器写入回调类型
  using WriteRegCb = std::function<void(RegAddr addr, RegValue old_val, RegValue new_val)>;

  /// @brief 注册寄存器写入回调
  /// @param cb 回调函数，参数为 (地址, 旧值, 新值)
  void register_write_callback(WriteRegCb cb);

  /// @brief 获取当前实际寄存器地址
  RegAddr get_regaddr();

  /// @brief 设置当前实际寄存器地址
  void set_regaddr(RegAddr addr);

private:
  std::string listen_ip_;                              ///< 监听 IP 地址
  uint16_t port_;                                      ///< 监听端口号
  rclcpp::Logger logger_;                              ///< ROS2 日志记录器
  std::unordered_map<RegAddr, RegValue> holding_regs_; ///< 保持寄存器映射表
  int server_fd_ = -1;                                 ///< 服务器 socket 文件描述符
  std::atomic<bool> run_flag_{false};                  ///< 运行标志（线程安全）
  std::thread server_thread_;                          ///< 服务器监听线程
  WriteRegCb write_cb_;                                ///< 寄存器写入回调
  std::atomic<RegAddr> real_reg_addr_{0};                              ///< 当前实际寄存器地址
  modbus_t* ctx_ = nullptr;                            ///< libmodbus 上下文指针
  std::mutex reg_mutex_;                               ///< 寄存器访问互斥锁

  /// @brief 服务器主循环（在独立线程中运行）
  void server_loop();

  /// @brief 处理 Modbus 请求
  /// @param query 请求报文缓冲区
  /// @param query_len 请求报文长度
  void handle_modbus_request(uint8_t* query, int query_len);
};
}

#endif // TASK_MANAGER__MODBUS_SLAVE_HPP_
