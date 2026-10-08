// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Author: gene.wang
// Date: 2026
//
// Description:
// Modbus TCP 从站（Slave）实现。
// 基于 libmodbus 库实现 Modbus TCP 服务器，支持：
// - 保持寄存器的读写操作
// - 单寄存器写入（MODBUS_FC_WRITE_SINGLE_REGISTER）
// - 保持寄存器读取（MODBUS_FC_READ_HOLDING_REGISTERS）
// - 寄存器写入事件回调通知
// - 多客户端连接管理

#include "task_manager/modbus_slave.hpp"
#include "task_manager/log_helper.hpp"
#include <cstring>
#include <unistd.h>
#include <modbus/modbus.h>
#include <thread>
#include <chrono>
#include <poll.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <cerrno>
#include <vector>
#include <algorithm>

namespace task_modbus
{
/// @brief 构造函数：创建 Modbus TCP 上下文
/// @param ip 监听 IP 地址
/// @param port 监听端口号
/// @param logger ROS2 日志记录器
ModbusTcpSlave::ModbusTcpSlave(const std::string& ip, uint16_t port, rclcpp::Logger logger)
: listen_ip_(ip), port_(port), logger_(logger), server_fd_(-1), run_flag_(false)
{
  // 创建 Modbus TCP 上下文
  ctx_ = modbus_new_tcp(listen_ip_.c_str(), port_);
  if (!ctx_)
  {
    LOG_ERROR_CRITICAL(logger_, task_log::Tag::MODBUS, "Create modbus context failed: " << modbus_strerror(errno));
    return;
  }
  modbus_set_slave(ctx_, 1);
  modbus_set_debug(ctx_, false);
}

/// @brief 析构函数：停止服务并释放 Modbus 上下文
ModbusTcpSlave::~ModbusTcpSlave()
{
  stop();
  if (ctx_)
  {
    modbus_free(ctx_);
    ctx_ = nullptr;
  }
}

/// @brief 启动 Modbus TCP 服务
/// 创建监听 socket 并启动服务器线程
void ModbusTcpSlave::start()
{
  if (!ctx_) return;
  run_flag_ = true;

  // 创建 TCP 监听
  server_fd_ = modbus_tcp_listen(ctx_, MODBUS_LISTEN_BACKLOG);
  if (server_fd_ == -1)
  {
    LOG_ERROR_CRITICAL(logger_, task_log::Tag::MODBUS, "Modbus listen failed: " << modbus_strerror(errno));
    run_flag_ = false;
    return;
  }

  // 启动服务器线程
  server_thread_ = std::thread(&ModbusTcpSlave::server_loop, this);
  LOG_INFO(logger_, task_log::Tag::MODBUS, "Modbus TCP Slave running on " << listen_ip_ << ":" << port_ << ", slave id=1");
}

/// @brief 停止 Modbus TCP 服务
/// 关闭 socket 并等待服务器线程退出
void ModbusTcpSlave::stop()
{
  run_flag_ = false;
  // The loop has a bounded poll timeout; it owns client sockets until it exits.
  if (server_thread_.joinable()) server_thread_.join();
  if (server_fd_ >= 0) {
    close(server_fd_);
    server_fd_ = -1;
  }

}

/// @brief 读取保持寄存器值（线程安全）
/// @param addr 寄存器地址
/// @return 寄存器当前值，未找到时返回 0
RegValue ModbusTcpSlave::get_reg(RegAddr addr)
{
  std::lock_guard<std::mutex> lock(reg_mutex_);
  auto iter = holding_regs_.find(addr);
  return iter != holding_regs_.end() ? iter->second : 0;
}

/// @brief 写入保持寄存器值（线程安全）
/// @param addr 寄存器地址
/// @param val 要写入的值
void ModbusTcpSlave::set_reg(RegAddr addr, RegValue val)
{
  std::lock_guard<std::mutex> lock(reg_mutex_);
  holding_regs_[addr] = val;
  LOG_DEBUG(logger_, task_log::Tag::MODBUS, "set_reg addr=" << addr << ", value=" << val);
}

/// @brief 注册寄存器写入回调
/// @param cb 回调函数
void ModbusTcpSlave::register_write_callback(WriteRegCb cb)
{
  write_cb_ = std::move(cb);
}

RegAddr ModbusTcpSlave::get_regaddr()
{
  return real_reg_addr_;
}

void ModbusTcpSlave::set_regaddr(RegAddr addr)
{
  real_reg_addr_ = addr;
}

/// @brief 服务器主循环
/// 接受客户端连接并处理 Modbus 请求
void ModbusTcpSlave::server_loop()
{
  struct Client {
    int fd;
    std::vector<uint8_t> input;
    std::chrono::steady_clock::time_point partial_since;
  };
  std::vector<Client> clients;
  // Never wait for a whole request on one client: fragmented/idle peers must not
  // prevent other clients from receiving their replies.
  fcntl(server_fd_, F_SETFL, fcntl(server_fd_, F_GETFL, 0) | O_NONBLOCK);
  while (run_flag_) {
    std::vector<pollfd> events{{server_fd_, POLLIN, 0}};
    for (const auto& client : clients) events.push_back({client.fd, POLLIN, 0});
    const int ready = poll(events.data(), events.size(), 20);
    if (ready < 0) {
      if (errno == EINTR) continue;
      break;
    }
    // Existing clients first; descriptors accepted below join the next poll.
    for (size_t i = clients.size(); i-- > 0;) {
      auto& client = clients[i];
      bool drop = events[i+1].revents & (POLLERR | POLLHUP | POLLNVAL);
      if (!drop && (events[i+1].revents & POLLIN)) {
        uint8_t data[MODBUS_TCP_MAX_ADU_LENGTH * 8];
        const auto n = recv(client.fd, data, sizeof(data), MSG_DONTWAIT);
        if (n > 0) {
          if (client.input.empty()) client.partial_since = std::chrono::steady_clock::now();
          client.input.insert(client.input.end(), data, data+n);
          if (client.input.size() > MODBUS_TCP_MAX_ADU_LENGTH * 16U) drop = true;
        } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
          drop = true;
        }
      }
      // Do not execute a queued write after its sender has already disconnected.
      uint8_t probe;
      if (!drop && recv(client.fd, &probe, 1, MSG_PEEK | MSG_DONTWAIT) == 0) drop = true;
      for (int budget = 0; !drop && budget < 8 && client.input.size() >= 7; ++budget) {
        const auto& b = client.input;
        const size_t length = (static_cast<size_t>(b[4]) << 8) | b[5];
        if (b[2] != 0 || b[3] != 0 || length < 2 || length + 6 > MODBUS_TCP_MAX_ADU_LENGTH) {
          drop = true;
          break;
        }
        const size_t total = length + 6;
        if (b.size() < total) break;
        modbus_set_socket(ctx_, client.fd);
        handle_modbus_request(client.input.data(), static_cast<int>(total));
        client.input.erase(client.input.begin(), client.input.begin()+total);
        client.partial_since = std::chrono::steady_clock::now();
      }
      if (!client.input.empty() &&
          std::chrono::steady_clock::now()-client.partial_since > std::chrono::seconds(1)) drop = true;
      if (drop) {
        close(client.fd);
        clients.erase(clients.begin()+i);
      }
    }
    if (events[0].revents & POLLIN) {
      const int fd = accept(server_fd_, nullptr, nullptr);
      if (fd >= 0) {
        if (clients.size() >= 64) close(fd);
        else {
          // A peer not reading replies also cannot block this loop indefinitely.
          timeval timeout{0, 100000};
          setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
          clients.push_back({fd, {}, std::chrono::steady_clock::now()});
        }
      }
    }
  }
  for (const auto& client : clients) close(client.fd);
  modbus_set_socket(ctx_, -1);

}

/// @brief 处理 Modbus 请求
/// 支持写单寄存器（0x06）和读保持寄存器（0x03）功能码
/// @param query 请求报文缓冲区
/// @param query_len 请求报文长度
void ModbusTcpSlave::handle_modbus_request(uint8_t* query, int query_len)
{
  // Modbus TCP 最小报文长度检查
  if (query_len < 8)
  {
    LOG_WARN(logger_, task_log::Tag::MODBUS, "Invalid modbus request length: " << query_len);
    return;
  }

  // 获取功能码（报文第 8 字节）
  uint8_t func = query[7];
  if ((func == MODBUS_FC_WRITE_SINGLE_REGISTER || func == MODBUS_FC_READ_HOLDING_REGISTERS) && query_len != 12) {
    modbus_reply_exception(ctx_, query, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
    return;
  }
  LOG_DEBUG(logger_, task_log::Tag::MODBUS, "Modbus request received, func=0x" << std::hex << (int)func << std::dec);

  // 创建 Modbus 映射结构用于响应
  modbus_mapping_t *mb_mapping = modbus_mapping_new(0, 0, MODBUS_MAX_REGISTERS, 0);
  if (!mb_mapping)
  {
    LOG_ERROR(logger_, task_log::Tag::MODBUS, "Failed to allocate modbus mapping");
    return;
  }

  int rc = -1;

  if (func == MODBUS_FC_WRITE_SINGLE_REGISTER)
  {
    // 写单寄存器（功能码 0x06）
    // 寄存器地址：报文第 9-10 字节
    uint16_t logic_addr = (query[8] << 8) | query[9];
    // 写入值：报文第 11-12 字节
    uint16_t write_val = (query[10] << 8) | query[11];

    // 逻辑地址转换为实际地址（Modbus 协议偏移）
    const RegAddr write_addr = logic_addr + MODBUS_PROTOCOL_ADDR_OFFSET;
    LOG_INFO(logger_, task_log::Tag::MODBUS,
      "Write single register: logic_addr=" << logic_addr << ", real_addr=" << write_addr << ", value=" << write_val);

    if (logic_addr >= MODBUS_MAX_REGISTERS) {
      modbus_reply_exception(ctx_, query, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
      modbus_mapping_free(mb_mapping);
      return;
    }

    // 获取旧值
    uint16_t old_val = get_reg(write_addr);

    // 写入寄存器并触发回调
    set_reg(write_addr, write_val);
    if (write_cb_)
      write_cb_(write_addr, old_val, write_val);

    LOG_INFO(logger_, task_log::Tag::MODBUS, "Write reg " << write_addr << " value: " << write_val << " (old: " << old_val << ")");

    // 发送响应
    rc = modbus_reply(ctx_, query, query_len, mb_mapping);
  }
  else if (func == MODBUS_FC_READ_HOLDING_REGISTERS)
  {
    // 读保持寄存器（功能码 0x03）
    // 起始地址：报文第 9-10 字节
    uint16_t start_logic = (query[8] << 8) | query[9];
    // 寄存器数量：报文第 11-12 字节
    uint16_t reg_count = (query[10] << 8) | query[11];

    // 检查寄存器范围合法性
    if (reg_count == 0 || reg_count > 125 || static_cast<uint32_t>(start_logic) + reg_count > MODBUS_MAX_REGISTERS)
    {
      LOG_WARN(logger_, task_log::Tag::MODBUS, "Invalid register range: start=" << start_logic << ", count=" << reg_count);
      modbus_reply_exception(ctx_, query, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
      modbus_mapping_free(mb_mapping);
      return;
    }

    // 填充保持寄存器值
    for (uint16_t i = 0; i < reg_count; i++)
    {
      RegAddr real_addr = start_logic + i + MODBUS_PROTOCOL_ADDR_OFFSET;
      uint16_t val = get_reg(real_addr);
      mb_mapping->tab_registers[start_logic + i] = val;
    }

    // 发送响应
    rc = modbus_reply(ctx_, query, query_len, mb_mapping);
  }
  else
  {
    // 不支持的功能码
    LOG_WARN(logger_, task_log::Tag::MODBUS, "Unsupported modbus function code: 0x" << std::hex << (int)func << std::dec);
    rc = modbus_reply_exception(ctx_, query, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
  }

  // 检查响应是否成功
  if (rc == -1)
  {
    LOG_ERROR(logger_, task_log::Tag::MODBUS, "Failed to send modbus reply: " << modbus_strerror(errno));
  }

  // 释放 Modbus 映射内存
  modbus_mapping_free(mb_mapping);
}

} // namespace task_modbus
