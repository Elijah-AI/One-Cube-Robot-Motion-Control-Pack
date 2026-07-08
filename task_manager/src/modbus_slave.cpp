#include "task_manager/modbus_slave.hpp"
#include <cstring>
#include <unistd.h>
#include <modbus/modbus.h>
#include <thread>
#include <chrono>

namespace task_modbus
{
ModbusTcpSlave::ModbusTcpSlave(const std::string& ip, uint16_t port, rclcpp::Logger logger)
: listen_ip_(ip), port_(port), logger_(logger), server_fd_(-1), run_flag_(false)
{
  ctx_ = modbus_new_tcp(listen_ip_.c_str(), port_);
  if (!ctx_)
  {
    RCLCPP_ERROR(logger_, "Create modbus ctx failed: %s", modbus_strerror(errno));
    return;
  }
  modbus_set_slave(ctx_, 1);
  modbus_set_debug(ctx_, false);
}

ModbusTcpSlave::~ModbusTcpSlave()
{
  stop();
  if (ctx_)
  {
    modbus_free(ctx_);
    ctx_ = nullptr;
  }
}

void ModbusTcpSlave::start()
{
  if (!ctx_) return;
  run_flag_ = true;
  server_fd_ = modbus_tcp_listen(ctx_, 5);
  if (server_fd_ == -1)
  {
    RCLCPP_ERROR(logger_, "Modbus listen fail: %s", modbus_strerror(errno));
    run_flag_ = false;
    return;
  }
  server_thread_ = std::thread(&ModbusTcpSlave::server_loop, this);
  RCLCPP_INFO(logger_, "Modbus TCP Slave running %s:%d, slave id=1", listen_ip_.c_str(), port_);
}

void ModbusTcpSlave::stop()
{
  run_flag_ = false;
  if (server_fd_ > 0)
  {
    close(server_fd_);
    server_fd_ = -1;
  }
  if (server_thread_.joinable())
    server_thread_.join();
}

RegValue ModbusTcpSlave::get_reg(RegAddr addr)
{
  std::lock_guard<std::mutex> lock(reg_mutex_);
  auto iter = holding_regs_.find(addr);
  return iter != holding_regs_.end() ? iter->second : 0;
}

void ModbusTcpSlave::set_reg(RegAddr addr, RegValue val)
{
  std::lock_guard<std::mutex> lock(reg_mutex_);
  
  holding_regs_[addr] = val;
  RCLCPP_INFO(logger_, "----------set_reg addr: %ld, value: %ld--------------", addr, val);
}

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

void ModbusTcpSlave::server_loop()
{
  uint8_t query_buf[MODBUS_TCP_MAX_ADU_LENGTH];
  int recv_len;
  
  while (run_flag_)
  {
    // 接受客户端连接
    int client_fd = modbus_tcp_accept(ctx_, &server_fd_);
    
    if (client_fd <= 0)
    {
      // 如果没有客户端连接，等待一段时间后继续
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    
    // 设置客户端socket
    modbus_set_socket(ctx_, client_fd);
    
    // 处理客户端请求
    while (run_flag_)
    {
      recv_len = modbus_receive(ctx_, query_buf);
      if (recv_len > 0)
      {
        handle_modbus_request(query_buf, recv_len);
      }
      else if (recv_len == -1)
      {
        // 接收错误或连接关闭
        break;
      }
    }
    
    // 关闭客户端连接
    close(client_fd);
    modbus_set_socket(ctx_, -1);
  }
}

void ModbusTcpSlave::handle_modbus_request(uint8_t* query, int query_len)
{
  if (query_len < 12)  // Modbus TCP最小报文长度检查
  {
    RCLCPP_WARN(logger_, "Invalid modbus request length: %d", query_len);
    return;
  }
  
  // 正确获取功能码：从数组索引7获取
  uint8_t func = query[7];
  RCLCPP_INFO(logger_, "func: %d", func);
  // 创建 modbus_mapping_t 结构体用于存储响应数据
  // 参数分别表示：线圈数量、离散输入数量、保持寄存器数量、输入寄存器数量
  modbus_mapping_t *mb_mapping = modbus_mapping_new(0, 0, 65535, 0);
  if (!mb_mapping)
  {
    RCLCPP_ERROR(logger_, "Failed to allocate modbus mapping");
    return;
  }
  RCLCPP_INFO(logger_, "----------------");
  int rc = -1;
  if (func == MODBUS_FC_WRITE_SINGLE_REGISTER)
  {
    // 正确解析寄存器地址：从数组索引8-9获取
    uint16_t logic_addr = (query[8] << 8) | query[9];
    RCLCPP_INFO(logger_, "logic_addr = %d", logic_addr);
    // 正确解析要写入的寄存器值：从数组索引10-11获取
    uint16_t write_val = (query[10] << 8) | query[11];
    RCLCPP_INFO(logger_, "write_val = %d", write_val);
    // 将逻辑地址转换为实际寄存器地址
    real_reg_addr_ = logic_addr + 40001;
    RCLCPP_INFO(logger_, "set_reg %d %d", real_reg_addr_, write_val);


    // 写入寄存器
    set_reg(real_reg_addr_, write_val);
    if (write_cb_)
    write_cb_(real_reg_addr_, 0, write_val);

    set_reg(real_reg_addr_, 0);
    RCLCPP_INFO(logger_, "Write reg %u value: %u", real_reg_addr_, write_val);
    
    // 对于写操作，直接使用 modbus_reply 回复
    rc = modbus_reply(ctx_, query, query_len, mb_mapping);
    
  }
  else if (func == MODBUS_FC_READ_HOLDING_REGISTERS)
  {
    // 正确解析起始寄存器地址：从数组索引8-9获取
    uint16_t start_logic = (query[8] << 8) | query[9];
    
    // 正确解析要读取的寄存器数量：从数组索引10-11获取
    uint16_t reg_count = (query[10] << 8) | query[11];
    
    // 检查寄存器数量是否合法
    if (start_logic + reg_count > 65535)
    {
      RCLCPP_WARN(logger_, "Invalid register range: start=%u, count=%u", start_logic, reg_count);
      modbus_mapping_free(mb_mapping);
      return;
    }
    
    // 将寄存器值填充到 mb_mapping 的保持寄存器数组中
    for (uint16_t i = 0; i < reg_count; i++)
    {
      RegAddr real_addr = start_logic + i + 40001;
      uint16_t val = get_reg(real_addr);
      mb_mapping->tab_registers[start_logic + i] = val;
    }
    
    // 使用 modbus_reply 发送响应
    rc = modbus_reply(ctx_, query, query_len, mb_mapping);
  }
  else
  {
    RCLCPP_WARN(logger_, "Unsupported modbus function code: 0x%02X", func);
    // 对于不支持的功能码，发送异常响应
    rc = modbus_reply_exception(ctx_, query, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
  }
  
  // 检查响应是否成功
  if (rc == -1)
  {
    RCLCPP_ERROR(logger_, "Failed to send modbus reply: %s", modbus_strerror(errno));
  }
  
  // 释放内存
  modbus_mapping_free(mb_mapping);
}


} // namespace task_modbus
