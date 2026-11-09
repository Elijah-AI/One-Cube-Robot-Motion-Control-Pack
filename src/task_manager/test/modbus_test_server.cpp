#include "task_manager/modbus_slave.hpp"
#include <csignal>
#include <iostream>
#include <thread>
volatile std::sig_atomic_t done=0;
void stop_test(int){done=1;}
int main(int argc,char**argv) {
  if(argc!=2)return 2;
  const int port=std::stoi(argv[1]);
  rclcpp::init(argc,argv);
  std::signal(SIGINT,stop_test);std::signal(SIGTERM,stop_test);
  task_modbus::ModbusTcpSlave server("127.0.0.1",port,rclcpp::get_logger("modbus_test"));
  server.set_regaddr(40010);
  server.set_reg(50010,42);
  server.register_write_callback([&](uint32_t,uint16_t,uint16_t){
    server.set_reg(50011,server.get_reg(50011)+1);
    server.set_reg(50012,server.get_regaddr()==40010?1:0);
  });
  server.start();
  std::cout<<"READY"<<std::endl;
  while(!done)std::this_thread::sleep_for(std::chrono::milliseconds(10));
  server.stop();rclcpp::shutdown();
}
