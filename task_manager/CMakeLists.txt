cmake_minimum_required(VERSION 3.8)
project(task_manager)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(com_interfaces REQUIRED)

# Discover both system installs and isolated offline dependency prefixes.
find_path(MODBUS_INCLUDE_DIR NAMES modbus/modbus.h)
find_library(MODBUS_LIB NAMES modbus)
if(NOT MODBUS_INCLUDE_DIR OR NOT MODBUS_LIB)
  message(FATAL_ERROR "libmodbus development files are required; install libmodbus-dev or set CMAKE_PREFIX_PATH")
endif()
include_directories(${MODBUS_INCLUDE_DIR})

# 项目头文件目录
include_directories(include)

# 源码列表
set(SRC_LIST
  src/fsm_state.cpp
  src/task_config_parser.cpp
  src/modbus_slave.cpp
  src/task_action_client.cpp
  src/behavior_tree_engine.cpp
  src/task_manager_node.cpp
)

add_executable(task_manager_node ${SRC_LIST})

# 链接库
target_link_libraries(task_manager_node
  yaml-cpp
  ${MODBUS_LIB}
  uuid
  pthread
)

# 添加所有必要的依赖
ament_target_dependencies(task_manager_node
  rclcpp 
  rclcpp_action 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  com_interfaces
)

# 安装规则
install(TARGETS
  task_manager_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
