cmake_minimum_required(VERSION 3.8)
project(safety_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(com_interfaces REQUIRED)

include_directories(include)

add_executable(safety_monitor_node
  src/safety_monitor_node.cpp
)

ament_target_dependencies(safety_monitor_node
  rclcpp
  std_msgs
  com_interfaces
)

install(TARGETS
  safety_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()