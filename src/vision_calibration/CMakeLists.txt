cmake_minimum_required(VERSION 3.8)
project(vision_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(com_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(calibration_node
  src/calibration_node.cpp
  src/handeye_calibrator.cpp
  src/main.cpp
)

ament_target_dependencies(calibration_node
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  com_interfaces
  cv_bridge
)

target_include_directories(calibration_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(calibration_node ${OpenCV_LIBS})

install(TARGETS calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp rclcpp_action)
ament_package()
