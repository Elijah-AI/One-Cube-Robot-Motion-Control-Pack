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

# 源码列表（按功能模块拆分，每文件尽量 ~800 行）
set(SRC_LIST
  src/fsm_state.cpp
  src/task_config_parser.cpp
  src/modbus_slave.cpp
  # TaskActionClient 按功能模块拆分
  src/task_action_client.cpp
  src/task_action_client_vision.cpp
  src/task_action_client_arm.cpp
  src/task_action_client_gripper.cpp
  src/task_action_client_selfcheck.cpp
  src/task_action_client_calibration.cpp
  src/task_action_client_compound.cpp
  # 行为树拆分为节点实现 + 引擎
  src/behavior_tree_nodes.cpp
  src/behavior_tree_engine.cpp
  src/precondition_manager.cpp
  src/task_manager_node.cpp
  src/task_manager_legacy_safety.cpp
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


if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME precision_arrival_contract COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/run_precision_arrival_contract.py
    ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  set_tests_properties(precision_arrival_contract PROPERTIES TIMEOUT 120)
  add_executable(test_modbus_trigger_gate test/test_modbus_trigger_gate.cpp)
  target_link_libraries(test_modbus_trigger_gate pthread)
  add_test(NAME modbus_trigger_gate COMMAND test_modbus_trigger_gate)
  add_executable(modbus_test_server test/modbus_test_server.cpp src/modbus_slave.cpp)
  target_link_libraries(modbus_test_server ${MODBUS_LIB} pthread)
  ament_target_dependencies(modbus_test_server rclcpp)
  add_test(NAME modbus_tcp_multi_client COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_modbus_tcp.py $<TARGET_FILE:modbus_test_server>)
  set_tests_properties(modbus_tcp_multi_client PROPERTIES TIMEOUT 20)
  add_test(NAME leg_body_motion_contract COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/run_leg_body_motion_contract.py
    ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  set_tests_properties(leg_body_motion_contract PROPERTIES TIMEOUT 120)
  add_test(NAME ik_leg_recovery COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/run_ik_leg_recovery.py
    ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  set_tests_properties(ik_leg_recovery PROPERTIES TIMEOUT 180)
  add_test(NAME waist_leg_parallel_task COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/run_waist_leg_task.py
    ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  set_tests_properties(waist_leg_parallel_task PROPERTIES TIMEOUT 120)

  add_executable(test_vision_half_turn
    test/test_vision_half_turn.cpp src/task_config_parser.cpp)
  target_compile_features(test_vision_half_turn PRIVATE cxx_std_17)
  target_link_libraries(test_vision_half_turn yaml-cpp)
  ament_target_dependencies(test_vision_half_turn
    rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs com_interfaces)
  add_test(NAME vision_half_turn COMMAND test_vision_half_turn
    "${CMAKE_CURRENT_SOURCE_DIR}/config/tasks/laptop_assembly_two_piece.yaml"
    "${CMAKE_CURRENT_BINARY_DIR}/half_turn_parser_case.yaml")
  add_executable(test_placement_vertical_down
    test/test_placement_vertical_down.cpp src/task_config_parser.cpp)
  target_compile_features(test_placement_vertical_down PRIVATE cxx_std_17)
  target_link_libraries(test_placement_vertical_down yaml-cpp)
  ament_target_dependencies(test_placement_vertical_down
    rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs com_interfaces)
  add_test(NAME placement_vertical_down COMMAND test_placement_vertical_down
    "${CMAKE_CURRENT_SOURCE_DIR}/config/tasks/laptop_assembly_two_piece.yaml"
    "${CMAKE_CURRENT_BINARY_DIR}/vertical_down_parser_case.yaml")
endif()

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME rigid_leg_contract COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/run_rigid_leg_contract.py
    ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  set_tests_properties(rigid_leg_contract PROPERTIES TIMEOUT 120)
endif()

ament_package()
