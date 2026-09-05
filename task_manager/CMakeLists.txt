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

# 单元测试依赖
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

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

# ============================================================================
# 单元测试
# ============================================================================
if(BUILD_TESTING)
  # ----- test_fsm_state -----
  ament_add_gtest(test_fsm_state
    test/test_fsm_state.cpp
    src/fsm_state.cpp
  )
  target_include_directories(test_fsm_state PRIVATE include)
  ament_target_dependencies(test_fsm_state
    rclcpp
    com_interfaces
  )

  # ----- test_prioritized_task -----
  ament_add_gtest(test_prioritized_task
    test/test_prioritized_task.cpp
  )
  target_include_directories(test_prioritized_task PRIVATE include ${CMAKE_CURRENT_BINARY_DIR})
  ament_target_dependencies(test_prioritized_task
    rclcpp
    rclcpp_action
    std_msgs
    com_interfaces
  )

  # ----- test_add_task_action -----
  ament_add_gtest(test_add_task_action
    test/test_add_task_action.cpp
  )
  target_include_directories(test_add_task_action PRIVATE include ${CMAKE_CURRENT_BINARY_DIR})
  ament_target_dependencies(test_add_task_action
    rclcpp
    rclcpp_action
    std_msgs
    com_interfaces
  )

  # ----- test_task_config_parser -----
  ament_add_gtest(test_task_config_parser
    test/test_task_config_parser.cpp
    src/task_config_parser.cpp
  )
  target_include_directories(test_task_config_parser PRIVATE include)
  target_compile_definitions(test_task_config_parser PRIVATE
    TASK_MANAGER_LOCAL_PARALLEL_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/local_parallel_task_config.yaml"
  )
  target_link_libraries(test_task_config_parser yaml-cpp)
  ament_target_dependencies(test_task_config_parser rclcpp)

  # ----- test_behavior_tree_engine -----
  ament_add_gtest(test_behavior_tree_engine
    test/test_behavior_tree_engine.cpp
    src/behavior_tree_engine.cpp
    src/task_action_client.cpp
  )
  target_include_directories(test_behavior_tree_engine PRIVATE include ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_behavior_tree_engine uuid)
  ament_target_dependencies(test_behavior_tree_engine
    rclcpp
    rclcpp_action
    std_msgs
    geometry_msgs
    sensor_msgs
    com_interfaces
  )
endif()

ament_package()
