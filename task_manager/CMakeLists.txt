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

# libmodbus /usr/include/modbus 适配
include_directories("/usr/include")
set(MODBUS_LIB modbus)

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

  # ----- test_task_config_parser：验证主级+次级配置分级加载 -----
  ament_add_gtest(test_task_config_parser
    test/test_task_config_parser.cpp
    src/task_config_parser.cpp
  )
  target_include_directories(test_task_config_parser PRIVATE include)
  target_compile_definitions(test_task_config_parser PRIVATE SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_task_config_parser yaml-cpp)
  ament_target_dependencies(test_task_config_parser
    rclcpp
    com_interfaces
  )

endif()

ament_package()
