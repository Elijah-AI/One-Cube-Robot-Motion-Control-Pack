cmake_minimum_required(VERSION 3.8)
project(task_manager)

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
find_package(vision_detection_action REQUIRED) 
find_package(motion_control_pkg REQUIRED) 
# libmodbus /usr/include/modbus 适配
include_directories("/usr/include")
set(MODBUS_LIB modbus)

# 自定义接口
set(msg_files
  "msg/Heartbeat.msg"
  "msg/NodeStatus.msg"
)
set(srv_files
  "srv/GetSystemStatus.srv"
)
set(action_files
  "action/VisionDetection.action"
  "action/MoveArm.action"
  "action/ControlGripper.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

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
)

ament_target_dependencies(task_manager_node
  rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs vision_detection_action motion_control_pkg
)
rosidl_target_interfaces(task_manager_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

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