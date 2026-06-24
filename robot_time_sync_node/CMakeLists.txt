cmake_minimum_required(VERSION 3.8)
project(robot_time_sync_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 编译自定义msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SyncedRobotVision.msg"
  "msg/NodeStatus.msg"
  "msg/HeartBeat.msg"
  "msg/MotorInfo.msg"
  "msg/CameraRawData.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs builtin_interfaces
)

# 生成可执行文件
add_executable(time_sync_node src/time_sync.cpp)
ament_target_dependencies(time_sync_node
  rclcpp std_msgs sensor_msgs geometry_msgs builtin_interfaces
)
rosidl_target_interfaces(time_sync_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 安装可执行文件
install(TARGETS
  time_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
