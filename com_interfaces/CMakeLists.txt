cmake_minimum_required(VERSION 3.8)
project(com_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 接口生成依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 生成srv接口代码
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetSystemStatus.srv"
  "msg/HeartBeat.msg"
  "msg/NodeStatus.msg"
  "msg/MotorStatus.msg"
  "msg/MotorInfo.msg"
  "msg/CameraDriver.msg"
  "msg/SyncedRobotVision.msg"
  "action/ControlGripper.action"
  "action/ControlSuck.action"
  "action/MovePose.action"
  "action/HardwareCheck.action"
  "action/HardwareReset.action"
  "action/VisionDetection.action"
  "action/VisionCheckSelf.action"
  "action/PickAndPlace.action"
  "action/AdjustBasePose.action"
  "action/MoveJoints.action"
  "action/AddTask.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# 导出依赖供其他包调用
ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
)

# 安装规则
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
