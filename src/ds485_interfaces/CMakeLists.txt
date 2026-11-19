cmake_minimum_required(VERSION 3.8)
project(ds485_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # 夹爪（外部模块通过话题使用）
  "msg/GripperCommand.msg"
  "msg/GripperStop.msg"
  "msg/GripperResult.msg"
  "msg/ServoState.msg"
  "msg/GripperHealth.msg"
  # 舵机直接控制（调试服务）
  "srv/ServoReadRegister.srv"
  "srv/ServoWriteRegister.srv"
  "srv/ServoMoveAngle.srv"
  "srv/ServoSetSwitch.srv"
  "srv/ServoSetTorque.srv"
  "srv/ServoSetLimits.srv"
  "srv/ServoGetInfo.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
