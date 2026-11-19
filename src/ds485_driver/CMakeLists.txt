cmake_minimum_required(VERSION 3.8)
project(ds485_driver)

# 自制夹爪（德晟 485 舵机）驱动。分层：
#   ds485_rs485   第 1 层 通用 485 收发（Marvin 机械臂透传 / USB-485 串口）
#   ds485_core    第 2、3 层 舵机协议 + 夹爪（不依赖 SDK、ROS）
#   ds485_node    第 4 层 ROS2 节点

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ds485_interfaces REQUIRED)

# ---------------- MarvinSDK（只用末端透传相关接口）----------------
# libMarvinSDK.so 没有 SONAME，按 -L/-l 方式链接，运行时从可执行文件同目录加载（RPATH=$ORIGIN）
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(MARVIN_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/marvin_sdk/lib/aarch64)
else()
  set(MARVIN_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/marvin_sdk/lib/x64)
endif()
message(STATUS "ds485_driver: MarvinSDK 库目录 ${MARVIN_LIB_DIR}")

add_library(marvin_sdk SHARED IMPORTED)
set_target_properties(marvin_sdk PROPERTIES
  IMPORTED_LOCATION "${MARVIN_LIB_DIR}/libMarvinSDK.so"
  IMPORTED_NO_SONAME TRUE
)

# ---------------- 第 2、3 层：舵机协议 + 夹爪（纯 C++）----------------
set(ds485_core_sources
  src/servo/ds485_protocol.cpp
  src/servo/serial_transaction.cpp
  src/servo/ds485_protection.cpp
  src/servo/ds485_servo.cpp
  src/gripper/trajectory_controller.cpp
  src/rs485/bus_worker.cpp
  src/gripper/ds485_gripper.cpp
)
add_library(ds485_core STATIC ${ds485_core_sources})
target_include_directories(ds485_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(ds485_core PUBLIC Threads::Threads)
set_target_properties(ds485_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------- 第 1 层：485 收发 ----------------
add_library(ds485_rs485 STATIC
  src/rs485/marvin_rs485_port.cpp
  src/rs485/serial_rs485_port.cpp
)
target_include_directories(ds485_rs485 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(ds485_rs485 PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/marvin_sdk/include
)
target_link_libraries(ds485_rs485 PUBLIC marvin_sdk pthread rt)
set_target_properties(ds485_rs485 PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------- 第 4 层：ROS2 节点 ----------------
set(ds485_node_sources src/node/ds485_node.cpp src/node/ds485_setup.cpp
  src/node/ds485_interfaces.cpp src/node/ds485_torque.cpp)
add_library(ds485_control STATIC ${ds485_core_sources} ${ds485_node_sources}
  src/node/embedded_torque.cpp src/gripper/gripper_controller.cpp src/gripper/gripper_config.cpp
  src/rs485/serial_rs485_port.cpp src/rs485/marvin_rs485_port.cpp)
target_include_directories(ds485_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  PRIVATE third_party/marvin_sdk/include)
target_link_libraries(ds485_control PUBLIC Threads::Threads yaml-cpp)
ament_target_dependencies(ds485_control PUBLIC rclcpp std_srvs ds485_interfaces rcutils)
set_target_properties(ds485_control PROPERTIES POSITION_INDEPENDENT_CODE ON)
install(TARGETS ds485_control EXPORT export_ds485_control ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_ds485_control HAS_LIBRARY_TARGET)
ament_export_dependencies(Threads yaml-cpp rclcpp std_srvs ds485_interfaces rcutils)

add_executable(ds485_node src/node/main.cpp ${ds485_node_sources})
target_link_libraries(ds485_node ds485_core ds485_rs485)
ament_target_dependencies(ds485_node rclcpp ds485_interfaces std_srvs rcutils)
target_link_options(ds485_node PRIVATE "-Wl,--disable-new-dtags")
set_target_properties(ds485_node PROPERTIES INSTALL_RPATH "\$ORIGIN")

# ---------------- 安装 ----------------
install(TARGETS ds485_node DESTINATION lib/${PROJECT_NAME})
install(FILES ${MARVIN_LIB_DIR}/libMarvinSDK.so DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/gripper scripts/servo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch doc DESTINATION share/${PROJECT_NAME})

# ---------------- 离线测试（不需要机械臂和舵机）----------------
if(BUILD_TESTING)
  add_executable(ds485_offline_test
    test/test_bus_worker.cpp
    test/test_trajectory.cpp
    test/test_transaction.cpp
    test/test_ds485.cpp
    test/fake_servo_port.cpp
  )
  target_include_directories(ds485_offline_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(ds485_offline_test ds485_core)
  target_compile_options(ds485_offline_test PRIVATE -UNDEBUG)
  add_test(NAME ds485_offline_test COMMAND ds485_offline_test)
  add_executable(test_gripper_controller test/test_gripper_controller.cpp
    test/fake_servo_port.cpp test/marvin_link_spy.cpp)
  target_include_directories(test_gripper_controller PRIVATE test)
  target_link_libraries(test_gripper_controller ds485_control)
  add_test(NAME ds485_controller COMMAND test_gripper_controller)
  add_executable(test_gripper_config test/test_gripper_config.cpp)
  target_link_libraries(test_gripper_config ds485_control)
  add_test(NAME ds485_config COMMAND test_gripper_config
    ${CMAKE_CURRENT_SOURCE_DIR}/config/ds485_driver.yaml 4800)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME ds485_ros_pty COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros_pty.py $<TARGET_FILE:ds485_node>)
  add_test(NAME ds485_torque_pty COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_torque_pty.py $<TARGET_FILE:ds485_node>)
  set_tests_properties(ds485_ros_pty ds485_torque_pty PROPERTIES TIMEOUT 60)
endif()

ament_package()
