cmake_minimum_required(VERSION 3.8)
project(vision_detection_pose_cloud)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(com_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Native ARM64 builds regenerate the NPU library from the matching API/source.
# The existing binary remains available for object-only validation on x86_64.
set(_npu_source_default OFF)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(_npu_source_default ON)
endif()
option(VISION_BUILD_NPU_FROM_SOURCE "Build Rockchip NPU library from source" ${_npu_source_default})
if(VISION_BUILD_NPU_FROM_SOURCE)
  add_subdirectory(third_party/sort_detection_npu sort_detection_npu)
  set(VISION_NPU_LIBRARY sort_detection)
  set(VISION_NPU_BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}/sort_detection_npu/lib")
  set_target_properties(sort_detection PROPERTIES INSTALL_RPATH "$ORIGIN;$ORIGIN/..")
else()
  set(VISION_NPU_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/third_party/sort_detection_npu/lib/libsort_detection.so")
  set(VISION_NPU_BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/sort_detection_npu/lib")
endif()

add_executable(detection_pose_cloud_server
  src/detection_pose_cloud_server.cpp
  src/vision_data_subscriber.cpp
  src/main.cpp
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(detection_pose_cloud_server PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(detection_pose_cloud_server
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  com_interfaces
  cv_bridge
  pcl_conversions
)

target_include_directories(detection_pose_cloud_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  # NPU 检测库头文件: sort_dectect.h (API) + 内部头
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/sort_detection_npu/API
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/sort_detection_npu/include
)

target_link_libraries(detection_pose_cloud_server
  ${PCL_LIBRARIES}
  # NPU 检测库 + RKNN 运行时 + glog (libsort_detection 内部用 LOG())
  ${VISION_NPU_LIBRARY}
  rknnrt
  glog
)

# Install the rebuilt library, preserving SONAME links.
if(VISION_BUILD_NPU_FROM_SOURCE)
  install(TARGETS sort_detection LIBRARY DESTINATION lib/${PROJECT_NAME})
else()
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/sort_detection_npu/lib/libsort_detection.so
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/sort_detection_npu/lib/libsort_detection.so.1
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/sort_detection_npu/lib/libsort_detection.so.1.1.0.1
  DESTINATION lib/${PROJECT_NAME}
)

endif()
install(DIRECTORY third_party/sort_detection_npu/models/
  DESTINATION share/${PROJECT_NAME}/models
)

# 设置 RPATH：运行时在可执行文件同级目录找 libsort_detection.so
set_target_properties(detection_pose_cloud_server PROPERTIES
  BUILD_RPATH "${VISION_NPU_BUILD_RPATH}"
  INSTALL_RPATH "$ORIGIN;$ORIGIN/.."
)

install(TARGETS detection_pose_cloud_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp rclcpp_action)
ament_package()
