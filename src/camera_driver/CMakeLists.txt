cmake_minimum_required(VERSION 3.8)
project(camera_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wno-pedantic -Wno-array-bounds)
endif()

# find dependencies
set(dependencies
    rclcpp
    rclcpp_components
    rclcpp_action
    sensor_msgs
    std_msgs
    com_interfaces
)

foreach(dep IN LISTS dependencies)
    find_package(${dep} REQUIRED)
endforeach()

find_package(Threads REQUIRED)

# Detect platform for Orbbec SDK
execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT)
message(STATUS "camera_driver Machine : ${MACHINES}")
message(STATUS "camera_driver Machine Bits : ${MACHINES_BIT}")

if((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64"))
    set(HOST_PLATFORM "x64")
elseif((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
    set(HOST_PLATFORM "arm64")
else()
    message(FATAL_ERROR "Unsupported platform: ${MACHINES} ${MACHINES_BIT}")
endif()

# Orbbec SDK paths (local copy within camera_driver package)
set(ORBBEC_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/SDK")
set(ORBBEC_SDK_INCLUDE_DIR "${ORBBEC_SDK_DIR}/include")
set(ORBBEC_SDK_LIBS_DIR "${ORBBEC_SDK_DIR}/lib/${HOST_PLATFORM}")

message(STATUS "Orbbec SDK include: ${ORBBEC_SDK_INCLUDE_DIR}")
message(STATUS "Orbbec SDK libs: ${ORBBEC_SDK_LIBS_DIR}")

set(CMAKE_BUILD_RPATH "${CMAKE_BUILD_RPATH}:${ORBBEC_SDK_LIBS_DIR}")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${ORBBEC_SDK_LIBS_DIR}")

set(COMMON_INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ORBBEC_SDK_INCLUDE_DIR}
)

set(COMMON_LIBRARIES
    Threads::Threads
    -L${ORBBEC_SDK_LIBS_DIR}
    -lOrbbecSDK
    -lrt
)

# Main camera driver node
add_executable(camera_driver_node
    src/camera_driver_node.cpp
)

target_include_directories(camera_driver_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(camera_driver_node ${COMMON_LIBRARIES})
ament_target_dependencies(camera_driver_node ${dependencies})

# Install rules
install(TARGETS camera_driver_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install Orbbec SDK shared libraries
install(DIRECTORY ${ORBBEC_SDK_LIBS_DIR}/ DESTINATION lib/
        FILES_MATCHING PATTERN "*.so*")

# Install extensions directory
file(GLOB_RECURSE EXTENSION_FILES "${ORBBEC_SDK_LIBS_DIR}/extensions/*")
if(EXTENSION_FILES)
    install(FILES ${EXTENSION_FILES} DESTINATION lib/extensions/)
endif()

ament_package()
