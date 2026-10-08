cmake_minimum_required(VERSION 3.8)
project(berxel_camera_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_FLAGS "-O2 -fPIE -fPIC -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(com_interfaces REQUIRED)
find_package(Threads REQUIRED)

# -----------------------------------------------------------------------------
# Berxel Hawk SDK discovery
#
# Priority:
#   1) -DBERXEL_SDK_DIR=<path to SDK root>     (explicit override, recommended
#      when a different/upgraded SDK must be used)
#   2) $BERXEL_SDK_LIBRARY env var             (exported by <SDK>/berxel_set_env.sh)
#   3) bundled local SDK copy                  (default, self-contained like
#      camera_driver/SDK): berxel_camera_driver/SDK/{Include,x86,arm}
#
# The bundled SDK keeps the shared C++ headers in "Include/" and splits the
# native libraries into two top-level folders so the SAME source tree builds
# natively on an x86_64 WSL host AND on an aarch64 RK3576 target without
# cross-compiling:
#
#   berxel_camera_driver/SDK/Include/       C++ headers (shared, 5 .h)
#   berxel_camera_driver/SDK/x86/           x86_64 .so + params*.bin
#   berxel_camera_driver/SDK/arm/           aarch64 .so + params*.bin
#
# Each arch folder holds libBerxelHawk.so, its companion shared libraries
# (dlopen'ed at runtime: libBerxelInterface.so / libBerxelLogDriver.so / ...)
# and the depth-calibration params*.bin files used by the UVC/Net drivers.
# An external SDK root passed via -DBERXEL_SDK_DIR may use either the same
# "x86|arm" layout or the original flat "libs/" layout (auto-detected below).
# -----------------------------------------------------------------------------
if(NOT DEFINED BERXEL_SDK_DIR)
  if(DEFINED ENV{BERXEL_SDK_LIBRARY})
    set(BERXEL_SDK_DIR "$ENV{BERXEL_SDK_LIBRARY}")
  else()
    set(BERXEL_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/SDK")
  endif()
endif()

# --- target architecture selection: "x86" (x86_64) vs "arm" (aarch64) ------
# CMAKE_SYSTEM_PROCESSOR is authoritative (pre-filled by CMake on a native build
# and set by the toolchain file when cross-compiling); fall back to the
# camera_driver-style `uname -m` + `getconf LONG_BIT` detection.
set(_BERXEL_ARCH "")
if(CMAKE_SYSTEM_PROCESSOR)
  string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _BERXEL_PROC)
  if(_BERXEL_PROC MATCHES "aarch64|arm64|armv8")
    set(_BERXEL_ARCH "arm")
  elseif(_BERXEL_PROC MATCHES "x86_64|amd64|x64")
    set(_BERXEL_ARCH "x86")
  endif()
endif()
if(NOT _BERXEL_ARCH)
  execute_process(COMMAND uname -m OUTPUT_VARIABLE _BERXEL_MACH OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE _BERXEL_BIT OUTPUT_STRIP_TRAILING_WHITESPACE)
  if((_BERXEL_MACH MATCHES "x86_64") AND (_BERXEL_BIT MATCHES "64"))
    set(_BERXEL_ARCH "x86")
  elseif((_BERXEL_MACH MATCHES "aarch64|arm64") AND (_BERXEL_BIT MATCHES "64"))
    set(_BERXEL_ARCH "arm")
  endif()
endif()
if(NOT _BERXEL_ARCH)
  message(FATAL_ERROR
    "Could not determine the target CPU architecture for the Berxel SDK "
    "(CMAKE_SYSTEM_PROCESSOR='${CMAKE_SYSTEM_PROCESSOR}', uname='${_BERXEL_MACH}'). "
    "Supported: x86_64 -> x86, aarch64 -> arm.")
endif()

message(STATUS "Berxel SDK dir  : ${BERXEL_SDK_DIR}")
message(STATUS "Berxel SDK arch : ${_BERXEL_ARCH}")
set(BERXEL_SDK_INCLUDE_DIR "${BERXEL_SDK_DIR}/Include")

# Pick the library folder: prefer the bundled "<sdk>/<arch>" folder, fall back
# to the original flat "<sdk>/libs" folder of an external SDK root.
set(BERXEL_SDK_ARCH_LIB_DIR "${BERXEL_SDK_DIR}/${_BERXEL_ARCH}")
set(BERXEL_SDK_LIB_DIR "")
if(EXISTS "${BERXEL_SDK_ARCH_LIB_DIR}")
  set(BERXEL_SDK_LIB_DIR "${BERXEL_SDK_ARCH_LIB_DIR}")
elseif(EXISTS "${BERXEL_SDK_DIR}/libs")
  set(BERXEL_SDK_LIB_DIR "${BERXEL_SDK_DIR}/libs")
endif()

if(NOT EXISTS "${BERXEL_SDK_INCLUDE_DIR}" OR NOT EXISTS "${BERXEL_SDK_LIB_DIR}")
  message(FATAL_ERROR
    "Berxel Hawk SDK not found at '${BERXEL_SDK_DIR}' for arch '${_BERXEL_ARCH}'.\n"
    "Expected '<sdk>/Include' and '<sdk>/${_BERXEL_ARCH}' (bundled layout: "
    "<pkg>/SDK/{x86,arm}) or a flat '<sdk>/libs' folder, or pass "
    "-DBERXEL_SDK_DIR=<sdk-root> / export BERXEL_SDK_LIBRARY (source berxel_set_env.sh).")
endif()

include_directories(
  include
  ${BERXEL_SDK_INCLUDE_DIR}
)
link_directories(${BERXEL_SDK_LIB_DIR})

add_executable(camera_driver_node
  src/camera_driver_node.cpp
)

# Runtime library resolution
# ---------------------------
# The installed SDK shared libraries are placed in <prefix>/lib (the ament lib
# directory) which setup.bash automatically appends to LD_LIBRARY_PATH.  The SDK
# loads its companion drivers (libBerxelInterface.so, libBerxelLogDriver.so,
# libBerxelUvcDriver.so, ...) with dlopen("...so") at runtime, so they must be
# reachable through LD_LIBRARY_PATH.  A "$ORIGIN/.." rpath is added as a fallback
# so the direct NEEDED dependency libBerxelHawk.so is also found when the node is
# executed without sourcing the install environment.
set_target_properties(camera_driver_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
  BUILD_RPATH_USE_ORIGIN TRUE
)

target_link_libraries(camera_driver_node BerxelHawk Threads::Threads)

ament_target_dependencies(camera_driver_node
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  geometry_msgs
  com_interfaces
)

install(TARGETS camera_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Ship the whole local SDK with the package so the installed node is
# self-contained (mirrors how camera_driver bundles SDK/).
#   * shared libraries   -> <prefix>/lib   (ament LD_LIBRARY_PATH + $ORIGIN/..)
#   * params*.bin        -> <prefix>/lib   (read by the UVC/Net device drivers)
#   * headers            -> <prefix>/include/<pkg>
file(GLOB BERXEL_SDK_SHARED_LIBS ${BERXEL_SDK_LIB_DIR}/*.so)
file(GLOB BERXEL_SDK_PARAM_BINS ${BERXEL_SDK_LIB_DIR}/*.bin)
file(GLOB BERXEL_SDK_HEADERS ${BERXEL_SDK_INCLUDE_DIR}/*.h)
if(BERXEL_SDK_SHARED_LIBS)
  install(FILES ${BERXEL_SDK_SHARED_LIBS} DESTINATION lib)
endif()
if(BERXEL_SDK_PARAM_BINS)
  install(FILES ${BERXEL_SDK_PARAM_BINS} DESTINATION lib)
endif()
if(BERXEL_SDK_HEADERS)
  install(FILES ${BERXEL_SDK_HEADERS} DESTINATION include/${PROJECT_NAME})
endif()

# Berxel runtime config files the SDK drivers look for NEXT TO their own .so in
# <prefix>/lib (e.g. libBerxelUvcDriver.so resolves berxelCamera.ini from its own
# directory, as shown by the SDK log "Camera Config File Path = .../lib/...ini").
# Both ini are small, architecture-independent and identical across SDK releases;
# they configure the hotplug scan (berxelCamera.ini) and the SDK log sink
# (berxelLog.ini). Installed together with the shared libraries so the SDK can
# discover them at runtime on the target (RK3576) as well as on the host.
install(FILES
  ${BERXEL_SDK_DIR}/berxelCamera.ini
  ${BERXEL_SDK_DIR}/berxelLog.ini
  DESTINATION lib
  OPTIONAL
)

# First-run USB support: non-root access to the Berxel camera requires the udev
# rules to be installed once per host (SDK readMe: run the install script as root
# and replug the device). Ship both the rules and a one-click installer.
install(FILES udev/berxel-usb.rules DESTINATION share/${PROJECT_NAME}/udev)
install(PROGRAMS udev/install_berxel_udev.sh DESTINATION share/${PROJECT_NAME}/udev)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
