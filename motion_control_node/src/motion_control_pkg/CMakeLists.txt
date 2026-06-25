cmake_minimum_required(VERSION 3.8)
project(motion_control_pkg)

set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "\$ORIGIN")


# 1. 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. 查找 ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickAndPlace.action"
  "action/HardwareReset.action"
  "action/HardwareCheck.action"
  "action/MoveArm.action"
  "action/ControlGripper.action"
  "action/AdjustBasePose.action"
  "msg/HeartBeat.msg"
  "msg/DeviceStatus.msg"
  "msg/MotorStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# 如果其他模块需要，在此添加更多 find_package，例如:
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)

# ==============================================================================
# 3. 配置第三方库 (Third-party Libraries)
# ==============================================================================

# --- 3.1 配置  (静态库，区分平台) ---
set(dirEigen "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/eigen3")
set(dirGlfw  "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/glfw")
set(dirPino  "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/pinocchio")
set(dirJson  "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/jsoncpp")
set(dirQuill "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/quill")
set(dirQP    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/qpOASES")
set(libURDF  "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/urdfdom")
set(incFmt   "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/quill/quill/bundled")
set(incBoost "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/boost")
set(incMujoco "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/mujoco")
set(allDir ${dirEigen} ${dirGlfw} ${dirPino} ${dirJson} ${dirQuill} ${dirQP} )
set(allLib ${allDir} ${libURDF})
set(allInc ${allDir} ${incFmt} ${incBoost} ${incMujoco})
include_directories(${allInc})
include_directories(${OMPL_INCLUDE_DIRS})
message(${CMAKE_SYSTEM_PROCESSOR})
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	list(APPEND allLib "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/mujoco/lin_arm64")
	set(mujocoLib "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/mujoco/lin_arm64/libmujoco.so.3.1.1")
else()
	list(APPEND allLib "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/mujoco/lin_x64")
	set(mujocoLib "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/mujoco/lin_x64/libmujoco.so.3.1.1")
endif()
link_directories(${allLib})

#链接库
set(corLibs pinocchio urdfdom_model tinyxml console_bridge jsoncpp quill qpOASES)
set(simLibs glfw3)#mujoco区分x86/arm的方式不一样，在上面按link目录区分

#根据架构，处理连接库
set(sysCoreLibs)
set(sysSimLibs)
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	message("linux arm64架构")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_arm64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_arm64)
	endforeach()
	list(APPEND sysSimLibs dl)#arm 需要 -ldl
else()
	message("linux x64架构")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_x64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_x64)
	endforeach()
endif()
message(${sysCoreLibs})


## 3.1.1 配置第三方库配置  (动态库) --- 
##1.定义第三方库路径 
set(dirCCD "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ccd")
set(dirOctomap "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/octomap")
set(dirFCL  "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/fcl")
set(dirToppRA "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/toppra")
set(dirRuckig "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ruckig")
set(dirOmpl "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ompl")
## 2.设置包含目录 (Include Directories)
include_directories(
    ${dirCCD}/include
    ${dirOctomap}/include
    ${dirFCL}/include
    ${dirToppRA}/include
    ${dirRuckig}/include
    ${dirOmpl}/include/ompl-1.6
)
## 3. 设置库文件搜索路径 (Link Directories) 
link_directories(
    ${dirCCD}/lib
    ${dirOctomap}/lib
    ${dirFCL}/lib
    ${dirToppRA}/lib
    ${dirRuckig}/lib
    ${dirOmpl}/lib
)
## 4. 导入ruckig.so
add_library(ruckig_lib SHARED IMPORTED)
set_target_properties(ruckig_lib PROPERTIES
    IMPORTED_LOCATION "${dirRuckig}/lib/libruckig.so"
)
#导入octomap
add_library(octomap_lib SHARED IMPORTED)
set_target_properties(octomap_lib PROPERTIES
    IMPORTED_LOCATION "${dirOctomap}/lib/liboctomap.so.1.10"
    # 如果有头文件依赖，也可以在这里指定，但 include_directories 已经做了
)
add_library(octomath_lib SHARED IMPORTED)
set_target_properties(octomath_lib PROPERTIES
    IMPORTED_LOCATION "${dirOctomap}/lib/liboctomath.so.1.10"
)

add_library(ccd_lib SHARED IMPORTED)
set_target_properties(ccd_lib PROPERTIES
    IMPORTED_LOCATION "${dirCCD}/lib/libccd.so.2"
)
#导入fcl
add_library(fcl_lib SHARED IMPORTED)
set_target_properties(fcl_lib PROPERTIES
    IMPORTED_LOCATION "${dirFCL}/lib/libfcl.so.0.7"
)
add_library(toppra_lib SHARED IMPORTED)
set_target_properties(toppra_lib PROPERTIES
    IMPORTED_LOCATION "${dirToppRA}/lib/libtoppra.so"
)

#ompl
add_library(ompl_lib SHARED IMPORTED)
set_target_properties(ompl_lib PROPERTIES
    IMPORTED_LOCATION "${dirOmpl}/lib/libompl.so.1.6.0"
)

# --- 3.2 配置 MarvinSDK 和 Kine (动态库) ---
set(SDK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")

# 导入 libMarvinSDK.so
add_library(marvin_sdk SHARED IMPORTED)
set_target_properties(marvin_sdk PROPERTIES
    IMPORTED_LOCATION "${SDK_LIB_DIR}/libMarvinSDK.so"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include/motion_control_pkg/MarvinSDK"
)

# 导入 libKine.so
add_library(kine_lib SHARED IMPORTED)
set_target_properties(kine_lib PROPERTIES
    IMPORTED_LOCATION "${SDK_LIB_DIR}/libKine.so"
)

# ==============================================================================
# 4. 构建当前包的源文件
# ==============================================================================

# 收集源文件
# 注意：这里显式列出文件，或者使用 file(GLOB ...) 但推荐显式列出以避免缓存问题
set(SOURCES
    src/main_node.cpp
    src/ArmControlNode.cpp
    src/common/ctrl/PVT_ctrl.cpp
    src/common/mpc/mpc.cpp
    src/common/pino/pino_kin_dyn.cpp
    src/common/wbc/priority_tasks.cpp
    src/common/wbc/wbc_priority.cpp
    src/common/gripper‌/Modbus_Jodell.cpp
    src/common/motion/fcl_wrapper.cpp
    src/common/motion/planner_factory.cpp
    src/common/motion/ruckig_planner.cpp
    src/common/motion/toppra_planner.cpp
    src/common/motion/ompl_wrapper.cpp
    src/utils/math/bezier_1D.cpp
    src/utils/math/LPF_fst.cpp
    src/utils/math/ramp_trajectory.cpp
    src/utils/math/useful_math.cpp   
)

# 创建可执行节点
add_executable(motion_control_node ${SOURCES})

# 设置包含目录
# 1. 当前包的头文件 (include/motion_control_pkg)
# 2. JsonCpp 的头文件 (3rdparty/jsoncpp)
# 3. MarvinSDK 的头文件 (已在 target_properties 中通过 INTERFACE_INCLUDE_DIRECTORIES 设置，也可在此再次确认)
target_include_directories(motion_control_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/motion_control_pkg/ctrl
    ${CMAKE_CURRENT_SOURCE_DIR}/include/motion_control_pkg/mpc
    ${CMAKE_CURRENT_SOURCE_DIR}/include/motion_control_pkg/pino
    ${CMAKE_CURRENT_SOURCE_DIR}/include/motion_control_pkg/wbc
    ${CMAKE_CURRENT_SOURCE_DIR}/include/motion_control_pkg/gripper‌
    ${CMAKE_CURRENT_SOURCE_DIR}/include/motion_control_pkg/motion
    ${CMAKE_CURRENT_SOURCE_DIR}/include/motion_control_pkg
    ${dirOmpl}/include/ompl-1.6
)

target_link_options(motion_control_node PRIVATE "-Wl,--disable-new-dtags")

set_target_properties(motion_control_node PROPERTIES
    INSTALL_RPATH "\$ORIGIN"
)
# 链接库
# 1. ROS 2 依赖
ament_target_dependencies(motion_control_node 
    rclcpp
    rclcpp_action
    std_msgs
    geometry_msgs)

# 链接生成的 Action 接口库
# 注意：这一步至关重要，否则找不到 action 的头文件
rosidl_target_interfaces(motion_control_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)


# 2. 第三方库
target_link_libraries(motion_control_node
    marvin_sdk
    kine_lib
    ompl_lib
    ${sysCoreLibs}
    ${sysSimLibs} 
    fcl_lib 
    ccd_lib
    octomap_lib
    octomath_lib
    toppra_lib  
    ruckig_lib  
    pthread 
    dl
)

# 如果 MarvinSDK 或 Kine 依赖系统库（如 pthread, dl, rt 等），需在此补充
# target_link_libraries(motion_control_node pthread dl)
# ==============================================================================
# 5. 安装规则 (Install Rules)
# ==============================================================================

# 安装可执行文件
install(TARGETS motion_control_node
    DESTINATION lib/${PROJECT_NAME}
)

# 安装第三方动态库 (.so) 到 lib 目录，以便运行时能找到
# 注意：静态库 (.a) 不需要安装到运行时环境，只需要在编译时链接即可   
 
install(DIRECTORY ${dirOmpl}/lib/
        DESTINATION lib/${PROJECT_NAME}
        FILES_MATCHING PATTERN "libompl*")

install(DIRECTORY ${dirFCL}/lib/
        DESTINATION lib/${PROJECT_NAME}
        FILES_MATCHING PATTERN "libfcl*")
        
install(DIRECTORY ${dirOctomap}/lib/
        DESTINATION lib/${PROJECT_NAME}
        FILES_MATCHING PATTERN "libocto*")

install(DIRECTORY ${dirCCD}/lib/
        DESTINATION lib/${PROJECT_NAME}
        FILES_MATCHING PATTERN "libccd*")
       
install(FILES
    ${SDK_LIB_DIR}/libMarvinSDK.so
    ${SDK_LIB_DIR}/libKine.so
    ${dirRuckig}/lib/libruckig.so
    ${dirToppRA}/lib/libtoppra.so
    DESTINATION lib/${PROJECT_NAME})
   
# 安装配置文件
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

# 安装头文件 (如果其他包需要依赖此包的头文件)
install(DIRECTORY include/
    DESTINATION include
)

# 安装 action 文件（可选，但推荐，以便其他包引用）
install(DIRECTORY action/
  DESTINATION share/${PROJECT_NAME}/action
)

# 导出包配置
ament_package()
