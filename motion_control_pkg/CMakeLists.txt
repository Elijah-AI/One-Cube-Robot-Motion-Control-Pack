cmake_minimum_required(VERSION 3.8)
project(motion_control_pkg)

set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "\$ORIGIN")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1. 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. 查找 ROS 2 及第三方依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(com_interfaces REQUIRED)
# --- 查找可通过 apt 安装的第三方库 ---
find_package(Eigen3 REQUIRED)
# find_package(glfw3 REQUIRED)
find_package(urdfdom REQUIRED)
# find_package(console_bridge REQUIRED)
# find_package(TinyXML REQUIRED)
find_package(fcl REQUIRED)
find_package(ccd REQUIRED)
# find_package(octomap REQUIRED) 解决FCL (系统级)依赖的octomap库和OctoMap (ROS 级)的冲突
find_package(octomap REQUIRED PATHS /usr NO_DEFAULT_PATH)

find_package(ompl REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ruckig REQUIRED)
# find_package(glfw3 REQUIRED)
# find_package(console_bridge REQUIRED)
# find_package(TinyXML REQUIRED)
find_package(jsoncpp REQUIRED) 



# 如果其他模块需要，在此添加更多 find_package，例如:
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)

# ==============================================================================
# 3. 配置第三方库 (Third-party Libraries)
# ==============================================================================

# --- 3.1 配置  (静态库，区分平台) ---
set(dirQuill "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/quill")
set(dirQP    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/qpOASES")
set(incFmt   "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/quill/quill/bundled")
set(allDir ${dirQuill} ${dirQP} )
set(allLib ${allDir})
set(allInc ${allDir} ${incFmt})
include_directories(${allInc})
message(${CMAKE_SYSTEM_PROCESSOR})
link_directories(${allLib})

#链接库
set(corLibs quill qpOASES)
set(simLibs )#mujoco区分x86/arm的方式不一样，在上面按link目录区分

#根据架构，处理连接库
set(sysCoreLibs)
set(sysSimLibs)

## 配置第三方库配置  (动态库) --- 
##1.定义第三方库路径 
set(dirToppRA "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/toppra")
## 2.设置包含目录 (Include Directories)
include_directories(
    ${dirToppRA}/include
)
## 3. 设置库文件搜索路径 (Link Directories) 
link_directories(
    ${dirToppRA}/lib
)
add_library(toppra_lib SHARED IMPORTED)


if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	message("linux arm64架构")
    set_target_properties(toppra_lib PROPERTIES
        IMPORTED_LOCATION "${dirToppRA}/lib/aarch64/libtoppra.so"
    )
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_arm64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_arm64)
	endforeach()
	list(APPEND sysSimLibs dl)#arm 需要 -ldl
else()
	message("linux x64架构")
    set_target_properties(toppra_lib PROPERTIES
        IMPORTED_LOCATION "${dirToppRA}/lib/x64/libtoppra.so"
    )
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_x64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_x64)
	endforeach()
endif()
message(${sysCoreLibs})


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
    src/loong_driver_sdk_stub.cpp
    src/ethercat_bus_manager.cpp
    src/main_node.cpp
    src/dual_arm_module.cpp
    src/miracle_module.cpp
    src/miracle_head_module.cpp
    src/miracle_lower_body_module.cpp
    src/miracle_m_algo.cpp
    # src/common/ctrl/PVT_ctrl.cpp
    # src/common/mpc/mpc.cpp
    # src/common/pino/pino_kin_dyn.cpp
    # src/common/wbc/priority_tasks.cpp
    # src/common/wbc/wbc_priority.cpp
    src/common/gripper‌/Modbus_Jodell.cpp
    src/common/motion/fcl_wrapper.cpp
    src/common/motion/planner_factory.cpp
    src/common/motion/ruckig_planner.cpp
    src/common/motion/toppra_planner.cpp
    src/common/motion/ompl_wrapper.cpp
    # src/utils/math/bezier_1D.cpp
    # src/utils/math/LPF_fst.cpp
    # src/utils/math/ramp_trajectory.cpp
    # src/utils/math/useful_math.cpp   
)

# 创建可执行节点
add_executable(motion_control_node ${SOURCES})

# 设置包含目录
# 1. 当前包的头文件 (include/motion_control_pkg)
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
    ${Eigen3_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    # ${console_bridge_INCLUDE_DIRS}
    # ${TinyXML_INCLUDE_DIRS}
    ${fcl_INCLUDE_DIRS}
    ${ccd_INCLUDE_DIRS}
    ${octomap_INCLUDE_DIRS}
    ${ompl_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
    ${ruckig_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
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
    com_interfaces
    std_msgs
    geometry_msgs)


# 2. 第三方库
target_link_libraries(motion_control_node
    marvin_sdk
    kine_lib
    ${sysCoreLibs}
    ${sysSimLibs} 
    toppra_lib  
    pthread 
    dl
    Eigen3::Eigen
    # glfw
    urdfdom::urdfdom_model
    # console_bridge::console_bridge
    fcl
    ccd
    octomap
    octomath
    ompl::ompl
    pinocchio::pinocchio
    ruckig::ruckig
    jsoncpp
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
       
install(FILES
    ${SDK_LIB_DIR}/libMarvinSDK.so
    ${SDK_LIB_DIR}/libKine.so
    ${dirToppRA}/lib/x64/libtoppra.so
    DESTINATION lib/${PROJECT_NAME})
   
# 安装配置文件
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch   
  DESTINATION share/${PROJECT_NAME}
)

# 安装头文件 (如果其他包需要依赖此包的头文件)
# install(DIRECTORY include/
#     DESTINATION include
# )

# 安装 action 文件（可选，但推荐，以便其他包引用）
# install(DIRECTORY action/
#   DESTINATION share/${PROJECT_NAME}/action
# )

# 导出包配置
ament_package()
