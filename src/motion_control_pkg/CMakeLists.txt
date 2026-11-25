cmake_minimum_required(VERSION 3.8)
project(motion_control_pkg)

set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
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
option(WBC_MODEL_ONLY "Build the URDF model and read-only observer without hardware drivers" OFF)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(pinocchio REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Threads REQUIRED)
include(cmake/wbc_model.cmake)
if(WBC_MODEL_ONLY)
    ament_package()
    return()
endif()
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ds485_driver REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(com_interfaces REQUIRED)
# --- 查找可通过 apt 安装的第三方库 ---
# find_package(glfw3 REQUIRED)
# find_package(console_bridge REQUIRED)
# find_package(TinyXML REQUIRED)
find_package(fcl REQUIRED)
find_package(ccd REQUIRED)
# find_package(octomap REQUIRED) 解决FCL (系统级)依赖的octomap库和OctoMap (ROS 级)的冲突
find_package(octomap REQUIRED PATHS /usr NO_DEFAULT_PATH)

find_package(ompl REQUIRED)
# 新整机框架统一使用 Coal，并与 Pinocchio 的发行版本保持一致。
find_package(coal REQUIRED CONFIG)
find_package(ruckig REQUIRED)
# find_package(glfw3 REQUIRED)
# find_package(console_bridge REQUIRED)
# find_package(TinyXML REQUIRED)

find_package(PCL 1.12 REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
include_directories(${coal_INCLUDE_DIRS})


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
set(ONECUBE_DRIVER_SDK_ROOT "/opt/onecube/driver_sdk" CACHE PATH
    "DriverSDK install prefix used for both linking and bundled runtime libraries")

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


if(CMAKE_CXX_COMPILER MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
	message("linux arm64架构")
    set(_real_driver_default ON)
    set(ONECUBE_DRIVER_ARCH a64)
    set(ONECUBE_DRIVER_MACHINE aarch64)
    # set(REAL_DRIVER_SDK_LIB_DIR "/opt/onecube/driver_sdk/lib")
    set(TOPPRA_LIB_PATH "${dirToppRA}/lib/aarch64/libtoppra.so")
    set_target_properties(toppra_lib PROPERTIES
        IMPORTED_LOCATION "${TOPPRA_LIB_PATH}"
        IMPORTED_NO_SONAME TRUE
    )
    set(SDK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64")

	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_arm64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_arm64)
	endforeach()
	list(APPEND sysSimLibs dl)#arm 需要 -ldl
else()
	message("linux x64架构")
    set(_real_driver_default OFF)
    set(ONECUBE_DRIVER_ARCH x64)
    set(ONECUBE_DRIVER_MACHINE x86_64)
    set(TOPPRA_LIB_PATH "${dirToppRA}/lib/x64/libtoppra.so")
    set_target_properties(toppra_lib PROPERTIES
        IMPORTED_LOCATION "${TOPPRA_LIB_PATH}"
        IMPORTED_NO_SONAME TRUE
    )
    set(SDK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/x64")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_x64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_x64)
	endforeach()
endif()
message(${sysCoreLibs})

# x86 offline builds retain their original default. Robot builds explicitly
# select the installed SDK, including its matching public ABI header.
option(ONECUBE_USE_REAL_DRIVER_SDK "Link the real EtherCAT DriverSDK" ${_real_driver_default})
option(ONECUBE_USE_REAL_MARVIN_SDK "Use the real dual-arm Marvin SDK adapter" ${ONECUBE_USE_REAL_DRIVER_SDK})
if(ONECUBE_USE_REAL_MARVIN_SDK)
    set_property(SOURCE src/sdk/marvin_dual_arm_sdk_adapter.cpp APPEND PROPERTY
        COMPILE_DEFINITIONS ONECUBE_USE_REAL_MARVIN_SDK)
    message(STATUS "Using real dual-arm Marvin SDK adapter")
endif()
if(ONECUBE_USE_REAL_DRIVER_SDK)
    set(NEED_PSEUDO_DRIVER_SDK OFF)
    if(NOT EXISTS "${ONECUBE_DRIVER_SDK_ROOT}/include/loong_driver_sdk.h")
        message(FATAL_ERROR "Missing matching DriverSDK header under ${ONECUBE_DRIVER_SDK_ROOT}")
    endif()
    add_compile_definitions(ONECUBE_DRIVER_SDK_HEADER="${ONECUBE_DRIVER_SDK_ROOT}/include/loong_driver_sdk.h")
    find_library(ONECUBE_ETHERCAT_LIBRARY NAMES ethercat
        PATHS "${ONECUBE_DRIVER_SDK_ROOT}/igh_install/lib" /usr/local/lib)
    if(NOT ONECUBE_ETHERCAT_LIBRARY)
        message(FATAL_ERROR "Real DriverSDK requires the matching IgH libethercat")
    endif()
    get_filename_component(ONECUBE_ETHERCAT_REALPATH "${ONECUBE_ETHERCAT_LIBRARY}" REALPATH)
    message(STATUS "Real DriverSDK: ${ONECUBE_DRIVER_SDK_ROOT}, ${ONECUBE_DRIVER_MACHINE}")
else()
    set(NEED_PSEUDO_DRIVER_SDK ON)
    message(STATUS "Using offline pseudo EtherCAT DriverSDK")
endif()


# 导入 libMarvinSDK.so
add_library(marvin_sdk SHARED IMPORTED)
set_target_properties(marvin_sdk PROPERTIES
    IMPORTED_LOCATION "${SDK_LIB_DIR}/libMarvinSDK.so"
    IMPORTED_NO_SONAME TRUE
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include/motion_control_pkg/MarvinSDK"
)

# 导入 libKine.so
add_library(kine_lib SHARED IMPORTED)
set_target_properties(kine_lib PROPERTIES
    IMPORTED_LOCATION "${SDK_LIB_DIR}/libKine.so"
    IMPORTED_NO_SONAME TRUE
)

# ==============================================================================
# 4. 构建当前包的源文件
# ==============================================================================

# SDK 命令门是厂商无关的内部核心库，便于使用 Fake SDK 做离线事务测试。
add_library(dual_arm_sdk_core STATIC
    src/sdk/sdk_command_gate.cpp
    src/sdk/arm_drive_control.cpp
    src/sdk/arm_teach_recovery.cpp
    src/sdk/ethercat_drive_maintenance.cpp
    src/execution/motion_safety.cpp
)
target_include_directories(dual_arm_sdk_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(dual_arm_sdk_core PUBLIC Threads::Threads Eigen3::Eigen)

# 收集源文件
# 注意：这里显式列出文件，或者使用 file(GLOB ...) 但推荐显式列出以避免缓存问题
set(SOURCES
    src/main_node.cpp
    src/dual_arm_module.cpp
    src/sdk/marvin_dual_arm_sdk_adapter.cpp
    src/miracle_module.cpp
    src/miracle_head_module.cpp
    src/head_startup_execution.cpp
    src/miracle_lower_body_module.cpp
    src/leg_body_orientation_execution.cpp
    src/leg_body_motion_execution.cpp
    src/whole_body_module.cpp
    src/common/gripper/Modbus_Jodell.cpp
    # src/common/ctrl/PVT_ctrl.cpp
    # src/common/mpc/mpc.cpp
    # src/common/pino/pino_kin_dyn.cpp
    # src/common/wbc/priority_tasks.cpp
    # src/common/wbc/wbc_priority.cpp
    # src/common/motion/fcl_wrapper.cpp
    # Stage 0-7 的生产节点只负责 Marvin 双臂。头部、下肢和 EtherCAT
    # 仍保留源码，但在各自完成离线与真机验证前不编入本节点。
    # MovePose 直接复用 MiracleMAlgo 内部的 Pinocchio/Coal/OMPL 链；
    # 不再把旧的恒真碰撞 OMPLWrapper 编入生产节点。
    # src/utils/math/bezier_1D.cpp
    # src/utils/math/LPF_fst.cpp
    # src/utils/math/ramp_trajectory.cpp
    # src/utils/math/useful_math.cpp   
)

if(NEED_PSEUDO_DRIVER_SDK)
    list(APPEND SOURCES src/ethercat_bus_manager_fake.cpp)
    list(APPEND SOURCES src/loong_driver_sdk_stub.cpp)
    list(APPEND SOURCES src/common/suck/neardi47_gpio_fake.c)
else()
    list(APPEND SOURCES src/ethercat_bus_manager.cpp)
    if(ONECUBE_DRIVER_MACHINE STREQUAL "aarch64")
        list(APPEND SOURCES src/common/suck/neardi47_gpio.c)
    else()
        # The Neardi GPIO pin mapping is not available on an x86 controller.
        list(APPEND SOURCES src/common/suck/neardi47_gpio_unavailable.c)
    endif()
endif()

# 创建可执行节点
add_executable(motion_control_node ${SOURCES})

# Shared production planning core, independent of hardware SDKs.
add_library(motion_planning_core STATIC src/miracle_m_algo.cpp src/parallel_planning.cpp src/ik_recovery_client.cpp src/ik_recovery_solver.cpp src/motion/leg_body_orientation.cpp src/motion/leg_body_motion.cpp src/motion/tcp_level_constraint.cpp src/motion/rigid_grasp_constraint.cpp)
target_include_directories(motion_planning_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/motion_control_pkg
    ${PCL_INCLUDE_DIRS})
target_compile_definitions(motion_planning_core PUBLIC BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
ament_target_dependencies(motion_planning_core rclcpp sensor_msgs pcl_conversions com_interfaces)
target_link_libraries(motion_planning_core
    wbc_model
    Threads::Threads
    Eigen3::Eigen pinocchio::pinocchio coal::coal ompl::ompl ruckig::ruckig
    toppra_lib ${sysCoreLibs} octomap octomath jsoncpp ${PCL_LIBRARIES})

# Model-only executable: posix_spawn from ROS, then fork its workers before
# creating any threads. No motor SDK is linked; ROS is not initialized here.
add_executable(ik_recovery_worker src/ik_recovery_worker.cpp)
target_link_libraries(ik_recovery_worker motion_planning_core)
set_target_properties(ik_recovery_worker PROPERTIES INSTALL_RPATH "$ORIGIN;$ORIGIN/..")
install(TARGETS ik_recovery_worker DESTINATION lib/${PROJECT_NAME})

# Pinocchio 会把 Boost MPL 容器上限扩展到 30；必须同时禁用 Boost
# 自带的 20 元素预生成头，否则 OMPL 引入 Boost.MultiIndex 时会模板展开失败。
target_compile_definitions(motion_control_node PRIVATE
    BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
    ${PCL_DEFINITIONS}
)

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
    ${PCL_INCLUDE_DIRS}
    ${coal_INCLUDE_DIRS}
)

target_link_options(motion_control_node PRIVATE "-Wl,--disable-new-dtags")

# if(NOT NEED_PSEUDO_DRIVER_SDK)
#     target_link_options(motion_control_node PRIVATE
#         "-Wl,-rpath-link,${REAL_DRIVER_SDK_LIB_DIR}"
#         "-Wl,-rpath,${REAL_DRIVER_SDK_LIB_DIR}"
#     )
# endif()

set_target_properties(motion_control_node PROPERTIES
    INSTALL_RPATH "\$ORIGIN;\$ORIGIN/.."
)
# 链接库
# 1. ROS 2 依赖
ament_target_dependencies(motion_control_node 
    ament_index_cpp
    rclcpp
    rclcpp_action
    com_interfaces
    std_msgs
    std_srvs
    rcl_interfaces
    geometry_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    pcl_conversions
)


# 2. 第三方库
target_link_libraries(motion_control_node
    ds485_driver::ds485_control
    motion_planning_core
    dual_arm_sdk_core
    marvin_sdk
    kine_lib
    ${sysCoreLibs}
    ${sysSimLibs} 
    toppra_lib  
    Threads::Threads
    ${CMAKE_DL_LIBS}
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
    coal::coal
    ruckig::ruckig
    jsoncpp
    ${PCL_LIBRARIES}
    ${coal_LIBRARIES}
)

if(NOT NEED_PSEUDO_DRIVER_SDK)
    add_library(onecube_driver_sdk SHARED IMPORTED)
    set_target_properties(onecube_driver_sdk PROPERTIES
        IMPORTED_LOCATION "${ONECUBE_DRIVER_SDK_ROOT}/lib/libloong_driver_sdk_${ONECUBE_DRIVER_ARCH}.so"
        # The vendor library has no SONAME. Link by name so the installed node
        # resolves its bundled copy through $ORIGIN, not a build-time absolute path.
        IMPORTED_NO_SONAME TRUE)
    target_link_libraries(motion_control_node
        onecube_driver_sdk
        ${ONECUBE_DRIVER_SDK_ROOT}/loong_third_party/lib/libmodbus_${ONECUBE_DRIVER_MACHINE}.so
        ${ONECUBE_DRIVER_SDK_ROOT}/loong_third_party/lib/libnetlink_${ONECUBE_DRIVER_MACHINE}.so
        ${ONECUBE_DRIVER_SDK_ROOT}/loong_third_party/lib/libutil_${ONECUBE_DRIVER_MACHINE}.so
    )
    if(ONECUBE_DRIVER_MACHINE STREQUAL "aarch64")
        target_link_libraries(motion_control_node
            ${ONECUBE_DRIVER_SDK_ROOT}/loong_third_party/lib/libtinyxml2_aarch64.so)
    endif()
endif()


# 如果 MarvinSDK 或 Kine 依赖系统库（如 pthread, dl, rt 等），需在此补充
# target_link_libraries(motion_control_node pthread dl)
# ==============================================================================
# 6. 安装规则 (Install Rules)
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
    ${TOPPRA_LIB_PATH}
    DESTINATION lib/${PROJECT_NAME})

if(NOT NEED_PSEUDO_DRIVER_SDK)
    install(FILES
        ${ONECUBE_DRIVER_SDK_ROOT}/lib/libloong_driver_sdk_${ONECUBE_DRIVER_ARCH}.so
        ${ONECUBE_DRIVER_SDK_ROOT}/lib/libhbcanhal_${ONECUBE_DRIVER_MACHINE}.so
        ${ONECUBE_DRIVER_SDK_ROOT}/loong_third_party/lib/libmodbus_${ONECUBE_DRIVER_MACHINE}.so
        ${ONECUBE_DRIVER_SDK_ROOT}/loong_third_party/lib/libnetlink_${ONECUBE_DRIVER_MACHINE}.so
        ${ONECUBE_DRIVER_SDK_ROOT}/loong_third_party/lib/libutil_${ONECUBE_DRIVER_MACHINE}.so
        DESTINATION lib/${PROJECT_NAME})
    if(ONECUBE_DRIVER_MACHINE STREQUAL "aarch64")
        install(FILES
            ${ONECUBE_DRIVER_SDK_ROOT}/loong_third_party/lib/libtinyxml2_aarch64.so
            DESTINATION lib/${PROJECT_NAME})
    endif()
    install(FILES
        ${ONECUBE_ETHERCAT_REALPATH}
        DESTINATION lib/${PROJECT_NAME})
    install(FILES
        ${ONECUBE_ETHERCAT_REALPATH}
        DESTINATION lib/${PROJECT_NAME}
        RENAME libethercat.so.1)
endif()
   
# Both bare commands after sourcing the overlay and `ros2 run` entry points.
foreach(error_command arm_error body_error)
    install(PROGRAMS scripts/error_option DESTINATION bin RENAME ${error_command})
    install(PROGRAMS scripts/error_option DESTINATION lib/${PROJECT_NAME} RENAME ${error_command})
endforeach()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/error_options_path.dsv")
install(PROGRAMS scripts/joint_tf_pair DESTINATION lib/${PROJECT_NAME})

# 安装配置文件
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(FILES doc/ORDERLY_SHUTDOWN.txt DESTINATION share/${PROJECT_NAME}/doc)
install(FILES doc/ROBOT_TF.md DESTINATION share/${PROJECT_NAME}/doc)
install(FILES doc/TEACH_AND_RECOVERY.md doc/SDK_LIMITS_AND_TEACH_RELEASE.md doc/GX12-01_GRIPPER.md
    DESTINATION share/${PROJECT_NAME}/doc)

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

# Offline regression: real URDF/Pinocchio, no vendor SDK or hardware node.
if(BUILD_TESTING)
    add_executable(test_suction_collision test/suction_collision_test.cpp)
    target_link_libraries(test_suction_collision motion_planning_core)
    add_test(NAME suction_collision COMMAND test_suction_collision "${CMAKE_CURRENT_SOURCE_DIR}/../..")
    set_tests_properties(suction_collision PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)
    add_executable(test_sdk_arm_position_limits test/sdk_arm_position_limits_test.cpp)
    target_link_libraries(test_sdk_arm_position_limits motion_planning_core)
    add_dependencies(test_sdk_arm_position_limits ik_recovery_worker)
    add_test(NAME sdk_arm_position_limits COMMAND test_sdk_arm_position_limits "${CMAKE_CURRENT_SOURCE_DIR}/../..")
    set_tests_properties(sdk_arm_position_limits PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)
    add_executable(test_dual_arm_completion test/dual_arm_completion_test.cpp)
    target_include_directories(test_dual_arm_completion PRIVATE include)
    add_test(NAME dual_arm_completion COMMAND test_dual_arm_completion)
    set_tests_properties(dual_arm_completion PROPERTIES TIMEOUT 10)
    add_executable(test_ik_seed_sampler test/ik_seed_sampler_test.cpp)
    target_include_directories(test_ik_seed_sampler PRIVATE include)
    add_test(NAME ik_seed_sampler COMMAND test_ik_seed_sampler)
    add_executable(test_ik_recovery test/ik_recovery_test.cpp)
    target_link_libraries(test_ik_recovery motion_planning_core)
    add_dependencies(test_ik_recovery ik_recovery_worker)
    add_test(NAME ik_recovery COMMAND test_ik_recovery "${CMAKE_CURRENT_SOURCE_DIR}/../..")
    set_tests_properties(ik_recovery PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)
    get_filename_component(_rigid_interface_prefix "${com_interfaces_DIR}/../../.." ABSOLUTE)
    foreach(rigid_test leg_rigid_grasp leg_rigid_execution)
        add_executable(test_${rigid_test} test/${rigid_test}_test.cpp src/whole_body_module.cpp)
        target_include_directories(test_${rigid_test} PRIVATE
            $<TARGET_PROPERTY:motion_control_node,INCLUDE_DIRECTORIES>
            include/motion_control_pkg/MarvinSDK)
        target_link_libraries(test_${rigid_test} motion_planning_core dual_arm_sdk_core)
        ament_target_dependencies(test_${rigid_test} rclcpp_action tf2 tf2_ros tf2_geometry_msgs)
        add_test(NAME ${rigid_test} COMMAND test_${rigid_test} "${CMAKE_CURRENT_SOURCE_DIR}/../..")
        set_tests_properties(${rigid_test} PROPERTIES TIMEOUT 180
            ENVIRONMENT "ROS_DOMAIN_ID=225;ROS_LOCALHOST_ONLY=1;LD_LIBRARY_PATH=${_rigid_interface_prefix}/lib:$ENV{LD_LIBRARY_PATH};AMENT_PREFIX_PATH=${_rigid_interface_prefix}:$ENV{AMENT_PREFIX_PATH}"
            RUN_SERIAL TRUE)
    endforeach()
    add_executable(test_rigid_grasp test/rigid_grasp_test.cpp)
    target_link_libraries(test_rigid_grasp motion_planning_core dual_arm_sdk_core)
    ament_target_dependencies(test_rigid_grasp com_interfaces)
    add_test(NAME rigid_grasp COMMAND test_rigid_grasp "${CMAKE_CURRENT_SOURCE_DIR}/../..")
    set_tests_properties(rigid_grasp PROPERTIES TIMEOUT 180)
    add_test(NAME robot_tf COMMAND /usr/bin/python3
        "${CMAKE_CURRENT_SOURCE_DIR}/test/robot_tf_test.py"
        "${CMAKE_CURRENT_SOURCE_DIR}")
    set_tests_properties(robot_tf PROPERTIES TIMEOUT 60)
    add_executable(test_waist_leg_trajectory test/waist_leg_trajectory_test.cpp)
    target_link_libraries(test_waist_leg_trajectory motion_planning_core)
    add_test(NAME waist_leg_trajectory COMMAND test_waist_leg_trajectory "${CMAKE_CURRENT_SOURCE_DIR}/../..")
    set_tests_properties(waist_leg_trajectory PROPERTIES TIMEOUT 45)
    add_executable(test_latest_progress test/latest_progress_test.cpp)
    target_include_directories(test_latest_progress PRIVATE include)
    target_link_libraries(test_latest_progress PRIVATE Threads::Threads Eigen3::Eigen)
    add_test(NAME latest_progress COMMAND test_latest_progress)
    set_tests_properties(latest_progress PROPERTIES TIMEOUT 30)
    add_executable(test_error_tracking_option test/error_tracking_option_test.cpp)
    target_link_libraries(test_error_tracking_option dual_arm_sdk_core)
    add_test(NAME error_tracking_option COMMAND test_error_tracking_option)
    set_tests_properties(error_tracking_option PROPERTIES TIMEOUT 30)
    add_test(NAME error_option_cli COMMAND /usr/bin/python3
        "${CMAKE_CURRENT_SOURCE_DIR}/test/error_option_cli_test.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/error_option")
    set_tests_properties(error_option_cli PROPERTIES TIMEOUT 10)
    add_executable(test_orderly_shutdown test/orderly_shutdown_test.cpp)
    target_include_directories(test_orderly_shutdown PRIVATE include)
    add_test(NAME orderly_shutdown COMMAND test_orderly_shutdown)
    add_executable(test_shutdown_gate test/shutdown_gate_test.cpp)
    target_link_libraries(test_shutdown_gate dual_arm_sdk_core)
    add_test(NAME shutdown_gate COMMAND test_shutdown_gate)
    add_executable(test_shutdown_signal test/shutdown_signal_test.cpp)
    target_include_directories(test_shutdown_signal PRIVATE include)
    ament_target_dependencies(test_shutdown_signal rclcpp)
    add_test(NAME shutdown_signal COMMAND test_shutdown_signal)
    add_test(NAME shutdown_launch COMMAND /usr/bin/python3
        ${CMAKE_CURRENT_SOURCE_DIR}/test/shutdown_launch_test.py
        $<TARGET_FILE:test_shutdown_signal> ${CMAKE_CURRENT_SOURCE_DIR}/launch/bringup.launch.py)
    set_tests_properties(orderly_shutdown shutdown_gate shutdown_signal shutdown_launch
        PROPERTIES TIMEOUT 30 ENVIRONMENT "ROS_DOMAIN_ID=216;ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp;CYCLONEDDS_URI=")
    add_executable(test_parallel_planning test/parallel_planning_test.cpp)
    target_link_libraries(test_parallel_planning motion_planning_core Threads::Threads)
    add_test(NAME parallel_planning COMMAND test_parallel_planning "${CMAKE_CURRENT_SOURCE_DIR}/../..")
    set_tests_properties(parallel_planning PROPERTIES TIMEOUT 120)
    add_executable(test_ethercat_startup_readiness test/ethercat_startup_readiness_test.cpp)
    target_include_directories(test_ethercat_startup_readiness PRIVATE include)
    add_test(NAME ethercat_startup_readiness COMMAND test_ethercat_startup_readiness)
    add_executable(test_tracking_diagnostics test/tracking_diagnostics_test.cpp)
    target_include_directories(test_tracking_diagnostics PRIVATE include)
    target_link_libraries(test_tracking_diagnostics Eigen3::Eigen)
    add_test(NAME tracking_diagnostics COMMAND test_tracking_diagnostics)
    add_executable(test_measured_feedback test/measured_feedback_test.cpp)
    target_include_directories(test_measured_feedback PRIVATE include)
    target_link_libraries(test_measured_feedback Threads::Threads)
    ament_target_dependencies(test_measured_feedback sensor_msgs)
    add_test(NAME measured_feedback_snapshot COMMAND test_measured_feedback)
    add_executable(test_ethercat_initial_enable test/ethercat_initial_enable_test.cpp)
    target_link_libraries(test_ethercat_initial_enable dual_arm_sdk_core)
    add_test(NAME ethercat_initial_enable COMMAND test_ethercat_initial_enable)
    set_tests_properties(ethercat_initial_enable PROPERTIES TIMEOUT 20)
    add_executable(test_ethercat_feedback_validity test/ethercat_feedback_validity_test.cpp)
    target_include_directories(test_ethercat_feedback_validity PRIVATE include)
    add_test(NAME ethercat_feedback_validity COMMAND test_ethercat_feedback_validity)
    if(ONECUBE_USE_REAL_DRIVER_SDK AND ONECUBE_DRIVER_MACHINE STREQUAL "x86_64")
        # Loading the real SDK must not interpose an incompatible XML ABI into
        # Fast DDS. This creates only a ROS node; it never initializes hardware.
        add_executable(test_x86_driver_ros_init test/x86_driver_ros_init_test.cpp)
        target_link_options(test_x86_driver_ros_init PRIVATE "-Wl,--no-as-needed")
        target_link_libraries(test_x86_driver_ros_init onecube_driver_sdk marvin_sdk kine_lib)
        ament_target_dependencies(test_x86_driver_ros_init rclcpp)
        add_test(NAME x86_driver_ros_init COMMAND test_x86_driver_ros_init)
        set_tests_properties(x86_driver_ros_init PROPERTIES TIMEOUT 20
            ENVIRONMENT "ROS_DOMAIN_ID=216;ROS_LOCALHOST_ONLY=1")
    endif()
    add_executable(test_head_startup_home test/head_startup_home_test.cpp)
    target_link_libraries(test_head_startup_home dual_arm_sdk_core ruckig::ruckig)
    add_test(NAME head_startup_home COMMAND test_head_startup_home)
    set_tests_properties(head_startup_home PROPERTIES TIMEOUT 30)
    add_executable(test_head_startup_geometry test/head_startup_geometry_test.cpp)
    target_link_libraries(test_head_startup_geometry motion_planning_core)
    add_test(NAME head_startup_geometry COMMAND test_head_startup_geometry "${CMAKE_CURRENT_SOURCE_DIR}/../..")
    set_tests_properties(head_startup_geometry PROPERTIES TIMEOUT 30)
    enable_testing()
    add_executable(test_precision_arrival test/precision_arrival_test.cpp)
    target_include_directories(test_precision_arrival PRIVATE
        $<TARGET_PROPERTY:motion_control_node,INCLUDE_DIRECTORIES>)
    target_link_libraries(test_precision_arrival dual_arm_sdk_core ruckig::ruckig)
    ament_target_dependencies(test_precision_arrival com_interfaces)
    add_test(NAME precision_arrival COMMAND test_precision_arrival)
    set_tests_properties(precision_arrival PROPERTIES TIMEOUT 45)
    add_executable(test_drive_control test/drive_control_test.cpp)
    target_link_libraries(test_drive_control dual_arm_sdk_core)
    add_test(NAME drive_control COMMAND test_drive_control)
    add_executable(test_teach_recovery test/teach_recovery_test.cpp)
    target_link_libraries(test_teach_recovery dual_arm_sdk_core)
    add_test(NAME teach_recovery COMMAND test_teach_recovery)
    set_tests_properties(teach_recovery PROPERTIES TIMEOUT 60)
    set_tests_properties(drive_control PROPERTIES TIMEOUT 45)
    add_executable(test_tcp_level_constraint test/tcp_level_constraint_test.cpp)
    target_link_libraries(test_tcp_level_constraint motion_planning_core dual_arm_sdk_core)
    ament_target_dependencies(test_tcp_level_constraint com_interfaces)
    add_test(NAME tcp_level_constraint COMMAND test_tcp_level_constraint
        "${CMAKE_CURRENT_SOURCE_DIR}/../onecube_robot_description/urdf/onecube_robot_description.urdf")
    add_executable(test_tcp_level_planning test/tcp_level_planning_test.cpp src/whole_body_module.cpp)
    target_include_directories(test_tcp_level_planning PRIVATE
        $<TARGET_PROPERTY:motion_control_node,INCLUDE_DIRECTORIES>
        include/motion_control_pkg/MarvinSDK)
    target_link_libraries(test_tcp_level_planning motion_planning_core dual_arm_sdk_core)
    ament_target_dependencies(test_tcp_level_planning rclcpp_action tf2 tf2_ros tf2_geometry_msgs)
    add_test(NAME tcp_level_planning COMMAND test_tcp_level_planning "${CMAKE_CURRENT_SOURCE_DIR}/../..")
    set_tests_properties(tcp_level_planning PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=216;ROS_LOCALHOST_ONLY=1")
endif()

option(WBC_MUJOCO_VALIDATION "Build hardware-free trajectory exporter for MuJoCo" OFF)
if(WBC_MUJOCO_VALIDATION)
    add_executable(wbc_mujoco_export tools/mujoco_validation/export_trajectories.cpp
        tools/mujoco_validation/hardware_traps.cpp src/whole_body_module.cpp)
    target_include_directories(wbc_mujoco_export PRIVATE
        $<TARGET_PROPERTY:motion_control_node,INCLUDE_DIRECTORIES>
        include/motion_control_pkg/MarvinSDK)
    target_link_libraries(wbc_mujoco_export motion_planning_core dual_arm_sdk_core)
    ament_target_dependencies(wbc_mujoco_export rclcpp_action tf2 tf2_ros tf2_geometry_msgs)
endif()

# 导出包配置


ament_package()
