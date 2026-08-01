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
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(com_interfaces REQUIRED)
find_package(Threads REQUIRED)
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
# 新整机框架统一使用 Coal，并与 Pinocchio 的发行版本保持一致。
find_package(coal REQUIRED CONFIG)
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


if(CMAKE_CXX_COMPILER MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
	message("linux arm64架构")
    set(NEED_PSEUDO_DRIVER_SDK OFF)
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
    set(NEED_PSEUDO_DRIVER_SDK ON)
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
    src/execution/dual_arm_command_coordinator.cpp
    src/execution/synchronized_trajectory.cpp
    src/execution/dual_arm_task_arbiter.cpp
)
target_include_directories(dual_arm_sdk_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(dual_arm_sdk_core PUBLIC Threads::Threads)

# 收集源文件
# 注意：这里显式列出文件，或者使用 file(GLOB ...) 但推荐显式列出以避免缓存问题
set(SOURCES
    src/main_node.cpp
    src/dual_arm_module.cpp
    src/sdk/marvin_dual_arm_sdk_adapter.cpp
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
else()
    list(APPEND SOURCES src/ethercat_bus_manager.cpp)
endif()

# 创建可执行节点
add_executable(motion_control_node ${SOURCES})

# Pinocchio 会把 Boost MPL 容器上限扩展到 30；必须同时禁用 Boost
# 自带的 20 元素预生成头，否则 OMPL 引入 Boost.MultiIndex 时会模板展开失败。
target_compile_definitions(motion_control_node PRIVATE
    BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
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
)

target_link_options(motion_control_node PRIVATE "-Wl,--disable-new-dtags")

# if(NOT NEED_PSEUDO_DRIVER_SDK)
#     target_link_options(motion_control_node PRIVATE
#         "-Wl,-rpath-link,${REAL_DRIVER_SDK_LIB_DIR}"
#         "-Wl,-rpath,${REAL_DRIVER_SDK_LIB_DIR}"
#     )
# endif()

set_target_properties(motion_control_node PROPERTIES
    INSTALL_RPATH "\$ORIGIN"
)
# 链接库
# 1. ROS 2 依赖
ament_target_dependencies(motion_control_node 
    ament_index_cpp
    rclcpp
    rclcpp_action
    com_interfaces
    std_msgs
    geometry_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)


# 2. 第三方库
target_link_libraries(motion_control_node
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
)

if(NOT NEED_PSEUDO_DRIVER_SDK)
    target_link_libraries(motion_control_node
        /opt/onecube/driver_sdk/lib/libloong_driver_sdk_a64.so
        /opt/onecube/driver_sdk/loong_third_party/lib/libmodbus_aarch64.so
        /opt/onecube/driver_sdk/loong_third_party/lib/libnetlink_aarch64.so
        /opt/onecube/driver_sdk/loong_third_party/lib/libutil_aarch64.so
        /opt/onecube/driver_sdk/loong_third_party/lib/libtinyxml2_aarch64.so
    )
endif()


# ==============================================================================
# 5. 离线测试：只链接厂商无关核心库和 Fake 设备，禁止链接 MarvinSDK/Kine。
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  add_library(fake_dual_arm_device STATIC
    test/fake_dual_arm_device.cpp
  )
  target_include_directories(fake_dual_arm_device PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(fake_dual_arm_device PUBLIC Threads::Threads)

  ament_add_gtest(test_sdk_command_gate
    test/test_sdk_command_gate.cpp
  )
  target_include_directories(test_sdk_command_gate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(test_sdk_command_gate
    dual_arm_sdk_core
    fake_dual_arm_device
    Threads::Threads
  )

  ament_add_gtest(test_dual_arm_command_coordinator
    test/test_dual_arm_command_coordinator.cpp
  )
  target_include_directories(test_dual_arm_command_coordinator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(test_dual_arm_command_coordinator
    dual_arm_sdk_core
    fake_dual_arm_device
    Threads::Threads
  )

  ament_add_gtest(test_dual_arm_task_arbiter
    test/test_dual_arm_task_arbiter.cpp
  )
  target_link_libraries(test_dual_arm_task_arbiter
    dual_arm_sdk_core
    Threads::Threads
  )

  ament_add_gtest(test_dual_arm_pose_transforms
    test/test_dual_arm_pose_transforms.cpp
  )
  target_include_directories(test_dual_arm_pose_transforms PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_dual_arm_pose_transforms
    rclcpp
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )
  target_link_libraries(test_dual_arm_pose_transforms
    Eigen3::Eigen
  )
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
        /opt/onecube/driver_sdk/lib/libloong_driver_sdk_a64.so
        /opt/onecube/driver_sdk/lib/libhbcanhal_aarch64.so
        /opt/onecube/driver_sdk/loong_third_party/lib/libmodbus_aarch64.so
        /opt/onecube/driver_sdk/loong_third_party/lib/libnetlink_aarch64.so
        /opt/onecube/driver_sdk/loong_third_party/lib/libutil_aarch64.so
        /opt/onecube/driver_sdk/loong_third_party/lib/libtinyxml2_aarch64.so
        DESTINATION lib/${PROJECT_NAME})
    install(FILES
        /opt/onecube/driver_sdk/igh_install/lib/libethercat.so.1.1.0
        DESTINATION lib/${PROJECT_NAME})
    install(FILES
        /opt/onecube/driver_sdk/igh_install/lib/libethercat.so.1.1.0
        DESTINATION lib/${PROJECT_NAME}
        RENAME libethercat.so.1)
endif()
   
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
