# Robot delivery: model, read-only diagnostics and observer. No MuJoCo, test
# fixtures or generated validation data are required to configure or build.
add_library(wbc_model SHARED
  src/wbc/whole_body_state.cpp
  src/wbc/centroidal_model.cpp
  src/wbc/static_balance.cpp
  src/wbc/zmp.cpp
  src/wbc/trajectory.cpp
  src/wbc/support_model.cpp)
target_include_directories(wbc_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(wbc_model PRIVATE ${urdfdom_INCLUDE_DIRS})
target_link_libraries(wbc_model PUBLIC Eigen3::Eigen PRIVATE pinocchio::pinocchio ${urdfdom_LIBRARIES} Threads::Threads)

add_executable(wbc_model_audit src/wbc/model_audit.cpp)
target_link_libraries(wbc_model_audit wbc_model jsoncpp_lib)
add_executable(wbc_zmp_audit src/wbc/zmp_audit.cpp)
target_link_libraries(wbc_zmp_audit wbc_model jsoncpp_lib)
add_executable(wbc_model_node src/wbc/model_node.cpp)
target_link_libraries(wbc_model_node wbc_model jsoncpp_lib)
ament_target_dependencies(wbc_model_node rclcpp rcl_interfaces sensor_msgs geometry_msgs std_msgs)
set_target_properties(wbc_model_node wbc_model_audit wbc_zmp_audit PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(TARGETS wbc_model EXPORT wbc_modelTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS wbc_model_node wbc_model_audit wbc_zmp_audit DESTINATION lib/${PROJECT_NAME})
install(FILES
  include/motion_control_pkg/wbc/model.hpp
  include/motion_control_pkg/wbc/static_balance.hpp
  include/motion_control_pkg/wbc/zmp.hpp
  include/motion_control_pkg/wbc/trajectory.hpp
  include/motion_control_pkg/wbc/tracking.hpp
  DESTINATION include/motion_control_pkg/wbc)
install(FILES config/wbc_model.yaml config/wbc_stage2_observer.yaml DESTINATION share/${PROJECT_NAME}/config)
install(FILES launch/wbc_model.launch.py DESTINATION share/${PROJECT_NAME}/launch)
install(FILES docs/WBC_ROBOT_DELIVERY.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(wbc_modelTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pinocchio urdfdom)
