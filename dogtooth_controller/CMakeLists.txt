cmake_minimum_required(VERSION 3.16)
project(dogtooth_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

if(NOT CMAKE_CXX_STANDARD)

  set(CMAKE_CXX_STANDARD 14)

endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(-Wall -Wextra -Wpedantic)

endif()
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)
include_directories(
  include
)
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(dogtooth_controller_parameters
  src/dogtooth_controller_parameter.yaml
)
# add_library(dogtooth_hardware SHARED 
# hardware/dogtooth_hardware.cpp)
# 

add_library(dogtooth_controller SHARED
  src/dogtooth_controller.cpp
  src/dogtooth_hardware.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_compile_features(dogtooth_controller PUBLIC cxx_std_17)
target_include_directories(dogtooth_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/dogtooth_controller>
)
target_link_libraries(dogtooth_controller PUBLIC dogtooth_controller_parameters)
ament_target_dependencies(dogtooth_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(dogtooth_controller PRIVATE "DOGTOOTH_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface dogtooth_plugin.xml)


target_compile_definitions(${PROJECT_NAME} PRIVATE "DOGTOOTH_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface dogtooth_hardware_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_dogtooth_controller
    test/test_dogtooth_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_dogtooth_controller.yaml)
  target_link_libraries(test_dogtooth_controller
    dogtooth_controller
  )
  ament_target_dependencies(test_dogtooth_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

  ament_add_gmock(test_load_dogtooth_controller
    test/test_load_dogtooth_controller.cpp
  )
  ament_target_dependencies(test_load_dogtooth_controller
    controller_manager
    ros2_control_test_assets
  )
endif()
install(
  DIRECTORY include/dogtooth_hardware/
  DESTINATION include/dogtooth_hardware
)

install(
  DIRECTORY include/dogtooth_controller/
  DESTINATION include/dogtooth_controller
)
install(TARGETS dogtooth_controller dogtooth_controller_parameters
  EXPORT export_dogtooth_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_dogtooth_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
