cmake_minimum_required(VERSION 3.8)
project(hunter_controller LANGUAGES CXX)

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


generate_parameter_library(hunter_controller_parameters
  src/hunter_controller_parameter.yaml
)
# add_library(hunter_hardware SHARED 
# hardware/hunter_hardware.cpp)
# 

add_library(hunter_controller SHARED
  src/hunter_controller.cpp
  src/hunter_hardware.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

target_compile_features(hunter_controller PUBLIC cxx_std_17)
target_include_directories(hunter_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/hunter_controller>
)

target_link_libraries(hunter_controller PUBLIC hunter_controller_parameters)
ament_target_dependencies(hunter_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(hunter_controller PRIVATE "HUNTER_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface hunter_plugin.xml)

target_compile_definitions(${PROJECT_NAME} PRIVATE "HUNTER_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hunter_hardware_plugin.xml)
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

endif()

install(
  DIRECTORY include/hunter_hardware/
  DESTINATION include/hunter_hardware
)

install(
  DIRECTORY include/hunter_controller/
  DESTINATION include/hunter_controller
)
install(TARGETS hunter_controller hunter_controller_parameters
  EXPORT export_hunter_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_hunter_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
