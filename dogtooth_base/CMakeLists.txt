cmake_minimum_required(VERSION 3.8)
project(dogtooth_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  # hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
# find_package(ament_cmake REQUIRED)
# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# add_library(
#   SHARED
#   hardware/dogtooth_system.cpp
# )

# target_compile_features(dogtooth_base PUBLIC cxx_std_17)
# target_include_directories(dogtooth_base PUBLIC
# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
# $<INSTALL_INTERFACE:include/dogtooth_base>
# )



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# ament_target_dependencies(
#   dogtooth_base PUBLIC
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(${PROJECT_NAME} PRIVATE "DOGTOOTH_BASE_BUILDING_DLL")

# Export hardware plugins
# pluginlib_export_plugin_description_file(hardware_interface dogtooth_base.xml)

# install(
  # DIRECTORY hardware/include/
  # DESTINATION include/dogtooth_base
# )


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
# ament_export_targets(export_ros2_control_demo_example_2 HAS_LIBRARY_TARGET)
# ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
# ament_package()


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
