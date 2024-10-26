cmake_minimum_required(VERSION 3.8)
project(shi2d2_leg_group_ikfast_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(LAPACK REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

set(IKFAST_LIBRARY_NAME shi2d2_leg_group_ikfast_plugin)
add_library(${IKFAST_LIBRARY_NAME} src/shi2d2_left_leg_group_ikfast_moveit_plugin.cpp)
ament_target_dependencies(${IKFAST_LIBRARY_NAME}
  rclcpp
  moveit_core
  pluginlib
  tf2_kdl
  orocos_kdl
  tf2_eigen
  LAPACK
)
# suppress warnings about unused variables in OpenRave's solver code
target_compile_options(${IKFAST_LIBRARY_NAME} PRIVATE -Wno-unused-variable -Wno-unused-parameter)

ament_export_targets(${IKFAST_LIBRARY_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/${IKFAST_LIBRARY_NAME}
  DESTINATION include 
)

install(
  TARGETS ${IKFAST_LIBRARY_NAME}
  EXPORT ${IKFAST_LIBRARY_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
