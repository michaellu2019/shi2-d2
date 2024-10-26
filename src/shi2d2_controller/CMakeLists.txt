cmake_minimum_required(VERSION 3.8)
project(shi2d2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shi2d2_left_leg_group_ikfast_plugin REQUIRED)
# find_package(shi2d2_right_leg_group_ikfast_plugin REQUIRED)
find_package(shi2d2_leg_group_ikfast_plugin REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# include_directories(
#   ${shi2d2_left_leg_group_ikfast_plugin_INCLUDE_DIRS}
#   ${shi2d2_right_leg_group_ikfast_plugin_INCLUDE_DIRS}
# )

# add_executable(head_controller src/head_controller.cpp)
# ament_target_dependencies(head_controller rclcpp trajectory_msgs)
add_executable(leg_controller src/leg_controller.cpp)
ament_target_dependencies(leg_controller rclcpp trajectory_msgs shi2d2_leg_group_ikfast_plugin)# shi2d2_left_leg_group_ikfast_plugin)# shi2d2_right_leg_group_ikfast_plugin)
# add_executable(leg_ik_controller src/leg_ik_controller.cpp)
# ament_target_dependencies(leg_ik_controller rclcpp moveit_ros_planning_interface trajectory_msgs)

# target_link_libraries(leg_controller shi2d2_left_leg_group_ikfast_plugin::shi2d2_left_leg_group_ikfast_plugin)

install(TARGETS 
  # head_controller
  leg_controller
  # leg_ik_controller
  DESTINATION lib/${PROJECT_NAME})

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
