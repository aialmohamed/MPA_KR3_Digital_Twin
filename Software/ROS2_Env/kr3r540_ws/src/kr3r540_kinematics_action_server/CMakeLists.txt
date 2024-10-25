cmake_minimum_required(VERSION 3.8)
project(kr3r540_kinematics_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(kr3r540_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)


add_library(kr3r540_kinematics_action_server SHARED
  src/kr3r540_kinematics_action_server.cpp
  src/kinematics_solver.cpp
)
target_include_directories(kr3r540_kinematics_action_server
  PUBLIC
  include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(kr3r540_kinematics_action_server
  "orocos_kdl"
  "std_msgs"
  "kdl_parser"
  "rclcpp"
  "kr3r540_msgs"
  "rclcpp_action"
  "rclcpp_components"
  "sensor_msgs"
  "trajectory_msgs"
)
rclcpp_components_register_node(kr3r540_kinematics_action_server
  PLUGIN "kr3r540_kinematics_action_server::Kr3r540KinematicsActionServer"
  EXECUTABLE kr3r540_kinematics_action_server_node
)

install(TARGETS
  kr3r540_kinematics_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
