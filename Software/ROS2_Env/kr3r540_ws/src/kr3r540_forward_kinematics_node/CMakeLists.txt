cmake_minimum_required(VERSION 3.8)
project(kr3r540_forward_kinematics_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kr3r540_msgs REQUIRED)
find_package(std_msgs REQUIRED)



# Add library for the node
add_library(kr3r540_forward_kinematics_node SHARED
  src/kr3r540_forward_kinematics_node.cpp
  src/forward_kinematics_solver.cpp
)

# Include directories for headers
target_include_directories(kr3r540_forward_kinematics_node
  PUBLIC
  include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add dependencies
ament_target_dependencies(kr3r540_forward_kinematics_node
  rclcpp
  rclcpp_components
  kdl_parser
  kr3r540_msgs
  orocos_kdl
  sensor_msgs
  std_msgs
)

# Register the node
rclcpp_components_register_node(kr3r540_forward_kinematics_node
  PLUGIN "forward_kinematics_node::ForwardKinematicsNode"
  EXECUTABLE kr3r540_forward_kinematics_node_exec
)

# Install the library and headers
install(TARGETS
  kr3r540_forward_kinematics_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
