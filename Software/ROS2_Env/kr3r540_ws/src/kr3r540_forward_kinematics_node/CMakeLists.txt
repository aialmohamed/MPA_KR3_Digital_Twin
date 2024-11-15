cmake_minimum_required(VERSION 3.8)
project(kr3r540_forward_kinematics_node)

# Compiler flags for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kr3r540_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Add solver library
add_library(kr3r540_forward_kinematics_solver_library SHARED src/forward_kinematics_solver.cpp)
ament_target_dependencies(kr3r540_forward_kinematics_solver_library
  rclcpp
  kdl_parser
  orocos_kdl
  sensor_msgs
  kr3r540_msgs
  std_msgs
)

install(
  TARGETS kr3r540_forward_kinematics_solver_library
  EXPORT kr3r540_forward_kinematics_solver_library 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Add node library
add_library(kr3r540_forward_kinematics_node_library SHARED src/kr3r540_forward_kinematics_node.cpp)
target_link_libraries(kr3r540_forward_kinematics_node_library
  kr3r540_forward_kinematics_solver_library
)
ament_target_dependencies(kr3r540_forward_kinematics_node_library
  rclcpp
  kdl_parser
  orocos_kdl
  sensor_msgs
  kr3r540_msgs
  std_msgs
)

install(
  TARGETS kr3r540_forward_kinematics_node_library
  EXPORT kr3r540_forward_kinematics_node_library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Add executable
add_executable(kr3r540_forward_kinematics_node src/main.cpp)
target_link_libraries(kr3r540_forward_kinematics_node
  kr3r540_forward_kinematics_node_library
)
ament_target_dependencies(kr3r540_forward_kinematics_node 
  rclcpp
)

install(TARGETS
  kr3r540_forward_kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
