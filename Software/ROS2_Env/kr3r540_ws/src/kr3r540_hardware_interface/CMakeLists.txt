cmake_minimum_required(VERSION 3.8)
project(kr3r540_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(angles REQUIRED)

# Include custom Boost installation if needed
set(BOOST_ROOT "/home/robolab/boost_1_82_install")
set(BOOST_LIBRARIES 
    Boost::system
    Boost::thread)
find_package(Boost 1.82 REQUIRED COMPONENTS system thread)

# Set up your static libraries with POSITION_INDEPENDENT_CODE ON

# 1. TcpClientLibrary
set(TCPLIBRARY_NAME TcpClientLibrary)
add_library(${TCPLIBRARY_NAME} STATIC
    src/TcpClient/TcpClient.cpp)
target_include_directories(${TCPLIBRARY_NAME} PRIVATE include)
target_link_libraries(${TCPLIBRARY_NAME} ${BOOST_LIBRARIES})
# Enable position-independent code
set_target_properties(${TCPLIBRARY_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 2. messageLibrary
set(MESSAGELIBRARY_NAME messageLibrary)
add_library(${MESSAGELIBRARY_NAME} STATIC
    src/Message/Message.cpp)
target_include_directories(${MESSAGELIBRARY_NAME} PUBLIC include)
# Enable position-independent code
set_target_properties(${MESSAGELIBRARY_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 3. writeMessageLibrary
set(WRITEMESSAGELIBRARY_NAME writeMessageLibrary)
add_library(${WRITEMESSAGELIBRARY_NAME} STATIC
    src/WriteMessage/WriteMessage.cpp)
target_include_directories(${WRITEMESSAGELIBRARY_NAME} PUBLIC include)
target_link_libraries(${WRITEMESSAGELIBRARY_NAME} ${MESSAGELIBRARY_NAME})
# Enable position-independent code
set_target_properties(${WRITEMESSAGELIBRARY_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 4. readMessageLibrary
set(READMESSAGELIBRARY_NAME readMessageLibrary)
add_library(${READMESSAGELIBRARY_NAME} STATIC
    src/ReadMessage/ReadMessage.cpp)
target_include_directories(${READMESSAGELIBRARY_NAME} PUBLIC include)
target_link_libraries(${READMESSAGELIBRARY_NAME} ${MESSAGELIBRARY_NAME})
# Enable position-independent code
set_target_properties(${READMESSAGELIBRARY_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 5. responseMessageLibrary
set(RESPONSEMESSAGELIBRARY_NAME responseMessageLibrary)
add_library(${RESPONSEMESSAGELIBRARY_NAME} STATIC
    src/ResponseMessage/ResponseMessage.cpp)
target_include_directories(${RESPONSEMESSAGELIBRARY_NAME} PUBLIC include)
target_link_libraries(${RESPONSEMESSAGELIBRARY_NAME} ${MESSAGELIBRARY_NAME})
# Enable position-independent code
set_target_properties(${RESPONSEMESSAGELIBRARY_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 6. kukaClientMessageLibrary
set(KUKACLIENTLIBRARY_NAME kukaClientMessageLibrary)
add_library(${KUKACLIENTLIBRARY_NAME} STATIC
    src/KukaClient/KukaClient.cpp)
target_include_directories(${KUKACLIENTLIBRARY_NAME} PUBLIC include)
target_link_libraries(${KUKACLIENTLIBRARY_NAME} 
    ${BOOST_LIBRARIES}
    ${TCPLIBRARY_NAME}
    ${MESSAGELIBRARY_NAME}
    ${WRITEMESSAGELIBRARY_NAME}
    ${READMESSAGELIBRARY_NAME}
    ${RESPONSEMESSAGELIBRARY_NAME})
# Enable position-independent code
set_target_properties(${KUKACLIENTLIBRARY_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 7. myAxisTypeLibrary
set(MYAXISTYPELIBRARY_NAME myAxisTypeLibrary)
add_library(${MYAXISTYPELIBRARY_NAME} STATIC
    src/MYAXIS_type/MYAXIS_type.cpp)
target_include_directories(${MYAXISTYPELIBRARY_NAME} PUBLIC include)
# Enable position-independent code
set_target_properties(${MYAXISTYPELIBRARY_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Now, build your shared library (kr3r540_hardware_interface)
add_library(${PROJECT_NAME} SHARED
    src/kr3r540_interface.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
    include
    /home/robolab/boost_1_82_install/include
    $<BUILD_INTERFACE:${ament_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    ${BOOST_LIBRARIES}
    ${KUKACLIENTLIBRARY_NAME}
    ${MYAXISTYPELIBRARY_NAME})

ament_target_dependencies(${PROJECT_NAME}
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    angles
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_include_directories(
  include
  )
ament_export_libraries(
  ${PROJECT_NAME}
  )
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  angles
  )

# Export pluginlib plugins
pluginlib_export_plugin_description_file(hardware_interface kr3r540_hardware_interface_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
