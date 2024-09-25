cmake_minimum_required(VERSION 3.22)

project(CppOpenShowVarClient VERSION 1.0.0 LANGUAGES C CXX)

set(EXCUTABLE_NAME Excutable)
set(TCPLIBRARY_NAME TcpClientLibrary)

# Include ExternalProject for fetching libraries via git
include(ExternalProject)

# Include custom Boost installation if needed
set(BOOST_ROOT "/home/robolab/boost_1_82_install")
set(BOOST_LIBRARIES 
Boost::system
Boost::thread)
find_package(Boost 1.82 REQUIRED COMPONENTS system thread)

add_subdirectory(src)
add_subdirectory(include)
add_subdirectory(app)
