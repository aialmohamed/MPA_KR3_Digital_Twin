cmake_minimum_required(VERSION 3.22)

project(CppOpenShowVarClient VERSION 1.0.0 LANGUAGES C CXX)

set(EXCUTABLE_NAME Excutable)
set(TCPLIBRARY_NAME TcpClientLibrary)


add_subdirectory(app)
add_subdirectory(src)
add_subdirectory(include)