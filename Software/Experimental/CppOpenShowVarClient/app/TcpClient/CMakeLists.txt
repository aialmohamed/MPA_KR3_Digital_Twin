add_executable(${EXCUTABLE_NAME} main.cpp)
target_include_directories(${EXCUTABLE_NAME} PUBLIC
 "./"
 "include/TcpClient")
target_link_libraries(${EXCUTABLE_NAME} PUBLIC
${TCPLIBRARY_NAME}
${BOOST_LIBRARIES})