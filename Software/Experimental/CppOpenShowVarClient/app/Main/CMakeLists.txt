add_executable(${EXCUTABLE_NAME} main.cpp)
target_include_directories(${EXCUTABLE_NAME} PUBLIC
 "./"
 "include/TcpClient"
 "include/Message"
 "include/WriteMessage"
 "include/ReadMessage"
 "include/ResponseMessage"
 "include/MYAXIS_type")
target_link_libraries(${EXCUTABLE_NAME} PUBLIC ${KUKACLIENTLIBARAY_NAME} ${MYAXISTYPELIBRARY_NAME})