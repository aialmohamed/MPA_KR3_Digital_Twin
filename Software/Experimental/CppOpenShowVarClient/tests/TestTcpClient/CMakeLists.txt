enable_testing()

set(TCPCLIENTTEST_NAME TcpClient_tests)
add_executable(${TCPCLIENTTEST_NAME} test_TcpClient.cpp)
target_link_libraries(${TCPCLIENTTEST_NAME} PUBLIC  
                        gtest_main 
                        gmock_main 
                        ${TCPLIBRARY_NAME})