set(READMESSAGETEST_NAME ReadMessage_tests)
add_executable(${READMESSAGETEST_NAME} test_ReadMessage.cpp)
target_link_libraries(${READMESSAGETEST_NAME} PRIVATE Catch2::Catch2WithMain 
                        ${READMESSAGELIBARAY_NAME})