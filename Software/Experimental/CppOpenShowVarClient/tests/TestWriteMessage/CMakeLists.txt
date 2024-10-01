set(WRITEMESSAGETEST_NAME WriteMessage_tests)
add_executable(${WRITEMESSAGETEST_NAME} test_WriteMessage.cpp)
target_link_libraries(${WRITEMESSAGETEST_NAME} PRIVATE Catch2::Catch2WithMain 
                        ${WRTIEMESSAGELIBARAY_NAME})