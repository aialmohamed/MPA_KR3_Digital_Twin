set(RESPONSEMESSAGETEST_NAME ResponseMessage_tests)
add_executable(${RESPONSEMESSAGETEST_NAME} test_ResponseMessage.cpp)
target_link_libraries(${RESPONSEMESSAGETEST_NAME} PRIVATE Catch2::Catch2WithMain 
                        ${RESPONSEMESSAGELIBARAY_NAME})