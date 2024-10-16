set(MYAXISTYPETEST_NAME MYAXISType_tests)
add_executable(${MYAXISTYPETEST_NAME} test_MYAXIS_Type.cpp)
target_link_libraries(${MYAXISTYPETEST_NAME} PRIVATE Catch2::Catch2WithMain 
                        ${MYAXISTYPELIBRARY_NAME})