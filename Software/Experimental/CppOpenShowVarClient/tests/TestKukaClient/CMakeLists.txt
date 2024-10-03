set(KUKACLIENTTEST_NAME KukaClient_tests)
add_executable(${KUKACLIENTTEST_NAME} test_KukaClient.cpp)
target_link_libraries(${KUKACLIENTTEST_NAME} PUBLIC  
                        gtest_main 
                        gmock_main
                        ${TCPLIBRARY_NAME}
                        ${READMESSAGELIBARAY_NAME}
                        ${RESPONSEMESSAGELIBARAY_NAME}
                        ${WRTIEMESSAGELIBARAY_NAME}
                        ${KUKACLIENTLIBARAY_NAME})