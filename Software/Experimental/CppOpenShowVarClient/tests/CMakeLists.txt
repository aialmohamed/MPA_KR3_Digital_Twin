add_executable(test_KukaClient test_KukaClient.cpp test_main.cpp)
target_link_libraries(test_KukaClient PRIVATE ${KUKACLIENTLIBARAY_NAME} gtest gtest_main)
add_test(NAME test_KukaClient COMMAND test_KukaClient)