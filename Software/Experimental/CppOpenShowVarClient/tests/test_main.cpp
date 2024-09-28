// test_main.cpp

#include <gtest/gtest.h>

int main(int argc, char **argv) {
    // Initialize GoogleTest
    ::testing::InitGoogleTest(&argc, argv);

    // Run all tests
    return RUN_ALL_TESTS();
}