#define CATCH_CONFIG_MAIN  
#include <catch2/catch.hpp>
#include "MYAXIS_type.hpp"
#include <vector>
#include <cstdint>
#include <string>
#include <limits>
#include <cmath> 

TEST_CASE("Parsing functionality", "[MYAXIS_type]") {
    std::string data = "{AXIS: A1 15.0000, A2 -70.0000, A3 70.0000, A4 10.0000, A5 -50.0000, A6 0.0}";
    MYAXIS_type axis(data);

    REQUIRE(axis.getA1() == Approx(15.0000));
    REQUIRE(axis.getA2() == Approx(-70.0000));
    REQUIRE(axis.getA3() == Approx(70.0000));
    REQUIRE(axis.getA4() == Approx(10.0000));
    REQUIRE(axis.getA5() == Approx(-50.0000));
    REQUIRE(axis.getA6() == Approx(0.0));
}

TEST_CASE("Getters and setters", "[MYAXIS_type]") {
    MYAXIS_type axis;
    axis.setA1(25.5);
    axis.setA2(-30.75);
    axis.setA3(45.25);
    axis.setA4(60.0);
    axis.setA5(-15.0);
    axis.setA6(0.0);

    REQUIRE(axis.getA1() == Approx(25.5));
    REQUIRE(axis.getA2() == Approx(-30.75));
    REQUIRE(axis.getA3() == Approx(45.25));
    REQUIRE(axis.getA4() == Approx(60.0));
    REQUIRE(axis.getA5() == Approx(-15.0));
    REQUIRE(axis.getA6() == Approx(0.0));
}

TEST_CASE("Serialization to string", "[MYAXIS_type]") {
    MYAXIS_type axis;
    axis.setA1(20.1234);
    axis.setA2(-70.5678);
    axis.setA3(70.9101);
    axis.setA4(10.1121);
    axis.setA5(-50.3141);
    axis.setA6(0.0);

    std::string expected = "{AXIS: A1 20.1234, A2 -70.5678, A3 70.9101, A4 10.1121, A5 -50.3141, A6 0.0000}";
    std::string result = axis.toString();

    REQUIRE(result == expected);
}

TEST_CASE("Serialization to bytes", "[MYAXIS_type]") {
    MYAXIS_type axis;
    axis.setA1(1.0);
    axis.setA2(2.0);
    axis.setA3(3.0);
    axis.setA4(4.0);
    axis.setA5(5.0);
    axis.setA6(6.0);

    std::vector<uint8_t> bytes = axis.Joints();

    // Each double is 8 bytes, total of 48 bytes expected
    REQUIRE(bytes.size() == 48);

    // Reconstruct the doubles from the bytes and compare
    double values[6];
    for (int i = 0; i < 6; ++i) {
        std::memcpy(&values[i], &bytes[i * sizeof(double)], sizeof(double));
    }

    REQUIRE(values[0] == Approx(1.0));
    REQUIRE(values[1] == Approx(2.0));
    REQUIRE(values[2] == Approx(3.0));
    REQUIRE(values[3] == Approx(4.0));
    REQUIRE(values[4] == Approx(5.0));
    REQUIRE(values[5] == Approx(6.0));
}

TEST_CASE("Parsing with invalid input", "[MYAXIS_type]") {
    std::string data = "{AXIS: A1 abc, A2 -70.0000, A3 70.0000}";
    MYAXIS_type axis(data);

    // Since "abc" is invalid for A1, it should remain at its default value (e.g., 0.0)
    REQUIRE(axis.getA1() == Approx(0.0));
    REQUIRE(axis.getA2() == Approx(-70.0000));
    REQUIRE(axis.getA3() == Approx(70.0000));
    REQUIRE(axis.getA4() == Approx(0.0)); // Not provided, should be default
    REQUIRE(axis.getA5() == Approx(0.0)); // Not provided, should be default
    REQUIRE(axis.getA6() == Approx(0.0)); // Not provided, should be default
}


TEST_CASE("Setters enforce axis limits", "[MYAXIS_type]") {
    MYAXIS_type axis;

    // Test setting values within limits
    REQUIRE_NOTHROW(axis.setA1(0.0));
    REQUIRE_NOTHROW(axis.setA2(-100.0));
    REQUIRE_NOTHROW(axis.setA3(170.0));
    REQUIRE_NOTHROW(axis.setA4(-120.0));
    REQUIRE_NOTHROW(axis.setA5(170.0));
    REQUIRE_NOTHROW(axis.setA6(360.0));

    // Test setting values beyond limits
    REQUIRE_THROWS_AS(axis.setA1(180.0), std::out_of_range);
    REQUIRE_THROWS_AS(axis.setA2(-130.0), std::out_of_range);
    REQUIRE_THROWS_AS(axis.setA3(180.0), std::out_of_range);
    REQUIRE_THROWS_AS(axis.setA4(130.0), std::out_of_range);
    REQUIRE_THROWS_AS(axis.setA5(-180.0), std::out_of_range);
    REQUIRE_THROWS_AS(axis.setA6(370.0), std::out_of_range);
}

TEST_CASE("Parsing enforces axis limits", "[MYAXIS_type]") {
    // Valid data
    std::string valid_data = "{AXIS: A1 15.0, A2 -70.0, A3 70.0, A4 10.0, A5 -50.0, A6 0.0}";
    REQUIRE_NOTHROW(MYAXIS_type(valid_data));

    // Invalid data with A1 out of range
    std::string invalid_data_A1 = "{AXIS: A1 180.0, A2 -70.0, A3 70.0, A4 10.0, A5 -50.0, A6 0.0}";
    REQUIRE_THROWS_AS(MYAXIS_type(invalid_data_A1), std::out_of_range);

    // Invalid data with multiple axes out of range
    std::string invalid_data_multiple = "{AXIS: A1 15.0, A2 -130.0, A3 70.0, A4 10.0, A5 -50.0, A6 370.0}";
    REQUIRE_THROWS_AS(MYAXIS_type(invalid_data_multiple), std::out_of_range);
}