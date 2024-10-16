#ifndef MYAXIS_TYPE_HPP
#define MYAXIS_TYPE_HPP

#include <vector>
#include <cstdint>
#include <string>
#include <regex>
#include <iomanip>
#include <stdexcept>

class MYAXIS_type {
protected:
    double A1_;
    double A2_;
    double A3_;
    double A4_;
    double A5_;
    double A6_;
    std::string Joints_robot_;


public:


    // Axis limits
    static constexpr double A1_MIN = -27.0;
    static constexpr double A1_MAX = 60.0;
    static constexpr double A2_MIN = -110.0;
    static constexpr double A2_MAX = 50.0;
    static constexpr double A3_MIN = 25.0;
    static constexpr double A3_MAX = 155.0;
    static constexpr double A4_MIN = -175.0;
    static constexpr double A4_MAX = 175.0;
    static constexpr double A5_MIN = -120.0;
    static constexpr double A5_MAX = 120.0;
    static constexpr double A6_MIN = -350.0;
    static constexpr double A6_MAX = 350.0;
    MYAXIS_type();
    MYAXIS_type(const std::string& data_string);

    // Setters
    void setA1(double A1);
    void setA2(double A2);
    void setA3(double A3);
    void setA4(double A4);
    void setA5(double A5);
    void setA6(double A6);
    void setJoints_robot(const std::string& Joints_robot);

    // Getters
    double getA1() const;
    double getA2() const;
    double getA3() const;
    double getA4() const;
    double getA5() const;
    double getA6() const;
    std::string getJoints_robot() const;

    // Other methods
    std::vector<uint8_t> Joints() const;
    void parseFromString(const std::string& data_string);
    std::string toString() const;
};

#endif
