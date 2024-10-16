#include "MYAXIS_type.hpp"
#include <cstring>
#include <sstream>
#include <stdexcept>

// Default Constructor
MYAXIS_type::MYAXIS_type()
    : A1_(0), A2_(0), A3_(0), A4_(0), A5_(0), A6_(0), Joints_robot_("") {}

// Constructor with data string
MYAXIS_type::MYAXIS_type(const std::string& data_string) {
    parseFromString(data_string);
}

// Setters with limit checks
void MYAXIS_type::setA1(double A1) {
    if (A1 < A1_MIN || A1 > A1_MAX) {
        throw std::out_of_range("A1 value out of range");
    }
    A1_ = A1;
}

void MYAXIS_type::setA2(double A2) {
    if (A2 < A2_MIN || A2 > A2_MAX) {
        throw std::out_of_range("A2 value out of range");
    }
    A2_ = A2;
}

void MYAXIS_type::setA3(double A3) {
    if (A3 < A3_MIN || A3 > A3_MAX) {
        throw std::out_of_range("A3 value out of range");
    }
    A3_ = A3;
}

void MYAXIS_type::setA4(double A4) {
    if (A4 < A4_MIN || A4 > A4_MAX) {
        throw std::out_of_range("A4 value out of range");
    }
    A4_ = A4;
}

void MYAXIS_type::setA5(double A5) {
    if (A5 < A5_MIN || A5 > A5_MAX) {
        throw std::out_of_range("A5 value out of range");
    }
    A5_ = A5;
}

void MYAXIS_type::setA6(double A6) {
    if (A6 < A6_MIN || A6 > A6_MAX) {
        throw std::out_of_range("A6 value out of range");
    }
    A6_ = A6;
}

void MYAXIS_type::setJoints_robot(const std::string& Joints_robot) { Joints_robot_ = Joints_robot; }

// Getters
double MYAXIS_type::getA1() const { return A1_; }
double MYAXIS_type::getA2() const { return A2_; }
double MYAXIS_type::getA3() const { return A3_; }
double MYAXIS_type::getA4() const { return A4_; }
double MYAXIS_type::getA5() const { return A5_; }
double MYAXIS_type::getA6() const { return A6_; }
std::string MYAXIS_type::getJoints_robot() const { return Joints_robot_; }

// Parsing from string
void MYAXIS_type::parseFromString(const std::string& data_string) {
    std::regex rgx("A([1-6])\\s*(-?[0-9]+\\.?[0-9]*)");
    std::smatch match;
    std::string::const_iterator searchStart(data_string.cbegin());
    while (std::regex_search(searchStart, data_string.cend(), match, rgx)) {
        int axis_number = std::stoi(match[1]);
        double value = std::stod(match[2]);

        switch(axis_number) {
            case 1: setA1(value); break;
            case 2: setA2(value); break;
            case 3: setA3(value); break;
            case 4: setA4(value); break;
            case 5: setA5(value); break;
            case 6: setA6(value); break;
            default:
                throw std::invalid_argument("Invalid axis number");
        }

        searchStart = match.suffix().first;
    }
}

// Serialization to bytes
std::vector<uint8_t> MYAXIS_type::Joints() const {
    std::vector<uint8_t> joints;
    const double* axes[] = {&A1_, &A2_, &A3_, &A4_, &A5_, &A6_};
    for (const double* axis : axes) {
        uint8_t buffer[sizeof(double)];
        memcpy(buffer, axis, sizeof(double));
        joints.insert(joints.end(), buffer, buffer + sizeof(double));
    }
    return joints;
}

// Serialization to string
std::string MYAXIS_type::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << "{AXIS: "
        << "A1 " << A1_ << ", "
        << "A2 " << A2_ << ", "
        << "A3 " << A3_ << ", "
        << "A4 " << A4_ << ", "
        << "A5 " << A5_ << ", "
        << "A6 " << A6_ << "}";
    return oss.str();
}
