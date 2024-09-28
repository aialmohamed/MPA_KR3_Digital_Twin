// ResponseMessage.cpp

#include "ResponseMessage.hpp"
#include <iostream>
#include <iomanip>  // For formatted output
#include <sstream>  // For string formatting

// Constructor implementation
ResponseMessage::ResponseMessage()
    : message_id_(0),
      content_length_(0),
      mode_(0),
      variable_value_length_(0),
      variable_value_("") {
    // Initialize status code with zero values
    status_code_[0] = 0;
    status_code_[1] = 0;
    status_code_[2] = 0;
}

// Method to deserialize the response message from a byte vector (hex array)
void ResponseMessage::deserialize(const std::vector<uint8_t>& data) {
    // Ensure the data size is at least the minimum size required for deserialization
    if (data.size() >= MIN_RESPONSE_SIZE) {
        // Deserialize message ID (2 bytes)
        message_id_ = static_cast<uint16_t>(data[MESSAGE_ID_OFFSET]) << BYTE_SHIFT | static_cast<uint16_t>(data[MESSAGE_ID_OFFSET + 1]);

        // Deserialize content length (2 bytes)
        content_length_ = static_cast<uint16_t>(data[CONTENT_LENGTH_OFFSET]) << BYTE_SHIFT | static_cast<uint16_t>(data[CONTENT_LENGTH_OFFSET + 1]);

        // Deserialize mode (1 byte)
        mode_ = data[MODE_OFFSET];

        // Deserialize variable value length (2 bytes)
        variable_value_length_ = static_cast<uint16_t>(data[VARIABLE_VALUE_LENGTH_OFFSET]) << BYTE_SHIFT | static_cast<uint16_t>(data[VARIABLE_VALUE_LENGTH_OFFSET + 1]);

        // Calculate the offset where the variable value starts
        size_t variable_value_offset = VARIABLE_VALUE_OFFSET;

        // Deserialize variable value (N bytes)
        if (data.size() >= variable_value_offset + variable_value_length_) {
            variable_value_ = std::string(data.begin() + variable_value_offset, data.begin() + variable_value_offset + variable_value_length_);
        }

        // Calculate the offset where the status code starts
        size_t status_code_offset = variable_value_offset + variable_value_length_;

        // Deserialize status code (3 bytes)
        if (data.size() >= status_code_offset + STATUS_CODE_SIZE) {
            status_code_[0] = data[status_code_offset];
            status_code_[1] = data[status_code_offset + 1];
            status_code_[2] = data[status_code_offset + 2];
        }
    } else {
        std::cerr << "Data size is too small for deserialization." << std::endl;
    }
}

// Getters for the response attributes
uint16_t ResponseMessage::getMessageID() const {
    return message_id_;
}

uint16_t ResponseMessage::getContentLength() const {
    return content_length_;
}

uint8_t ResponseMessage::getMode() const {
    return mode_;
}

uint16_t ResponseMessage::getVariableValueLength() const {
    return variable_value_length_;
}

std::string ResponseMessage::getVariableValue() const {
    return variable_value_;
}

// Getter for status code as a formatted string
std::string ResponseMessage::getStatusCode() const {
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(status_code_[0]) << " "
       << std::setw(2) << static_cast<int>(status_code_[1]) << " "
       << std::setw(2) << static_cast<int>(status_code_[2]);
    return ss.str();
}

// Print the ResponseMessage details for debugging
void ResponseMessage::printMessageDetails() const {
    std::cout << "ResponseMessage Details:" << std::endl;
    std::cout << "Message ID: " << message_id_ << std::endl;
    std::cout << "Content Length: " << content_length_ << std::endl;
    std::cout << "Mode: " << static_cast<int>(mode_) << std::endl;
    std::cout << "Variable Value Length: " << variable_value_length_ << std::endl;
    std::cout << "Variable Value: " << variable_value_ << std::endl;
    std::cout << "Status Code: " << getStatusCode() << std::endl;
}
