// ResponseMessage.cpp

#include "ResponseMessage.hpp"
#include <iostream>
#include <iomanip>  
#include <sstream> 

ResponseMessage::ResponseMessage()
    : message_id_(0),
      content_length_(0),
      mode_(0),
      variable_value_length_(0),
      variable_value_("") {

    status_code_[0] = 0;
    status_code_[1] = 0;
    status_code_[2] = 0;
}


void ResponseMessage::deserialize(const std::vector<uint8_t>& data) {
    if (data.size() < MESSAGE_ID_SIZE + CONTENT_LENGTH_SIZE + MODE_SIZE + VARIABLE_VALUE_LENGTH_SIZE) {
        std::cerr << "Data size is too small for deserialization." << std::endl;
        return;
    }
    message_id_ = static_cast<uint16_t>(data[MESSAGE_ID_OFFSET]) << BYTE_SHIFT | static_cast<uint16_t>(data[MESSAGE_ID_OFFSET + 1]);
    if (data.size() < CONTENT_LENGTH_OFFSET + CONTENT_LENGTH_SIZE) {
        std::cerr << "Data size is too small for deserializing content length." << std::endl;
        return;
    }
    content_length_ = static_cast<uint16_t>(data[CONTENT_LENGTH_OFFSET]) << BYTE_SHIFT | static_cast<uint16_t>(data[CONTENT_LENGTH_OFFSET + 1]);
    if (data.size() < MODE_OFFSET + MODE_SIZE) {
        std::cerr << "Data size is too small for deserializing mode." << std::endl;
        return;
    }
    mode_ = data[MODE_OFFSET];
    if (data.size() < VARIABLE_VALUE_LENGTH_OFFSET + VARIABLE_VALUE_LENGTH_SIZE) {
        std::cerr << "Data size is too small for deserializing variable value length." << std::endl;
        return;
    }
    variable_value_length_ = static_cast<uint16_t>(data[VARIABLE_VALUE_LENGTH_OFFSET]) << BYTE_SHIFT | static_cast<uint16_t>(data[VARIABLE_VALUE_LENGTH_OFFSET + 1]);
    size_t variable_value_offset = VARIABLE_VALUE_OFFSET;
    if (data.size() >= variable_value_offset + variable_value_length_) {
        variable_value_ = std::string(data.begin() + variable_value_offset, data.begin() + variable_value_offset + variable_value_length_);
    } else {
        variable_value_ = "";
    }
    size_t status_code_offset = variable_value_offset + variable_value_length_;
    if (data.size() >= status_code_offset + STATUS_CODE_SIZE) {
        status_code_[0] = data[status_code_offset];
        status_code_[1] = data[status_code_offset + 1];
        status_code_[2] = data[status_code_offset + 2];
    } else {
        status_code_[0] = 0;
        status_code_[1] = 0;
        status_code_[2] = 0;
    }
}

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

std::string ResponseMessage::getStatusCode() const {
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(status_code_[0]) << " "
       << std::setw(2) << static_cast<int>(status_code_[1]) << " "
       << std::setw(2) << static_cast<int>(status_code_[2]);
    return ss.str();
}

void ResponseMessage::printMessageDetails() const {
    std::cout << "ResponseMessage Details:" << std::endl;
    std::cout << "Message ID: " << message_id_ << std::endl;
    std::cout << "Content Length: " << content_length_ << std::endl;
    std::cout << "Mode: " << static_cast<int>(mode_) << std::endl;
    std::cout << "Variable Value Length: " << variable_value_length_ << std::endl;
    std::cout << "Variable Value: " << variable_value_ << std::endl;
    std::cout << "Status Code: " << getStatusCode() << std::endl;
}
