// WriteMessage.cpp

#include "WriteMessage/WriteMessage.hpp"
#include <iostream>

WriteMessage::WriteMessage(uint16_t message_id, const std::string& variable_name, const std::string& variable_value)
    : Message(message_id, 0, 1),
      variable_name_length_(variable_name.length()),
      variable_name_(variable_name),
      variable_value_length_(variable_value.length()),
      variable_value_(variable_value) {
    content_length_ = 1 + 2 + variable_name_length_ + 2 + variable_value_length_;
}

std::vector<uint8_t> WriteMessage::serialize() const {
    std::vector<uint8_t> data;
    data.push_back((message_id_ >> 8) & 0xFF); 
    data.push_back(message_id_ & 0xFF);
    data.push_back((content_length_ >> 8) & 0xFF);
    data.push_back(content_length_ & 0xFF);

    data.push_back(mode_);
    data.push_back((variable_name_length_ >> 8) & 0xFF);
    data.push_back(variable_name_length_ & 0xFF);
    data.insert(data.end(), variable_name_.begin(), variable_name_.end());
    data.push_back((variable_value_length_ >> 8) & 0xFF); 
    data.push_back(variable_value_length_ & 0xFF);
    data.insert(data.end(), variable_value_.begin(), variable_value_.end());

    return data;
}

void WriteMessage::deserialize(const std::vector<uint8_t>& data) {
    if (data.size() < 7) {
        message_id_ = 0;
        content_length_ = 0;
        mode_ = 0;
        variable_name_length_ = 0;
        variable_name_ = "";
        variable_value_length_ = 0;
        variable_value_ = "";
        return;
    }
    message_id_ = (data[0] << 8) | data[1];
    content_length_ = (data[2] << 8) | data[3];
    mode_ = data[4];
    variable_name_length_ = (data[5] << 8) | data[6]; 
    if (data.size() < 7 + variable_name_length_ + 2) {
        variable_name_ = ""; 
        variable_value_ = "";
        return;
    }
    variable_name_ = std::string(data.begin() + 7, data.begin() + 7 + variable_name_length_);
    variable_value_length_ = (data[7 + variable_name_length_] << 8) | data[8 + variable_name_length_];
    if (data.size() < 9 + variable_name_length_ + variable_value_length_) {
        variable_value_ = "";
        return;
    }
    variable_value_ = std::string(data.begin() + 9 + variable_name_length_, data.begin() + 9 + variable_name_length_ + variable_value_length_);
}


std::string WriteMessage::getVariableName() const {
    return variable_name_;
}

void WriteMessage::setVariableName(const std::string& variable_name) {
    variable_name_ = variable_name;
    variable_name_length_ = variable_name.length();
    content_length_ = 1 + 2 + variable_name_length_ + 2 + variable_value_length_; // Update content length
}

std::string WriteMessage::getVariableValue() const {
    return variable_value_;
}

void WriteMessage::setVariableValue(const std::string& variable_value) {
    variable_value_ = variable_value;
    variable_value_length_ = variable_value.length();
    content_length_ = 1 + 2 + variable_name_length_ + 2 + variable_value_length_; // Update content length
}

uint16_t WriteMessage::getVariableNameLength() const {
    return variable_name_length_;
}
uint16_t WriteMessage::getVariableValueLength() const {
    return variable_value_length_;
}
void WriteMessage::printMessageDetails() const {
    std::cout << "WriteMessage Details:" << std::endl;
    std::cout << "Message ID: " << message_id_ << std::endl;
    std::cout << "Content Length: " << content_length_ << std::endl;
    std::cout << "Mode: " << static_cast<int>(mode_) << std::endl;
    std::cout << "Variable Name Length: " << variable_name_length_ << std::endl;
    std::cout << "Variable Name: " << variable_name_ << std::endl;
    std::cout << "Variable Value Length: " << variable_value_length_ << std::endl;
    std::cout << "Variable Value: " << variable_value_ << std::endl;
}
