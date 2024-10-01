// ReadMessage.cpp

#include "ReadMessage.hpp"
#include <iostream>


ReadMessage::ReadMessage(uint16_t message_id, const std::string& variable_name)
    : Message(message_id, 0, 0),
      variable_name_length_(variable_name.length()),
      variable_name_(variable_name) {


    content_length_ = 1 + 2 + variable_name_length_;
}


std::vector<uint8_t> ReadMessage::serialize() const {
    std::vector<uint8_t> data;

    data.push_back((message_id_ >> 8) & 0xFF); 
    data.push_back(message_id_ & 0xFF);


    data.push_back((content_length_ >> 8) & 0xFF);
    data.push_back(content_length_ & 0xFF);


    data.push_back(mode_);
    data.push_back((variable_name_length_ >> 8) & 0xFF); 
    data.push_back(variable_name_length_ & 0xFF); 
    data.insert(data.end(), variable_name_.begin(), variable_name_.end());

    return data;
}


void ReadMessage::deserialize(const std::vector<uint8_t>& data) {

    if (data.size() < 7) {
        message_id_ = 0;
        content_length_ = 0;
        mode_ = 0;
        variable_name_length_ = 0;
        variable_name_ = "";
        return;
    }

    message_id_ = (data[0] << 8) | data[1];
    content_length_ = (data[2] << 8) | data[3];
    mode_ = data[4];
    variable_name_length_ = data[6] | (data[5] << 8);

    if (data.size() < 7 + variable_name_length_) {
        return;
    }

    variable_name_ = std::string(data.begin() + 7, data.begin() + 7 + variable_name_length_);
}




std::string ReadMessage::getVariableName() const {
    return variable_name_;
}

void ReadMessage::setVariableName(const std::string& variable_name) {
    variable_name_ = variable_name;
    variable_name_length_ = variable_name.length();
    content_length_ = 1 + 2 + variable_name_length_;
}

uint16_t ReadMessage::getVariableNameLength() const {
    return variable_name_length_;
}
void ReadMessage::printMessageDetails() const {
    std::cout << "ReadMessage Details:" << std::endl;
    std::cout << "Message ID: " << message_id_ << std::endl;
    std::cout << "Content Length: " << content_length_ << std::endl;
    std::cout << "Mode: " << static_cast<int>(mode_) << std::endl;
    std::cout << "Variable Name Length: " << variable_name_length_ << std::endl;
    std::cout << "Variable Name: " << variable_name_ << std::endl;
}
