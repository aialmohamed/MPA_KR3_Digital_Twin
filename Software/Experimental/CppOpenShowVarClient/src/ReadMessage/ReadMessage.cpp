// ReadMessage.cpp

#include "ReadMessage.hpp"
#include <iostream>

// Constructor implementation
ReadMessage::ReadMessage(uint16_t message_id, const std::string& variable_name)
    : Message(message_id, 0, 0),  // Initialize content_length_ as 0 and mode as 0 for read
      variable_name_length_(variable_name.length()),
      variable_name_(variable_name) {

    // Set the content length after knowing the length of the variable name
    content_length_ = 1 + 2 + variable_name_length_; // 1 byte for mode + 2 bytes for variable name length + variable name length
}

// Method to serialize the read message into a byte vector
std::vector<uint8_t> ReadMessage::serialize() const {
    std::vector<uint8_t> data;

    // Serialize message ID
    data.push_back((message_id_ >> 8) & 0xFF); // High byte
    data.push_back(message_id_ & 0xFF);        // Low byte

    // Serialize content length (ensure it's up to date)
    data.push_back((content_length_ >> 8) & 0xFF); // High byte
    data.push_back(content_length_ & 0xFF);        // Low byte

    // Serialize mode (0 for read)
    data.push_back(mode_);

    // Serialize variable name length
    data.push_back((variable_name_length_ >> 8) & 0xFF); // High byte
    data.push_back(variable_name_length_ & 0xFF);        // Low byte

    // Serialize variable name
    data.insert(data.end(), variable_name_.begin(), variable_name_.end());

    return data;
}

// Method to deserialize the read message from a byte vector
void ReadMessage::deserialize(const std::vector<uint8_t>& data) {
    Message::deserialize(data); // Deserialize base class fields

    if (data.size() >= 7) { // Minimum size for deserialization
        // Get variable name length (2 bytes)
        variable_name_length_ = data[5] | (data[6] << 8);

        if (data.size() >= 7 + variable_name_length_) {
            // Get variable name
            variable_name_ = std::string(data.begin() + 7, data.begin() + 7 + variable_name_length_);
        }
    }
}

// Getters and Setters
std::string ReadMessage::getVariableName() const {
    return variable_name_;
}

void ReadMessage::setVariableName(const std::string& variable_name) {
    variable_name_ = variable_name;
    variable_name_length_ = variable_name.length();
    content_length_ = 1 + 2 + variable_name_length_; // Update content length
}

// Print the ReadMessage details for debugging
void ReadMessage::printMessageDetails() const {
    std::cout << "ReadMessage Details:" << std::endl;
    std::cout << "Message ID: " << message_id_ << std::endl;
    std::cout << "Content Length: " << content_length_ << std::endl;
    std::cout << "Mode: " << static_cast<int>(mode_) << std::endl;
    std::cout << "Variable Name Length: " << variable_name_length_ << std::endl;
    std::cout << "Variable Name: " << variable_name_ << std::endl;
}
