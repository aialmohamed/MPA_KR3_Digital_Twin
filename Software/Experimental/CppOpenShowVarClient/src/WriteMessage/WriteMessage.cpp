// WriteMessage.cpp

#include "WriteMessage.hpp"
#include <iostream>

// Constructor implementation
WriteMessage::WriteMessage(uint16_t message_id, const std::string& variable_name, const std::string& variable_value)
    : Message(message_id, 0, 1),  // Initialize content_length_ as 0 and mode as 1 for write
      variable_name_length_(variable_name.length()),
      variable_name_(variable_name),
      variable_value_length_(variable_value.length()),
      variable_value_(variable_value) {

    // Set the content length after knowing the lengths of the variable name and value
    content_length_ = 1 + 2 + variable_name_length_ + 2 + variable_value_length_;
}

// Method to serialize the write message into a byte vector
std::vector<uint8_t> WriteMessage::serialize() const {
    std::vector<uint8_t> data;

    // Serialize message ID
    data.push_back((message_id_ >> 8) & 0xFF); // High byte
    data.push_back(message_id_ & 0xFF);        // Low byte

    // Serialize content length (ensure it's up to date)
    data.push_back((content_length_ >> 8) & 0xFF); // High byte
    data.push_back(content_length_ & 0xFF);        // Low byte

    // Serialize mode
    data.push_back(mode_);

    // Serialize variable name length
    data.push_back((variable_name_length_ >> 8) & 0xFF); // High byte
    data.push_back(variable_name_length_ & 0xFF);        // Low byte

    // Serialize variable name
    data.insert(data.end(), variable_name_.begin(), variable_name_.end());

    // Serialize variable value length
    data.push_back((variable_value_length_ >> 8) & 0xFF); // High byte
    data.push_back(variable_value_length_ & 0xFF);        // Low byte

    // Serialize variable value
    data.insert(data.end(), variable_value_.begin(), variable_value_.end());

    return data;
}

// Method to deserialize the write message from a byte vector
void WriteMessage::deserialize(const std::vector<uint8_t>& data) {
    Message::deserialize(data); // Deserialize base class fields

    if (data.size() >= 7) { // Minimum size for deserialization
        // Get variable name length (2 bytes)
        variable_name_length_ = data[5] | (data[6] << 8);

        if (data.size() >= 7 + variable_name_length_ + 2) {
            // Get variable name
            variable_name_ = std::string(data.begin() + 7, data.begin() + 7 + variable_name_length_);

            // Get variable value length (2 bytes)
            variable_value_length_ = data[7 + variable_name_length_] | (data[8 + variable_name_length_] << 8);

            if (data.size() >= 9 + variable_name_length_ + variable_value_length_) {
                // Get variable value
                variable_value_ = std::string(data.begin() + 9 + variable_name_length_, data.begin() + 9 + variable_name_length_ + variable_value_length_);
            }
        }
    }
}

// Method that replicates the getWriteCommand logic from Java code
std::vector<uint8_t> WriteMessage::getWriteCommand() const {
    // 1. Convert variable name and value into byte arrays
    std::vector<uint8_t> cmd(variable_name_.begin(), variable_name_.end());
    std::vector<uint8_t> value(variable_value_.begin(), variable_value_.end());

    // 2. Create header and block vectors
    std::vector<uint8_t> header;
    std::vector<uint8_t> block;

    // 3. Add mode (1 byte)
    block.push_back(static_cast<uint8_t>(mode_));  // Mode: 1 for write

    // 4. Add variable name length (2 bytes)
    block.push_back(static_cast<uint8_t>((variable_name_length_ >> 8) & 0xFF)); // High byte
    block.push_back(static_cast<uint8_t>(variable_name_length_ & 0xFF));        // Low byte

    // 5. Add variable name (N bytes)
    block.insert(block.end(), cmd.begin(), cmd.end());

    // 6. Add variable value length (2 bytes)
    block.push_back(static_cast<uint8_t>((variable_value_length_ >> 8) & 0xFF)); // High byte
    block.push_back(static_cast<uint8_t>(variable_value_length_ & 0xFF));        // Low byte

    // 7. Add variable value (M bytes)
    block.insert(block.end(), value.begin(), value.end());

    // 8. Calculate the content length and update block length
    uint16_t content_length = static_cast<uint16_t>(block.size());

    // 9. Add header information
    header.push_back(static_cast<uint8_t>((message_id_ >> 8) & 0xFF)); // Message ID high byte
    header.push_back(static_cast<uint8_t>(message_id_ & 0xFF));        // Message ID low byte

    header.push_back(static_cast<uint8_t>((content_length >> 8) & 0xFF)); // Content length high byte
    header.push_back(static_cast<uint8_t>(content_length & 0xFF));        // Content length low byte

    // 10. Combine header and block into the final message
    block.insert(block.begin(), header.begin(), header.end());

    return block;  // Return the complete message as a vector of bytes
}

// Getters and Setters
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

// Print the WriteMessage details for debugging
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
