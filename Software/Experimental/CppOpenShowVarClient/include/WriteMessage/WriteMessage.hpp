// WriteMessage.hpp

#ifndef WRITEMESSAGE_HPP
#define WRITEMESSAGE_HPP

#include "Message.hpp"
#include <string>
#include <vector>
#include <cstdint>

class WriteMessage : public Message {
private:
    uint16_t variable_name_length_;  // Length of the variable name
    std::string variable_name_;      // Variable name to be written
    uint16_t variable_value_length_; // Length of the variable value
    std::string variable_value_;     // Value to be written to the variable

public:
    // Constructor
    WriteMessage(uint16_t message_id = 0, const std::string& variable_name = "", const std::string& variable_value = "");

    // Method to serialize the write message into a byte vector
    std::vector<uint8_t> serialize() const override;

    // Method to deserialize the write message from a byte vector
    void deserialize(const std::vector<uint8_t>& data) override;

    // Getters and Setters for variable name and value
    std::string getVariableName() const;
    void setVariableName(const std::string& variable_name);

    std::string getVariableValue() const;
    void setVariableValue(const std::string& variable_value);

    // Method that replicates the getWriteCommand logic from Java code
    std::vector<uint8_t> getWriteCommand() const;

    // Print the WriteMessage details for debugging
    void printMessageDetails() const;
};

#endif // WRITEMESSAGE_HPP
