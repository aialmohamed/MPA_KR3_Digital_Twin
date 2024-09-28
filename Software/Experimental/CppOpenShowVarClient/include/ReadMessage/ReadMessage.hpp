// ReadMessage.hpp

#ifndef READMESSAGE_HPP
#define READMESSAGE_HPP

#include "Message.hpp"
#include <string>
#include <vector>
#include <cstdint>

class ReadMessage : public Message {
private:
    uint16_t variable_name_length_;  // Length of the variable name
    std::string variable_name_;      // Variable name to be read

public:
    // Constructor
    ReadMessage(uint16_t message_id = 0, const std::string& variable_name = "");

    // Method to serialize the read message into a byte vector
    std::vector<uint8_t> serialize() const override;

    // Method to deserialize the read message from a byte vector
    void deserialize(const std::vector<uint8_t>& data) override;

    // Getters and Setters for variable name
    std::string getVariableName() const;
    void setVariableName(const std::string& variable_name);

    // Print the ReadMessage details for debugging
    void printMessageDetails() const;
};

#endif // READMESSAGE_HPP
