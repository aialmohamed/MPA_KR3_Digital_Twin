// ResponseMessage.hpp

#ifndef RESPONSEMESSAGE_HPP
#define RESPONSEMESSAGE_HPP

#include <string>
#include <vector>
#include <cstdint>

// Define constants for field sizes in the response message
#define MESSAGE_ID_SIZE 2            // Size of Message ID (2 bytes)
#define CONTENT_LENGTH_SIZE 2        // Size of Content Length (2 bytes)
#define MODE_SIZE 1                  // Size of Mode field (1 byte)
#define VARIABLE_VALUE_LENGTH_SIZE 2 // Size of Variable Value Length (2 bytes)
#define STATUS_CODE_SIZE 3           // Size of Status Code (3 bytes)

// Define offsets for each field in the message
#define MESSAGE_ID_OFFSET 0                           // Offset for Message ID
#define CONTENT_LENGTH_OFFSET (MESSAGE_ID_OFFSET + MESSAGE_ID_SIZE)                 // Offset for Content Length
#define MODE_OFFSET (CONTENT_LENGTH_OFFSET + CONTENT_LENGTH_SIZE)                   // Offset for Mode
#define VARIABLE_VALUE_LENGTH_OFFSET (MODE_OFFSET + MODE_SIZE)                      // Offset for Variable Value Length
#define VARIABLE_VALUE_OFFSET (VARIABLE_VALUE_LENGTH_OFFSET + VARIABLE_VALUE_LENGTH_SIZE) // Offset for Variable Value

// Define bit-shift values
#define BYTE_SHIFT 8 // Number of bits to shift for higher byte

// Define minimum size for a response message to be considered valid for deserialization
#define MIN_RESPONSE_SIZE (MESSAGE_ID_SIZE + CONTENT_LENGTH_SIZE + MODE_SIZE + VARIABLE_VALUE_LENGTH_SIZE + STATUS_CODE_SIZE)

class ResponseMessage {
private:
    uint16_t message_id_;            // Message ID (2 bytes)
    uint16_t content_length_;         // Content Length (2 bytes)
    uint8_t mode_;                    // Mode (1 byte)
    uint16_t variable_value_length_;  // Length of the variable value (2 bytes)
    std::string variable_value_;      // Variable value (N bytes)
    uint8_t status_code_[3];          // Status code (3 bytes)

public:
    // Constructor
    ResponseMessage();

    // Method to deserialize the response message from a byte vector
    void deserialize(const std::vector<uint8_t>& data);

    // Getters for the response attributes
    uint16_t getMessageID() const;
    uint16_t getContentLength() const;
    uint8_t getMode() const;
    uint16_t getVariableValueLength() const;
    std::string getVariableValue() const;
    std::string getStatusCode() const;

    // Print the ResponseMessage details for debugging
    void printMessageDetails() const;
};

#endif // RESPONSEMESSAGE_HPP
