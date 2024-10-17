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

#define MIN_RESPONSE_SIZE (MESSAGE_ID_SIZE + CONTENT_LENGTH_SIZE + MODE_SIZE + VARIABLE_VALUE_LENGTH_SIZE + STATUS_CODE_SIZE)

/**
 * @brief The ResponseMessage class is crucial for interpreting the server’s response and determining whether a read or write operation was successful or encountered an error.
 * 
 *  This class extends the Message base class and provides functionality to interpret the server's response, extract relevant data, 
 *  and determine the success or failure of the operation. The ResponseMessage typically includes fields such as the message ID, 
 *  content length, read/write mode, variable value, and status code (success or error).
 */
class ResponseMessage {
private:
    /**
     * @brief  message id of the message
     * 
     */
    uint16_t message_id_;
    /**
     * @brief  content length of the message
     * 
     */
    uint16_t content_length_;
    /**
     * @brief  mode of the message (read or write)
     * 
     */
    uint8_t mode_;
    /**
     * @brief   length of the variable value
     * 
     */
    uint16_t variable_value_length_;  
    /**
     * @brief  value of the variable
     * 
     */
    std::string variable_value_;
    /**
     * @brief  status code of the message (000 error or 011 success)
     * 
     */
    uint8_t status_code_[3];          

public:
    /**
     * @brief Construct a new Response Message object   
     * 
     */
    ResponseMessage();

    /**
     * @brief   serialize the ResponseMessage object
     * 
     * @param data  data to serialize
     */
    void deserialize(const std::vector<uint8_t>& data);

    /**
     * @brief Get the Message I D object
     * 
     * @return uint16_t 
     */
    uint16_t getMessageID() const;

    /**
     * @brief Get the Content Length object
     * 
     * @return uint16_t 
     */
    uint16_t getContentLength() const;

    /**
     * @brief Get the Mode object
     * 
     * @return uint8_t 
     */
    uint8_t getMode() const;

    /**
     * @brief Get the Variable Value Length object
     * 
     * @return uint16_t 
     */
    uint16_t getVariableValueLength() const;

    /**
     * @brief Get the Variable Value object
     * 
     * @return std::string 
     */
    std::string getVariableValue() const;

    /**
     * @brief Get the Status Code object
     * 
     * @return std::string 
     */
    std::string getStatusCode() const;

    /**
     * @brief  print the details of the message
     * 
     */
    void printMessageDetails() const;
};

#endif // RESPONSEMESSAGE_HPP
