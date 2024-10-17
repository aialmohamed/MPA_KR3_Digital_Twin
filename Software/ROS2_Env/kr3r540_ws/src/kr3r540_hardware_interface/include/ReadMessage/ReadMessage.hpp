// ReadMessage.hpp

#ifndef READMESSAGE_HPP
#define READMESSAGE_HPP

#include "Message/Message.hpp"
#include <string>
#include <vector>
#include <cstdint>


/**
 * @brief The ReadMessage class inherits from the Message base class and is specifically designed to handle read requests.
 * 
 *  This message type allows clients to request the value of specific variables from the server.
 *  The format of the ReadMessage includes an identifier, content length, read/write mode, 
 *  and the name of the variable to be read. This class is responsible for creating and serializing 
 *  read request messages that comply with the server’s expected format.
 */
class ReadMessage : public Message {
private:
    /**
     * @brief  length of the variable name
     * 
     */
    uint16_t variable_name_length_;

    /**
     * @brief  name of the variable to read
     * 
     */
    std::string variable_name_;
public:
    /**
     * @brief Construct a new Read Message object
     * 
     * @param message_id  message id of the message
     * @param variable_name  name of the variable to read
     */
    ReadMessage(uint16_t message_id = 0, const std::string& variable_name = "");

    /**
     * @brief  serialize the ReadMessage object (implementing the pure virtual function of the base class "Message")
     * 
     * @return std::vector<uint8_t>  serialized data
     */
    std::vector<uint8_t> serialize() const override;

    /**
     * @brief  deserialize the data into the ReadMessage object (implementing the pure virtual function of the base class "Message")
     * 
     * @param data  data to deserialize
     */
    void deserialize(const std::vector<uint8_t>& data) override;

    /**
     * @brief Get the Variable Name object
     * 
     * @return std::string 
     */
    std::string getVariableName() const;

    /**
     * @brief Get the Variable Name Length object
     * 
     * @return uint16_t 
     */
    uint16_t getVariableNameLength() const;

    /**
     * @brief Set the Variable Name object
     * 
     * @param variable_name 
     */
    void setVariableName(const std::string& variable_name);

    /**
     * @brief  print the details of the message (Debugging)
     * 
     */
    void printMessageDetails() const;
};

#endif // READMESSAGE_HPP
