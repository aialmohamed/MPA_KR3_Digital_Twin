// WriteMessage.hpp

#ifndef WRITEMESSAGE_HPP
#define WRITEMESSAGE_HPP

#include "Message.hpp"
#include <string>
#include <vector>
#include <cstdint>

class WriteMessage : public Message {
private:
    /**
     * @brief  length of the variable name
     * 
     */
    uint16_t variable_name_length_;
    /**
     * @brief  name of the variable to write
     * 
     */
    std::string variable_name_;
    /**
     * @brief  length of the variable value
     * 
     */
    uint16_t variable_value_length_;
    /**
     * @brief  value of the variable
     * 
     */
    std::string variable_value_;

public:
    /**
     * @brief Construct a new Write Message object
     * 
     * @param message_id  message id of the message
     * @param variable_name  name of the variable to write
     * @param variable_value  value of the variable
     */
    WriteMessage(uint16_t message_id = 0, const std::string& variable_name = "", const std::string& variable_value = "");
    /**
     * @brief  serialize the WriteMessage object (implementing the pure virtual function of the base class "Message")
     * 
     * @return std::vector<uint8_t> 
     */
    std::vector<uint8_t> serialize() const override;
    /**
     * @brief deserialize the data into the WriteMessage object (implementing the pure virtual function of the base class "Message")
     * 
     * @param data 
     */
    void deserialize(const std::vector<uint8_t>& data) override;
    /**
     * @brief Get the Variable Name object
     * 
     * @return std::string 
     */
    std::string getVariableName() const;
    /**
     * @brief Set the Variable Name object
     * 
     * @param variable_name 
     */
    void setVariableName(const std::string& variable_name);
    /**
     * @brief Get the Variable Name Length object
     * 
     * @return uint16_t 
     */
    uint16_t getVariableNameLength() const;
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
     * @brief Set the Variable Value object
     * 
     * @param variable_value 
     */
    void setVariableValue(const std::string& variable_value);
    /**
     * @brief  print the details of the message
     * 
     */
    void printMessageDetails() const;
};

#endif // WRITEMESSAGE_HPP
