// Message.hpp

#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <vector>
#include <cstdint>
#include <string>

class Message {
protected:
    /**
     * @brief message id of the message
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

public:

    /**
     * @brief Construct a new Message object
     * 
     * @param message_id  message id of the message
     * @param content_length  content length of the message
     * @param mode  mode of the message (read or write)
     */
    Message(uint16_t message_id = 0, uint16_t content_length = 0, uint8_t mode = 0);

    /**
     * @brief Destroy the Message object
     * 
     */
    virtual ~Message() = default;

    /**
     * @brief Set the Message ID object
     * 
     * @param message_id  message id of the message
     */
    void setMessageID(uint16_t message_id);
    
    /**
     * @brief Set the Content Length object
     * 
     * @param content_length  content length of the message
     */
    void setContentLength(uint16_t content_length);

    /**
     * @brief Set the Mode object
     * 
     * @param mode  mode of the message (read or write)
     */
    void setMode(uint8_t mode);

    /**
     * @brief Get the Message I D object
     * 
     * @return uint16_t  message id of the message
     */
    uint16_t getMessageID() const;

    /**
     * @brief Get the Content Length object
     * 
     * @return uint16_t  content length of the message
     */
    uint16_t getContentLength() const;

    /**
     * @brief Get the Mode object
     * 
     * @return uint8_t  mode of the message (read or write)
     */
    uint8_t getMode() const;

    /**
     * @brief  serialize the message object (virtual function)
     * 
     * @return std::vector<uint8_t>  serialized message
     */
    virtual std::vector<uint8_t> serialize() const;

    /**
     * @brief  deserialize the message object (virtual function)
     * 
     * @param data  serialized message
     */
    virtual void deserialize(const std::vector<uint8_t>& data);

    /**
     * @brief  display the message object for debugging (virtual function)
     * 
     */
    virtual void display() const;
};

#endif // MESSAGE_HPP
