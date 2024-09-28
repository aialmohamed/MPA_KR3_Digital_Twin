// Message.hpp

#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <vector>
#include <cstdint>
#include <string>

class Message {
protected:
    uint16_t message_id_;         // Message ID (2 bytes)
    uint16_t content_length_;     // Content Length (2 bytes)
    uint8_t mode_;                // Mode (1 byte)

public:
    // Constructor
    Message(uint16_t message_id = 0, uint16_t content_length = 0, uint8_t mode = 0);

    // Virtual destructor for polymorphic base class
    virtual ~Message() = default;

    // Setters
    void setMessageID(uint16_t message_id);
    void setContentLength(uint16_t content_length);
    void setMode(uint8_t mode);

    // Getters
    uint16_t getMessageID() const;
    uint16_t getContentLength() const;
    uint8_t getMode() const;

    // Serialize the message into a byte array
    virtual std::vector<uint8_t> serialize() const;

    // Deserialize the message from a byte array
    virtual void deserialize(const std::vector<uint8_t>& data);

    // Display the message content for debugging purposes
    virtual void display() const;
};

#endif // MESSAGE_HPP
