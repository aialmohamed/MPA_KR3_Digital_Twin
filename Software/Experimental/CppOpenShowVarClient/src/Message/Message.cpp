// Message.cpp

#include "Message.hpp"
#include <iostream>

// Constructor Implementation
Message::Message(uint16_t message_id, uint16_t content_length, uint8_t mode)
    : message_id_(message_id), content_length_(content_length), mode_(mode) {}

// Setters Implementation
void Message::setMessageID(uint16_t message_id) {
    message_id_ = message_id;
}

void Message::setContentLength(uint16_t content_length) {
    content_length_ = content_length;
}

void Message::setMode(uint8_t mode) {
    mode_ = mode;
}

// Getters Implementation
uint16_t Message::getMessageID() const {
    return message_id_;
}

uint16_t Message::getContentLength() const {
    return content_length_;
}

uint8_t Message::getMode() const {
    return mode_;
}

// Serialize the message into a byte array
std::vector<uint8_t> Message::serialize() const {
    std::vector<uint8_t> data;

    // Serialize the message ID (2 bytes)
    data.push_back(static_cast<uint8_t>(message_id_ & 0x00FF));
    data.push_back(static_cast<uint8_t>((message_id_ >> 8) & 0xFF00));

    // Serialize the content length (2 bytes)
    data.push_back(static_cast<uint8_t>(content_length_ & 0x00FF));
    data.push_back(static_cast<uint8_t>((content_length_ >> 8) & 0xFF00));

    // Serialize the mode (1 byte)
    data.push_back(mode_);

    return data;
}

// Deserialize the message from a byte array
void Message::deserialize(const std::vector<uint8_t>& data) {
    if (data.size() >= 5) {
        // Deserialize the message ID
        message_id_ = data[0] | (data[1] << 8);

        // Deserialize the content length
        content_length_ = data[2] | (data[3] << 8);

        // Deserialize the mode
        mode_ = data[4];
    }
}

// Display the message content for debugging purposes
void Message::display() const {
    std::cout << "Message ID: " << message_id_ << std::endl;
    std::cout << "Content Length: " << content_length_ << std::endl;
    std::cout << "Mode: " << static_cast<int>(mode_) << std::endl;
}
