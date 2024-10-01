// test_ReadMessage.cpp

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "ReadMessage.hpp"

    /* Read Request Message Format
    ---------------------------
    2 bytes Id (uint16) =
    2 bytes for content length (uint16)  
    1 byte for read/write mode (0=Read)
    2 bytes for the variable name length (uint16) 
    N bytes for the variable name to be read (ASCII)
    */

// Test Case 1: Verify Message ID Serialization
TEST_CASE("Verify Message ID serialization", "[ReadMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    ReadMessage readMessage(message_id, variable_name);
    std::vector<uint8_t> serialized_data = readMessage.serialize();

    // Message ID should be 0x00 0x01
    REQUIRE(serialized_data[0] == 0x00);  // High byte of message ID
    REQUIRE(serialized_data[1] == 0x01);  // Low byte of message ID
}

// Test Case 2: Verify Content Length Serialization
TEST_CASE("Verify Content Length serialization", "[ReadMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    ReadMessage readMessage(message_id, variable_name);
    std::vector<uint8_t> serialized_data = readMessage.serialize();

    // Content Length should be 0x00 0x0A (10 bytes)
    REQUIRE(serialized_data[2] == 0x00);  // High byte of content length
    REQUIRE(serialized_data[3] == 0x0A);  // Low byte of content length
}

// Test Case 3: Verify Mode Serialization
TEST_CASE("Verify Mode serialization", "[ReadMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    ReadMessage readMessage(message_id, variable_name);
    std::vector<uint8_t> serialized_data = readMessage.serialize();

    // Mode should be 0x00 (Read)
    REQUIRE(serialized_data[4] == 0x00);  // Read mode
}

// Test Case 4: Verify Variable Name Length Serialization
TEST_CASE("Verify Variable Name Length serialization", "[ReadMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    ReadMessage readMessage(message_id, variable_name);
    std::vector<uint8_t> serialized_data = readMessage.serialize();

    // Variable Name Length should be 0x00 0x07 (7 bytes)
    REQUIRE(serialized_data[5] == 0x00);  // High byte of variable name length
    REQUIRE(serialized_data[6] == 0x07);  // Low byte of variable name length
}

// Test Case 5: Verify Variable Name Serialization
TEST_CASE("Verify Variable Name serialization", "[ReadMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    ReadMessage readMessage(message_id, variable_name);
    std::vector<uint8_t> serialized_data = readMessage.serialize();

    // Variable name should be serialized as ASCII bytes for "$OV_PRO"
    std::vector<uint8_t> expected_variable_name = {0x24, 0x4F, 0x56, 0x5F, 0x50, 0x52, 0x4F};
    for (size_t i = 0; i < expected_variable_name.size(); ++i) {
        REQUIRE(serialized_data[7 + i] == expected_variable_name[i]);
    }
}

// Test Case 6: Verify Full Serialized Message
TEST_CASE("Verify full serialized message", "[ReadMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    ReadMessage readMessage(message_id, variable_name);
    std::vector<uint8_t> serialized_data = readMessage.serialize();

    // Expected full message: 0x00 0x01 0x00 0x0A 0x00 0x00 0x07 0x24 0x4F 0x56 0x5F 0x50 0x52 0x4F
    std::vector<uint8_t> expected_data = {0x00, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x07, 0x24, 0x4F, 0x56, 0x5F, 0x50, 0x52, 0x4F};
    
    REQUIRE(serialized_data.size() == expected_data.size());
    REQUIRE(serialized_data == expected_data);
}

// Test Case 7: Verify basic deserialization of a ReadMessage
TEST_CASE("Verify basic deserialization of ReadMessage", "[ReadMessage]") {
    // Serialized byte vector for ReadMessage with variable name "$OV_PRO"
    // Serialized message: 0x00 0x01 0x00 0x0A 0x00 0x00 0x07 0x24 0x4F 0x56 0x5F 0x50 0x52 0x4F
    std::vector<uint8_t> serialized_data = {0x00, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x07, 0x24, 0x4F, 0x56, 0x5F, 0x50, 0x52, 0x4F};

    // Create a ReadMessage object
    ReadMessage readMessage;

    // Call the deserialize method with the given data
    readMessage.deserialize(serialized_data);

    // Check that the message ID and content length are correctly deserialized
    REQUIRE(readMessage.getMessageID() == 1);
    REQUIRE(readMessage.getContentLength() == 10);

    // Check that the variable name length is correctly deserialized
    REQUIRE(readMessage.getVariableNameLength() == 7);

    // Check that the variable name is correctly deserialized
    REQUIRE(readMessage.getVariableName() == "$OV_PRO");
}

// Test Case 8: Verify deserialization with an empty byte vector
TEST_CASE("Verify deserialization with an empty byte vector", "[ReadMessage]") {
    std::vector<uint8_t> empty_data;

    ReadMessage readMessage;
    readMessage.deserialize(empty_data);

    // Check that fields remain unchanged or default
    REQUIRE(readMessage.getMessageID() == 0);  // Default ID, assuming it is initialized to 0
    REQUIRE(readMessage.getContentLength() == 0);  // Default content length
    REQUIRE(readMessage.getVariableNameLength() == 0);  // No variable name length
    REQUIRE(readMessage.getVariableName().empty());  // Variable name should be empty
}

// Test Case 9: Verify deserialization with incomplete byte vector
TEST_CASE("Verify deserialization with incomplete byte vector", "[ReadMessage]") {
    // Serialized byte vector with missing variable name part
    std::vector<uint8_t> incomplete_data = {0x00, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x07};

    ReadMessage readMessage;
    readMessage.deserialize(incomplete_data);

    // Check that the variable name is not deserialized since data is incomplete
    REQUIRE(readMessage.getMessageID() == 1);
    REQUIRE(readMessage.getContentLength() == 10);
    REQUIRE(readMessage.getVariableNameLength() == 7);
    REQUIRE(readMessage.getVariableName().empty());  // Variable name should not be deserialized
}

// Test Case 10: Verify deserialization of valid ReadMessage with different variable name
TEST_CASE("Verify deserialization with different variable name", "[ReadMessage]") {
    // Serialized byte vector for ReadMessage with variable name "$POS_ACT"
    // Serialized message: 0x00 0x01 0x00 0x0A 0x00 0x00 0x08 0x24 0x50 0x4F 0x53 0x5F 0x41 0x43 0x54
    std::vector<uint8_t> serialized_data = {0x00, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x08, 0x24, 0x50, 0x4F, 0x53, 0x5F, 0x41, 0x43, 0x54};

    // Create a ReadMessage object
    ReadMessage readMessage;

    // Call the deserialize method with the given data
    readMessage.deserialize(serialized_data);

    // Check that the variable name length and name are correctly deserialized
    REQUIRE(readMessage.getVariableNameLength() == 8);  // Length of "$POS_ACT" is 8
    REQUIRE(readMessage.getVariableName() == "$POS_ACT");
}