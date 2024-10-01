// test_ReadMessage.cpp

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "WriteMessage.hpp"

/*Write Request Message Format
---------------------------
2 bytes Id (uint16)
2 bytes for content length (uint16)
1 byte for read/write mode (1=Write)
2 bytes for the variable name length (uint16)
N bytes for the variable name to be written (ASCII)
2 bytes for the variable value length (uint16)
M bytes for the variable value to be written (ASCII)*/

// Test Case 1: Verify serialization of Message ID
TEST_CASE("Verify serialization of Message ID", "[WriteMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    std::string variable_value = "60";
    WriteMessage writeMessage(message_id, variable_name, variable_value);

    // Serialize the message
    std::vector<uint8_t> serialized_data = writeMessage.serialize();

    // Check that the message ID is serialized correctly (2 bytes)
    REQUIRE(serialized_data[0] == 0x00);  // High byte of Message ID
    REQUIRE(serialized_data[1] == 0x01);  // Low byte of Message ID
}

// Test Case 2: Verify serialization of Content Length
TEST_CASE("Verify serialization of Content Length", "[WriteMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    std::string variable_value = "60";
    WriteMessage writeMessage(message_id, variable_name, variable_value);

    // Serialize the message
    std::vector<uint8_t> serialized_data = writeMessage.serialize();

    // Expected content length = 1 (mode) + 2 (variable name length) + 7 (variable name) + 2 (variable value length) + 2 (variable value) = 14 bytes
    REQUIRE(serialized_data[2] == 0x00);  // High byte of content length
    REQUIRE(serialized_data[3] == 0x0E);  // Low byte of content length (14 in decimal)
}

// Test Case 3: Verify serialization of Mode
TEST_CASE("Verify serialization of Mode", "[WriteMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    std::string variable_value = "60";
    WriteMessage writeMessage(message_id, variable_name, variable_value);

    // Serialize the message
    std::vector<uint8_t> serialized_data = writeMessage.serialize();

    // Check that the mode is serialized correctly (1 byte)
    REQUIRE(serialized_data[4] == 0x01);  // Mode = 1 for Write
}

// Test Case 4: Verify serialization of Variable Name Length
TEST_CASE("Verify serialization of Variable Name Length", "[WriteMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    std::string variable_value = "60";
    WriteMessage writeMessage(message_id, variable_name, variable_value);

    // Serialize the message
    std::vector<uint8_t> serialized_data = writeMessage.serialize();

    // Check that the variable name length is serialized correctly (2 bytes)
    REQUIRE(serialized_data[5] == 0x00);  // High byte of variable name length
    REQUIRE(serialized_data[6] == 0x07);  // Low byte of variable name length (7 bytes)
}

// Test Case 5: Verify serialization of Variable Name
TEST_CASE("Verify serialization of Variable Name", "[WriteMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    std::string variable_value = "60";
    WriteMessage writeMessage(message_id, variable_name, variable_value);

    // Serialize the message
    std::vector<uint8_t> serialized_data = writeMessage.serialize();

    // Check that the variable name is serialized correctly (starts at index 7)
    std::vector<uint8_t> expected_variable_name = {0x24, 0x4F, 0x56, 0x5F, 0x50, 0x52, 0x4F};  // ASCII values of "$OV_PRO"
    for (size_t name_char_idx = 0; name_char_idx < expected_variable_name.size(); ++name_char_idx) {
        REQUIRE(serialized_data[7 + name_char_idx] == expected_variable_name[name_char_idx]);
    }
}

// Test Case 6: Verify serialization of Variable Value Length
TEST_CASE("Verify serialization of Variable Value Length", "[WriteMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    std::string variable_value = "60";
    WriteMessage writeMessage(message_id, variable_name, variable_value);

    // Serialize the message
    std::vector<uint8_t> serialized_data = writeMessage.serialize();

    // Check that the variable value length is serialized correctly (2 bytes)
    REQUIRE(serialized_data[14] == 0x00);  // High byte of variable value length
    REQUIRE(serialized_data[15] == 0x02);  // Low byte of variable value length (2 bytes)
}

// Test Case 7: Verify serialization of Variable Value
TEST_CASE("Verify serialization of Variable Value", "[WriteMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    std::string variable_value = "60";
    WriteMessage writeMessage(message_id, variable_name, variable_value);

    // Serialize the message
    std::vector<uint8_t> serialized_data = writeMessage.serialize();

    // Check that the variable value is serialized correctly (starts at index 16)
    std::vector<uint8_t> expected_variable_value = {0x36, 0x30};  // ASCII values of "60"
    for (size_t value_idx = 0; value_idx < expected_variable_value.size(); ++value_idx) {
        REQUIRE(serialized_data[16 + value_idx] == expected_variable_value[value_idx]);
    }
}
// Test Case 8: Verify the entire serialized message for WriteMessage
TEST_CASE("Verify full serialized message for WriteMessage", "[WriteMessage]") {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";   // Example variable name
    std::string variable_value = "60";       // Example value to write
    WriteMessage writeMessage(message_id, variable_name, variable_value);

    // Serialize the message
    std::vector<uint8_t> serialized_data = writeMessage.serialize();
    std::vector<uint8_t> expected_data = {
        0x00, 0x01,             // Message ID
        0x00, 0x0E,             // Content Length (14 bytes)
        0x01,                   // Mode (Write)
        0x00, 0x07,             // Variable Name Length (7 bytes)
        0x24, 0x4F, 0x56, 0x5F, 0x50, 0x52, 0x4F, // Variable Name ($OV_PRO)
        0x00, 0x02,             // Variable Value Length (2 bytes)
        0x36, 0x30              // Variable Value ("60")
    };

    // Check that the entire serialized message matches the expected message
    REQUIRE(serialized_data.size() == expected_data.size());
    REQUIRE(serialized_data == expected_data);
}

// Test Case 1: Verify deserialization of a complete WriteMessage
TEST_CASE("Verify deserialization of a complete WriteMessage", "[WriteMessage]") {
    // Serialized byte vector for a complete WriteMessage
    // Message ID (2 bytes): 0x00 0x01
    // Content Length (2 bytes): 0x00 0x0E (14 bytes)
    // Mode (1 byte): 0x01 (Write)
    // Variable Name Length (2 bytes): 0x00 0x07 (7 bytes)
    // Variable Name (7 bytes): $OV_PRO (ASCII: 0x24 0x4F 0x56 0x5F 0x50 0x52 0x4F)
    // Variable Value Length (2 bytes): 0x00 0x02 (2 bytes)
    // Variable Value (2 bytes): 60 (ASCII: 0x36 0x30)
    std::vector<uint8_t> serialized_data = {
        0x00, 0x01, 0x00, 0x0E, 0x01, 0x00, 0x07, 0x24, 0x4F, 0x56, 0x5F, 0x50, 0x52, 0x4F, 0x00, 0x02, 0x36, 0x30};

    // Create a WriteMessage object and deserialize the data
    WriteMessage writeMessage;
    writeMessage.deserialize(serialized_data);

    // Verify the deserialized fields
    REQUIRE(writeMessage.getMessageID() == 1);
    REQUIRE(writeMessage.getContentLength() == 14);
    REQUIRE(writeMessage.getMode() == 1);
    REQUIRE(writeMessage.getVariableNameLength() == 7);
    REQUIRE(writeMessage.getVariableName() == "$OV_PRO");
    REQUIRE(writeMessage.getVariableValueLength() == 2);
    REQUIRE(writeMessage.getVariableValue() == "60");
}

// Test Case 2: Verify deserialization with incomplete data (missing variable name)
TEST_CASE("Verify deserialization with incomplete data (missing variable name)", "[WriteMessage]") {
    // Serialized byte vector with missing variable name part
    std::vector<uint8_t> incomplete_data = {0x00, 0x01, 0x00, 0x0E, 0x01, 0x00, 0x07};

    WriteMessage writeMessage;
    writeMessage.deserialize(incomplete_data);

    // Check that fields are deserialized correctly up to the available data
    REQUIRE(writeMessage.getMessageID() == 1);
    REQUIRE(writeMessage.getContentLength() == 14);
    REQUIRE(writeMessage.getMode() == 1);
    REQUIRE(writeMessage.getVariableNameLength() == 7);

    // Variable name and value should remain empty as the data is insufficient
    REQUIRE(writeMessage.getVariableName().empty());
    REQUIRE(writeMessage.getVariableValue().empty());
}

// Test Case 3: Verify deserialization with incomplete data (missing variable value)
TEST_CASE("Verify deserialization with incomplete data (missing variable value)", "[WriteMessage]") {
    // Serialized byte vector with complete variable name but missing variable value part
    std::vector<uint8_t> incomplete_data = {
        0x00, 0x01, 0x00, 0x0E, 0x01, 0x00, 0x07, 0x24, 0x4F, 0x56, 0x5F, 0x50, 0x52, 0x4F, 0x00, 0x02};

    WriteMessage writeMessage;
    writeMessage.deserialize(incomplete_data);

    // Check that fields are deserialized correctly up to the available data
    REQUIRE(writeMessage.getMessageID() == 1);
    REQUIRE(writeMessage.getContentLength() == 14);
    REQUIRE(writeMessage.getMode() == 1);
    REQUIRE(writeMessage.getVariableNameLength() == 7);
    REQUIRE(writeMessage.getVariableName() == "$OV_PRO");
    REQUIRE(writeMessage.getVariableValueLength() == 2);

    // Variable value should remain empty as the data is insufficient
    REQUIRE(writeMessage.getVariableValue().empty());
}

// Test Case 4: Verify deserialization of minimum size byte vector
TEST_CASE("Verify deserialization of minimum size byte vector", "[WriteMessage]") {
    // Minimum size byte vector (less than 7 bytes)
    std::vector<uint8_t> min_size_data = {0x00, 0x01, 0x00, 0x02};

    WriteMessage writeMessage;
    writeMessage.deserialize(min_size_data);

    // Check that no fields are set, as the data is insufficient
    REQUIRE(writeMessage.getMessageID() == 0);  // Default ID
    REQUIRE(writeMessage.getContentLength() == 0);  // Default content length
    REQUIRE(writeMessage.getMode() == 0);  // Default mode
    REQUIRE(writeMessage.getVariableNameLength() == 0);  // No variable name length
    REQUIRE(writeMessage.getVariableName().empty());  // Variable name should remain empty
    REQUIRE(writeMessage.getVariableValueLength() == 0);  // No variable value length
    REQUIRE(writeMessage.getVariableValue().empty());  // Variable value should remain empty
}