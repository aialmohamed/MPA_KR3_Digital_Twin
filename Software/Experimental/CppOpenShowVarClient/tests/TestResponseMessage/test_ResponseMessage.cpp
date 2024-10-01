// test_ReadMessage.cpp

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "ResponseMessage.hpp"

/*Answer Message Format
---------------------------
2 bytes Id (uint16)
2 bytes for content length (uint16)
1 byte for read/write mode (0=Read, 1=Write, 2=ReadArray, 3=WriteArray)
2 bytes for the variable value length (uint16)
N bytes for the variable value (ASCII)
3 bytes for tail (000 on error, 011 on success)*/



// Test Case 1: Verify deserialization of a complete ResponseMessage
TEST_CASE("Verify deserialization of a complete ResponseMessage", "[ResponseMessage]") {
    // Serialized byte vector for a complete ResponseMessage
    // Message ID (2 bytes): 0x00 0x01
    // Content Length (2 bytes): 0x00 0x0A (10 bytes)
    // Mode (1 byte): 0x00 (Read)
    // Variable Value Length (2 bytes): 0x00 0x02 (2 bytes)
    // Variable Value (2 bytes): "50" (ASCII: 0x35 0x30)
    // Status Code (3 bytes): 0x00 0x01 0x11
    std::vector<uint8_t> serialized_data = {
        0x00, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x02, 0x35, 0x30, 0x00, 0x01, 0x11};

    // Create a ResponseMessage object and deserialize the data
    ResponseMessage responseMessage;
    responseMessage.deserialize(serialized_data);

    // Verify the deserialized fields
    REQUIRE(responseMessage.getMessageID() == 1);
    REQUIRE(responseMessage.getContentLength() == 10);
    REQUIRE(responseMessage.getMode() == 0);
    REQUIRE(responseMessage.getVariableValueLength() == 2);
    REQUIRE(responseMessage.getVariableValue() == "50");

    // Verify the status code (3 bytes)
    std::string expected_status_code = "00 01 11";
    REQUIRE(responseMessage.getStatusCode() == expected_status_code);
}

// Test Case 2: Verify deserialization with incomplete data (missing variable value)
TEST_CASE("Verify deserialization with incomplete data (missing variable value)", "[ResponseMessage]") {
    // Serialized byte vector with missing variable value part
    std::vector<uint8_t> incomplete_data = {0x00, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x02};

    ResponseMessage responseMessage;
    responseMessage.deserialize(incomplete_data);

    // Check that fields are deserialized correctly up to the available data
    REQUIRE(responseMessage.getMessageID() == 1);
    REQUIRE(responseMessage.getContentLength() == 10);
    REQUIRE(responseMessage.getMode() == 0);
    REQUIRE(responseMessage.getVariableValueLength() == 2);

    // Variable value should remain empty as the data is insufficient
    REQUIRE(responseMessage.getVariableValue().empty());

    // Status code should also remain unset
    REQUIRE(responseMessage.getStatusCode() == "00 00 00");
}

// Test Case 3: Verify deserialization with incomplete data (missing status code)
TEST_CASE("Verify deserialization with incomplete data (missing status code)", "[ResponseMessage]") {
    // Serialized byte vector with complete variable value but missing status code
    std::vector<uint8_t> incomplete_data = {0x00, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x02, 0x35, 0x30};

    ResponseMessage responseMessage;
    responseMessage.deserialize(incomplete_data);

    // Check that fields are deserialized correctly up to the available data
    REQUIRE(responseMessage.getMessageID() == 1);
    REQUIRE(responseMessage.getContentLength() == 10);
    REQUIRE(responseMessage.getMode() == 0);
    REQUIRE(responseMessage.getVariableValueLength() == 2);
    REQUIRE(responseMessage.getVariableValue() == "50");

    // Status code should remain unset as the data is insufficient
    REQUIRE(responseMessage.getStatusCode() == "00 00 00");
}

// Test Case 4: Verify deserialization of minimum size byte vector
TEST_CASE("Verify deserialization of minimum size byte vector", "[ResponseMessage]") {
    // Minimum size byte vector (less than MIN_RESPONSE_SIZE)
    std::vector<uint8_t> min_size_data = {0x00, 0x01, 0x00, 0x02};

    ResponseMessage responseMessage;
    responseMessage.deserialize(min_size_data);

    // Check that no fields are set, as the data is insufficient
    REQUIRE(responseMessage.getMessageID() == 0);  // Default ID
    REQUIRE(responseMessage.getContentLength() == 0);  // Default content length
    REQUIRE(responseMessage.getMode() == 0);  // Default mode
    REQUIRE(responseMessage.getVariableValueLength() == 0);  // No variable value length
    REQUIRE(responseMessage.getVariableValue().empty());  // Variable value should remain empty
    REQUIRE(responseMessage.getStatusCode() == "00 00 00");  // Status code should remain unset
}