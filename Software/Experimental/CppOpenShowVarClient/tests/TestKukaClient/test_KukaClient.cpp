// KukaClientTest.cpp
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "KukaClient.hpp"
#include <boost/asio.hpp>
#include "ReadMessage.hpp"
#include "ResponseMessage.hpp"
#include "WriteMessage.hpp"
#include <thread>
#include <chrono>
#include <iostream>

// Mock class for TCPClient
class MockTCPClient : public TCPClient {
public:
    MockTCPClient(boost::asio::io_context& io_context, const std::string& host_ip, const std::string& port)
        : TCPClient(io_context, host_ip, port) {}

    MOCK_METHOD(void, async_tcp_connect, (std::function<void(boost::system::error_code, boost::asio::ip::tcp::endpoint)>), (override));
    MOCK_METHOD(void, async_send, (const std::vector<uint8_t>&, std::function<void(boost::system::error_code, std::size_t)>), (override));
    MOCK_METHOD(void, async_receive, (std::function<void(boost::system::error_code, const std::vector<uint8_t>&, std::size_t)>), (override));
    MOCK_METHOD(void, close, (), (override));
};

class KukaClientTest : public ::testing::Test {
    protected:
    // Test setup
    void SetUp() override {
        // Create io_context and work_guard to keep it running
        io_context_ = std::make_shared<boost::asio::io_context>();
        work_guard_ = std::make_shared<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(boost::asio::make_work_guard(*io_context_));

        // Create a mock TCP client
        mock_client_ = std::make_shared<MockTCPClient>(*io_context_, "172.31.1.197", "7000");

        // Create the KukaClient instance and inject the mock TCP client
        kuka_client_ = std::make_unique<KukaClient>(*io_context_, "172.31.1.197", "7000", mock_client_);

        // Run the io_context in a separate thread
        io_thread_ = std::make_shared<std::thread>([this]() {
            io_context_->run();
        });
    }

    // Test cleanup
    void TearDown() override {
        // Close the Kuka client and stop the io_context
        kuka_client_->close();
        work_guard_.reset();
        io_context_->stop();
        if (io_thread_->joinable()) {
            io_thread_->join();
        }
        io_context_->reset();
    }

    std::shared_ptr<boost::asio::io_context> io_context_;    // io_context for the tests
    std::shared_ptr<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>> work_guard_; // work guard to keep io_context running
    std::shared_ptr<MockTCPClient> mock_client_;  // Mock TCP client instance
    std::unique_ptr<KukaClient> kuka_client_;     // KukaClient instance under test
    std::shared_ptr<std::thread> io_thread_;      // Thread running the io_context
};


TEST_F(KukaClientTest, ConnectionTest)
{
    std::promise<void> connect_promise;
    auto connect_future = connect_promise.get_future();

    // Expectation: async_tcp_connect should be called once and should invoke the callback with no error
    EXPECT_CALL(*mock_client_, async_tcp_connect(testing::_))
        .WillOnce(testing::Invoke([&connect_promise](std::function<void(boost::system::error_code, boost::asio::ip::tcp::endpoint)> callback) {
            boost::system::error_code ec;  // No error
            boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::make_address("172.31.1.197"), 7000);
            callback(ec, endpoint);
            connect_promise.set_value();
        }));

    // Call connect on KukaClient
    kuka_client_->connect([&](boost::system::error_code ec) {
        ASSERT_FALSE(ec) << "Failed to connect to KUKA robot: " << ec.message();
    });

    // Wait for the connection to complete
    ASSERT_EQ(connect_future.wait_for(std::chrono::seconds(5)), std::future_status::ready) << "Connection timed out!";
}

TEST_F(KukaClientTest, ReadVariableSuccess) {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    std::string expected_value = "100";

    // Create a ReadMessage and serialize it
    ReadMessage read_message(message_id, variable_name);
    std::vector<uint8_t> expected_read_request = read_message.serialize();

    // Mock response message for the variable with correct structure
    std::vector<uint8_t> mock_response_message = {
        0x00, 0x01, // Message ID
        0x00, 0x0B, // Content Length (1 byte mode + 2 bytes variable value length + 3 bytes for tail + variable value length)
        0x00,       // Mode (read mode, 0 = Read)
        0x00, 0x03, // Variable value length (3 bytes)
        '1', '0', '0', // Variable value: "100"
        0x00, 0x11, 0x00 // Tail indicating success (011)
    };

    std::promise<void> receive_promise;
    auto receive_future = receive_promise.get_future();
    bool callback_called = false;  // Track if callback is called

    // Expect the async_send method to be called with the serialized read request message
    EXPECT_CALL(*mock_client_, async_send(expected_read_request, testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([this](const std::vector<uint8_t>& message, std::function<void(boost::system::error_code, std::size_t)> callback) {
            boost::system::error_code ec;  // No error
            callback(ec, message.size());
        }));

    // Expect the async_receive method to be called and return the mock response message
    EXPECT_CALL(*mock_client_, async_receive(testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([&mock_response_message, &receive_promise, &callback_called](std::function<void(boost::system::error_code, const std::vector<uint8_t>&, std::size_t)> callback) {
            boost::system::error_code ec;  // No error
            callback(ec, mock_response_message, mock_response_message.size());
            if (!callback_called) {  // Ensure promise is only set once
                receive_promise.set_value();
                callback_called = true;
            }
        }));

    // Read variable and verify the callback is called with the correct response message
    kuka_client_->readVariable(message_id, variable_name, [&](boost::system::error_code ec, ResponseMessage response) {
        ASSERT_FALSE(ec) << "Failed to read variable: " << ec.message();
        EXPECT_EQ(response.getVariableValue(), expected_value);  // Check if the response value matches expected
        EXPECT_EQ(response.getStatusCode(), "00 11 00");  // Verify the status code matches success indicator
        if (!callback_called) {  // Ensure promise is only set once
            receive_promise.set_value();
            callback_called = true;
        }
    });

    // Wait for the receive operation to complete
    ASSERT_EQ(receive_future.wait_for(std::chrono::seconds(5)), std::future_status::ready) << "Read operation timed out!";
}

TEST_F(KukaClientTest, WriteVariableSuccess) {
    uint16_t message_id = 2;
    std::string variable_name = "$OV_PRO";
    std::string variable_value = "50";
    std::string expected_status_code = "00 11 00";  // Success status code

    // Create a WriteMessage and serialize it
    WriteMessage write_message(message_id, variable_name, variable_value);
    std::vector<uint8_t> expected_write_request = write_message.serialize();

    // Mock response message for the variable with the correct structure
    std::vector<uint8_t> mock_response_message = {
        0x00, 0x02,  // Message ID (matches the one we send)
        0x00, 0x0B,  // Content Length (1 byte mode + 2 bytes variable value length + 3 bytes for tail + variable value length)
        0x01,        // Mode (write mode, 1 = Write)
        0x00, 0x02,  // Variable value length (2 bytes)
        '5', '0',    // Variable value: "50"
        0x00, 0x11, 0x00 // Tail indicating success (011)
    };

    std::promise<void> receive_promise;
    auto receive_future = receive_promise.get_future();
    bool callback_called = false;  // Track if callback is called

    // Expect the async_send method to be called with the serialized write request message
    EXPECT_CALL(*mock_client_, async_send(expected_write_request, testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([this](const std::vector<uint8_t>& message, std::function<void(boost::system::error_code, std::size_t)> callback) {
            boost::system::error_code ec;  // No error
            callback(ec, message.size());
        }));

    // Expect the async_receive method to be called and return the mock response message
    EXPECT_CALL(*mock_client_, async_receive(testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([&mock_response_message, &receive_promise, &callback_called](std::function<void(boost::system::error_code, const std::vector<uint8_t>&, std::size_t)> callback) {
            boost::system::error_code ec;  // No error
            callback(ec, mock_response_message, mock_response_message.size());
            if (!callback_called) {  // Ensure promise is only set once
                receive_promise.set_value();
                callback_called = true;
            }
        }));

    // Write variable and verify the callback is called with the correct response message
    kuka_client_->writeVariable(message_id, variable_name, variable_value, [&](boost::system::error_code ec, ResponseMessage response) {
        ASSERT_FALSE(ec) << "Failed to write variable: " << ec.message();
        EXPECT_EQ(response.getVariableValue(), variable_value);  // Check if the response value matches expected
        EXPECT_EQ(response.getStatusCode(), expected_status_code);  // Verify the status code matches success indicator
        if (!callback_called) {  // Ensure promise is only set once
            receive_promise.set_value();
            callback_called = true;
        }
    });

    // Wait for the receive operation to complete
    ASSERT_EQ(receive_future.wait_for(std::chrono::seconds(5)), std::future_status::ready) << "Write operation timed out!";
}
// Test when the readVariable operation fails (e.g., status code "000" indicates an error)
TEST_F(KukaClientTest, ReadVariableFailure) {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    std::string expected_variable_value = "";  // Empty value for failure
    std::string expected_status_code = "00 00 00";  // Failure status code

    // Create the expected ReadMessage and serialize it
    ReadMessage read_message(message_id, variable_name);
    std::vector<uint8_t> expected_read_request = read_message.serialize();

    // Create a mock response message indicating failure
    std::vector<uint8_t> mock_response_message = {
        0x00, 0x01,  // Message ID (matches the one we send)
        0x00, 0x07,  // Content Length (1 byte mode + 2 bytes variable value length + 3 bytes for tail)
        0x00,        // Mode (read mode, 0 = Read)
        0x00, 0x00,  // Variable value length (0 bytes for failure)
        0x00, 0x00, 0x00 // Tail indicating failure (000)
    };

    std::promise<void> receive_promise;
    auto receive_future = receive_promise.get_future();
    bool callback_called = false;

    // Expect the async_send method to be called with the serialized read request
    EXPECT_CALL(*mock_client_, async_send(expected_read_request, testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([this](const std::vector<uint8_t>& message, std::function<void(boost::system::error_code, std::size_t)> callback) {
            boost::system::error_code ec;  // No error
            callback(ec, message.size());
        }));

    // Expect the async_receive method to be called and return the mock response message
    EXPECT_CALL(*mock_client_, async_receive(testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([&mock_response_message, &receive_promise, &callback_called](std::function<void(boost::system::error_code, const std::vector<uint8_t>&, std::size_t)> callback) {
            boost::system::error_code ec;  // No error
            callback(ec, mock_response_message, mock_response_message.size());
            if (!callback_called) {  // Ensure promise is only set once
                receive_promise.set_value();
                callback_called = true;
            }
        }));

    // Perform the read operation and check the response
    kuka_client_->readVariable(message_id, variable_name, [&](boost::system::error_code ec, ResponseMessage response) {
        ASSERT_FALSE(ec) << "Failed to read variable: " << ec.message();
        EXPECT_EQ(response.getVariableValue(), expected_variable_value);  // Check if the response value is empty
        EXPECT_EQ(response.getStatusCode(), expected_status_code);  // Verify the status code matches failure indicator
        if (!callback_called) {
            receive_promise.set_value();
            callback_called = true;
        }
    });

    // Wait for the receive operation to complete
    ASSERT_EQ(receive_future.wait_for(std::chrono::seconds(5)), std::future_status::ready) << "Read operation timed out!";
}

// Test when the writeVariable operation fails (e.g., status code "000" indicates an error)
TEST_F(KukaClientTest, WriteVariableFailure) {
    uint16_t message_id = 2;
    std::string variable_name = "$OV_PRO";
    std::string variable_value = "50";
    std::string expected_status_code = "00 00 00";  // Failure status code

    // Create a WriteMessage and serialize it
    WriteMessage write_message(message_id, variable_name, variable_value);
    std::vector<uint8_t> expected_write_request = write_message.serialize();

    // Create a mock response message indicating failure
    std::vector<uint8_t> mock_response_message = {
        0x00, 0x02,  // Message ID (matches the one we send)
        0x00, 0x07,  // Content Length (1 byte mode + 2 bytes variable value length + 3 bytes for tail)
        0x01,        // Mode (write mode, 1 = Write)
        0x00, 0x00,  // Variable value length (0 bytes for failure)
        0x00, 0x00, 0x00 // Tail indicating failure (000)
    };

    std::promise<void> receive_promise;
    auto receive_future = receive_promise.get_future();
    bool callback_called = false;

    // Expect the async_send method to be called with the serialized write request message
    EXPECT_CALL(*mock_client_, async_send(expected_write_request, testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([this](const std::vector<uint8_t>& message, std::function<void(boost::system::error_code, std::size_t)> callback) {
            boost::system::error_code ec;  // No error
            callback(ec, message.size());
        }));

    // Expect the async_receive method to be called and return the mock response message
    EXPECT_CALL(*mock_client_, async_receive(testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([&mock_response_message, &receive_promise, &callback_called](std::function<void(boost::system::error_code, const std::vector<uint8_t>&, std::size_t)> callback) {
            boost::system::error_code ec;  // No error
            callback(ec, mock_response_message, mock_response_message.size());
            if (!callback_called) {  // Ensure promise is only set once
                receive_promise.set_value();
                callback_called = true;
            }
        }));

    // Write variable and verify the callback is called with the correct response message
    kuka_client_->writeVariable(message_id, variable_name, variable_value, [&](boost::system::error_code ec, ResponseMessage response) {
        ASSERT_FALSE(ec) << "Failed to write variable: " << ec.message();
        EXPECT_EQ(response.getVariableValue(), "");  // Check if the response value is empty due to failure
        EXPECT_EQ(response.getStatusCode(), expected_status_code);  // Verify the status code matches failure indicator
        if (!callback_called) {
            receive_promise.set_value();
            callback_called = true;
        }
    });

    // Wait for the receive operation to complete
    ASSERT_EQ(receive_future.wait_for(std::chrono::seconds(5)), std::future_status::ready) << "Write operation timed out!";
}
