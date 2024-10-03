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
    void SetUp() override {

        io_context_ = std::make_shared<boost::asio::io_context>();
        work_guard_ = std::make_shared<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(boost::asio::make_work_guard(*io_context_));
        mock_client_ = std::make_shared<MockTCPClient>(*io_context_, "172.31.1.197", "7000");
        kuka_client_ = std::make_unique<KukaClient>(*io_context_, "172.31.1.197", "7000", mock_client_);
        io_thread_ = std::make_shared<std::thread>([this]() {
            io_context_->run();
        });
    }
    void TearDown() override {
        kuka_client_->close();
        work_guard_.reset();
        io_context_->stop();
        if (io_thread_->joinable()) {
            io_thread_->join();
        }
        io_context_->reset();
    }

    std::shared_ptr<boost::asio::io_context> io_context_;
    std::shared_ptr<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>> work_guard_;
    std::shared_ptr<MockTCPClient> mock_client_;
    std::unique_ptr<KukaClient> kuka_client_;
    std::shared_ptr<std::thread> io_thread_;
};


TEST_F(KukaClientTest, ConnectionTest)
{
    std::promise<void> connect_promise;
    auto connect_future = connect_promise.get_future();
    EXPECT_CALL(*mock_client_, async_tcp_connect(testing::_))
        .WillOnce(testing::Invoke([&connect_promise](std::function<void(boost::system::error_code, boost::asio::ip::tcp::endpoint)> callback) {
            boost::system::error_code ec;
            boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::make_address("172.31.1.197"), 7000);
            callback(ec, endpoint);
            connect_promise.set_value();
        }));
    kuka_client_->connect([&](boost::system::error_code ec) {
        ASSERT_FALSE(ec) << "Failed to connect to KUKA robot: " << ec.message();
    });
    ASSERT_EQ(connect_future.wait_for(std::chrono::seconds(5)), std::future_status::ready) << "Connection timed out!";
}

TEST_F(KukaClientTest, ReadVariableSuccess) {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    std::string expected_value = "100";
    ReadMessage read_message(message_id, variable_name);
    std::vector<uint8_t> expected_read_request = read_message.serialize();


    std::vector<uint8_t> mock_response_message = {
        0x00, 0x01, 
        0x00, 0x0B, 
        0x00,       
        0x00, 0x03,
        '1', '0', '0',
        0x00, 0x11, 0x00
    };

    std::promise<void> receive_promise;
    auto receive_future = receive_promise.get_future();
    bool callback_called = false;
    EXPECT_CALL(*mock_client_, async_send(expected_read_request, testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([this](const std::vector<uint8_t>& message, std::function<void(boost::system::error_code, std::size_t)> callback) {
            boost::system::error_code ec;
            callback(ec, message.size());
        }));
    EXPECT_CALL(*mock_client_, async_receive(testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([&mock_response_message, &receive_promise, &callback_called](std::function<void(boost::system::error_code, const std::vector<uint8_t>&, std::size_t)> callback) {
            boost::system::error_code ec;  
            callback(ec, mock_response_message, mock_response_message.size());
            if (!callback_called) {
                receive_promise.set_value();
                callback_called = true;
            }
        }));
    kuka_client_->readVariable(message_id, variable_name, [&](boost::system::error_code ec, ResponseMessage response) {
        ASSERT_FALSE(ec) << "Failed to read variable: " << ec.message();
        EXPECT_EQ(response.getVariableValue(), expected_value);
        EXPECT_EQ(response.getStatusCode(), "00 11 00");
        if (!callback_called) {
            receive_promise.set_value();
            callback_called = true;
        }
    });
    ASSERT_EQ(receive_future.wait_for(std::chrono::seconds(5)), std::future_status::ready) << "Read operation timed out!";
}

TEST_F(KukaClientTest, WriteVariableSuccess) {
    uint16_t message_id = 2;
    std::string variable_name = "$OV_PRO";
    std::string variable_value = "50";
    std::string expected_status_code = "00 11 00";
    WriteMessage write_message(message_id, variable_name, variable_value);
    std::vector<uint8_t> expected_write_request = write_message.serialize();
    std::vector<uint8_t> mock_response_message = {
        0x00, 0x02,  
        0x00, 0x0B, 
        0x01,       
        0x00, 0x02,  
        '5', '0',   
        0x00, 0x11, 0x00 
    };
    std::promise<void> receive_promise;
    auto receive_future = receive_promise.get_future();
    bool callback_called = false;
    EXPECT_CALL(*mock_client_, async_send(expected_write_request, testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([this](const std::vector<uint8_t>& message, std::function<void(boost::system::error_code, std::size_t)> callback) {
            boost::system::error_code ec;
            callback(ec, message.size());
        }));
    EXPECT_CALL(*mock_client_, async_receive(testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([&mock_response_message, &receive_promise, &callback_called](std::function<void(boost::system::error_code, const std::vector<uint8_t>&, std::size_t)> callback) {
            boost::system::error_code ec; 
            callback(ec, mock_response_message, mock_response_message.size());
            if (!callback_called) {
                receive_promise.set_value();
                callback_called = true;
            }
        }));
    kuka_client_->writeVariable(message_id, variable_name, variable_value, [&](boost::system::error_code ec, ResponseMessage response) {
        ASSERT_FALSE(ec) << "Failed to write variable: " << ec.message();
        EXPECT_EQ(response.getVariableValue(), variable_value); 
        EXPECT_EQ(response.getStatusCode(), expected_status_code); 
        if (!callback_called) {
            receive_promise.set_value();
            callback_called = true;
        }
    });

    ASSERT_EQ(receive_future.wait_for(std::chrono::seconds(5)), std::future_status::ready) << "Write operation timed out!";
}

TEST_F(KukaClientTest, ReadVariableFailure) {
    uint16_t message_id = 1;
    std::string variable_name = "$OV_PRO";
    std::string expected_variable_value = "";  
    std::string expected_status_code = "00 00 00"; 
    ReadMessage read_message(message_id, variable_name);
    std::vector<uint8_t> expected_read_request = read_message.serialize();

    std::vector<uint8_t> mock_response_message = {
        0x00, 0x01, 
        0x00, 0x07, 
        0x00,        
        0x00, 0x00, 
        0x00, 0x00, 0x00
    };

    std::promise<void> receive_promise;
    auto receive_future = receive_promise.get_future();
    bool callback_called = false;
    EXPECT_CALL(*mock_client_, async_send(expected_read_request, testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([this](const std::vector<uint8_t>& message, std::function<void(boost::system::error_code, std::size_t)> callback) {
            boost::system::error_code ec;
            callback(ec, message.size());
        }));
    EXPECT_CALL(*mock_client_, async_receive(testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([&mock_response_message, &receive_promise, &callback_called](std::function<void(boost::system::error_code, const std::vector<uint8_t>&, std::size_t)> callback) {
            boost::system::error_code ec; 
            callback(ec, mock_response_message, mock_response_message.size());
            if (!callback_called) {
                receive_promise.set_value();
                callback_called = true;
            }
        }));

    kuka_client_->readVariable(message_id, variable_name, [&](boost::system::error_code ec, ResponseMessage response) {
        ASSERT_FALSE(ec) << "Failed to read variable: " << ec.message();
        EXPECT_EQ(response.getVariableValue(), expected_variable_value);  
        EXPECT_EQ(response.getStatusCode(), expected_status_code);
        if (!callback_called) {
            receive_promise.set_value();
            callback_called = true;
        }
    });
    ASSERT_EQ(receive_future.wait_for(std::chrono::seconds(5)), std::future_status::ready) << "Read operation timed out!";
}

TEST_F(KukaClientTest, WriteVariableFailure) {
    uint16_t message_id = 2;
    std::string variable_name = "$OV_PRO";
    std::string variable_value = "50";
    std::string expected_status_code = "00 00 00"; 
    WriteMessage write_message(message_id, variable_name, variable_value);
    std::vector<uint8_t> expected_write_request = write_message.serialize();

    std::vector<uint8_t> mock_response_message = {
        0x00, 0x02,  
        0x00, 0x07,  
        0x01, 
        0x00, 0x00,
        0x00, 0x00, 0x00
    };
    std::promise<void> receive_promise;
    auto receive_future = receive_promise.get_future();
    bool callback_called = false;

    EXPECT_CALL(*mock_client_, async_send(expected_write_request, testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([this](const std::vector<uint8_t>& message, std::function<void(boost::system::error_code, std::size_t)> callback) {
            boost::system::error_code ec;
            callback(ec, message.size());
        }));
    EXPECT_CALL(*mock_client_, async_receive(testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([&mock_response_message, &receive_promise, &callback_called](std::function<void(boost::system::error_code, const std::vector<uint8_t>&, std::size_t)> callback) {
            boost::system::error_code ec; 
            callback(ec, mock_response_message, mock_response_message.size());
            if (!callback_called) {
                receive_promise.set_value();
                callback_called = true;
            }
        }));

    kuka_client_->writeVariable(message_id, variable_name, variable_value, [&](boost::system::error_code ec, ResponseMessage response) {
        ASSERT_FALSE(ec) << "Failed to write variable: " << ec.message();
        EXPECT_EQ(response.getVariableValue(), ""); 
        EXPECT_EQ(response.getStatusCode(), expected_status_code);
        if (!callback_called) {
            receive_promise.set_value();
            callback_called = true;
        }
    });
    ASSERT_EQ(receive_future.wait_for(std::chrono::seconds(5)), std::future_status::ready) << "Write operation timed out!";
}
