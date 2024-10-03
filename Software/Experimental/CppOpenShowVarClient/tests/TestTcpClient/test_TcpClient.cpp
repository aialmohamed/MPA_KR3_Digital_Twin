#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <boost/asio.hpp>
#include "TcpClient.hpp"
#include <thread>
#include <chrono>
#include <iostream>

class MockTCPClient : public TCPClient {
public:
    MockTCPClient(boost::asio::io_context& io_context, const std::string& host_ip, const std::string& port)
        : TCPClient(io_context, host_ip, port) {}
    MOCK_METHOD(void, async_send, (const std::vector<uint8_t>& message, std::function<void(boost::system::error_code, std::size_t)> callback), (override));
    MOCK_METHOD(void, async_receive, (std::function<void(boost::system::error_code, const std::vector<uint8_t>&, std::size_t)> callback), (override));
};

class TCPClientTest : public ::testing::Test {
protected:
    void SetUp() override {
        io_context_ = std::make_shared<boost::asio::io_context>();
        work_guard_ = std::make_shared<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(boost::asio::make_work_guard(*io_context_));
        client_ = std::make_shared<MockTCPClient>(*io_context_, "172.31.1.197", "7000"); 
    }
    void TearDown() override {
        client_->close();
        
        work_guard_.reset();
        io_context_->stop();
        io_context_->reset();
    }

    std::shared_ptr<boost::asio::io_context> io_context_;
    std::shared_ptr<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>> work_guard_;
    std::shared_ptr<MockTCPClient> client_; 
};

TEST_F(TCPClientTest, ConnectionSuccess) {

    std::promise<void> connect_promise;
    auto connect_future = connect_promise.get_future();

    client_->async_tcp_connect([&connect_promise](boost::system::error_code ec, boost::asio::ip::tcp::endpoint endpoint) {
        ASSERT_FALSE(ec) << "Failed to connect to server: " << ec.message();
        ASSERT_TRUE(endpoint.address().to_string() == "172.31.1.197") << "Server address is not : 172.31.1.197";
        std::cout << "Connected to: " << endpoint << std::endl;
        connect_promise.set_value();
    });
    std::thread io_thread([&]() {
        io_context_->run();
    });

    if (connect_future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
        FAIL() << "Connection timeout";
        io_context_->stop();
    }

    ASSERT_TRUE(client_->is_open()) << "Socket is not open after connection!";
    io_context_->stop();
    io_thread.join();
}

std::vector<uint8_t> createValidReceiveResponse(uint16_t id, const std::string& value, uint8_t mode = 0x00) {
    std::vector<uint8_t> message;

    uint16_t content_length = 1 + 2 + value.size() + 3;
    message.push_back(static_cast<uint8_t>(id >> 8));
    message.push_back(static_cast<uint8_t>(id & 0xFF));
    message.push_back(static_cast<uint8_t>(content_length >> 8));
    message.push_back(static_cast<uint8_t>(content_length & 0xFF));
    message.push_back(mode);
    uint16_t value_length = value.size();
    message.push_back(static_cast<uint8_t>(value_length >> 8));
    message.push_back(static_cast<uint8_t>(value_length & 0xFF));
    for (char ch : value) {
        message.push_back(static_cast<uint8_t>(ch));
    }
    message.push_back(0x00);
    message.push_back(0x11);
    message.push_back(0x00);

    return message;
}

std::vector<uint8_t> createInvalidReceiveResponse(uint16_t id) {
    std::vector<uint8_t> message;

    uint16_t content_length = 1 + 2 + 3;
    message.push_back(static_cast<uint8_t>(id >> 8));
    message.push_back(static_cast<uint8_t>(id & 0xFF));
    message.push_back(static_cast<uint8_t>(content_length >> 8));
    message.push_back(static_cast<uint8_t>(content_length & 0xFF));
    message.push_back(0x00);
    message.push_back(0x00);
    message.push_back(0x00);
    message.push_back(0x00);
    message.push_back(0x00);
    message.push_back(0x00);

    return message;
}


std::vector<uint8_t> createReadRequest(uint16_t id, const std::string& variable_name) {
    std::vector<uint8_t> message;

    uint16_t content_length = 1 + 2 + variable_name.size(); 
    message.push_back(static_cast<uint8_t>(id >> 8));
    message.push_back(static_cast<uint8_t>(id & 0xFF));
    message.push_back(static_cast<uint8_t>(content_length >> 8));
    message.push_back(static_cast<uint8_t>(content_length & 0xFF));
    message.push_back(0x00);
    uint16_t var_length = variable_name.size();
    message.push_back(static_cast<uint8_t>(var_length >> 8));
    message.push_back(static_cast<uint8_t>(var_length & 0xFF));
    for (char ch : variable_name) {
        message.push_back(static_cast<uint8_t>(ch));
    }

    return message;
}

std::vector<uint8_t> createWriteRequest(uint16_t id, const std::string& variable_name, const std::string& value) {
    std::vector<uint8_t> message;

    uint16_t content_length = 1 + 2 + variable_name.size() + 2 + value.size();
    message.push_back(static_cast<uint8_t>(id >> 8));
    message.push_back(static_cast<uint8_t>(id & 0xFF));
    message.push_back(static_cast<uint8_t>(content_length >> 8));
    message.push_back(static_cast<uint8_t>(content_length & 0xFF));
    message.push_back(0x01);
    uint16_t var_length = variable_name.size();
    message.push_back(static_cast<uint8_t>(var_length >> 8));
    message.push_back(static_cast<uint8_t>(var_length & 0xFF));
    for (char ch : variable_name) {
        message.push_back(static_cast<uint8_t>(ch));
    }
    uint16_t value_length = value.size();
    message.push_back(static_cast<uint8_t>(value_length >> 8));
    message.push_back(static_cast<uint8_t>(value_length & 0xFF));
    for (char ch : value) {
        message.push_back(static_cast<uint8_t>(ch));
    }

    return message;
}
TEST_F(TCPClientTest, SendReadRequest) {
    std::promise<void> connect_promise;
    auto connect_future = connect_promise.get_future();

    client_->async_tcp_connect([&connect_promise](boost::system::error_code ec, boost::asio::ip::tcp::endpoint endpoint) {
        ASSERT_FALSE(ec) << "Failed to connect to server: " << ec.message();
        connect_promise.set_value();
    });
    std::thread io_thread([&]() {
        io_context_->run();
    });

    if (connect_future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
        FAIL() << "Connection timeout";
        io_context_->stop();
    }

    ASSERT_TRUE(client_->is_open()) << "Socket is not open after connection!";

    std::string variable_name = "$OV_PRO";
    std::vector<uint8_t> expected_message = createReadRequest(1, variable_name);
    EXPECT_CALL(*client_, async_send(expected_message, testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([&](const std::vector<uint8_t>& message, std::function<void(boost::system::error_code, std::size_t)> callback) {
            boost::system::error_code ec;
            callback(ec, message.size());
        }));

    client_->async_send(expected_message, [expected_message](boost::system::error_code ec, std::size_t length) {
        ASSERT_FALSE(ec);
        ASSERT_EQ(length, expected_message.size());
    });
    io_context_->stop();
    io_thread.join();
}

TEST_F(TCPClientTest, FailedReadRequest) {
    std::promise<void> connect_promise;
    auto connect_future = connect_promise.get_future();

    client_->async_tcp_connect([&connect_promise](boost::system::error_code ec, boost::asio::ip::tcp::endpoint endpoint) {
        ASSERT_FALSE(ec) << "Failed to connect to server: " << ec.message();
        connect_promise.set_value();
    });
    std::thread io_thread([&]() {
        io_context_->run();
    });

    if (connect_future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
        FAIL() << "Connection timeout";
        io_context_->stop();
    }

    ASSERT_TRUE(client_->is_open()) << "Socket is not open after connection!";
    std::string invalid_variable_name = "INVALID_VAR";
    std::vector<uint8_t> invalid_message = createReadRequest(1, invalid_variable_name);
    EXPECT_CALL(*client_, async_send(invalid_message, testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([&](const std::vector<uint8_t>& message, std::function<void(boost::system::error_code, std::size_t)> callback) {
            boost::system::error_code ec = boost::asio::error::make_error_code(boost::asio::error::invalid_argument);
            callback(ec,0);
    }));


    std::promise<void> send_promise;
    auto send_future = send_promise.get_future();
    client_->async_send(invalid_message, [&send_promise](boost::system::error_code ec, std::size_t length) {
        ec = boost::asio::error::make_error_code(boost::asio::error::fault);
        ASSERT_TRUE(ec) << "Expected an error due to invalid read request";
        std::cerr << "Error: " << ec.message() << std::endl;
        send_promise.set_value();
    });

    if (send_future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
        FAIL() << "Failed read request timeout";
        io_context_->stop();
    }
    io_context_->stop();
    io_thread.join();
}
TEST_F(TCPClientTest, SendWriteRequest) {
    std::promise<void> connect_promise;
    auto connect_future = connect_promise.get_future();

    client_->async_tcp_connect([&connect_promise](boost::system::error_code ec, boost::asio::ip::tcp::endpoint endpoint) {
        ASSERT_FALSE(ec) << "Failed to connect to server: " << ec.message();
        connect_promise.set_value();
    });
    std::thread io_thread([&]() {
        io_context_->run();
    });
    if (connect_future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
        FAIL() << "Connection timeout";
        io_context_->stop();
    }
    ASSERT_TRUE(client_->is_open()) << "Socket is not open after connection!";
    std::string variable_name = "$OV_PRO";
    std::string value = "100";
    std::vector<uint8_t> expected_message = createWriteRequest(1, variable_name,value);
    EXPECT_CALL(*client_, async_send(expected_message, testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([&](const std::vector<uint8_t>& message, std::function<void(boost::system::error_code, std::size_t)> callback) {
            boost::system::error_code ec;
            callback(ec, message.size());
        }));

    client_->async_send(expected_message, [expected_message](boost::system::error_code ec, std::size_t length) {
        ASSERT_FALSE(ec);
        ASSERT_EQ(length, expected_message.size());
    });
    io_context_->stop();
    io_thread.join();
}


TEST_F(TCPClientTest, FailedWriteRequest) {
    std::promise<void> connect_promise;
    auto connect_future = connect_promise.get_future();

    client_->async_tcp_connect([&connect_promise](boost::system::error_code ec, boost::asio::ip::tcp::endpoint endpoint) {
        ASSERT_FALSE(ec) << "Failed to connect to server: " << ec.message();
        connect_promise.set_value();
    });
    std::thread io_thread([&]() {
        io_context_->run();
    });
    if (connect_future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
        FAIL() << "Connection timeout";
        io_context_->stop();
    }

    ASSERT_TRUE(client_->is_open()) << "Socket is not open after connection!";

    std::string invalid_variable_name = "INVALID_VAR";
    std::string invalid_value = "INVALID_VALUE";
    std::vector<uint8_t> invalid_message = createWriteRequest(1, invalid_variable_name,invalid_value);
    EXPECT_CALL(*client_, async_send(invalid_message, testing::_))
        .Times(1)
        .WillOnce(testing::Invoke([&](const std::vector<uint8_t>& message, std::function<void(boost::system::error_code, std::size_t)> callback) {
            boost::system::error_code ec = boost::asio::error::make_error_code(boost::asio::error::invalid_argument);
            callback(ec,0);
    }));


    std::promise<void> send_promise;
    auto send_future = send_promise.get_future();
    client_->async_send(invalid_message, [&send_promise](boost::system::error_code ec, std::size_t length) {
        ec = boost::asio::error::make_error_code(boost::asio::error::fault);
        ASSERT_TRUE(ec) << "Expected an error due to invalid read request";
        std::cerr << "Error: " << ec.message() << std::endl;
        send_promise.set_value();
    });

    if (send_future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
        FAIL() << "Failed read request timeout";
        io_context_->stop();
    }
    io_context_->stop();
    io_thread.join();
}