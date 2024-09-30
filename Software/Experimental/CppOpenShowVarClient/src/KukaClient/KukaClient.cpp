// KukaClient.cpp

#include "KukaClient.hpp"

// Constructor to initialize the KukaClient with io_context and IP/Port
KukaClient::KukaClient(boost::asio::io_context& io_context, const std::string& ip, const std::string& port)
    : client_(io_context, ip, port), ip_(ip), port_(port), io_context_(io_context), reconnect_attempts_(0) {}

// Function to establish a connection with the KUKA robot
void KukaClient::connect(std::function<void(boost::system::error_code)> callback) {
    std::cout << "Initiating asynchronous connection to KUKA robot..." << std::endl;
    client_.async_tcp_connect([this, callback](boost::system::error_code ec, boost::asio::ip::tcp::endpoint endpoint) {
        if (!ec) {
            std::cout << "Successfully connected to KUKA robot at: " << endpoint.address().to_string() << ":" << endpoint.port() << std::endl;
            reconnect_attempts_ = 0;  // Reset reconnect attempts on successful connection
            callback(ec);
        } else {
            std::cerr << "Failed to connect to KUKA robot: " << ec.message() << std::endl;
            callback(ec);
            attempt_reconnection();  // Attempt to reconnect on failure
        }
    });
}

// Function to attempt reconnection with the KUKA robot
void KukaClient::attempt_reconnection() {
    if (reconnect_attempts_ < max_reconnect_attempts_) {
        ++reconnect_attempts_;
        std::cout << "Reconnection attempt " << reconnect_attempts_ << " of " << max_reconnect_attempts_ << "..." << std::endl;

        // Use a timer to delay reconnection attempts
        reconnect_timer_ = std::make_shared<boost::asio::steady_timer>(io_context_, std::chrono::seconds(2));
        reconnect_timer_->async_wait([this](boost::system::error_code ec) {
            if (!ec) {
                std::cout << "Retrying connection to KUKA robot..." << std::endl;

                // Close and reset the existing client before reconnecting
                client_.close();  // Ensure the socket is properly closed
                client_ = TCPClient(io_context_, ip_, port_);  // Recreate the client to reset the socket

                connect([](boost::system::error_code) {});  // Re-attempt connection
            }
        });
    } else {
        std::cerr << "Max reconnection attempts reached. Giving up." << std::endl;
    }
}

// Function to send a read request for a specific variable
void KukaClient::readVariable(uint16_t message_id, const std::string& variable_name, std::function<void(boost::system::error_code, ResponseMessage)> callback) {
    ReadMessage read_msg(message_id, variable_name);
    std::vector<uint8_t> serialized_read_msg = read_msg.serialize();

    client_.async_send(serialized_read_msg, [this, callback](boost::system::error_code ec, std::size_t /* length */) {
        if (!ec) {
            client_.async_receive([this, callback](boost::system::error_code ec, const std::vector<uint8_t>& data, std::size_t /* length */) {
                if (!ec) {
                    ResponseMessage response_msg;
                    response_msg.deserialize(data);
                    callback(ec, response_msg);
                } else {
                    std::cerr << "Failed to receive response: " << ec.message() << std::endl;
                    callback(ec, ResponseMessage());
                    handle_disconnection(ec);  // Handle disconnection if receive failed
                }
            });
        } else {
            std::cerr << "Failed to send read request: " << ec.message() << std::endl;
            callback(ec, ResponseMessage());
            handle_disconnection(ec);  // Handle disconnection if send failed
        }
    });
}

// Function to send a write request for a specific variable
void KukaClient::writeVariable(uint16_t message_id, const std::string& variable_name, const std::string& value, std::function<void(boost::system::error_code, ResponseMessage)> callback) {
    WriteMessage write_msg(message_id, variable_name, value);
    std::vector<uint8_t> serialized_write_msg = write_msg.serialize();

    client_.async_send(serialized_write_msg, [this, callback](boost::system::error_code ec, std::size_t /* length */) {
        if (!ec) {
            client_.async_receive([this, callback](boost::system::error_code ec, const std::vector<uint8_t>& data, std::size_t /* length */) {
                if (!ec) {
                    ResponseMessage response_msg;
                    response_msg.deserialize(data);
                    callback(ec, response_msg);
                } else {
                    std::cerr << "Failed to receive response: " << ec.message() << std::endl;
                    callback(ec, ResponseMessage());
                    handle_disconnection(ec);  // Handle disconnection if receive failed
                }
            });
        } else {
            std::cerr << "Failed to send write request: " << ec.message() << std::endl;
            callback(ec, ResponseMessage());
            handle_disconnection(ec);  // Handle disconnection if send failed
        }
    });
}

// Function to handle disconnection scenarios
void KukaClient::handle_disconnection(boost::system::error_code ec) {
    if (ec == boost::asio::error::connection_reset || ec == boost::asio::error::connection_aborted ||
        ec == boost::asio::error::eof || ec == boost::asio::error::not_connected || ec == boost::asio::error::broken_pipe) {
        std::cout << "Connection lost or broken pipe detected. Attempting to reconnect..." << std::endl;
        attempt_reconnection();
    }
}

// Function to close the TCP connection
void KukaClient::close() {
    client_.close();
}
