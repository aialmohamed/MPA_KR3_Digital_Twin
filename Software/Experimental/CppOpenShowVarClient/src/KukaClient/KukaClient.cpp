// KukaClient.cpp

#include "KukaClient.hpp"

KukaClient::KukaClient(boost::asio::io_context& io_context, const std::string& ip, const std::string& port)
    : client_(io_context, ip, port), ip_(ip), port_(port), io_context_(io_context), reconnect_attempts_(0) {}

void KukaClient::connect(std::function<void(boost::system::error_code)> callback) {
    client_.async_tcp_connect([this, callback](boost::system::error_code ec, boost::asio::ip::tcp::endpoint endpoint) {
        if (!ec) {
            reconnect_attempts_ = 0;
            callback(ec);
        } else {
            std::cerr << "Failed to connect to KUKA robot: " << ec.message() << std::endl;
            callback(ec);
            attempt_reconnection();
        }
    });
}
void KukaClient::attempt_reconnection() {
    if (reconnect_attempts_ < max_reconnect_attempts_) {
        ++reconnect_attempts_;
        std::cout << "Reconnection attempt " << reconnect_attempts_ << " of " << max_reconnect_attempts_ << "..." << std::endl;
        reconnect_timer_ = std::make_shared<boost::asio::steady_timer>(io_context_, std::chrono::seconds(2));
        reconnect_timer_->async_wait([this](boost::system::error_code ec) {
            if (!ec) {
                std::cout << "Retrying connection to KUKA robot..." << std::endl;
                client_.close();
                client_ = TCPClient(io_context_, ip_, port_);
                connect([](boost::system::error_code) {});
            }
        });
    } else {
        std::cerr << "Max reconnection attempts reached. Giving up." << std::endl;
    }
}

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
                    handle_disconnection(ec);
                }
            });
        } else {
            std::cerr << "Failed to send read request: " << ec.message() << std::endl;
            callback(ec, ResponseMessage());
            handle_disconnection(ec);
        }
    });
}
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
                    handle_disconnection(ec);
                }
            });
        } else {
            std::cerr << "Failed to send write request: " << ec.message() << std::endl;
            callback(ec, ResponseMessage());
            handle_disconnection(ec);
        }
    });
}
void KukaClient::handle_disconnection(boost::system::error_code ec) {
    if (ec == boost::asio::error::connection_reset || ec == boost::asio::error::connection_aborted ||
        ec == boost::asio::error::eof || ec == boost::asio::error::not_connected || ec == boost::asio::error::broken_pipe) {
        std::cout << "Connection lost or broken pipe detected. Attempting to reconnect..." << std::endl;
        attempt_reconnection();
    }
}
void KukaClient::close() {
    client_.close();
}
