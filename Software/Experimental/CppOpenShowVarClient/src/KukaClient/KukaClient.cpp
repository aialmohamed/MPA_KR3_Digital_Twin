// KukaClient.cpp

#include "KukaClient.hpp"

// Constructor implementation
KukaClient::KukaClient(boost::asio::io_context& io_context, const std::string& ip, const std::string& port)
    : client_(io_context, ip, port), ip_(ip), port_(port), io_context_(io_context) {}

// Establish a connection to the KUKA robot asynchronously
void KukaClient::connect(std::function<void(boost::system::error_code)> callback) {
    client_.async_tcp_connect([this, callback](boost::system::error_code ec, boost::asio::ip::tcp::endpoint endpoint) {
        if (!ec) {
            std::cout << "Successfully connected to KUKA robot at: " << endpoint << std::endl;
        } else {
            std::cerr << "Failed to connect to KUKA robot: " << ec.message() << std::endl;
        }
        callback(ec); // Call the callback function with the connection result
    });
}

// Send a read request for a specific variable
void KukaClient::readVariable(uint16_t message_id, const std::string& variable_name, std::function<void(boost::system::error_code, ResponseMessage)> callback) {
    // Create a ReadMessage object
    ReadMessage read_msg(message_id, variable_name);

    // Serialize the read message to a byte vector
    std::vector<uint8_t> serialized_read_msg = read_msg.serialize();

    // Send the read message asynchronously
    client_.async_send(serialized_read_msg, [this, callback](boost::system::error_code ec, std::size_t /* length */) {
        if (!ec) {
            // After sending, wait for the response asynchronously
            client_.async_receive([callback](boost::system::error_code ec, const std::vector<uint8_t>& data, std::size_t /* length */) {
                if (!ec) {
                    // Deserialize the received data into a ResponseMessage object
                    ResponseMessage response_msg;
                    response_msg.deserialize(data);
                    callback(ec, response_msg); // Pass the deserialized response to the callback
                } else {
                    std::cerr << "Failed to receive response: " << ec.message() << std::endl;
                    callback(ec, ResponseMessage()); // Pass an empty ResponseMessage on error
                }
            });
        } else {
            std::cerr << "Failed to send read request: " << ec.message() << std::endl;
            callback(ec, ResponseMessage()); // Pass an empty ResponseMessage on error
        }
    });
}

// Send a write request for a specific variable
void KukaClient::writeVariable(uint16_t message_id, const std::string& variable_name, const std::string& value, std::function<void(boost::system::error_code, ResponseMessage)> callback) {
    // Create a WriteMessage object
    WriteMessage write_msg(message_id, variable_name, value);

    // Serialize the write message to a byte vector
    std::vector<uint8_t> serialized_write_msg = write_msg.serialize();

    // Send the write message asynchronously
    client_.async_send(serialized_write_msg, [this, callback](boost::system::error_code ec, std::size_t /* length */) {
        if (!ec) {
            // After sending, wait for the response asynchronously
            client_.async_receive([callback](boost::system::error_code ec, const std::vector<uint8_t>& data, std::size_t /* length */) {
                if (!ec) {
                    // Deserialize the received data into a ResponseMessage object
                    ResponseMessage response_msg;
                    response_msg.deserialize(data);
                    callback(ec, response_msg); // Pass the deserialized response to the callback
                } else {
                    std::cerr << "Failed to receive response: " << ec.message() << std::endl;
                    callback(ec, ResponseMessage()); // Pass an empty ResponseMessage on error
                }
            });
        } else {
            std::cerr << "Failed to send write request: " << ec.message() << std::endl;
            callback(ec, ResponseMessage()); // Pass an empty ResponseMessage on error
        }
    });
}

// Close the TCP connection
void KukaClient::close() {
    client_.close();
}
