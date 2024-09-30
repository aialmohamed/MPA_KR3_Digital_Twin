#include <iostream>
#include "TcpClient.hpp"


// Constructor to initialize the socket and resolver
TCPClient::TCPClient(boost::asio::io_context& io_context, const std::string& host_ip, const std::string& port)
    : socket_(io_context), resolver_(io_context), host_ip_(host_ip), port_(port) 
    {
        endpoints_ = resolver_.resolve(host_ip_,port_);
    }

// Asynchronous connection to the TCP server
void TCPClient::async_tcp_connect(std::function<void(boost::system::error_code,boost::asio::ip::tcp::endpoint)> callback) {
   // Debug statement to confirm the function is called
    std::cout << "Resolving and attempting connection to: " << host_ip_ << ":" << port_ << std::endl;
    try {
         // Loop through resolved endpoints and print them
    for (auto& endpoint_entry : endpoints_) {
        auto endpoint = endpoint_entry.endpoint();  // Get the endpoint from resolver entry
        std::cout << "Resolved endpoint: " << endpoint.address().to_string() << ":" << endpoint.port() << std::endl;
    }

    // Start async connection
    boost::asio::async_connect(socket_, endpoints_,
        [this, callback](boost::system::error_code ec, boost::asio::ip::tcp::endpoint connection_endpoint) {
            std::cout << "Async connect callback - Error: " << ec.message() << std::endl; // Debug statement
            callback(ec, connection_endpoint); // Invoke the callback with the error code and endpoint
        });
    }catch(const std::exception& ex) {
        std::cerr << "Exception during connection: " << ex.what() << std::endl;
    }
   
}

void TCPClient::async_send(const std::vector<uint8_t>& message, std::function<void(boost::system::error_code , std::size_t)> callback)
{
    boost::asio::async_write(socket_,boost::asio::buffer(message),
    [this,callback](boost::system::error_code ec, std::size_t sent_message_length){
        callback(ec,sent_message_length);
    });
}

void TCPClient::async_receive(std::function<void(boost::system::error_code,const std::vector<uint8_t>&,std::size_t)> callback)
{
     std::cout << "Setting up async receive..." << std::endl; // Debug statement
    auto buffer = std::make_shared<boost::asio::streambuf>();
    boost::asio::async_read(socket_, *buffer, boost::asio::transfer_at_least(1),
        [this, buffer, callback](boost::system::error_code ec, std::size_t received_data_length) {
            std::cout << "Async receive callback : " << ec.message() << " | Data length: " << received_data_length << std::endl; // Debug statement
            if (!ec) {
                std::istream stream(buffer.get());
                std::vector<uint8_t> data((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
                callback(ec, data, received_data_length);
            } else {
                std::cerr << "Receive error: " << ec.message() << std::endl; // Debug on receive error
                callback(ec, {}, 0);
            }
        });
}

void TCPClient::close () {
    if(socket_.is_open())
    {
        boost::system::error_code ec;
        socket_.close(ec);
        if(ec) {
            std::cerr <<"Error closing socket : " << ec.message() << std::endl;
        }
    }
}
