#include <iostream>
#include "TcpClient.hpp"


TCPClient::TCPClient(boost::asio::io_context& io_context, const std::string& host_ip, const std::string& port)
    : socket_(io_context), resolver_(io_context), host_ip_(host_ip), port_(port) 
    {
        endpoints_ = resolver_.resolve(host_ip_,port_);
    }

void TCPClient::async_tcp_connect(std::function<void(boost::system::error_code,boost::asio::ip::tcp::endpoint)> callback) {
    try {
    for (auto& endpoint_entry : endpoints_) {
        auto endpoint = endpoint_entry.endpoint();
    }
    boost::asio::async_connect(socket_, endpoints_,
        [this, callback](boost::system::error_code ec, boost::asio::ip::tcp::endpoint connection_endpoint) {
            callback(ec, connection_endpoint);
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
    auto buffer = std::make_shared<boost::asio::streambuf>();
    boost::asio::async_read(socket_, *buffer, boost::asio::transfer_at_least(1),
        [this, buffer, callback](boost::system::error_code ec, std::size_t received_data_length) {
            if (!ec) {
                std::istream stream(buffer.get());
                std::vector<uint8_t> data((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
                callback(ec, data, received_data_length);
            } else {
                std::cerr << "Receive error: " << ec.message() << std::endl;
                callback(ec, {}, 0);
            }
        });
}

    bool TCPClient::is_open() const {
        return socket_.is_open();
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
