#ifndef TCP_CLIENT_HPP
#define TCP_CLIENT_HPP

#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <thread>

class TCPClient{

public:

    TCPClient(boost::asio::io_context& io_context,const std::string& host_ip,const std::string& port);
    
    // Asynchronous methods for connecting, sending, and receiving
    void async_tcp_connect(std::function<void(boost::system::error_code,boost::asio::ip::tcp::endpoint)> callback);
    void async_send(const std::vector<uint8_t>& message, std::function<void(boost::system::error_code , std::size_t)> callback);
    void async_receive(std::function<void(boost::system::error_code,const std::vector<uint8_t>&,std::size_t)> callback);
    void close();

private:
    boost::asio::ip::tcp::socket socket_; // Socket for communication
    boost::asio::ip::tcp::resolver resolver_; // Resolver for host name and port
    boost::asio::ip::tcp::resolver::results_type endpoints_; // Resolved endpoints
    std::string host_ip_; // IP of the host
    std::string port_; // Port of the host
};


#endif // TCP_CLIENT_HPP