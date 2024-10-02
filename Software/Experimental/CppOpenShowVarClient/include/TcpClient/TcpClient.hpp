#ifndef TCP_CLIENT_HPP
#define TCP_CLIENT_HPP

#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <thread>

/**
 * @brief The TCPClient class is responsible for managing the TCP/IP communication between the client and the server
 * 
 * It provides asynchronous methods for connecting, sending, 
 * and receiving data over a TCP connection. The TCPClient class 
 * leverages the Boost.Asio library for efficient and scalable asynchronous 
 * operations, ensuring that the client can handle multiple I/O operations without blocking 
 * the main thread. This class is used as the foundation for sending ReadMessage and WriteMessage 
 * objects to the server.
 */
class TCPClient{

private:
    /**
     * @brief  a tcp socket object to handle the connection based on boost.asio
     * 
     */
    boost::asio::ip::tcp::socket socket_;
    /**
     * @brief  a resolver object to resolve the host name to an ip address
     * 
     */
    boost::asio::ip::tcp::resolver resolver_; 
    /**
     * @brief  a list of endpoints to connect to
     * 
     */
    boost::asio::ip::tcp::resolver::results_type endpoints_;
    /**
     * @brief  host ip address
     * 
     */
    std::string host_ip_;
    /**
     * @brief  port number
     * 
     */
    std::string port_;

public:
    /**
     * @brief Construct a new TCPClient object
     * 
     * @param io_context  io_context object to handle the asynchronous operations
     * @param host_ip  host ip address
     * @param port  port number
     */
    TCPClient(boost::asio::io_context& io_context,const std::string& host_ip,const std::string& port);
    /**
     * @brief connect to the server asynchronously and call the callback function
     * 
     * @param callback  callback function to call after the connection is established
     */
   virtual void async_tcp_connect(std::function<void(boost::system::error_code,boost::asio::ip::tcp::endpoint)> callback);
   /**
    * @brief  send the message to the server asynchronously and call the callback function
    * 
    * @param message  message to send
    * @param callback  callback function to call after the message is sent
    */
   virtual void async_send(const std::vector<uint8_t>& message, std::function<void(boost::system::error_code , std::size_t)> callback);
   /**
    * @brief  receive the message from the server asynchronously and call the callback function
    * 
    * @param callback  callback function to call after the message is received
    */
   virtual void async_receive(std::function<void(boost::system::error_code,const std::vector<uint8_t>&,std::size_t)> callback);

    /**
     * @brief  check if the connection is open
     * 
     * @return true 
     * @return false 
     */
    bool is_open() const;


   /**
    * @brief  close the connection
    * 
    */
   virtual void close();
};


#endif // TCP_CLIENT_HPP