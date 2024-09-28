// KukaClient.hpp

#ifndef KUKACLIENT_HPP
#define KUKACLIENT_HPP

#include "TcpClient.hpp"
#include "ReadMessage.hpp"
#include "WriteMessage.hpp"
#include "ResponseMessage.hpp"
#include <boost/asio.hpp>
#include <functional>

class KukaClient {
private:
    TCPClient client_;                 // TCPClient instance for managing TCP connections
    std::string ip_;                   // IP address of the KUKA robot
    std::string port_;                 // Port number for the TCP connection
    boost::asio::io_context& io_context_; // Reference to the I/O context

public:
    // Constructor to initialize the KukaClient with IP, port, and io_context
    KukaClient(boost::asio::io_context& io_context, const std::string& ip, const std::string& port);

    // Establish a connection to the KUKA robot asynchronously
    void connect(std::function<void(boost::system::error_code)> callback);

    // Send a read request for a specific variable
    void readVariable(uint16_t message_id, const std::string& variable_name, std::function<void(boost::system::error_code, ResponseMessage)> callback);

    // Send a write request for a specific variable
    void writeVariable(uint16_t message_id, const std::string& variable_name, const std::string& value, std::function<void(boost::system::error_code, ResponseMessage)> callback);

    // Close the TCP connection
    void close();
};

#endif // KUKACLIENT_HPP
