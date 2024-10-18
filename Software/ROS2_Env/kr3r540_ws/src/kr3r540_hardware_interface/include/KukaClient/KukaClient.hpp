// KukaClient.hpp

#ifndef KUKACLIENT_HPP
#define KUKACLIENT_HPP

#include "TcpClient/TcpClient.hpp"
#include "ReadMessage/ReadMessage.hpp"
#include "WriteMessage/WriteMessage.hpp"
#include "ResponseMessage/ResponseMessage.hpp"
#include <boost/asio.hpp>
#include <functional>

/**
 * @brief  This class handles communication with the KUKA robot.
 *
 * The `KukaClient` class provides methods for sending and receiving data
 * to and from a KUKA robot via a specified communication protocol.
 */
class KukaClient
{
private:
    /**
     * @brief  TCPClient object to handle the connection to the server
     *
     */
    std::shared_ptr<TCPClient> client_;

    /**
     * @brief  ip address of the server to connect to
     *
     */
    std::string ip_;

    /**
     * @brief  port number of the server to connect to
     *
     */
    std::string port_;

    /**
     * @brief io_context object to handle the asynchronous operations of TCPClient
     *
     */
    boost::asio::io_context &io_context_;

    /**
     * @brief  number of attempts to reconnect to the server after a disconnection
     *
     */
    int reconnect_attempts_;

    /**
     * @brief maximum number of attempts to reconnect to the server after a disconnection
     *
     */
    const int max_reconnect_attempts_ = 5;

    /**
     * @brief reconnect timer to schedule reconnection attempts
     *
     */
    std::shared_ptr<boost::asio::steady_timer> reconnect_timer_;

    /**
     * @brief Attempt to reconnect to the server after a disconnection
     *
     */
    void attempt_reconnection();

    /**
     * @brief handle the disconnection of the client from the server
     *
     * @param ec error code of the disconnection
     */
    void handle_disconnection(boost::system::error_code ec);

public:
    /**
     * @brief Set the Client object
     *
     * @param client
     */
    void setClient(std::shared_ptr<TCPClient> client)
    {
        client_ = client;
    }
    /**
     * @brief Get the Client object
     *
     * @return TCPClient&
     */
    std::shared_ptr<TCPClient> getClient() const
    {
        return client_;
    }
    /**
     * @brief Construct a new Kuka Client object
     *
     * @param io_context io_context object to handle the asynchronous operations of TCPClient
     * @param ip ip address of the server to connect to
     * @param port port number of the server to connect to
     */
    KukaClient(boost::asio::io_context &io_context, const std::string &ip, const std::string &port);

    /**
     * @brief Construct a new Kuka Client object (only used for testing purposes)
     *
     * @param io_context
     * @param ip
     * @param port
     * @param client
     */
    KukaClient(boost::asio::io_context &io_context, const std::string &ip, const std::string &port,
               std::shared_ptr<TCPClient> client);

    /**
     * @brief connect to the server asynchronously
     *
     * @param callback callback function to be called after the connection is established
     */
    void connect(std::function<void(boost::system::error_code)> callback);

    /**
     * @brief read the value of a variable from the server asynchronously
     *
     * @param message_id  message id of the request
     * @param variable_name  name of the variable to read
     * @param callback  callback function to be called after the response is received
     */
    void readVariable(uint16_t message_id, const std::string &variable_name, std::function<void(boost::system::error_code, ResponseMessage)> callback);

    /**
     * @brief  write the value of a variable to the server asynchronously
     *
     * @param message_id  message id of the request
     * @param variable_name  name of the variable to write
     * @param value  value to write
     * @param callback  callback function to be called after the response is received
     */
    void writeVariable(uint16_t message_id, const std::string &variable_name, const std::string &value, std::function<void(boost::system::error_code, ResponseMessage)> callback);

    /**
     * @brief  send a custom message to the server
     *
     */
    void close();
};

#endif // KUKACLIENT_HPP
