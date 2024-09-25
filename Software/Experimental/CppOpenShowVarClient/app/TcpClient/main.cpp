#include <iostream>
#include "TcpClient.hpp"
#include <boost/asio.hpp>


int main() {

        // Boost.Asio I/O context for managing asynchronous operations
        boost::asio::io_context io_context;
        std::string ip = "172.31.1.197";
        std::string port = "7000";

        // Create the TCPClient with the IP and port of the server (KUKA robot or testing server)
        TCPClient client(io_context, ip ,port); // Replace with your server IP and port

        // connect to the socket : 
        client.async_tcp_connect([&client](boost::system::error_code ec,boost::asio::ip::tcp::endpoint connection_endpoints) {
            if(!ec) {
                std::cout << "connecting to : " << connection_endpoints << std::endl;
                // Construct the message to write $OV_PRO = 60
                std::vector<uint8_t> message;
                // Message ID (2 bytes, example: 1)
                uint16_t message_id = 1;
                message.push_back((message_id >> 8) & 0xFF);  // High byte
                message.push_back(message_id & 0xFF);         // Low byte

                // Content Length (2 bytes)
                // Mode (1 byte) + Variable Name Length (2 bytes) + Variable Name (7 bytes) + 
                // Variable Value Length (2 bytes) + Variable Value (2 bytes)
                uint16_t content_length = 1 + 2 + 7 + 2 + 2;
                message.push_back((content_length >> 8) & 0xFF);  // High byte
                message.push_back(content_length & 0xFF);         // Low byte

                // Mode (1 byte, 1 for write)
                uint8_t mode = 1;
                message.push_back(mode);

                // Variable name: $OV_PRO (7 bytes)
                std::string variable_name = "$OV_PRO";
                uint16_t var_name_len = variable_name.size();
                message.push_back((var_name_len >> 8) & 0xFF);  // High byte
                message.push_back(var_name_len & 0xFF);         // Low byte
                message.insert(message.end(), variable_name.begin(), variable_name.end());

                // Variable value: 60 (2 bytes, in ASCII)
                std::string value = "40";
                uint16_t value_len = value.size();
                message.push_back((value_len >> 8) & 0xFF);  // High byte
                message.push_back(value_len & 0xFF);         // Low byte
                message.insert(message.end(), value.begin(), value.end());

                // Send the message asynchronously
                client.async_send(message, [&client](boost::system::error_code ec, std::size_t sent_message_length){
                    if(!ec){
                        std::cout << "Sent message with length of : " << sent_message_length << std::endl;

                        // recive response
                        client.async_receive([](boost::system::error_code ec,const std::vector<uint8_t>& data,std::size_t received_data_length){
                            if(!ec) {
                                std::cout << "Received response: ";
                                for(auto byte : data) {
                                    std::cout << std::hex << static_cast<int>(byte) << " ";
                                }
                                std::cout << std::endl;
                            } else {
                                std::cerr << "Failed to receive response: " << ec.message() << std::endl;
                            }
                        });
                    }else {
                        std::cerr << "could not send the message error :" << ec.message() << std::endl;
                    }
                });
            }else {
                std::cerr << "could not connect to the socket error : " << ec.message() << std::endl;
            }
        });
        // Run the I/O context to initiate the asynchronous operations
        io_context.run();
        

    return 0;
}