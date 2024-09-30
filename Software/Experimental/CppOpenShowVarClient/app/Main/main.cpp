#include <iostream>
#include <boost/asio.hpp>
#include "KukaClient.hpp"
#include <thread>
#include <sstream>


int main() {
    // Get the IP address and port from the user
    std::string ip_address;
    std::string port;

    std::cout << "Enter the IP address of the KUKA robot: ";
    std::cin >> ip_address;
    std::cout << "Enter the port number: ";
    std::cin >> port;

    // Create an io_context for asynchronous operations
    boost::asio::io_context io_context;

    // Create a work guard to keep the io_context alive
    auto work_guard = boost::asio::make_work_guard(io_context);

    // Instantiate the KukaClient object
    KukaClient kukaClient(io_context, ip_address, port);

    // Run io_context.run() on a separate thread
    std::thread io_context_thread([&io_context]() {
        std::cout << "Starting io_context.run() in separate thread..." << std::endl;
        io_context.run();
        std::cout << "io_context.run() has exited." << std::endl;
    });

    std::cout << "Attempting to connect to the KUKA robot..." << std::endl;

    // Start the connection asynchronously; connection errors and reconnections are handled internally
    kukaClient.connect([](boost::system::error_code ec) {
        if (!ec) {
            std::cout << "Connected to KUKA robot!" << std::endl;
        } else {
            std::cerr << "Failed to connect initially. Reconnection attempts will continue in the background." << std::endl;
        }
    });

    std::string command;
    std::cout << "Enter commands in the following format:\n"
              << "w varname value  - to write a variable\n"
              << "r varname        - to read a variable\n"
              << "Type 'exit' to close the connection and exit the program.\n";

    // Command loop
    while (true) {
        std::cout << "> ";
        std::getline(std::cin, command);

        // Check if the user wants to exit
        if (command == "exit") {
            break;
        }

        // Parse the command
        std::istringstream command_stream(command);
        std::string action, varname, value;
        command_stream >> action >> varname;

        if (action == "r") {
            // Read variable command
            kukaClient.readVariable(1, varname, [&varname](boost::system::error_code ec, ResponseMessage response) {
                if (!ec) {
                    std::cout << "Read Variable Response:" << std::endl;
                    std::cout << "Variable: " << varname << " value: " << response.getVariableValue() << std::endl;
                } else {
                    std::cerr << "Failed to read variable: " << ec.message() << std::endl;
                }
            });
        } else if (action == "w") {
            // Write variable command
            std::getline(command_stream >> std::ws, value); // Get the rest of the command as the value
            if (value.empty()) {
                std::cerr << "Please provide a value to write to the variable." << std::endl;
                continue;
            }

            kukaClient.writeVariable(2, varname, value, [](boost::system::error_code ec, ResponseMessage response) {
                if (!ec) {
                    std::cout << "Write Variable Response:" << std::endl;
                    response.printMessageDetails();
                } else {
                    std::cerr << "Failed to write variable: " << ec.message() << std::endl;
                }
            });
        } else {
            std::cerr << "Unknown command. Please use 'r varname' to read or 'w varname value' to write." << std::endl;
        }
    }

    // Close the connection properly before exiting
    std::cout << "Closing KUKA client connection..." << std::endl;
    kukaClient.close();

    // Remove the work guard to allow io_context.run() to exit
    work_guard.reset();

    // Stop the io_context and join the thread
    io_context.stop();
    if (io_context_thread.joinable()) {
        io_context_thread.join();
    }

    return 0;
}
