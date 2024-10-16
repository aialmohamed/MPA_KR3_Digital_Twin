#include <iostream>
#include <boost/asio.hpp>
#include "KukaClient.hpp"
#include "MYAXIS_type.hpp"
#include <thread>
#include <sstream>
#include <limits>   // For std::numeric_limits
#include <stdexcept> // For exception handling

int main() {
    // Get the IP address and port from the user
    std::string ip_address;
    std::string port;

    std::cout << "Enter the IP address of the KUKA robot: ";
    std::cin >> ip_address;
    std::cout << "Enter the port number: ";
    std::cin >> port;

    // Clear the newline character left in the input stream
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // Create an io_context for asynchronous operations
    boost::asio::io_context io_context;

    // Create a work guard to keep the io_context alive
    auto work_guard = boost::asio::make_work_guard(io_context);

    // Instantiate the KukaClient object
    KukaClient kukaClient(io_context, ip_address, port);

    // Run io_context.run() on a separate thread
    std::thread io_context_thread([&io_context]() {
        io_context.run();
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
    std::cout << "\nEnter commands:\n"
              << "read_axis            - to read the MYAXIS data from the robot\n"
              << "modify_axis          - to modify the MYAXIS data locally\n"
              << "write_axis           - to write MYAXIS data to the robot\n"
              << "exit                 - to exit the program\n";

    // Create a MYAXIS_type object to hold the MYAXIS data
    MYAXIS_type axisData;

    // Command loop
    while (true) {
        std::cout << "\n> ";
        std::getline(std::cin, command);

        // Check if the user wants to exit
        if (command == "exit") {
            break;
        } else if (command == "read_axis") {
            // Read the MYAXIS data from the robot
            kukaClient.readVariable(1, "$AXIS_ACT", [&axisData](boost::system::error_code ec, ResponseMessage response) {
                if (!ec) {
                    std::cout << "MYAXIS Data Received:" << std::endl;
                    std::string axisString = response.getVariableValue();
                    std::cout << "Raw MYAXIS Data: " << axisString << std::endl; // Optional: display raw data
                    try {
                        axisData.parseFromString(axisString);
                        std::cout << axisData.toString() << std::endl;
                    } catch (const std::exception& e) {
                        std::cerr << "Error parsing MYAXIS data: " << e.what() << std::endl;
                    }
                } else {
                    std::cerr << "Failed to read MYAXIS data: " << ec.message() << std::endl;
                }
            });
        } else if (command == "modify_axis") {
            // Modify the MYAXIS data locally
            double value;
            bool valid_input = true;
            try {
                std::cout << "Enter new value for A1 (" << MYAXIS_type::A1_MIN << " to " << MYAXIS_type::A1_MAX << "): ";
                std::cin >> value;
                axisData.setA1(value);

                std::cout << "Enter new value for A2 (" << MYAXIS_type::A2_MIN << " to " << MYAXIS_type::A2_MAX << "): ";
                std::cin >> value;
                axisData.setA2(value);

                std::cout << "Enter new value for A3 (" << MYAXIS_type::A3_MIN << " to " << MYAXIS_type::A3_MAX << "): ";
                std::cin >> value;
                axisData.setA3(value);

                std::cout << "Enter new value for A4 (" << MYAXIS_type::A4_MIN << " to " << MYAXIS_type::A4_MAX << "): ";
                std::cin >> value;
                axisData.setA4(value);

                std::cout << "Enter new value for A5 (" << MYAXIS_type::A5_MIN << " to " << MYAXIS_type::A5_MAX << "): ";
                std::cin >> value;
                axisData.setA5(value);

                std::cout << "Enter new value for A6 (" << MYAXIS_type::A6_MIN << " to " << MYAXIS_type::A6_MAX << "): ";
                std::cin >> value;
                axisData.setA6(value);
            } catch (const std::out_of_range& e) {
                std::cerr << "Input error: " << e.what() << std::endl;
                valid_input = false;
            } catch (const std::exception& e) {
                std::cerr << "Unexpected error: " << e.what() << std::endl;
                valid_input = false;
            }

            // Clear the newline character left in the input stream
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            if (valid_input) {
                std::cout << "Modified MYAXIS Data:" << std::endl;
                std::cout << axisData.toString() << std::endl;
            } else {
                std::cerr << "Modification failed due to invalid input." << std::endl;
            }
        } else if (command == "write_axis") {
            // Write the MYAXIS data back to the robot
            std::string axisString = axisData.toString();
            kukaClient.writeVariable(2, "MYAXIS", axisString, [](boost::system::error_code ec, ResponseMessage response) {
                if (!ec) {
                    std::cout << "MYAXIS Data Written Successfully." << std::endl;
                } else {
                    std::cerr << "Failed to write MYAXIS data: " << ec.message() << std::endl;
                }
            });
        } else {
            std::cerr << "Unknown command. Please use 'read_axis', 'modify_axis', 'write_axis', or 'exit'." << std::endl;
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
