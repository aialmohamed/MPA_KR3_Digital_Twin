#include <iostream>
#include <boost/asio.hpp>
#include "KukaClient.hpp"
#include <thread>

// Function to run the io_context in a separate thread
void runIoContext(boost::asio::io_context& io_context) {
    io_context.run();
}

int main() {
    boost::asio::io_context io_context;
    KukaClient kukaClient(io_context, "172.31.1.197", "7000");

    // Run the io_context in a separate thread
    std::thread io_context_thread(runIoContext, std::ref(io_context));

    // Connect to the KUKA robot asynchronously
    kukaClient.connect([&](boost::system::error_code ec) {
        if (!ec) {
            std::cout << "Connected to KUKA robot!" << std::endl;

            // Read the value of a variable after connection is established
            kukaClient.readVariable(1, "$OV_PRO", [](boost::system::error_code ec, ResponseMessage response) {
                if (!ec) {
                    std::cout << "Read Variable Response:" << std::endl;
                    response.printMessageDetails();
                } else {
                    std::cerr << "Failed to read variable: " << ec.message() << std::endl;
                }
            });
        }
    });

    // Assume some delay or external trigger for another operation
    std::this_thread::sleep_for(std::chrono::seconds(2));  // Simulate some delay

    // After some delay or condition, perform a write operation without re-connecting
    kukaClient.writeVariable(2, "$OV_PRO", "77", [](boost::system::error_code ec, ResponseMessage response) {
        if (!ec) {
            std::cout << "Write Variable Response:" << std::endl;
            response.printMessageDetails();
        } else {
            std::cerr << "Failed to write variable: " << ec.message() << std::endl;
        }
    });

    // Wait for the io_context to finish processing
    io_context_thread.join();

    return 0;
}
