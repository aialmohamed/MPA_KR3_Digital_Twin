#include <iostream>
#include <boost/asio.hpp>
#include "KukaClient.hpp"
#include <thread>
#include <future> // Include future to synchronize operations

// Function to run the io_context in a separate thread
void runIoContext(boost::asio::io_context& io_context) {
    std::cout << "Running io_context..." << std::endl;
    io_context.run();
}

int main() {
    boost::asio::io_context io_context;
    KukaClient kukaClient(io_context, "172.31.1.197", "7000");

    // Run the io_context in a separate thread
    std::thread io_context_thread(runIoContext, std::ref(io_context));

    // Use a promise to signal when the connection is established
    std::promise<void> connection_promise;
    std::future<void> connection_future = connection_promise.get_future();

    // Debug log to confirm main thread is running
    std::cout << "Main thread started. Attempting to connect..." << std::endl;

    // Connect to the KUKA robot asynchronously
    kukaClient.connect([&](boost::system::error_code ec) {
        if (!ec) {
            std::cout << "Connected to KUKA robot!" << std::endl;

            // Set the promise value when the connection is established
            connection_promise.set_value();

            // Read the value of a variable after connection is established
            kukaClient.readVariable(1, "MYAXIS", [](boost::system::error_code ec, ResponseMessage response) {
                if (!ec) {
                    std::cout << "Read Variable Response:" << std::endl;
                    response.printMessageDetails();
                } else {
                    std::cerr << "Failed to read variable: " << ec.message() << std::endl;
                }
            });
        } else {
            std::cerr << "Failed to connect: " << ec.message() << std::endl;
            connection_promise.set_value();  // Set promise to allow program to proceed even on failure
        }
    });

    // Add a timer to prevent hanging on connection
    boost::asio::steady_timer timer(io_context, std::chrono::seconds(5));  // 5-second timeout
    timer.async_wait([&](const boost::system::error_code& ec) {
        if (!ec && connection_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            std::cerr << "Connection timed out!" << std::endl;
            connection_promise.set_value();  // Unblock the main thread
            kukaClient.close();  // Close the client if the connection times out
        }
    });

    // Wait for the connection to be established (or fail)
    connection_future.wait();
    std::string Home = "{A1 0,A2 -90,A3 90,A4 0,A5 0,A6 0}";
    // After the connection is established, perform the write operation
    std::cout << "Attempting to write variable after connection is established..." << std::endl;
    kukaClient.writeVariable(2, "MYAXIS", "{A1 10,A2 -70 ,A3 50,A4 20,A5 15,A6 0}", [](boost::system::error_code ec, ResponseMessage response) {
        if (!ec) {
            std::cout << "Write Variable Response:" << std::endl;
            response.printMessageDetails();
        } else {
            std::cerr << "Failed to write variable: " << ec.message() << std::endl;
        }
    });

    // Join the io_context thread before exiting
    std::cout << "Joining io_context thread..." << std::endl;
    io_context_thread.join();

    // Close the connection properly
    std::cout << "Closing KUKA client connection..." << std::endl;
    kukaClient.close();

    return 0;
}
