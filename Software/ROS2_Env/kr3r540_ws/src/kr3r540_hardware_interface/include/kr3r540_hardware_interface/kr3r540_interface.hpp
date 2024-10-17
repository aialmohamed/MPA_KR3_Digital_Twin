#ifndef KR3R540_INTERFACE_HPP_
#define KR3R540_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <thread>
#include <boost/asio.hpp>
#include <vector>
#include "KukaClient/KukaClient.hpp"

namespace kr3r540_hardware_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Kr3r540Interface : public hardware_interface::SystemInterface
{
public:
    Kr3r540Interface();
    virtual ~Kr3r540Interface();

    virtual CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    boost::asio::io_context io_context_;
    std::unique_ptr<std::thread> io_context_thread_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::unique_ptr<KukaClient> kuka_client_;
    std::string ip__;
    std::string port__;
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr kuka_callback_group_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    std::vector<double> position_commands_;
    std::vector<double> prev_position_commands_;
    std::vector<double> position_state_;
    std::atomic<bool> is_connected_{false}; // Add this member variable
    std::mutex command_mutex_;
    std::mutex state_mutex_;
    double command_threshold_ = 0.001;

    void parse_axis_act_data(const std::string& data);
};

}

#endif // KR3R540_INTERFACE_HPP_
