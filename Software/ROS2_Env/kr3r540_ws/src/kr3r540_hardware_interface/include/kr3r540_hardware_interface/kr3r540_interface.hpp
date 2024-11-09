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

#define INITIAL_A1 0.0
#define INITIAL_A2 -90.0
#define INITIAL_A3 90.0
#define INITIAL_A4 0.0
#define INITIAL_A5 0.0
#define INITIAL_A6 0.0
#define INITIAL_FINGER_1 0
#define INITIAL_FINGER_2 0
#define FINGER_1_OPEN 1
#define FINGER_2_OPEN 1
#define FINGER_1_OPEN_READ 0.004
#define FINGER_2_OPEN_READ 0.004
#define FINGER_1_CLOSE 0.001
#define FINGER_2_CLOSE 0.001

/* Limits  this shall break the robot ! it needs to be 3 degrees blow this*/
#define A1_MIN_  -27.0
#define A1_MAX_  60.0
#define A2_MIN_  -110.0
#define A2_MAX_  50.0
#define A3_MIN_ 25.0
#define A3_MAX_  155.0
#define A4_MIN_  -175.0
#define A4_MAX_ 175.0
#define A5_MIN_  -120.0
#define A5_MAX_  120.0
#define A6_MIN_  -350.0
#define A6_MAX_  350.0
#define TOLERANCE 3.0

namespace kr3r540_hardware_interface
{

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class Kr3r540Interface : public hardware_interface::SystemInterface
    {
    public:
        Kr3r540Interface();
        virtual ~Kr3r540Interface();

        virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

        virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        std::string ip_address_;
        std::string port_;
        std::vector<double> position_commands_;
        std::vector<double> prev_position_commands_;
        std::vector<double> position_state_;
        std::vector<double> previous_position_state_;
        std::unique_ptr<KukaClient> kuka_client_;
        boost::asio::io_context io_context_;
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
        std::atomic<bool> is_connected_ = false;
        std::thread io_thread_;
        bool local_gripper_flag_ = false;

        void parse_axis_data(const std::string &axis_data);
        std::string postion_commands_to_string(const std::vector<double> &position_commands);
        bool setup_ip_and_port(const hardware_interface::HardwareInfo &hardware_info);
        void init_positions();
        void limit_check(std::vector<double> &position_commands);
        double round_to_thousandth(double value);
        double round_to_hundredth(double value);
    };

}

#endif // KR3R540_INTERFACE_HPP_
