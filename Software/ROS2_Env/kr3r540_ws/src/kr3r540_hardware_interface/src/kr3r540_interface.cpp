#include "kr3r540_hardware_interface/kr3r540_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <angles/angles.h>
#include <regex>
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace kr3r540_hardware_interface
{
#pragma region Ros2_interface_methods
    Kr3r540Interface::Kr3r540Interface()
        : io_context_(),
          work_guard_(boost::asio::make_work_guard(io_context_))
    {
        io_thread_ = std::thread([this]()
                                 {
                RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Starting io_context in a separate thread...");
                io_context_.run(); });
    }
    Kr3r540Interface::~Kr3r540Interface()
    {
        RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Stopping io_context...");
        io_context_.stop();
    }
    CallbackReturn Kr3r540Interface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        bool res = setup_ip_and_port(hardware_info);
        if (!res)
        {
            return CallbackReturn::ERROR;
        }
        init_positions();

        try
        {
            kuka_client_ = std::make_unique<KukaClient>(io_context_, ip_address_, port_);
            RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "KukaClient initialized with IP: %s, Port: %s", ip_address_.c_str(), port_.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Kr3r540Interface"), "Failed to initialize KukaClient: %s", e.what());
            return CallbackReturn::ERROR;
        }

        return result;
    }
    std::vector<hardware_interface::StateInterface> Kr3r540Interface::export_state_interfaces()

    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t joint_idx = 0; joint_idx < info_.joints.size(); ++joint_idx)
        {
            RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Joint name: %s", info_.joints[joint_idx].name.c_str());
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[joint_idx].name, hardware_interface::HW_IF_POSITION, &position_state_[joint_idx]));
        }
        return state_interfaces;
    }
    std::vector<hardware_interface::CommandInterface> Kr3r540Interface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t joint_idx = 0; joint_idx < info_.joints.size(); ++joint_idx)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[joint_idx].name, hardware_interface::HW_IF_POSITION, &position_commands_[joint_idx]));
        }
        return command_interfaces;
    }
    CallbackReturn Kr3r540Interface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {

        prev_position_commands_ = position_commands_;

        kuka_client_->connect([this](boost::system::error_code ec)
                              {
            if (!ec) {
                RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Connected to KUKA robot successfully.");
                is_connected_ = true;
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("Kr3r540Interface"), "Failed to connect to KUKA robot: %s", ec.message().c_str());
                is_connected_ = false;
            } });

        // Wait for the connection to be established (add a timeout if necessary)
        while (!is_connected_)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        return CallbackReturn::SUCCESS;
    }
    CallbackReturn Kr3r540Interface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Deactivating Kr3r540Interface...");

        if (kuka_client_ && is_connected_)
        {
            kuka_client_->close();
            is_connected_ = false;
            RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "KUKA client connection closed.");
        }

        // Stop the io_context
        io_context_.stop();
        RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "io_context stopped.");

        // Join the io_context thread to ensure it shuts down cleanly
        if (io_thread_.joinable())
        {
            io_thread_.join();
            RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "io_context thread joined successfully.");
        }
        RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "io_context stopped.");
        work_guard_.reset();
        RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Work guard stopped.");

        position_commands_.clear();
        prev_position_commands_.clear();
        position_state_.clear();
        RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Command and state vectors cleared.");
        return CallbackReturn::SUCCESS;
    }
    hardware_interface::return_type Kr3r540Interface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if (!is_connected_)
        {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("Kr3r540Interface"), "KUKA robot is not connected yet. Waiting for connection...");
            return hardware_interface::return_type::ERROR;
        }

        // READ THE GRIPPER
        kuka_client_->readVariable(5, "GripperFlag", [this](boost::system::error_code ec, ResponseMessage response)
                                   {
                if (!ec) {
                    std::string gripper_flag = response.getVariableValue();
                    //RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Successfully received gripper flag: %s", gripper_flag.c_str());
                    if (gripper_flag == "TRUE") {
                        position_state_[6] = FINGER_1_OPEN_READ;
                        position_state_[7] = FINGER_2_OPEN_READ;
                        local_gripper_flag_ = true;
                    } else if(gripper_flag == "FALSE") {
                        position_state_[6] = FINGER_1_CLOSE;
                        position_state_[7] = FINGER_2_CLOSE;
                        local_gripper_flag_ = false;
                    }
                    previous_position_state_[6] = position_state_[6];
                    previous_position_state_[7] = position_state_[7];
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("Kr3r540Interface"), "Failed to read gripper flag: %s", ec.message().c_str());
                    position_state_[6] = previous_position_state_[6];          
                    position_state_[7] = previous_position_state_[7];      
                } });

        kuka_client_->readVariable(1, "$AXIS_ACT", [this](boost::system::error_code ec, ResponseMessage response)
                                   {
        if (!ec) {
            std::string axis_data = response.getVariableValue();
            //RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Successfully received joint data: %s", axis_data.c_str());
            for(size_t joint_idx = 0; joint_idx < 6; ++joint_idx)
            {
                previous_position_state_[joint_idx] = position_state_[joint_idx];
            }
            try {
                parse_axis_data(axis_data);
            } catch (const std::exception& e) {
                //RCLCPP_ERROR(rclcpp::get_logger("Kr3r540Interface"), "Error parsing joint data: %s", e.what());
                for( size_t joint_idx = 0; joint_idx < 6; ++joint_idx)
                {
                    position_state_[joint_idx] = previous_position_state_[joint_idx];
                }
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("Kr3r540Interface"), "Failed to read joint data: %s", ec.message().c_str());
            } });

        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type Kr3r540Interface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {

        if (!is_connected_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Kr3r540Interface"), "KUKA robot is not connected. Cannot send commands.");
            return hardware_interface::return_type::ERROR;
        }

        // Apply threshold to ignore near-zero values
        for (auto &command : position_commands_)
        {
            if (std::abs(command) < 1e-5)
            {
                command = 0.0;
            }
        }
        

        std::vector<double> arm_commands(position_commands_.begin(), position_commands_.begin() + 6);
        std::string command_data = postion_commands_to_string(position_commands_);
        
        std::ostringstream ss;
        /*
        ss << "position_commands_: [";
        for (size_t i = 0; i < command_data.size(); ++i)
        {
            ss << command_data[i];
            if (i < command_data.size() - 1)
            {
                ss << ", ";
            }
        }
        ss << "]";
        */
        // Print the formatted vector
        //RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "%s", ss.str().c_str());
        //RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Converted command_data: %s", command_data.c_str());

        kuka_client_->writeVariable(2, "MYAXIS", command_data, [this](boost::system::error_code ec, ResponseMessage response)
                                    {
            if (!ec) {
                prev_position_commands_ = position_commands_;
            } else {
                //position_commands_ = prev_position_commands_;
                RCLCPP_ERROR(rclcpp::get_logger("Kr3r540Interface"), "Failed to write joint data: %s", ec.message().c_str());
            } });

        bool new_gripper_flag = (round_to_thousandth(position_commands_[6]) >= FINGER_1_OPEN_READ);
        // RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Gripper finger1: %f", position_commands_[6]);

        if (new_gripper_flag != local_gripper_flag_)
        {
            kuka_client_->writeVariable(5, "GripperFlag", new_gripper_flag ? "TRUE" : "FALSE", [this, new_gripper_flag](boost::system::error_code ec, ResponseMessage response)
                                        {
        if (!ec) {
            local_gripper_flag_ = new_gripper_flag;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("Kr3r540Interface"), "Failed to write gripper flag: %s", ec.message().c_str());
        } });
        }

        return hardware_interface::return_type::OK;
    }

#pragma endregion
#pragma region private_methods
    // Function to round to the nearest thousandth (0.001 precision)
    double Kr3r540Interface::round_to_thousandth(double value)
    {
        return std::round(value * 1000.0) / 1000.0;
    }
    double Kr3r540Interface::round_to_hundredth(double value)
    {
        return std::round(value * 100.0) / 100.0;
    }
    void Kr3r540Interface::parse_axis_data(const std::string &axis_data)
    {

        // Regular expression to capture the A1 through A6 values
        std::regex axis_regex(R"(A1\s*(-?\d+\.\d+),\s*A2\s*(-?\d+\.\d+),\s*A3\s*(-?\d+\.\d+),\s*A4\s*(-?\d+\.\d+),\s*A5\s*(-?\d+\.\d+),\s*A6\s*(-?\d+\.\d+))");
        std::smatch matches;

        // Use regex to search for A1 through A6 values
        if (std::regex_search(axis_data, matches, axis_regex) && matches.size() == 7)
        {
            // Extract and convert the joint values (A1 to A6) to doubles
            for (size_t i = 1; i <= 6; ++i)
            {
                double angle_in_degrees = std::stod(matches[i].str());
                position_state_[i - 1] = angles::from_degrees(angle_in_degrees);
            }
        }
        else
        {
            throw std::runtime_error("Failed to parse joint data with regex.");
        }
    }
    std::string Kr3r540Interface::postion_commands_to_string(const std::vector<double> &position_commands)
    {
        std::vector<double> position_commands_copy = position_commands;
        limit_check(position_commands_copy);
        std::vector<double> position_commands_deg(info_.joints.size(), 0.0);
        for (size_t position_idx = 0; position_idx < position_commands.size(); ++position_idx)
        {
           // position_commands_copy[position_idx] = std::round(position_commands_copy[position_idx] * 10.0) / 10.0;
            position_commands_deg[position_idx] = angles::to_degrees(position_commands_copy[position_idx]);
        }

        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << "{A1 " << position_commands_deg[0] << ", "
           << "A2 " << position_commands_deg[1] << ", "
           << "A3 " << position_commands_deg[2] << ", "
           << "A4 " << position_commands_deg[3] << ", "
           << "A5 " << position_commands_deg[4] << ", "
           << "A6 " << position_commands_deg[5] << "}";

        std::string command_data = ss.str();
        // RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "position_commands_ (degrees) after conversion: %s", command_data.c_str());

        return command_data;
    }
    bool Kr3r540Interface::setup_ip_and_port(const hardware_interface::HardwareInfo &hardware_info)
    {
        bool are_set = false;
        try
        {
            ip_address_ = hardware_info.hardware_parameters.at("ip");
            port_ = hardware_info.hardware_parameters.at("port");
            RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "IP address:port : %s:%s", ip_address_.c_str(), port_.c_str());
            are_set = true;
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Kr3r540Interface"), "Missing hardware parameters: %s", e.what());
            are_set = false;
        }
        return are_set;
    }
    void Kr3r540Interface::init_positions()
    {
        RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Activating the Kr3r540Interface");
        position_commands_.resize(info_.joints.size(), 0.0);
        prev_position_commands_.resize(info_.joints.size(), 0.0);
        position_state_.resize(info_.joints.size(), 0.0);
        previous_position_state_.resize(info_.joints.size(), 0.0);
        RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Initialized command and state vectors.");

        position_commands_[0] = angles::from_degrees(INITIAL_A1);
        position_commands_[1] = angles::from_degrees(INITIAL_A2);
        position_commands_[2] = angles::from_degrees(INITIAL_A3);
        position_commands_[3] = angles::from_degrees(INITIAL_A4);
        position_commands_[4] = angles::from_degrees(INITIAL_A5);
        position_commands_[5] = angles::from_degrees(INITIAL_A6);
        position_commands_[6] = INITIAL_FINGER_1;
        position_commands_[7] = INITIAL_FINGER_2;
    }
    void Kr3r540Interface::limit_check(std::vector<double> &position_commands)
    {
        const double min_limits[] = {A1_MIN_, A2_MIN_, A3_MIN_, A4_MIN_, A5_MIN_, A6_MIN_};
        const double max_limits[] = {A1_MAX_, A2_MAX_, A3_MAX_, A4_MAX_, A5_MAX_, A6_MAX_};

        for (size_t joint_idx = 0; joint_idx < 6; ++joint_idx)
        {
            double command_in_degrees = angles::to_degrees(position_commands[joint_idx]);

            if (command_in_degrees < min_limits[joint_idx] + TOLERANCE)
            {
                RCLCPP_WARN_ONCE(rclcpp::get_logger("Kr3r540Interface"),
                                 "Joint %zu command %.2f is below the minimum limit %.2f. Clamping to minimum + tolerance %.2f.",
                                 joint_idx + 1, command_in_degrees, min_limits[joint_idx], min_limits[joint_idx] + TOLERANCE);
                position_commands[joint_idx] = angles::from_degrees(min_limits[joint_idx] + TOLERANCE);
            }
            else if (command_in_degrees > max_limits[joint_idx] - TOLERANCE)
            {
                RCLCPP_WARN_ONCE(rclcpp::get_logger("Kr3r540Interface"),
                                 "Joint %zu command %.2f is above the maximum limit %.2f. Clamping to maximum - tolerance %.2f.",
                                 joint_idx + 1, command_in_degrees, max_limits[joint_idx], max_limits[joint_idx] - TOLERANCE);
                position_commands[joint_idx] = angles::from_degrees(max_limits[joint_idx] - TOLERANCE);
            }
        }
    }

#pragma endregion
}

PLUGINLIB_EXPORT_CLASS(kr3r540_hardware_interface::Kr3r540Interface, hardware_interface::SystemInterface)
