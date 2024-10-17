#include "kr3r540_hardware_interface/kr3r540_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <angles/angles.h>
#include <regex>
#include "MYAXIS_type/MYAXIS_type.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace kr3r540_hardware_interface 
{
    Kr3r540Interface::Kr3r540Interface()
    : work_guard_(boost::asio::make_work_guard(io_context_))
    , io_context_thread_(nullptr)
    , kuka_client_(nullptr)
    {
            // Create a node handle for managing ROS 2 callback groups
        node_ = std::make_shared<rclcpp::Node>("kr3r540_interface_node");

        // Create a callback group for KUKA client
        kuka_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Assign a separate thread executor for the callback group
        executor_.add_callback_group(kuka_callback_group_, node_->get_node_base_interface());

        kuka_client_ = std::make_unique<KukaClient>(io_context_, "172.31.1.197", "7000");
        RCLCPP_INFO(node_->get_logger(), "created kuka client ");
        io_context_thread_ = std::make_unique<std::thread>([this]() {
            try {
                RCLCPP_INFO(node_->get_logger(), "Starting io_context thread.");
                io_context_.run();
                RCLCPP_INFO(node_->get_logger(), "io_context run() has exited.");
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "Exception in io_context thread: %s", e.what());
            }
        });
        
    }

    Kr3r540Interface::~Kr3r540Interface()
    {
        try 
        {
            if (kuka_client_) {
                kuka_client_->close();
            }
            // Remove the work guard to allow io_context.run() to exit
            work_guard_.reset();
            // Stop the io_context and join the thread
            io_context_.stop();
            if (io_context_thread_ && io_context_thread_->joinable()) {
                io_context_thread_->join();
            }
        } catch(...) {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("Kr3r540Interface"), "Error while closing the KukaClient " << ip__ << ":" << port__);
        }
    }

    CallbackReturn Kr3r540Interface::on_init(const hardware_interface::HardwareInfo & hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != CallbackReturn::SUCCESS) {
            return result;
        }
        



        // Retrieve 'ip' parameter
        if (info_.hardware_parameters.find("ip") != info_.hardware_parameters.end())
            ip__ = info_.hardware_parameters.at("ip");
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("Kr3r540Interface"), "'ip' parameter is missing.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Retrieve 'port' parameter
        if (info_.hardware_parameters.find("port") != info_.hardware_parameters.end())
            port__ = info_.hardware_parameters.at("port");
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("Kr3r540Interface"), "'port' parameter is missing.");
            return hardware_interface::CallbackReturn::ERROR;
        }
         RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Initialized with IP: %s, Port: %s", ip__.c_str(), port__.c_str());
        std::thread([this]() {
                executor_.spin();
            }).detach(); // Detach to let the executor run independently

        position_commands_.reserve(hardware_info.joints.size());
        prev_position_commands_.reserve(hardware_info.joints.size());
        position_state_.reserve(hardware_info.joints.size());

        return result;
    }

    std::vector<hardware_interface::StateInterface> Kr3r540Interface::export_state_interfaces() 
    
    {
        std::vector<hardware_interface::StateInterface> state_interfaces; 
        for(size_t joint_idx =0; joint_idx < info_.joints.size(); ++joint_idx) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[joint_idx].name, hardware_interface::HW_IF_POSITION, &position_state_[joint_idx]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> Kr3r540Interface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for(size_t joint_idx = 0; joint_idx < info_.joints.size(); ++joint_idx) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[joint_idx].name, hardware_interface::HW_IF_POSITION, &position_commands_[joint_idx]));
        }
        return command_interfaces;

    }
    
    CallbackReturn Kr3r540Interface::on_activate(const rclcpp_lifecycle::State & previous_state) 
    {
            RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Activating the Kr3r540Interface");
                    RCLCPP_INFO(node_->get_logger(), "Activating the Kr3r540Interface");

            // Start the callback group processing
            //executor_.spin_some();

            // Proceed with connecting to the KUKA client
            kuka_client_->connect([this](boost::system::error_code ec) {
                if (!ec) {
                    RCLCPP_INFO(node_->get_logger(), "Successfully connected to KUKA robot.");
                    is_connected_ = true;
                }
                else {
                    RCLCPP_ERROR(node_->get_logger(), "Error while connecting to KUKA robot");
                    is_connected_ = false;
                    return CallbackReturn::FAILURE;
                }
            });
        position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        prev_position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        position_state_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Kr3r540Interface::on_deactivate(const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Deactivating the Kr3r540Interface");
        try
        {
            kuka_client_->close();
            is_connected_ = false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("Kr3r540Interface"), "Error while closing the KukaClient:");
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("Kr3r540Interface"), "Successfully disconnected from KUKA robot.");
        return CallbackReturn::SUCCESS;

    }


    hardware_interface::return_type Kr3r540Interface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        if (!is_connected_)
        {
            RCLCPP_WARN(rclcpp::get_logger("Kr3r540Interface"), "Not connected to robot. Skipping read.");
            return hardware_interface::return_type::OK;
        }

        std::string variable_name = "$AXIS_ACT"; 
        kuka_client_->readVariable(0, variable_name,
            [this](boost::system::error_code ec, ResponseMessage response) {
                if (!ec)
                {
                    std::string value = response.getVariableValue();
                    try
                    {
                        
                        parse_axis_act_data(value);
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("Kr3r540Interface"), "Exception while parsing joint positions: %s", e.what());
                    }
                }
                else
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("Kr3r540Interface"), "Failed to read joint positions: %s", ec.message().c_str());
                }
            });

    return hardware_interface::return_type::OK;

    }
    hardware_interface::return_type Kr3r540Interface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
         if (!is_connected_)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("Kr3r540Interface"), "Not connected to robot. Skipping write.");
        return hardware_interface::return_type::OK;
    }

    // Check if commands have changed significantly
    bool commands_changed = false;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        for (size_t i = 0; i < 6; ++i) // Only A1 to A6
        {
            if (std::abs(position_commands_[i] - prev_position_commands_[i]) > command_threshold_)
            {
                commands_changed = true;
                break;
            }
        }
    }

    if (commands_changed)
    {
        // Update previous commands
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            for (size_t i = 0; i < 6; ++i)
            {
                prev_position_commands_[i] = position_commands_[i];
            }
        }

        // Construct the command using MYAXIS_type
        MYAXIS_type axis_cmd;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            // Convert radians to degrees before setting values
            axis_cmd.setA1(angles::to_degrees(position_commands_[0]));
            axis_cmd.setA2(angles::to_degrees(position_commands_[1]));
            axis_cmd.setA3(angles::to_degrees(position_commands_[2]));
            axis_cmd.setA4(angles::to_degrees(position_commands_[3]));
            axis_cmd.setA5(angles::to_degrees(position_commands_[4]));
            axis_cmd.setA6(angles::to_degrees(position_commands_[5]));
        }

        std::string value = axis_cmd.toString(); // Get the command string

        // Variable name to write joint commands
        std::string variable_name = "MYAXIS"; // As per your robot's configuration

        // Initiate asynchronous write without blocking
        kuka_client_->writeVariable(0, variable_name, value,
            [this](boost::system::error_code ec, ResponseMessage response) {
                if (!ec)
                {
                    RCLCPP_DEBUG(
                        rclcpp::get_logger("Kr3r540Interface"), "Successfully wrote joint commands");
                }
                else
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("Kr3r540Interface"), "Failed to write joint commands");
                }
            });
    }

    // Return immediately without blocking
    return hardware_interface::return_type::OK;

    }

    void Kr3r540Interface::parse_axis_act_data(const std::string& data)
    {
        // Regular expression to match A1 to A6 values
        std::regex axis_regex(R"(A(\d+)\s*([-+]?\d*\.?\d+))");
        std::smatch match;
        std::string::const_iterator search_start(data.cbegin());

        // Temporary map to hold axis values
        std::map<int, double> axis_values;

        while (std::regex_search(search_start, data.cend(), match, axis_regex))
        {
            int axis_number = std::stoi(match[1].str());
            if (axis_number >= 1 && axis_number <= 6) // Only consider A1 to A6
            {
                double axis_value = std::stod(match[2].str());
                axis_values[axis_number] = axis_value;
            }

            // Move to the next position in the string
            search_start = match.suffix().first;
        }

        // Check if we have all expected joint values
        if (axis_values.size() != 6) // Expecting 6 axes
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("Kr3r540Interface"), "Received joint positions do not match expected size.");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
                // Update position_state_
                for (size_t i = 0; i < 6; ++i)
                {
                    int axis_number = static_cast<int>(i) + 1; // A1 to A6
                    double degrees = axis_values[axis_number];
                    double radians = angles::from_degrees(degrees);
                    position_state_[i] = radians;
                }
        }
    }
}

PLUGINLIB_EXPORT_CLASS(kr3r540_hardware_interface::Kr3r540Interface, hardware_interface::SystemInterface)
