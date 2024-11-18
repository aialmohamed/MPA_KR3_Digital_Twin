#include "kr3r540_forward_kinematics_node/kr3r540_forward_kinematics_node.hpp"

namespace kr3r540_forward_kinematics_node
{
    Kr3r540ForwardKinematicsNode::Kr3r540ForwardKinematicsNode(const rclcpp::NodeOptions &options)
        : Node("kr3r540_forward_kinematics", options),
            last_time_real_(this->now()),
            last_time_sim_(this->now())
    {
        // Subscribe to the robot description
        robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/kr3r540_real/robot_description",
            rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
            std::bind(&Kr3r540ForwardKinematicsNode::robotDescriptionCallback, this, std::placeholders::_1));

        // Subscribe to real robot joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/kr3r540_real/joint_states",
            10,
            std::bind(&Kr3r540ForwardKinematicsNode::jointStateCallback, this, std::placeholders::_1));

        cartesian_pose_pub_ = this->create_publisher<kr3r540_msgs::msg::CartesianPose>("/opcua/real_robot_cartesian_pose", 10);

        // for simulation
        joint_state_sim_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/kr3r540_sim/joint_states",
            10,
            std::bind(&Kr3r540ForwardKinematicsNode::SimJointStateCallback, this, std::placeholders::_1));
        // and we publish

        cartesian_pose_pub_sim_ = this->create_publisher<kr3r540_msgs::msg::CartesianPose>("/opcua/sim_robot_cartesian_pose", 10);

        RCLCPP_INFO(this->get_logger(), "Kr3r540ForwardKinematicsNode has been initialized.");
    }

    void Kr3r540ForwardKinematicsNode::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
    {

        solver_ = std::make_shared<ForwardKinematicsSolver>(msg->data);

        if (!solver_->initialize("base_link", "link_6"))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize forward kinematics solver.");
            return;
        }

        is_solver_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "KDL solver initialized successfully.");
    }

    void Kr3r540ForwardKinematicsNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {

        if (!is_solver_initialized_)
        {
            RCLCPP_WARN(this->get_logger(), "Solver not initialized yet. Skipping joint state processing.");
            return;
        }
        rclcpp::Duration throttle_duration(std::chrono::milliseconds(1000));

        if ((this->now() - last_time_real_) < throttle_duration)
        {
            RCLCPP_INFO(this->get_logger(), "Throttling joint state callback.");
            return;
        }
        last_time_real_ = this->now();

        if (!solver_)
        {
            RCLCPP_WARN(this->get_logger(), "Solver not initialized yet.");
            return;
        }

        if (msg->position.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty joint positions.");
            return;
        }

        std::vector<double> joint_positions = msg->position;

        size_t num_joints = solver_->getNumberOfJoints();
        if (joint_positions.size() < num_joints)
        {
            RCLCPP_ERROR(this->get_logger(), "Insufficient joint positions provided.");
            return;
        }

        std::vector<double> corrected_joint_positions(6);

        // Map the positions to the correct order this is  bug in the tf where it swaps joint 2 and 5
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "joint_1")
                corrected_joint_positions[0] = msg->position[i];
            else if (msg->name[i] == "joint_2")
                corrected_joint_positions[1] = msg->position[i];
            else if (msg->name[i] == "joint_3")
                corrected_joint_positions[2] = msg->position[i];
            else if (msg->name[i] == "joint_4")
                corrected_joint_positions[3] = msg->position[i];
            else if (msg->name[i] == "joint_5")
                corrected_joint_positions[4] = msg->position[i];
            else if (msg->name[i] == "joint_6")
                corrected_joint_positions[5] = msg->position[i];
        }

        double x, y, z, roll, pitch, yaw;
        if (solver_->computeForwardKinematics(corrected_joint_positions, x, y, z, roll, pitch, yaw))
        {
            auto cartesian_pose_msg = kr3r540_msgs::msg::CartesianPose();
            cartesian_pose_msg.x = x;
            cartesian_pose_msg.y = y;
            cartesian_pose_msg.z = z;
            cartesian_pose_msg.roll = roll;
            cartesian_pose_msg.pitch = pitch;
            cartesian_pose_msg.yaw = yaw;

            cartesian_pose_msg.gripper = cartesian_pose_msg.gripper = (msg->position[6] > 0.002);
             RCLCPP_INFO(this->get_logger(),
                      "Published Real Cartesian Pose: [x: %.2f, y: %.2f, z: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f, gripper: %s]",
                    x, y, z, roll, pitch, yaw, cartesian_pose_msg.gripper ? "true" : "false");

            cartesian_pose_pub_->publish(cartesian_pose_msg);

            // RCLCPP_INFO(this->get_logger(),
            //           "Published Cartesian Pose: [x: %.2f, y: %.2f, z: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f, gripper: %s]",
            //         x, y, z, roll, pitch, yaw, cartesian_pose_msg.gripper ? "true" : "false");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute forward kinematics.");
        }
    }
    void Kr3r540ForwardKinematicsNode::SimJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!is_solver_initialized_)
        {
            RCLCPP_WARN(this->get_logger(), "Solver not initialized yet. Skipping joint state processing.");
            return;
        }


        rclcpp::Duration throttle_duration(std::chrono::milliseconds(1000));

        if ((this->now() - last_time_sim_) < throttle_duration)
        {
            RCLCPP_INFO(this->get_logger(), "Throttling joint state callback.");
            return;
        }
        last_time_sim_ = this->now();

        if (!solver_)
        {
            RCLCPP_WARN(this->get_logger(), "Solver not initialized yet.");
            return;
        }

        if (msg->position.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty joint positions.");
            return;
        }

        std::vector<double> joint_positions = msg->position;

        size_t num_joints = solver_->getNumberOfJoints();
        if (joint_positions.size() < num_joints)
        {
            RCLCPP_ERROR(this->get_logger(), "Insufficient joint positions provided.");
            return;
        }

        std::vector<double> corrected_joint_positions(6);

        // Map the positions to the correct order this is  bug in the tf where it swaps joint 2 and 5
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "joint_1")
                corrected_joint_positions[0] = msg->position[i];
            else if (msg->name[i] == "joint_2")
                corrected_joint_positions[1] = msg->position[i];
            else if (msg->name[i] == "joint_3")
                corrected_joint_positions[2] = msg->position[i];
            else if (msg->name[i] == "joint_4")
                corrected_joint_positions[3] = msg->position[i];
            else if (msg->name[i] == "joint_5")
                corrected_joint_positions[4] = msg->position[i];
            else if (msg->name[i] == "joint_6")
                corrected_joint_positions[5] = msg->position[i];
        }

        double x, y, z, roll, pitch, yaw;
        if (solver_->computeForwardKinematics(corrected_joint_positions, x, y, z, roll, pitch, yaw))
        {
            auto cartesian_pose_msg = kr3r540_msgs::msg::CartesianPose();
            cartesian_pose_msg.x = x;
            cartesian_pose_msg.y = y;
            cartesian_pose_msg.z = z;
            cartesian_pose_msg.roll = roll;
            cartesian_pose_msg.pitch = pitch;
            cartesian_pose_msg.yaw = yaw;
           
            cartesian_pose_msg.gripper = cartesian_pose_msg.gripper = (msg->position[6] > 0.002);
              RCLCPP_INFO(this->get_logger(),
                      "Published Sim Cartesian Pose: [x: %.2f, y: %.2f, z: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f, gripper: %s]",
                    x, y, z, roll, pitch, yaw, cartesian_pose_msg.gripper ? "true" : "false");

            cartesian_pose_pub_sim_->publish(cartesian_pose_msg);

            // RCLCPP_INFO(this->get_logger(),
            //           "Published Cartesian Pose: [x: %.2f, y: %.2f, z: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f, gripper: %s]",
            //         x, y, z, roll, pitch, yaw, cartesian_pose_msg.gripper ? "true" : "false");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute forward kinematics.");
        }
    }

}
