#ifndef FORWARD_KINEMATICS_NODE_HPP
#define FORWARD_KINEMATICS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "forward_kinematics_solver.hpp"
#include <std_msgs/msg/string.hpp>
#include "kr3r540_msgs/msg/cartesian_pose.hpp"


namespace kr3r540_forward_kinematics_node
{
    class Kr3r540ForwardKinematicsNode : public rclcpp::Node
    {
    public:
        explicit Kr3r540ForwardKinematicsNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        bool is_solver_initialized_ = false;
        void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;

        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

        void SimJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sim_sub_;

        rclcpp::Publisher<kr3r540_msgs::msg::CartesianPose>::SharedPtr cartesian_pose_pub_;
        rclcpp::Publisher<kr3r540_msgs::msg::CartesianPose>::SharedPtr cartesian_pose_pub_sim_;

        rclcpp::Time last_time_real_;
        rclcpp::Time last_time_sim_;
        std::shared_ptr<ForwardKinematicsSolver> solver_;
    };
} // namespace forward_kinematics_node

#endif // FORWARD_KINEMATICS_NODE_HPP