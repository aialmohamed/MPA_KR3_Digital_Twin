#ifndef KR3R540_KINEMATICS_ACTION_SERVER_HPP_
#define KR3R540_KINEMATICS_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <kr3r540_msgs/action/kr3r540_kinemactis.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include "kr3r540_kinematics_action_server/kinematics_solver.hpp"

#define FINGER_MAX 0.004
#define FINGER_MIN 0.001
namespace kr3r540_kinematics_action_server
{
    class Kr3r540KinematicsActionServer : public rclcpp::Node
    {
    public:
        explicit Kr3r540KinematicsActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_real;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_sim;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
        rclcpp_action::Server<kr3r540_msgs::action::Kr3r540Kinemactis>::SharedPtr action_server_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
        

        sensor_msgs::msg::JointState::SharedPtr current_joint_state_sim_;
        sensor_msgs::msg::JointState::SharedPtr current_joint_state_real_;
        std::mutex joint_state_mutex_real_;
        std::mutex joint_state_mutex_sim_;

        KinematicsSolver kinematics_solver_;


        void robotDescriptionCallback(const std_msgs::msg::String &msg);
        rclcpp_action::GoalResponse goalCallback(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const kr3r540_msgs::action::Kr3r540Kinemactis::Goal> goal);
        void acceptedCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<kr3r540_msgs::action::Kr3r540Kinemactis>> goal_handle);
        void execute(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<kr3r540_msgs::action::Kr3r540Kinemactis>> goal_handle);
        rclcpp_action::CancelResponse cancelCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<kr3r540_msgs::action::Kr3r540Kinemactis>> goal_handle);

            void simJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
            void realJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    };
}

#endif // KR3R540_KINEMATICS_ACTION_SERVER_HPP_
