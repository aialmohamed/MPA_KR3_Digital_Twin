#include "kr3r540_kinematics_action_server/kr3r540_kinematics_action_server.hpp"
#include <tuple>

using namespace std::placeholders;

namespace kr3r540_kinematics_action_server
{
    Kr3r540KinematicsActionServer::Kr3r540KinematicsActionServer(const rclcpp::NodeOptions &options)
        : Node("kr3r540_kinematics_action_server", options),
          kinematics_solver_("base_link", "link_6")
    {
        action_server_ = rclcpp_action::create_server<kr3r540_msgs::action::Kr3r540Kinemactis>(
            this, "kr3r540_kinematics",
            std::bind(&Kr3r540KinematicsActionServer::goalCallback, this, _1, _2),
            std::bind(&Kr3r540KinematicsActionServer::cancelCallback, this, _1),
            std::bind(&Kr3r540KinematicsActionServer::acceptedCallback, this, _1));

        robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/kr3r540_sim/robot_description",
            rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
            std::bind(&Kr3r540KinematicsActionServer::robotDescriptionCallback, this, _1));

        joint_state_sub_sim = this->create_subscription<sensor_msgs::msg::JointState>(
            "/kr3r540_sim/joint_states",
            rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE),
            std::bind(&Kr3r540KinematicsActionServer::simJointStateCallback, this, _1));

        joint_state_sub_real = this->create_subscription<sensor_msgs::msg::JointState>(
            "/kr3r540_real/joint_states",
            rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE),
            std::bind(&Kr3r540KinematicsActionServer::realJointStateCallback, this, _1));

        // "/kr3r540_digital_twin/digital_twin_joint_trajectory",
        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/kr3r540_digital_twin/digital_twin_joint_trajectory",
            10);
    }

    void Kr3r540KinematicsActionServer::simJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_sim_);
        current_joint_state_sim_ = msg;
    }

    void Kr3r540KinematicsActionServer::realJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_real_);
        current_joint_state_real_ = msg;
    }

    void Kr3r540KinematicsActionServer::robotDescriptionCallback(const std_msgs::msg::String &msg)
    {
        if (kinematics_solver_.initialize(msg.data))
        {

            RCLCPP_INFO(get_logger(), "Kinematics solver initialized with robot description.");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize kinematics solver.");
        }
    }

    rclcpp_action::GoalResponse Kr3r540KinematicsActionServer::goalCallback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const kr3r540_msgs::action::Kr3r540Kinemactis::Goal> goal)
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void Kr3r540KinematicsActionServer::acceptedCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<kr3r540_msgs::action::Kr3r540Kinemactis>> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Executing goal ");
        std::thread{std::bind(&Kr3r540KinematicsActionServer::execute, this, _1), goal_handle}.detach();
    }

    void Kr3r540KinematicsActionServer::execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<kr3r540_msgs::action::Kr3r540Kinemactis>> goal_handle)
    {
        auto goal = goal_handle->get_goal();

        // Extract x, y, z, roll, pitch for IK solving
        std::vector<double> position_goal(goal->cartesian_goal.begin(), goal->cartesian_goal.begin() + 5);

        // Extract gripper positions separately
        double finger_1_goal = goal->cartesian_goal[5];
        double finger_2_goal = goal->cartesian_goal[6];
        std::vector<double> joint_positions;
        rclcpp::Rate loop_rate(20);

        if (!kinematics_solver_.solveIK(position_goal, joint_positions))
        {
            RCLCPP_ERROR(get_logger(), "Inverse kinematics solver failed.");
            goal_handle->abort(std::make_shared<kr3r540_msgs::action::Kr3r540Kinemactis::Result>());
            return;
        }

        // Prepare the trajectory message
        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        trajectory_msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "flange_finger_1", "flange_finger_2"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_positions;

        // Add gripper goals to the positions
        point.positions.push_back(finger_1_goal);
        point.positions.push_back(finger_2_goal);
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);

        trajectory_msg.points.push_back(point);
        trajectory_pub_->publish(trajectory_msg);

        RCLCPP_INFO(get_logger(), "Published IK solution to arm controller.");

        auto feedback = std::make_shared<kr3r540_msgs::action::Kr3r540Kinemactis::Feedback>();
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_sim_);

            if (current_joint_state_sim_)
            {

                feedback->joints_cartesian_feedback.clear();
                for (const auto &position : current_joint_state_sim_->position)
                {
                    float adjusted_position = (std::abs(position) < 1e-6) ? 0.0f : static_cast<float>(position);
                    feedback->joints_cartesian_feedback.push_back(adjusted_position);
                }
                goal_handle->publish_feedback(feedback);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "No joint state available for feedback.");
            }
        }

        if (rclcpp::ok())
        {

            auto result = std::make_shared<kr3r540_msgs::action::Kr3r540Kinemactis::Result>();
            std::lock_guard<std::mutex> lock(joint_state_mutex_real_);
            {
                if (current_joint_state_real_)
                {
                    result->joints_result.clear();
                    for (const auto &position : current_joint_state_real_->position)
                    {
                        float adjusted_position = (std::abs(position) < 1e-6) ? 0.0f : static_cast<float>(position);
                        result->joints_result.push_back(adjusted_position);
                    }
                    goal_handle->succeed(result);
                    RCLCPP_INFO(get_logger(), "Goal succeeded, result sent.");
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "No joint state available for feedback.");
                }
            }
        }
    }

    rclcpp_action::CancelResponse Kr3r540KinematicsActionServer::cancelCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<kr3r540_msgs::action::Kr3r540Kinemactis>> goal_handle)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(kr3r540_kinematics_action_server::Kr3r540KinematicsActionServer)
