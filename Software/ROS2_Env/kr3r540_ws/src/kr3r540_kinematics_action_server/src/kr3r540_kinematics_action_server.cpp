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

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/kr3r540_sim/joint_states",
            rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE),
            std::bind(&Kr3r540KinematicsActionServer::jointStateCallback, this, _1));

        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/kr3r540_real/arm_controller/joint_trajectory",
            10);
    }

    void Kr3r540KinematicsActionServer::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        current_joint_state_ = msg;
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
        /*
        if (goal->cartesian_goal[0] <= -0.4 || goal->cartesian_goal[0] >= 0.4 ||
            goal->cartesian_goal[1] <= -0.3 || goal->cartesian_goal[1] >= 0.3 ||
            goal->cartesian_goal[2] <= 0 || goal->cartesian_goal[2] >= 0.8)
        {
            RCLCPP_ERROR(get_logger(), "Goal is out of bounds.");
            return rclcpp_action::GoalResponse::REJECT;
        }*/
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
        std::vector<double> cartesian_goal(goal->cartesian_goal.begin(), goal->cartesian_goal.end());
        std::vector<double> joint_positions;

        RCLCPP_INFO(get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);

        // Solve IK and check success
        if (!kinematics_solver_.solveIK(cartesian_goal, joint_positions))
        {
            RCLCPP_ERROR(get_logger(), "Inverse kinematics solver failed.");
            goal_handle->abort(std::make_shared<kr3r540_msgs::action::Kr3r540Kinemactis::Result>());
            return;
        }
        // Publish joint trajectory to move the arm
        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        trajectory_msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"}; // Replace with actual joint names

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_positions;
        point.time_from_start = rclcpp::Duration::from_seconds(2.0); // Adjust time as needed

        trajectory_msg.points.push_back(point);
        trajectory_pub_->publish(trajectory_msg);
        loop_rate.sleep();
        RCLCPP_INFO(get_logger(), "Published IK solution to arm controller.");

        // Publish feedback with the current joint state
        auto feedback = std::make_shared<kr3r540_msgs::action::Kr3r540Kinemactis::Feedback>();
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            if (current_joint_state_)
            {
                feedback->joints_cartesian_feedback.clear();
                for (const auto &position : current_joint_state_->position)
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

        // Send the result once the trajectory has been sent
        auto result = std::make_shared<kr3r540_msgs::action::Kr3r540Kinemactis::Result>();
        result->joints_result.assign(joint_positions.begin(), joint_positions.end());
        goal_handle->succeed(result);

        RCLCPP_INFO(get_logger(), "Goal succeeded, result sent.");
    }

    rclcpp_action::CancelResponse Kr3r540KinematicsActionServer::cancelCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<kr3r540_msgs::action::Kr3r540Kinemactis>> goal_handle)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(kr3r540_kinematics_action_server::Kr3r540KinematicsActionServer)
