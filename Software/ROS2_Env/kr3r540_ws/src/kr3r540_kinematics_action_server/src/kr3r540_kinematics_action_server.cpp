#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <kr3r540_msgs/action/kr3r540_kinemactis.hpp>
#include <thread>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <std_msgs/msg/string.hpp>

using namespace std::placeholders;

namespace kr3r540_kinematics_action_server
{
    class Kr3r540KinematicsActionServer : public rclcpp::Node
    {
    public:
        explicit Kr3r540KinematicsActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("kr3r540_kinematics_action_server", options)
        {
            action_server_ = rclcpp_action::create_server<kr3r540_msgs::action::Kr3r540Kinemactis>(this, "kr3r540_kinematics",
                                                                                                   std::bind(&Kr3r540KinematicsActionServer::goalCallback, this, _1, _2),
                                                                                                   std::bind(&Kr3r540KinematicsActionServer::cancelCallback, this, _1),
                                                                                                   std::bind(&Kr3r540KinematicsActionServer::acceptedCallback, this, _1));

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "kr3r540_kinematics_action_server has been started.");

            robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
                "/kr3r540_sim/robot_description",
                rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
                std::bind(&Kr3r540KinematicsActionServer::robotDescriptionCallback, this, _1));

            joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/kr3r540_sim/joint_states",
                rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE),
                std::bind(&Kr3r540KinematicsActionServer::jointStateCallback, this, _1));
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
        std::mutex joint_state_mutex_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
        rclcpp_action::Server<kr3r540_msgs::action::Kr3r540Kinemactis>::SharedPtr action_server_;
        KDL::Tree tree_;
        KDL::Chain chain_;
        KDL::JntArray joint_positions_init_;

        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            current_joint_state_ = msg; // Store the current joint state
        }

        void robotDescriptionCallback(const std_msgs::msg::String &msg)
        {
            const std::string urdf = msg.data;
            if (kdl_parser::treeFromString(urdf, tree_))
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully parsed robot description into KDL tree");
                tree_.getChain("base_link", "greifer", chain_);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Chain base_link to greifer has %d joints", chain_.getNrOfJoints());
                joint_positions_init_ = KDL::JntArray(chain_.getNrOfJoints());
                for (int joint_idx = 0; joint_idx < chain_.getNrOfJoints(); joint_idx++)
                {
                    joint_positions_init_(joint_idx) = -0.1;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized joint_%d positions to %d", joint_idx, joint_positions_init_(joint_idx));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to parse URDF into KDL tree");
            }
        }
        rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const kr3r540_msgs::action::Kr3r540Kinemactis::Goal> goal)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received goal request with order: ");

            // values : xmin = -0.4 xmax = 0.4 ymin = -0.3 ymax = 0.3 zmin = 0 zmax = 0.8
            if (goal->cartesian_goal[0] <= -0.4 || goal->cartesian_goal[0] >= 0.4 || goal->cartesian_goal[1] <= -0.3 || goal->cartesian_goal[1] >= 0.3 || goal->cartesian_goal[2] <= 0 || goal->cartesian_goal[2] >= 0.8)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal is out of bounds.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            // Check if the goal contains a vector and print each element
            std::ostringstream oss;
            for (const auto &val : goal->cartesian_goal)
            {
                oss << val << " "; // concatenate each value into the stream
            }

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Cartesian goal: " << oss.str()); // later here we need to extract the goal and use kdl to calculate the joint angles

            // also we need to check if the goal is bounded by the joint limits or not and return the appropriate response

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<kr3r540_msgs::action::Kr3r540Kinemactis>> goal_handle)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal has been accepted.");

            // we need to use threading here otherwise the action server will be blocked
            std::thread{std::bind(&Kr3r540KinematicsActionServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<kr3r540_msgs::action::Kr3r540Kinemactis>> goal_handle)
        {

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing goal.");
            rclcpp::Rate loop_rate(1);

            // Create a solver
            auto solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);

            // Extract the goal from the action request
            const auto goal = goal_handle->get_goal();

            // Check if the goal has the correct number of coordinates (x, y, z)
            if (goal->cartesian_goal.size() < 3)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid goal: less than 3 coordinates provided.");
                goal_handle->abort(std::make_shared<kr3r540_msgs::action::Kr3r540Kinemactis::Result>());
                return;
            }

            // Assign the x, y, z coordinates from the cartesian_goal field
            double x = goal->cartesian_goal[0];
            double y = goal->cartesian_goal[1];
            double z = goal->cartesian_goal[2];

            // Create a KDL Frame using the extracted goal values
            const KDL::Frame p_in(KDL::Vector(x, y, z));
            KDL::JntArray q_out(chain_.getNrOfJoints());

            // Perform the inverse kinematics
            if (solver_->CartToJnt(joint_positions_init_, p_in, q_out) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Inverse kinematics solver failed.");
                goal_handle->abort(std::make_shared<kr3r540_msgs::action::Kr3r540Kinemactis::Result>());
                return;
            }

            // Prepare feedback for the client
            auto feedback = std::make_shared<kr3r540_msgs::action::Kr3r540Kinemactis::Feedback>();
            auto &cartesian_goal_feedback = feedback->joints_cartesian_feedback;
            
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                cartesian_goal_feedback.clear();
                for (size_t i = 0; i < current_joint_state_->position.size(); ++i)
                {
                    cartesian_goal_feedback.push_back(current_joint_state_->position[i]);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint %d: %f", i, current_joint_state_->position[i]);
                }
            }
            // If execution reaches here, assume success
            auto result = std::make_shared<kr3r540_msgs::action::Kr3r540Kinemactis::Result>();
            for (size_t i = 0; i < current_joint_state_->position.size(); ++i)
            {
                result->joints_result.push_back(current_joint_state_->position[i]);
            }
            goal_handle->succeed(result);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded.");
            // Sleep to simulate processing
            // loop_rate.sleep();
        }

        rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<kr3r540_msgs::action::Kr3r540Kinemactis>> goal_handle)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to cancel goal.");
            return rclcpp_action::CancelResponse::ACCEPT;
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(kr3r540_kinematics_action_server::Kr3r540KinematicsActionServer)
