#include <rclcpp/rclcpp.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <urdf/model.h>
#include <std_msgs/msg/string.hpp>

class MyRobotKinematics : public rclcpp::Node
{
public:
    MyRobotKinematics() : Node("my_robot_kinematics")
    {
        // Subscribe to the robot description topic
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/kr3r540_sim/robot_description", 10,
            std::bind(&MyRobotKinematics::robot_description_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Waiting for robot description on topic: /kr3r540_sim/robot_description");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;

    void robot_description_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received robot description, parsing URDF...");

        // Parse the URDF string
        urdf::Model urdf_model;
        if (!urdf_model.initString(msg->data))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
            return;
        }

        // Convert URDF model to KDL tree
        if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
            return;
        }

        // Extract a chain from the tree
        std::string base_link = "base_link";  // Adjust these names as per your robot model
        std::string tip_link = "greifer";       // Assuming "tool0" is the end effector for KR3 R540
        if (!kdl_tree_.getChain(base_link, tip_link, kdl_chain_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from tree");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully created KDL chain with %d segments", kdl_chain_.getNrOfSegments());

        // Now you can work with the KDL chain
        for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
        {
            KDL::Segment segment = kdl_chain_.getSegment(i);
            RCLCPP_INFO(this->get_logger(), "Segment %d: %s", i, segment.getName().c_str());
            // You can access more properties of the segment here
        }

        // Unsubscribe after processing to avoid repeated processing
        subscription_.reset();
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyRobotKinematics>());
    rclcpp::shutdown();
    return 0;
}
