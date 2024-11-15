
#include "kr3r540_forward_kinematics_node/kr3r540_forward_kinematics_node.hpp"

int main (int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kr3r540_forward_kinematics_node::Kr3r540ForwardKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}