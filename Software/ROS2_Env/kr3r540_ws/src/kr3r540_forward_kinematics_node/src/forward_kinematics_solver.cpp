#include "kr3r540_forward_kinematics_node/forward_kinematics_solver.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>

ForwardKinematicsSolver::ForwardKinematicsSolver(const std::string &robot_description)
{
    // Parse the URDF into a KDL Tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromString(robot_description, kdl_tree))
    {
        throw std::runtime_error("Failed to parse robot description.");
    }

    // Save the tree for later initialization of the chain
    kdl_tree_ = std::make_shared<KDL::Tree>(kdl_tree);
}

bool ForwardKinematicsSolver::initialize(const std::string &base_link, const std::string &end_effector)
{
    // Extract the chain from the tree
    if (!kdl_tree_->getChain(base_link, end_effector, kdl_chain_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("ForwardKinematicsSolver"),
                     "Failed to extract KDL chain from %s to %s", base_link.c_str(), end_effector.c_str());
        return false;
    }

    // Initialize the FK solver
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    return true;
}

size_t ForwardKinematicsSolver::getNumberOfJoints() const
{
    return kdl_chain_.getNrOfJoints();
}


bool ForwardKinematicsSolver::computeForwardKinematics(
    const std::vector<double> &joint_positions,
    double &x, double &y, double &z,
    double &roll, double &pitch, double &yaw)
{
    if (joint_positions.size() != kdl_chain_.getNrOfJoints())
    {
        RCLCPP_ERROR(rclcpp::get_logger("ForwardKinematicsSolver"), 
                     "Mismatch in number of joint positions and KDL chain joints. Expected %d, got %ld.",
                     kdl_chain_.getNrOfJoints(), joint_positions.size());
        return false;
    }

    KDL::JntArray joint_array(kdl_chain_.getNrOfJoints());
    for (size_t i = 0; i < joint_positions.size(); ++i)
    {
        joint_array(i) = joint_positions[i];
    }


    KDL::Frame end_effector_frame;
    int fk_result = fk_solver_->JntToCart(joint_array, end_effector_frame);
    if (fk_result < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ForwardKinematicsSolver"),
                     "KDL solver returned error code: %d", fk_result);
        return false;
    }

    x = end_effector_frame.p.x();
    y = end_effector_frame.p.y();
    z = end_effector_frame.p.z();
    double r, p, y_rpy;
    end_effector_frame.M.GetRPY(r, p, y_rpy);
    roll = r;
    pitch = p;
    yaw = y_rpy;

    return true;
}
