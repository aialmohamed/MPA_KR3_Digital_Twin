#ifndef FORWARD_KINEMATICS_SOLVER_HPP
#define FORWARD_KINEMATICS_SOLVER_HPP

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class ForwardKinematicsSolver
{
public:
    explicit ForwardKinematicsSolver(const std::string &robot_description);

    bool initialize(const std::string &base_link, const std::string &end_effector);

    bool computeForwardKinematics(
        const std::vector<double> &joint_positions,
        double &x, double &y, double &z,
        double &roll, double &pitch, double &yaw);
        size_t getNumberOfJoints() const;

private:
    KDL::Chain kdl_chain_;
    std::shared_ptr<KDL::Tree> kdl_tree_; // Store parsed tree
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
};

#endif // FORWARD_KINEMATICS_SOLVER_HPP
