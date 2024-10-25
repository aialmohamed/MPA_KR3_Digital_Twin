#include "kr3r540_kinematics_action_server/kinematics_solver.hpp"

KinematicsSolver::KinematicsSolver(const std::string &base_link, const std::string &end_effector)
    : base_link_(base_link), end_effector_(end_effector) {}

bool KinematicsSolver::initialize(const std::string &urdf)
{
    if (!kdl_parser::treeFromString(urdf, tree_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("KinematicsSolver"), "Failed to parse URDF into KDL tree");
        return false;
    }
    if (!tree_.getChain(base_link_, end_effector_, chain_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("KinematicsSolver"), "Failed to extract KDL chain from %s to %s", base_link_.c_str(), end_effector_.c_str());
        return false;
    }

    unsigned int nj = chain_.getNrOfJoints();
    joint_positions_min_.resize(nj);
    joint_positions_max_.resize(nj);
    joint_positions_last_.resize(nj);

    solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);

    return true;
}

bool KinematicsSolver::solveIK(const std::vector<double> &cartesian_goal, std::vector<double> &joint_positions)
{
    KDL::Frame p_in(KDL::Vector(cartesian_goal[0], cartesian_goal[1], cartesian_goal[2]));
    KDL::JntArray q_out(chain_.getNrOfJoints());
    joint_positions_last_.resize(chain_.getNrOfJoints());
    joint_positions_last_(0) =0.0;
    joint_positions_last_(1) =-1,57;
    joint_positions_last_(2) =1,57;
    joint_positions_last_(3) =0.0;
    joint_positions_last_(4) =0.0;
    joint_positions_last_(5) =0.0;


    int result = solver_->CartToJnt(joint_positions_last_, p_in, q_out);

    if (result < 0 )
    {
        RCLCPP_ERROR(rclcpp::get_logger("KinematicsSolver"), "IK solution not found or out of joint limits");
        return false;
    }

    joint_positions.resize(q_out.rows());
    for (size_t i = 0; i < q_out.rows(); ++i)
    {
        joint_positions[i] = q_out(i);
        RCLCPP_INFO(rclcpp::get_logger("KinematicsSolver"), "Joint %ld: %f", i, q_out(i));
    }
    joint_positions_last_ = q_out;

    return true;
}
void KinematicsSolver::setJointPositions(const std::vector<double> &joint_positions) {
    for (size_t i = 0; i < joint_positions.size() && i < joint_positions_last_.rows(); ++i) {
        joint_positions_last_(i) = joint_positions[i];
    }
}
