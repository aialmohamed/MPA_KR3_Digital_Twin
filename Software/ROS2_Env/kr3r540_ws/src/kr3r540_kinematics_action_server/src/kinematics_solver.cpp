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
    joint_positions_last_(0) = 0;
    joint_positions_last_(1) = -1.57;
    joint_positions_last_(2) = 1.57;
    joint_positions_last_(3) = 0;
    joint_positions_last_(4) = 0;
    joint_positions_last_(5) = 0;

    solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);

    return true;
}

bool KinematicsSolver::solveIK(const std::vector<double> &cartesian_goal, std::vector<double> &joint_positions)
{
    double roll = cartesian_goal[3] * M_PI / 180.0;
    double pitch = cartesian_goal[4] * M_PI / 180.0;
    KDL::Rotation orientation = KDL::Rotation::RPY(roll, pitch,0);
    KDL::Vector cart(cartesian_goal[0], cartesian_goal[1], cartesian_goal[2]);
    KDL::Frame p_in(orientation,cart);
    KDL::JntArray q_out(chain_.getNrOfJoints());
    
    joint_positions_last_.resize(chain_.getNrOfJoints());
    int result = solver_->CartToJnt(joint_positions_last_, p_in, q_out);

    if (result < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("KinematicsSolver"), "IK solution not found.");
        return false;
    }

    if (!areJointsWithinLimits(q_out))
    {
        RCLCPP_WARN(rclcpp::get_logger("KinematicsSolver"), "Joint angles out of bounds, clamping to limits.");
        clampJointsToLimits(q_out);
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

bool KinematicsSolver::computeForwardKinematics(const KDL::JntArray &joint_positions, KDL::Frame &end_effector_pose)
{
    KDL::ChainFkSolverPos_recursive fk_solver(chain_);
    int result = fk_solver.JntToCart(joint_positions, end_effector_pose);
    return result >= 0;
}

void KinematicsSolver::set_joint_positions_last_(const std::vector<double> &joint_positions)
{
    for (size_t i = 0; i < joint_positions.size() && i < joint_positions_last_.rows(); ++i)
    {
        joint_positions_last_(i) = joint_positions[i];
    }
}

bool KinematicsSolver::areJointsWithinLimits(const KDL::JntArray &joint_positions)
{
    std::vector<double> min_limits = {A1_MIN_, A2_MIN_, A3_MIN_, A4_MIN_, A5_MIN_, A6_MIN_};
    std::vector<double> max_limits = {A1_MAX_, A2_MAX_, A3_MAX_, A4_MAX_, A5_MAX_, A6_MAX_};
    
    for (size_t i = 0; i < joint_positions.rows(); ++i)
    {
        double position_deg = joint_positions(i) * 180.0 / M_PI; // Convert to degrees

        if (position_deg < (min_limits[i] + TOLERANCE) || position_deg > (max_limits[i] - TOLERANCE))
        {
            RCLCPP_WARN(rclcpp::get_logger("KinematicsSolver"),
                        "Joint %ld out of limits: %f (limit: [%f, %f])", i,
                        position_deg, min_limits[i] + TOLERANCE, max_limits[i] - TOLERANCE);
            return false;
        }
    }
    return true;
}
void KinematicsSolver::clampJointsToLimits(KDL::JntArray &joint_positions)
{
    std::vector<double> min_limits = {A1_MIN_, A2_MIN_, A3_MIN_, A4_MIN_, A5_MIN_, A6_MIN_};
    std::vector<double> max_limits = {A1_MAX_, A2_MAX_, A3_MAX_, A4_MAX_, A5_MAX_, A6_MAX_};
    
    for (size_t i = 0; i < joint_positions.rows(); ++i)
    {
        double position_deg = joint_positions(i) * 180.0 / M_PI; // Convert to degrees

        if (position_deg < (min_limits[i] + TOLERANCE))
        {
            joint_positions(i) = (min_limits[i] + TOLERANCE) * M_PI / 180.0; // Convert back to radians
            RCLCPP_WARN(rclcpp::get_logger("KinematicsSolver"),
                        "Clamping Joint %ld to min limit: %f", i, joint_positions(i));
        }
        else if (position_deg > (max_limits[i] - TOLERANCE))
        {
            joint_positions(i) = (max_limits[i] - TOLERANCE) * M_PI / 180.0; // Convert back to radians
            RCLCPP_WARN(rclcpp::get_logger("KinematicsSolver"),
                        "Clamping Joint %ld to max limit: %f", i, joint_positions(i));
        }
    }
}
