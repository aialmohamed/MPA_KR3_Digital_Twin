#ifndef KINEMATICS_SOLVER_HPP_
#define KINEMATICS_SOLVER_HPP_

#include <string>
#include <vector>
#include <tuple>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>

// Limits and tolerance
#define A1_MIN_  -27.0
#define A1_MAX_  60.0
#define A2_MIN_  -110.0
#define A2_MAX_  50.0
#define A3_MIN_  25.0
#define A3_MAX_  155.0
#define A4_MIN_  -175.0
#define A4_MAX_  175.0
#define A5_MIN_  -120.0
#define A5_MAX_  120.0
#define A6_MIN_  -350.0
#define A6_MAX_  350.0
#define TOLERANCE 3.0

class KinematicsSolver
{
public:
    KinematicsSolver(const std::string &base_link, const std::string &end_effector);
    bool initialize(const std::string &urdf);
    bool solveIK(const std::vector<double> &cartesian_goal, std::vector<double> &joint_positions);
    void setJointPositions(const std::vector<double> &joint_positions);
    bool computeForwardKinematics(const KDL::JntArray &joint_positions, KDL::Frame &end_effector_pose);
    void set_joint_positions_last_(const std::vector<double> &joint_positions);

private:
    bool areJointsWithinLimits(const KDL::JntArray &joint_positions);
    void clampJointsToLimits(KDL::JntArray &joint_positions);
    KDL::JntArray joint_positions_last_;
    KDL::JntArray joint_positions_min_;
    KDL::JntArray joint_positions_max_;
    std::string base_link_;
    std::string end_effector_;
    KDL::Tree tree_;
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_;
};

#endif // KINEMATICS_SOLVER_HPP_
