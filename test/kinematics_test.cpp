#define EIGEN_MKL_USE_ALL

#include "StateSpace/JointSpace.hpp"
#include "StateSpace/rn.hpp"
#include "StateSpace/SE3.hpp"
#include "StateSpace/SO3.hpp"
#include "Kinematics/Kinematics.h"
#include "Kinematics/custom_kinematics.hpp"
#include "planner/Planner.hpp"

int main() {
    state_space::R6 temp_twist = planner::randomState<state_space::Rn>(6, nullptr).Vector();
    state_space::SE3 A(temp_twist);
    kinematics::Kinematics aubo_i5_kinematics("../config/aubo_i5.yaml");

    aubo_i5_kinematics.setAnalyticalIK(kinematics::aubo_i5_analytical_IK);


    Eigen::MatrixX2d bounds_for_aubo(6, 2);
    bounds_for_aubo << Eigen::Matrix<double, 6, 1>().setConstant(3.05), Eigen::Matrix<double, 6, 1>().setConstant(
            -3.05);

    size_t nearest_counter{0}, all_valid_counter{0}, ik_counter{0};
    const size_t max_iterations = 1e6;
    const double zero_thresh = 1e-3;
    double nearest_time = 0, all_time = 0;
    for (size_t index = 0; index < max_iterations; ++index) {
        auto test_joint = planner::randomState<state_space::JointSpace>( 6,&bounds_for_aubo);

        auto desired_pose = aubo_i5_kinematics.fk(test_joint);
        auto refer_joint = test_joint;
        refer_joint[4] -= 0.1;
        clock_t start(clock());
        auto result = aubo_i5_kinematics.nearestIkSolution(desired_pose, refer_joint, true);
        clock_t end(clock());
        nearest_time += double(end - start) / CLOCKS_PER_SEC;
        if (planner::distance(result, test_joint) < zero_thresh)
            nearest_counter++;


        if (planner::distance(aubo_i5_kinematics.fk(test_joint),
                              aubo_i5_kinematics.fk(result))
            < zero_thresh)
            ik_counter++;


        Eigen::MatrixXd joint_solutions;
        start = clock();
        bool all_valid_bool = aubo_i5_kinematics.allValidIkSolutions(joint_solutions, desired_pose.SE3Matrix(),
                                                                     &refer_joint);
        end = clock();
        all_time += double(end - start) / CLOCKS_PER_SEC;
        if (all_valid_bool) {
            for (int i = 0; i < joint_solutions.cols(); ++i) {
                if ((joint_solutions.col(i) - test_joint.Vector()).norm() < zero_thresh)
                    all_valid_counter++;
            }
        }
    }
    std::cout << "exactly the same IK solution coverage rate: " << (double) (nearest_counter) / (double) max_iterations
              << std::endl;
    std::cout << "exactly the same IK solution using all valid method coverage rate: "
              << (double) (all_valid_counter) / (double) max_iterations << std::endl;
    std::cout << "IK solution (equals to fk) coverage rate: " << (double) (ik_counter) / (double) max_iterations
              << std::endl;
    std::cout << "the average nearest_time consumption of nearest ik method: " << 1e6 * nearest_time / max_iterations
              << "us\n";
    std::cout << "the average nearest_time consumption of all valid ik method: " << 1e6 * all_time / max_iterations
              << "us\n";
}


