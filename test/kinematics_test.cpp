#define EIGEN_MKL_USE_ALL

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "util/logger.hpp"
#include "Kinematics/custom_kinematics.hpp"
#include "Kinematics/Kinematics.h"
#include "planner/Planner.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

TEST(KinematicsTest, AuboAnalyticalTest)
{
    my_kinematics::Kinematics aubo_i5_kinematics("/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                 my_kinematics::aubo_i5_analytical_IK);

    Eigen::MatrixX2d bounds_for_aubo(6, 2);
    bounds_for_aubo << Eigen::Matrix<double, 6, 1>().setConstant(3.05), Eigen::Matrix<double, 6, 1>().setConstant(
            -3.05);

    size_t ik_analytical_only_counter{}, ik_analytical_with_right_reference{}, ik_analytical_exactly_the_same_counter{};
    const size_t max_iterations = 1e6;
    const double zero_thresh = 1e-3;
    double all_time = 0;

    for (size_t index = 0; index < max_iterations; ++index) {

        auto test_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);
        auto desired_pose = aubo_i5_kinematics.fk(test_joint);
        //check analytical ik without right reference (make \theta6 as 0)
        Eigen::MatrixXd joint_solutions;
        clock_t start = clock();
        auto ret_code = aubo_i5_kinematics.analyticalIkSolutions(joint_solutions, desired_pose.SE3Matrix(),
                                                                 nullptr);
        clock_t end = clock();
        if (joint_solutions.cols() != 0 &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(state_space::JointSpace(joint_solutions.col(0))))
            < zero_thresh)
            ik_analytical_only_counter++;
        //check analytical ik with right reference
        joint_solutions.resize(0, 0);
        auto refer_joint = test_joint;
        ret_code = aubo_i5_kinematics.analyticalIkSolutions(joint_solutions, desired_pose.SE3Matrix(),
                                                            &refer_joint);
        if (joint_solutions.cols() != 0 &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(state_space::JointSpace(joint_solutions.col(0))))
            < zero_thresh) {
            ik_analytical_with_right_reference++;
            for (size_t i = 0; i < joint_solutions.cols(); ++i) {
                if (planner::distance(test_joint, state_space::JointSpace(joint_solutions.col(i))) < zero_thresh) {
                    ik_analytical_exactly_the_same_counter++;
                    break;
                }
            }
        }

        all_time += double(end - start) / CLOCKS_PER_SEC;
    }
    EXPECT_GE(ik_analytical_with_right_reference, 0.9999973 * max_iterations)
                        << "Coverage rate of analytical IK solution (equals to fk) doesn't reach 5 sigma level, but"
                        << ik_analytical_with_right_reference << "/" << max_iterations;

    EXPECT_GE(ik_analytical_only_counter, 0.9999973 * max_iterations)
                        << "Coverage rate of analytical IK solution with zero reference doesn't reach 5 sigma level, but"
                        << ik_analytical_only_counter << "/" << max_iterations;

    EXPECT_GE(ik_analytical_exactly_the_same_counter, 0.9973 * max_iterations)
                        << "Coverage rate of exactly the same analytical IK solution with zero reference doesn't reach 3 sigma level, but"
                        << ik_analytical_exactly_the_same_counter << "/" << max_iterations;

    LOG(INFO) << "the average time consumption of analytical ik method: " << 1e6 * all_time / max_iterations
              << "us\n";
}

TEST(KinematicsTest, NumericalTest)
{
    my_kinematics::Kinematics aubo_i5_kinematics("/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml");

    Eigen::MatrixX2d bounds_for_aubo(6, 2);
    bounds_for_aubo << Eigen::Matrix<double, 6, 1>().setConstant(3.05), Eigen::Matrix<double, 6, 1>().setConstant(
            -3.05);

    size_t ik_numerical_only_counter{}, ik_numerical_with_right_reference{}, ik_numerical_exactly_the_same_counter{};
    const size_t max_iterations = 1e3;
    const double zero_thresh = 1e-3;
    double all_time = 0;

    for (size_t index = 0; index < max_iterations; ++index) {

        auto test_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);

        auto desired_pose = aubo_i5_kinematics.fk(test_joint);

        //check analytical ik without right reference (make \theta6 as 0)
        state_space::JointSpace raw_solution;
        clock_t start = clock();
        bool found_ik = aubo_i5_kinematics.nIk(desired_pose, raw_solution);
        clock_t end = clock();

        if (found_ik &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(raw_solution))
            < zero_thresh)
            ik_numerical_only_counter++;

        //check analytical ik with right reference
        auto direction_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);
        raw_solution = planner::extend(test_joint,direction_joint,0.01);
        start = clock();
        found_ik = aubo_i5_kinematics.nIk(desired_pose, raw_solution);
        end = clock();
        if (found_ik &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(raw_solution))
            < zero_thresh) {
            ik_numerical_with_right_reference++;
            if (planner::distance(test_joint, raw_solution) < zero_thresh)
                ik_numerical_exactly_the_same_counter++;
        }
        all_time += double(end - start) / CLOCKS_PER_SEC;
    }
    EXPECT_EQ(max_iterations, ik_numerical_with_right_reference)
                        << "Coverage rate of numerical IK solution (equals to fk) with right reference is not 100%, but"
                        << ik_numerical_with_right_reference << "/" << max_iterations;

    //EXPECT_GE(ik_numerical_only_counter, 0.9973*max_iterations)<<"Coverage rate of numerical IK solution with zero reference doesn't reach 3 sigma level, but"
    // << ik_numerical_only_counter <<"/"<<max_iterations;

    EXPECT_GE(ik_numerical_exactly_the_same_counter, 0.99 * max_iterations)
                        << "Coverage rate of numerical IK solution with zero reference doesn't reach 99 level, but"
                        << ik_numerical_exactly_the_same_counter << "/" << max_iterations;

    LOG(INFO) << "the average time consumption of numerical ik method: " << 1e6 * all_time / max_iterations
              << "us\n";
}


TEST(KinematicsTest, CombinedIKTest)
{
    my_kinematics::Kinematics aubo_i5_kinematics("/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                 my_kinematics::aubo_i5_analytical_IK);

    Eigen::MatrixX2d bounds_for_aubo(6, 2);
    bounds_for_aubo << Eigen::Matrix<double, 6, 1>().setConstant(3.05), Eigen::Matrix<double, 6, 1>().setConstant(
            -3.05);

    size_t ik_combined_only_counter{},ik_combined_with_right_reference{}, ik_combined_exactly_the_same_counter{};
    const size_t max_iterations = 1e6;
    const double zero_thresh = 1e-3;
    double all_time = 0;

    for (size_t index = 0; index < max_iterations; ++index) {

        auto test_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);

        auto desired_pose = aubo_i5_kinematics.fk(test_joint);

        //check analytical ik without right reference (make \theta6 as 0)
        Eigen::MatrixXd joint_solutions;
        clock_t start = clock();
        bool found_ik =  aubo_i5_kinematics.allValidIkSolutions(joint_solutions, desired_pose.SE3Matrix(), nullptr);
        clock_t end = clock();

        if (found_ik &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(state_space::JointSpace(joint_solutions.col(0))))
            < zero_thresh) {
            ik_combined_only_counter++;
        }
        //check analytical ik with right reference
        auto refer_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);
        refer_joint = planner::extend(test_joint,refer_joint,0.01);

        joint_solutions.resize(0,0);
        start = clock();
        found_ik =  aubo_i5_kinematics.allValidIkSolutions(joint_solutions, desired_pose.SE3Matrix(), &refer_joint);
        end = clock();

        if (found_ik &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(state_space::JointSpace(joint_solutions.col(0))))
            < zero_thresh) {
            ik_combined_with_right_reference++;
            for (size_t i = 0; i < joint_solutions.cols(); ++i) {
                if (planner::distance(test_joint, state_space::JointSpace(joint_solutions.col(i))) < zero_thresh) {
                    ik_combined_exactly_the_same_counter++;
                    break;
                }
            }
        }

        all_time += double(end - start) / CLOCKS_PER_SEC;
    }
    EXPECT_GE(ik_combined_only_counter,0.9999943*max_iterations)<< "Coverage rate of combined IK solution with zero reference doesn't reach 5 sigma level, but"
            << ik_combined_only_counter << "/" << max_iterations;;

    EXPECT_EQ(max_iterations, ik_combined_with_right_reference)
                        << "Coverage rate of combined IK solution (equals to fk) with right reference is not 100%, but"
                        << ik_combined_with_right_reference << "/" << max_iterations;


    EXPECT_GE(ik_combined_exactly_the_same_counter, 0.9973 * max_iterations)
                        << "Coverage rate of exactly the same combined IK solution with right reference doesn't reach 3 sigma level, but"
                        << ik_combined_exactly_the_same_counter << "/" << max_iterations;

    LOG(INFO) << "the average time consumption of numerical ik method: " << 1e6 * all_time / max_iterations
              << "us\n";
}

TEST(KinematicsTest, NearestIKTest)
{
    my_kinematics::Kinematics aubo_i5_kinematics("/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                 my_kinematics::aubo_i5_analytical_IK);

    Eigen::MatrixX2d bounds_for_aubo(6, 2);
    bounds_for_aubo << Eigen::Matrix<double, 6, 1>().setConstant(3.05), Eigen::Matrix<double, 6, 1>().setConstant(
            -3.05);

    size_t ik_nearest_only_counter{}, ik_nearest_with_right_reference{}, ik_nearest_exactly_the_same_counter{};
    const size_t max_iterations = 1e6;
    const double zero_thresh = 1e-3;
    double all_time = 0;

    for (size_t index = 0; index < max_iterations; ++index) {

        auto test_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);

        auto desired_pose = aubo_i5_kinematics.fk(test_joint);

        auto refer_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);
        refer_joint = planner::extend(test_joint,refer_joint,0.01);

        //check analytical ik without right reference (make \theta6 as 0)
        state_space::JointSpace raw_solution;
        clock_t start = clock();
        bool found_ik = aubo_i5_kinematics.nearestIkSolution(raw_solution, desired_pose.SE3Matrix(), refer_joint,
                                                             false);
        clock_t end = clock();

        if (found_ik &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(raw_solution))
            < zero_thresh)
            ik_nearest_only_counter++;

        //check analytical ik with right reference
        found_ik = aubo_i5_kinematics.nearestIkSolution(raw_solution, desired_pose.SE3Matrix(), refer_joint, true);
        if (found_ik &&
            planner::distance(desired_pose, aubo_i5_kinematics.fk(raw_solution))
            < zero_thresh) {
            ik_nearest_with_right_reference++;
            if (planner::distance(test_joint, raw_solution) < zero_thresh) {
                ik_nearest_exactly_the_same_counter++;
            }
        }

        all_time += double(end - start) / CLOCKS_PER_SEC;
    }
    EXPECT_EQ(max_iterations, ik_nearest_with_right_reference)
                        << "Coverage rate of nearest IK solution (equals to fk) with right reference is not 100%, but"
                        << ik_nearest_with_right_reference << "/" << max_iterations;

    EXPECT_GE(ik_nearest_only_counter, 0.9999973 * max_iterations)
                        << "Coverage rate of nearest IK solution with zero reference doesn't reach 5 sigma level, but"
                        << ik_nearest_only_counter << "/" << max_iterations;

    EXPECT_GE(ik_nearest_exactly_the_same_counter, 0.9973 * max_iterations)
                        << "Coverage rate of exactly the same nearest IK solution with right reference doesn't reach 3 sigma level, but"
                        << ik_nearest_exactly_the_same_counter << "/" << ik_nearest_with_right_reference;

    LOG(INFO) << "the average time consumption of numerical ik method: " << 1e6 * all_time / max_iterations
              << "us\n";
}
int main(int argc, char **argv)
{
    logger lg(argv[0],"/home/xcy/log");
    logger::setFlagLogToStd(true);
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc,argv,"kTest");
    return RUN_ALL_TESTS();
}



