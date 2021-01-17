#include <gtest/gtest.h>
#include "util/logger.hpp"
#include <ros/ros.h>
#include "collision_detection/MoveItCollisionHelper.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_eigen/tf2_eigen.h>

TEST(collision_test, get_current_state_test)
{
    const std::string PLANNING_GROUP = "manipulator_i5";
    ros::AsyncSpinner spinner(3);
    spinner.start();

    my_collision_detection::MoveItCollisionHelper moveItCollisionHelper(
            PLANNING_GROUP,
            "/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
            my_kinematics::aubo_i5_analytical_IK);


    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    auto namedTargets = move_group.getNamedTargets();

    for (const auto &namedTarget : namedTargets) {
        move_group.setNamedTarget(namedTarget);
        move_group.move();
        auto current_joint = state_space::JointSpace(move_group.getCurrentJointValues());
        Eigen::Affine3d current_affine;
        tf2::fromMsg(move_group.getCurrentPose(move_group.getEndEffectorLink()).pose, current_affine);
        auto current_pose = state_space::SE3(current_affine.matrix());
        auto my_current_joint = moveItCollisionHelper.getCurrentJointAngles();
        auto my_current_pose = moveItCollisionHelper.getEndEffectorPose();
        EXPECT_NEAR(0, planner::distance(current_pose, my_current_pose), 1e-3)
                            << "my current pose failed in " << namedTarget << " case";
        EXPECT_NEAR(0, planner::distance(current_joint, my_current_joint), 1e-3)
                            << "my current joints failed in " << namedTarget << " case";;
    }
    spinner.stop();
}

TEST(collision_test, ik_with_collision_check_test)
{
    const std::string PLANNING_GROUP = "manipulator_i5";
    ros::AsyncSpinner spinner(3);
    spinner.start();

    my_collision_detection::MoveItCollisionHelper moveItCollisionHelper(
            PLANNING_GROUP,
            "/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
            my_kinematics::aubo_i5_analytical_IK);

    const auto &aubo_i5_kinematics = moveItCollisionHelper.getKinematicsPtr();
    Eigen::MatrixX2d bounds_for_aubo(6, 2);
    bounds_for_aubo << Eigen::Matrix<double, 6, 1>().setConstant(3.05), Eigen::Matrix<double, 6, 1>().setConstant(
            -3.05);

    size_t ik_with_one_collision_counter{}, ik_all_with_collision_counter{};
    const size_t max_iterations = 1e6;
    double all_time = 0, one_time = 0;

    for (size_t index = 0; index < max_iterations; ++index) {

        //generate a collision-free state
        auto test_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);
        bool state_valid = moveItCollisionHelper.isStateValid(test_joint);
        while(!state_valid)
        {
            test_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);
            state_valid = moveItCollisionHelper.isStateValid(test_joint);
        }

        auto desired_pose = aubo_i5_kinematics->fk(test_joint);
        //check analytical ik without right reference (make \theta6 as 0)
        Eigen::MatrixXd joint_solutions;
        auto direction_joint = planner::randomState<state_space::JointSpace>(6, &bounds_for_aubo);
        auto raw_solution = planner::extend(test_joint,direction_joint,0.5);
        clock_t start = clock();
        bool found_ik = moveItCollisionHelper.nearestSolution(raw_solution, desired_pose,
                                                              raw_solution,
                                                              true);
        clock_t end = clock();
        all_time += double(end - start) / CLOCKS_PER_SEC;
        if (found_ik)
            ik_with_one_collision_counter++;
        //check analytical ik with right reference
        joint_solutions.resize(0, 0);
        auto refer_joint = planner::extend(test_joint,direction_joint,0.5);
        state_space::vector_JointSpace raw_solutions;
        start = clock();
        found_ik = moveItCollisionHelper.allValidSolutions(raw_solutions, desired_pose, &refer_joint);
        end = clock();
        if (found_ik) {
            ik_all_with_collision_counter++;
        }
        one_time += double(end - start) / CLOCKS_PER_SEC;
    }
    EXPECT_EQ(max_iterations,ik_with_one_collision_counter)<<"the state valid coverage rate of neaerest is not 100%, but"
                                                            <<ik_with_one_collision_counter<<" / "<<max_iterations;

    EXPECT_EQ(max_iterations,ik_all_with_collision_counter)<<"the state valid coverage rate of neaerest is not 100%, but"
                                                           <<ik_all_with_collision_counter<<" / "<<max_iterations;
    LOG(INFO)<<"time consumption of all valid solution "<<1e6 * all_time/max_iterations<<"us\n";
    LOG(INFO)<<"time consumption of nearest valid solution"<<1e6 * one_time/max_iterations<<"us\n";
    spinner.stop();
}

int main(int argc, char **argv)
{
    logger lg(argv[0]);
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "cTest");

    return RUN_ALL_TESTS();
}

