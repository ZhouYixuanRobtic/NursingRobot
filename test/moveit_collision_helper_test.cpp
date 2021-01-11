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


int main(int argc, char **argv)
{
    logger lg(argv[0], "./log");
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "cTest");
    return RUN_ALL_TESTS();
}

