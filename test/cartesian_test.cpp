#include <gtest/gtest.h>
#include "util/logger.hpp"
#include <ros/ros.h>
#include "planner/CartesianPlanner.h"

TEST(cartesian_test,linearTest)
{
    ros::AsyncSpinner spinner(2);
    spinner.start();
    my_collision_detection::MoveItCollisionHelperPtr moveItCollisionHelper(new my_collision_detection::MoveItCollisionHelper("manipulator_i5",
                                                                                                                             "/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                                                                                             my_kinematics::aubo_i5_analytical_IK));

    const auto & kinematic_ptr = moveItCollisionHelper->getKinematicsPtr();
    auto current_pose = kinematic_ptr->fk(state_space::JointSpace::Zero());
    state_space::vector_SE3 cartesian_path;
    planner::CartesianPlanner::getCartesianLine(cartesian_path,current_pose,Eigen::Vector3d{-0.3,0.0,0});
    state_space::vector_JointSpace trajectory;
    clock_t start(clock());
    double percentage = planner::CartesianPlanner::computeCartesianPath(state_space::JointSpace::Zero(),
                                                                        cartesian_path,
                                                                        trajectory,
                                                                        planner::MaxEEFStep(0.01,0.0),
                                                                        planner::JumpThreshold::MIN(),
                                                                        kinematic_ptr->getBaseName(),
                                                                        moveItCollisionHelper);
    clock_t end(clock());
    EXPECT_FLOAT_EQ(percentage,1.0)<<"cartesian planner failed near zero";
    LOG(INFO)<<"time consumption of "<<trajectory.size()<<" points linear path is "<<(double)(end-start)/CLOCKS_PER_SEC<<" s";
    spinner.stop();
}
TEST(cartesian_test, circleTest)
{
    ros::AsyncSpinner spinner(2);
    spinner.start();
    my_collision_detection::MoveItCollisionHelperPtr moveItCollisionHelper(new my_collision_detection::MoveItCollisionHelper("manipulator_i5",
                                                                        "/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                                        my_kinematics::aubo_i5_analytical_IK));

    const auto & kinematic_ptr = moveItCollisionHelper->getKinematicsPtr();
    auto current_pose = kinematic_ptr->fk(state_space::JointSpace::Zero());
    state_space::vector_SE3 cartesian_path;
    planner::CartesianPlanner::getCartesianCircle(cartesian_path,state_space::SE3(state_space::SO3::Zero(),Eigen::Vector3d{-0.03,0,0}),
                                                  state_space::SE3(state_space::SO3::Zero(),Eigen::Vector3d{-0.23,0,0}),
                                                  -2*M_PI);
    state_space::vector_JointSpace trajectory;
    clock_t start(clock());
    double percentage = planner::CartesianPlanner::computeCartesianPath(state_space::JointSpace::Zero(),
                                                                        cartesian_path,
                                                                        trajectory,
                                                                        planner::MaxEEFStep(0.0,0.01),
                                                                        planner::JumpThreshold::MIN(),
                                                                        kinematic_ptr->getEndEffectorName(),
                                                                        moveItCollisionHelper);
    clock_t end(clock());
    EXPECT_FLOAT_EQ(percentage,1.0)<<"cartesian planner failed near zero";
    LOG(INFO)<<"time consumption of "<<trajectory.size()<<" points circle path is "<<(double)(end-start)/CLOCKS_PER_SEC <<" s";
    spinner.stop();
}

int main(int argc, char**argv)
{
    logger lg(argv[0]);
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "carTest");
    return RUN_ALL_TESTS();
}