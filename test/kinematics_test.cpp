#define EIGEN_MKL_USE_ALL
#include <gtest/gtest.h>
#include "util/logger.hpp"
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <collision_detection/MoveItCollisionHelper.h>

TEST(KinematicsTest,AuboIKWithCollisionTest)
{
    const std::string PLANNING_GROUP = "manipulator_i5";
    ros::AsyncSpinner spinner(2);
    spinner.start();

    my_collision_detection::MoveItCollisionHelper moveItCollisionHelper(
            PLANNING_GROUP,
            "/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
            my_kinematics::aubo_i5_analytical_IK);

    Eigen::MatrixX2d bounds_for_aubo(6, 2);
    bounds_for_aubo << Eigen::Matrix<double, 6, 1>().setConstant(3.05), Eigen::Matrix<double, 6, 1>().setConstant(
            -3.05);

    const auto& aubo_i5_kinematics = moveItCollisionHelper.getKinematicsPtr();
    size_t ik_counter{0};
    const size_t max_iterations = 1;
    double nearest_time = 0, all_time = 0;
    for (size_t index = 0; index < max_iterations; ++index) {
        //auto test_joint = planner::randomState<state_space::JointSpace>( 6,&bounds_for_aubo);
        state_space::JointSpace test_joint(std::vector<double>{-1.8253,-1.78352, -2.65591 ,1.90189 ,0.373454 -0.792052});
        auto desired_pose = aubo_i5_kinematics->fk(test_joint);
        auto refer_joint = test_joint;
        refer_joint[4] -= 0.01;
        clock_t start(clock());
        auto nresult = aubo_i5_kinematics->nIk(desired_pose,refer_joint);
        auto result = moveItCollisionHelper.allValidSolutions(desired_pose, &refer_joint, true);
        clock_t end(clock());
        all_time += double(end - start) / CLOCKS_PER_SEC;
        if(!result.empty())
            ik_counter++;
        else {
            LOG(INFO) << test_joint << std::endl;
            LOG(INFO)<< desired_pose.SE3Matrix()<<std::endl;
        }
    }

    EXPECT_DOUBLE_EQ(1.0,(double) (ik_counter) / (double) max_iterations);
    LOG_IF(INFO,1 != (double) (ik_counter) / (double) max_iterations)<<"IK solution (equals to fk) coverage rate is not 100, but"
                                                                     << ik_counter<<"/"<<max_iterations;

    LOG(INFO) << "the average time consumption of valid ik method: " << 1e6 * all_time / max_iterations
              << "us\n";
    spinner.stop();
}
int main(int argc, char** argv)
{
    logger lg(argv[0],"./log");
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "cTest");
    return RUN_ALL_TESTS();
}



