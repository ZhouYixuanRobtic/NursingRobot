#include "planner/RRT.hpp"
#include <gtest/gtest.h>
#include "util/logger.hpp"
#include <ros/ros.h>
#include "collision_detection/MoveItCollisionHelper.h"
#include <moveit_visual_tools/moveit_visual_tools.h>

template<typename SPCIFIC_STATE>
void test_no_collision_func(int dimensions = -1, const Eigen::MatrixX2d *bounds_ptr = nullptr)
{
    auto start_state = planner::randomState<SPCIFIC_STATE>(dimensions, bounds_ptr);
    auto goal_state = planner::randomState<SPCIFIC_STATE>(dimensions, bounds_ptr);

    planner::RRT<SPCIFIC_STATE> rrt_2d(planner::hash<SPCIFIC_STATE>, start_state.Dimensions());


    rrt_2d.setStepLen(0.05);
    rrt_2d.setGoalMaxDist(0.05);
    rrt_2d.constructPlan(planner::PLAN_REQUEST<SPCIFIC_STATE>(start_state, goal_state, 50));
    std::vector<SPCIFIC_STATE> path;
    clock_t start(clock());
    if (rrt_2d.planning()) {
        clock_t end(clock());
        LOG(INFO) << "planning time consumption: " << static_cast<double>(end - start) / CLOCKS_PER_SEC;
        path = rrt_2d.GetPath();
        for (const auto &it: path) {
            LOG(INFO) << it;
        }
    }
    EXPECT_TRUE(!path.empty()) << "No path found";
}

template<typename SPCIFIC_STATE>
void test_with_collision_func(int dimensions = -1, const Eigen::MatrixX2d *bounds_ptr = nullptr)
{
    ros::AsyncSpinner spinner(2);
    spinner.start();
    my_collision_detection::MoveItCollisionHelper moveItCollisionHelper("manipulator_i5",
                                                                        "/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                                        my_kinematics::aubo_i5_analytical_IK);


    auto start_state = planner::randomState<SPCIFIC_STATE>(dimensions, bounds_ptr);
    auto goal_state = planner::randomState<SPCIFIC_STATE>(dimensions, bounds_ptr);
    clock_t start(clock());
    double time{};
    while (true) {
        time += (double) (clock() - start) / CLOCKS_PER_SEC;
        CHECK_LT(time, 60) << "Generate valid state failed out of 1 mins";
        start_state = planner::randomState<SPCIFIC_STATE>(dimensions, bounds_ptr);
        goal_state = planner::randomState<SPCIFIC_STATE>(dimensions, bounds_ptr);
        if (moveItCollisionHelper.isStateValid(start_state) && moveItCollisionHelper.isStateValid(goal_state))
            break;
    }

    planner::RRT<SPCIFIC_STATE> rrt_2d(planner::hash<SPCIFIC_STATE>, start_state.Dimensions());


    rrt_2d.setStateValidator(std::function<bool(const SPCIFIC_STATE &, const SPCIFIC_STATE &)>(
            std::bind(&my_collision_detection::MoveItCollisionHelper::isPathValid<SPCIFIC_STATE>,
                      &moveItCollisionHelper, std::placeholders::_1, std::placeholders::_2)
    ));


    rrt_2d.setStepLen(0.05);
    rrt_2d.setGoalMaxDist(0.05);
    rrt_2d.setMaxIterations(1e7);
    rrt_2d.constructPlan(planner::PLAN_REQUEST<SPCIFIC_STATE>(start_state, goal_state, 1e7));
    std::vector<SPCIFIC_STATE> path;
    if (rrt_2d.planning()) {
        clock_t end(clock());
        LOG(INFO) << "planning time consumption: " << static_cast<double>(end - start) / CLOCKS_PER_SEC;
        path = rrt_2d.GetPath();
        for (const auto &it: path) {
            LOG(INFO) << it;
        }
    }
    EXPECT_TRUE(!path.empty()) << "No path found";
    spinner.stop();
}


TEST(RRTTest, J6WithoutCollisionTest)
{
    test_no_collision_func<state_space::JointSpace>(6);
}

TEST(RRTTest, R2WithoutCollisionTest)
{
    test_no_collision_func<state_space::Rn>(2);
}

TEST(RRTTest, R3WithoutCollisionTest)
{
    test_no_collision_func<state_space::Rn>(3);
}

TEST(RRTTest, SE3WithoutCollisionTest)
{
    test_no_collision_func<state_space::SE3>();
}

TEST(RRTTest, SO3WithoutCollisionTest)
{
    test_no_collision_func<state_space::SO3>();
}

TEST(RRTTest, J6withCollisionTest)
{
    Eigen::MatrixX2d bounds;
    bounds.resize(6, 2);
    bounds << state_space::R6().setConstant(3.05),
            state_space::R6().setConstant(-3.05);
    test_with_collision_func<state_space::JointSpace>(6, &bounds);
}

TEST(RRTTest, SE3withCollisionTest)
{
    test_with_collision_func<state_space::SE3>();
}

int main(int argc, char **argv)
{
    logger lg(argv[0], "./log");
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "RRTTest");
    return RUN_ALL_TESTS();
}
