#include <gtest/gtest.h>
#include "util/logger.hpp"
#include <ros/ros.h>
#include "collision_detection/MoveItCollisionHelper.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "opencv2/opencv.hpp"
#include "planner/PlannerMethod.hpp"

bool isPathValid_2d(const state_space::Rn &from, const state_space::Rn &to)
{
    //only accessible for 2d rn space
    if (from.Dimensions() != 2 || from.Dimensions() != 2)
        return true;
    static cv::Mat img{cv::imread("/home/xcy/Cspace/SampleBasedPlanningMethods/newmap.png", 0)} ;

    // a simple collision check using cv methods
    double checked_length{.0};
    while (checked_length <= planner::distance(from, to)) {
        auto intermediate_state = planner::extend(from, to, checked_length);
        if (img.at<uchar>((int) intermediate_state.Vector()[1], (int) intermediate_state.Vector()[0]) == 0)
            return false;
        checked_length += 0.5;
    }
    return true;
}/*
template<typename SPCIFIC_STATE>
void test_no_collision_func(int dimensions = -1, const Eigen::MatrixX2d *bounds_ptr = nullptr)
{
    auto start_state = planner::randomState<SPCIFIC_STATE>(dimensions, bounds_ptr);
    auto goal_state = planner::randomState<SPCIFIC_STATE>(dimensions, bounds_ptr);

    planner::RRT<SPCIFIC_STATE> rrt_2d(planner::hash<SPCIFIC_STATE>, start_state.Dimensions());


    rrt_2d.constructPlan(planner::PLAN_REQUEST<SPCIFIC_STATE>(start_state, goal_state, 50,0.1,false, 0.1));
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
        time = (double) (clock() - start) / CLOCKS_PER_SEC;
        CHECK_LT(time, 5) << "Generate valid state failed out of 5 s";
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
    rrt_2d.constructPlan(planner::PLAN_REQUEST<SPCIFIC_STATE>(start_state, goal_state, 10));
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
    test_with_collision_func<state_space::JointSpace>(1, nullptr);
}

TEST(RRTTest, SE3withCollisionTest)
{
    test_with_collision_func<state_space::SE3>();
}*/

TEST(RRTTest, R2WithAnimationTest)
{
    /*
    cv::Mat img = cv::imread("/home/xcy/Cspace/SampleBasedPlanningMethods/newmap.png", cv::COLOR_BGR2GRAY);
    Eigen::MatrixX2d bounds;
    bounds.resize(2, 2);
    bounds << Eigen::Vector2d{img.cols, img.rows},
            Eigen::Vector2d::Zero();
    //auto start_state = planner::randomState<state_space::Rn>(2,&bounds);
    //auto goal_state = planner::randomState<state_space::Rn>(2,&bounds);
    state_space::Rn start_state{std::vector<double>{1,1}};
    state_space::Rn goal_state{std::vector<double>{1,1}};

    cv::circle(img, cv::Point(start_state.Vector()[0], start_state.Vector()[1]), 3, cv::Scalar{0, 0, 255}, 3);
    cv::circle(img, cv::Point(goal_state.Vector()[0], goal_state.Vector()[1]), 3, cv::Scalar{0, 0, 255}, 3);

    auto rrt_based_planner_ptr = planner::createPlanner<state_space::Rn>(planner::RRT_SIMPLE,start_state.Dimensions());

    rrt_based_planner_ptr->setStateValidator(std::function<bool(const state_space::Rn &, const state_space::Rn &)>(isPathValid_2d));

    rrt_based_planner_ptr->setSampleBounds(&bounds);

    rrt_based_planner_ptr->constructPlan(planner::PLAN_REQUEST<state_space::Rn>(start_state, goal_state, 50,30,false,30));
    std::vector<state_space::Rn> path;
    clock_t start(clock());
    if (rrt_based_planner_ptr->planning()) {
        clock_t end(clock());
        LOG(INFO) << "planning time consumption: " << static_cast<double>(end - start) / CLOCKS_PER_SEC;
        path = rrt_based_planner_ptr->GetPath();
        for (std::size_t i = 1; i < path.size(); ++i) {
            cv::circle(img, cv::Point(path[i].Vector()[0], path[i].Vector()[1]), 3, cv::Scalar{255, 0, 0}, 1);
            cv::line(img, cv::Point(path[i].Vector()[0], path[i].Vector()[1]),
                     cv::Point(path[i - 1].Vector()[0], path[i - 1].Vector()[1]), cv::Scalar{255, 0, 0}, 2);
        }
        cv::imshow("result", img);
        cv::waitKey(0);
    }
    EXPECT_TRUE(!path.empty()) << "No path found";*/
    state_space::JointSpace from{std::vector<double>{0,0}};
    state_space::JointSpace to{std::vector<double>{0,1}};

    auto a = planner::extend(from,to,0.05);
    std::cout<<a<<std::endl;
    Eigen::MatrixX2d bounds;
    bounds.resize(2, 2);
    bounds << Eigen::Vector2d().setConstant(-1),
            Eigen::Vector2d().setConstant(0);
    auto b= planner::randomState<state_space::JointSpace>(2, &bounds);
    std::cout<<b<<std::endl;
}

int main(int argc, char **argv)
{
    logger lg(argv[0]);
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "RRTTest");
    return RUN_ALL_TESTS();
}
