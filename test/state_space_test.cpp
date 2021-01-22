#include "planner/Planner.hpp"
#include <gtest/gtest.h>
#include "util/logger.hpp"
#include <ros/ros.h>

template<typename  T>
double getMaxExtent(int dimensions = -1, const Eigen::MatrixX2d *bounds_ptr = nullptr)
{
    std::size_t counter=0;
    const std::size_t max_iterations = 1e6;
    double max = std::numeric_limits<double>::min();
    while(++counter < max_iterations)
    {
        auto start_state = planner::randomState<T>(dimensions, bounds_ptr);
        auto goal_state = planner::randomState<T>(dimensions, bounds_ptr);
        double dist = planner::distance(start_state,goal_state);
        max = dist > max ? dist : max;
    }
    return max;
}

TEST(state_space_test,joint_space_test)
{
    double max_extent = getMaxExtent<state_space::JointSpace>();
    EXPECT_GT(max_extent,0);
    LOG(INFO)<<"max extent of Jointspace free"<< planner::distance(state_space::JointSpace(std::vector<double>{3.05,3.05,3.05,3.05,3.05,3.05}),
                                                                   state_space::JointSpace(std::vector<double>{-3.05,-3.05,-3.05,-3.05,-3.05,-3.05}))<<std::endl;
}

int main(int argc, char **argv)
{
    logger lg(argv[0]);
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "RRTTest");
    return RUN_ALL_TESTS();
}

