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
    LOG(INFO)<<"max extent of Jointspace free is "<< planner::distance(state_space::JointSpace(std::vector<double>{3.05,3.05,3.05,3.05,3.05,3.05}),
                                                                   state_space::JointSpace(std::vector<double>{-3.05,-3.05,-3.05,-3.05,-3.05,-3.05}))<<std::endl;
    //Joint Space operator += test
    state_space::JointSpace a(std::vector<double>{-1});
    state_space::JointSpace b(std::vector<double>{-2});

    LOG(INFO)<<"distance of "<<a<<" to "<< b<<"is"<<planner::distance(a,b);

    LOG(INFO)<<"interpolate from "<<a<<" to "<< b<<" by 0.2 is "<<planner::interpolate(a,b,0.2);
    LOG(INFO)<<"extend from "<<a<<" to "<< b<<" by 1 is "<<planner::extend(a,b,-1);
}


int main(int argc, char **argv)
{
    logger lg(argv[0]);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

