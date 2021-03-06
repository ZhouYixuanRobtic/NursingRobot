#include <gtest/gtest.h>
#include "util/logger.hpp"
#include <ros/ros.h>
#include "planner/PlanningInterface.h"

TEST(planning_interface_test,compute_joint_path_test)
{
    ros::AsyncSpinner spinner(2);
    spinner.start();
    planner::PlanningInterface planningInterface(planner::RRT_SIMPLE,
                               "manipulator_i5",
                               "/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                               my_kinematics::aubo_i5_analytical_IK);
    const std::size_t max_iterations = 1;
    std::size_t iter_index = 0;
    std::size_t time{};
    while(++iter_index<=max_iterations){
        //auto goal_valid = planningInterface.getRandomValidJointState();
        auto goal_valid = state_space::JointSpace(std::vector<double>{-1.438079781134842,-2.4659506418562516,-0.79238488741972768,2.8083971615457237,-0.20534463596476682,-0.22776415840239927});
        auto start_valid = state_space::JointSpace(std::vector<double>{0,0,0,0,0,0});
        std::cout<<"the distance between start and goal is "<<planner::distance(goal_valid,start_valid)<<std::endl;

        state_space::vector_JointSpace path;
        auto plan_request = planner::PLAN_REQUEST<state_space::JointSpace>(state_space::JointSpace(std::vector<double>{0,0,0,0,0,0}),
                                                                  goal_valid,
                                                                  300,
                                                                  1,false,0.5);
        clock_t start(clock());
        bool found_path = planningInterface.computeJointPath(path,plan_request);
        time += clock()-start;
        ASSERT_TRUE(found_path)<<"No path found at "<<iter_index<<"th try"<<goal_valid;
        if(found_path)
        {
            for(const auto & it : path)
            {
                std::cout<<it<<std::endl;
            }
        }
    }
    LOG(INFO)<<" average time consumption is "<< (double)time/max_iterations/CLOCKS_PER_SEC<<" s";
    spinner.stop();
}


int main(int argc, char**argv)
{
    logger lg(argv[0]);
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "carTest");
    return RUN_ALL_TESTS();
}