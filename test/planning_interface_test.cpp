#include <gtest/gtest.h>
#include "util/logger.hpp"
#include <ros/ros.h>
#include "planner/PlanningInterface.h"

TEST(planning_interface_test,compute_joint_path_test)
{
    ros::AsyncSpinner spinner(2);
    spinner.start();
    planner::PlanningInterface planningInterface(planner::RRT_CONNECT,
                               "manipulator_i5",
                               "/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                               my_kinematics::aubo_i5_analytical_IK);
    const std::size_t max_iterations = 1E3;
    std::size_t iter_index = 0;
    std::size_t time{};
    while(++iter_index<=max_iterations){
        auto goal_valid = planningInterface.getRandomValidJointState();
       // auto start_valid = planningInterface.getRandomValidJointState();
        //auto goal_valid = state_space::JointSpace(std::vector<double>{-1.080161872088678,-0.39970868869990683,-1.5038783055001514,-3.0265962510776103,-0.014835318820423549,-2.3235883017994894});
        auto start_valid = state_space::JointSpace(std::vector<double>{0,0,0,0,0,0});

        state_space::vector_JointSpace path;
        auto plan_request = planner::PLAN_REQUEST<state_space::JointSpace>(start_valid,
                                                                  goal_valid,
                                                                  5,
                                                                  0.1,false,0.1);
        clock_t start(clock());
        bool found_path = planningInterface.computeJointPath(path,plan_request);
        time += clock()-start;
        ASSERT_TRUE(found_path)<<"No path found at "<<iter_index<<"th try"<<goal_valid<<std::endl<<start_valid;
        /*if(found_path)
        {
            for(const auto & it : path)
            {
                std::cout<<it<<std::endl;
            }
        }*/
    }
    LOG(INFO)<<" average time consumption is "<< (double)time/max_iterations/CLOCKS_PER_SEC<<" s";
    spinner.stop();
}

int main(int argc, char**argv)
{
    state_space::SE3 a(std::vector<double>{-0.000, 0.2155, 0.8865,0.000, std::sqrt(0.5), std::sqrt(0.5), 0.000});
    state_space::SE3 b(std::vector<double>{0.000, 0.000, 0.1215,0,0,1,0});
    state_space::SE3 c(std::vector<double>{-0.000, 0.2155, 0.8865,0.000, std::sqrt(0.5), std::sqrt(0.5), 0.000});
    std::cout<<(b+a).SE3Matrix()<<std::endl;
    std::cout<<(b+a).Vector().transpose()<<std::endl;
    std::cout<<c.Vector().transpose()<<std::endl;
    std::cout<<std::sqrt(0.5)<<std::endl;
    logger lg(argv[0]);
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "carTest");
    return RUN_ALL_TESTS();
}