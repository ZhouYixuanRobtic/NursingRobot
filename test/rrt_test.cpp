#include <gtest/gtest.h>
#include "util/logger.hpp"
#include <ros/ros.h>
#include "collision_detection/MoveItCollisionHelper.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "opencv2/opencv.hpp"
#include "planner/PlannerMethod.hpp"
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
inline bool collision_check(const state_space::Rn& test_state){
    static cv::Mat environment{cv::imread("/home/xcy/Cspace/SampleBasedPlanningMethods/3.png", 0)};
    Eigen::Vector2d bounds{environment.cols,environment.rows};
    return test_state.Vector().minCoeff()>=0&&
           (test_state.Vector()-bounds).maxCoeff()<=0
           &&environment.at<uchar>((int) test_state.Vector()[1], (int) test_state.Vector()[0]) == 255;
}
double vdc(int n,unsigned int bits) {
    int reverse = 0;
    while (n){
        int pos = log2(n & -n) + 1;
        reverse = reverse | (1 << (bits - pos));
        n = n & (n - 1);
    }
    return reverse;
}
std::size_t check_times;
bool isPathValid_vdc(const state_space::Rn& from, const state_space::Rn& to){
    const double min_distance=0.5;
    check_times++;
    if(!collision_check(to))
        return false;
    unsigned int K = ceil(log2(ceil(planner::distance(from,to)/min_distance)));
    unsigned int bits = K;
    K = 1<<K;
    for(int k=0; k<K;++k){
        check_times++;
        auto intermediate_state = planner::interpolate(from, to, vdc(k,bits)/K);
        if (!collision_check(intermediate_state))
            return false;
    }
    return true;
}
bool isPathValid_incremental(const state_space::Rn& from, const state_space::Rn& to){
    const double min_distance=0.5;
    unsigned int K= ceil(planner::distance(from,to)/min_distance);
    K = K? K:1;
    for(int k=0;k<=K;++k) {
        check_times++;
        auto intermediate_state = planner::interpolate(from, to, (double)k/K);
        if(!collision_check(intermediate_state))
            return false;
    }
    return true;
}
template<typename T>
void _animate(cv::Mat& draw, planner::Vertex<T> * vertex)
{
    cv::circle(draw, cv::Point((int)vertex->state().Vector()[0], (int)vertex->state().Vector()[1]), 3, cv::Scalar{255, 0, 0}, 1);
    for(auto & it : vertex->children())
    {
        cv::line(draw, cv::Point((int)vertex->state().Vector()[0], (int)vertex->state().Vector()[1]),
                 cv::Point((int)it->state().Vector()[0], (int)it->state().Vector()[1]), cv::Scalar{255, 0, 0}, 2);
        _animate(draw,it);
    }
}
template<typename T>
void animate(cv::Mat &draw, const std::vector<planner::Vertex<T>*>& roots)
{
    boost::this_thread::interruption_enabled();
    while(true){
        boost::this_thread::interruption_point();
        for(const auto& item: roots){
            _animate(draw,item);
        }
        cv::Mat canvas;
        cv::resize(draw,canvas,cv::Size{800,800});
        cv::imshow("check: ",canvas);
        cv::waitKey(10);
    }

}
/*
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
    //load environment
    cv::Mat img = cv::imread("/home/xcy/Cspace/SampleBasedPlanningMethods/map6.png");
    //set sample bounds
    Eigen::MatrixX2d bounds;
    bounds.resize(2, 2);
    bounds << Eigen::Vector2d{img.cols, img.rows},
            Eigen::Vector2d::Zero();

    //set start state
    state_space::Rn start_state{std::vector<double>{1,1}};
    //set valid goal state
    state_space::Rn goal_state{std::vector<double>{1990,1990}};
    std::size_t i=0;
    const std::size_t max_iterations=1e4;
    const std::string base_path{"/home/xcy/RRT_EXP2D/"};
    const std::string goal_list{"Goals.txt"};

    std::string final_path = base_path+goal_list;
    const std::vector<std::string> txt_names{{"/time.txt"},{"/pathLength.txt"},{"/ccTimes.txt"},
                                             {"/nodes.txt"},{"/invalidGoals.txt"},{"/performance.txt"}};

    std::size_t  iter_index=0;
    std::size_t total_time{};
    std::size_t valid_times=0;
    double total_length{};
    std::size_t total_path_size{};
    std::size_t total_nodes{};
    std::size_t total_ccTimes{};

    std::ifstream goal_src(final_path.c_str());

    auto rrt_based_planner_ptr = planner::createPlanner<state_space::Rn,flann::L2_Simple<double>>(planner::RRT_SIMPLE,start_state.Dimensions());

    rrt_based_planner_ptr->setStateValidator(std::function<bool(const state_space::Rn &, const state_space::Rn &)>(
            isPathValid_incremental));
    rrt_based_planner_ptr->setSampleBounds(&bounds);

    std::vector<std::ofstream> outfile_vector{};
    outfile_vector.resize(txt_names.size());
    for(int i=0;i<txt_names.size();++i){
        outfile_vector[i].open(base_path+rrt_based_planner_ptr->getName()+txt_names[i],std::ios::out | std::ios::trunc);
    }

    while(iter_index++<max_iterations){
        std::string s;getline(goal_src,s);
        std::istringstream stringGet(s);
        stringGet>>start_state[0]>>start_state[1]>>goal_state[0]>>goal_state[1];
        rrt_based_planner_ptr->constructPlan(planner::PLAN_REQUEST<state_space::Rn,flann::L2_Simple<double>>(start_state, goal_state, 10,200,false,30,0));
        std::vector<state_space::Rn> path;
        clock_t start(clock());
        if (rrt_based_planner_ptr->planning()) {
            path = rrt_based_planner_ptr->GetPath();
            //valid times
            valid_times++;
            //single time
            outfile_vector[0]<<(double)(clock()-start)/CLOCKS_PER_SEC<<std::endl;
            //single length
            double path_length{};
            for(int i=0; i<path.size()-1;++i){
                path_length += planner::distance(path[i],path[i+1]);
            }
            outfile_vector[1]<<path.size()<<" "<<path_length<<std::endl;
            //single cc times
            outfile_vector[2]<<check_times<<std::endl;
            //single iter times
            outfile_vector[3]<<rrt_based_planner_ptr->getTotalNodes()<<std::endl;

            //average usage
            total_time += clock() - start;
            total_length += path_length;
            total_path_size += path.size();
            total_ccTimes += check_times;
            total_nodes+=rrt_based_planner_ptr->getTotalNodes();
            check_times=0;
        }
        else{
            outfile_vector[4]<<goal_state<<std::endl;
        }
    }
    outfile_vector[5]<<"average planning time: "<<(double)total_time / valid_times / CLOCKS_PER_SEC<<"s"<<std::endl;
    outfile_vector[5]<<"average path length: "<<total_length/valid_times<<std::endl;
    outfile_vector[5]<<"average path size: "<<(double)total_path_size/valid_times<<std::endl;
    outfile_vector[5]<<"average cc Times: "<<(long double)total_ccTimes/valid_times<<std::endl;
    outfile_vector[5] << "average nodes: " << (long double)total_nodes / valid_times << std::endl;
    outfile_vector[5]<<"valid percent: "<<valid_times<<"/"<<max_iterations<<": "<<(double)valid_times/max_iterations<<std::endl;

    if(goal_src.is_open()) {
        goal_src.close();
    }
    for(int i=0; i<txt_names.size();++i){
        if(outfile_vector[i].is_open()){
            outfile_vector[i].close();
        }
    }
}
TEST(RRTTest, R2SingleTest){
    //load environment
    std::shared_ptr<boost::thread> thread_ptr_;
    cv::Mat img = cv::imread("/home/xcy/Cspace/SampleBasedPlanningMethods/3.png");
    //set sample bounds
    Eigen::MatrixX2d bounds;
    bounds.resize(2, 2);
    bounds << Eigen::Vector2d{img.cols, img.rows},
            Eigen::Vector2d::Zero();

    state_space::Rn start_state{std::vector<double>{190,245}};
    state_space::Rn goal_state{std::vector<double>{650,360}};

    auto rrt_based_planner_ptr = planner::createPlanner<state_space::Rn,flann::L2_Simple<double>>(planner::RRT_SIMPLE,start_state.Dimensions());
    rrt_based_planner_ptr->setStateValidator(std::function<bool(const state_space::Rn &, const state_space::Rn &)>(
            isPathValid_vdc));
    rrt_based_planner_ptr->setSampleBounds(&bounds);
    rrt_based_planner_ptr->constructPlan(planner::PLAN_REQUEST<state_space::Rn,flann::L2_Simple<double>>(start_state, goal_state, 5000,60,false,30,0.05));
    std::vector<state_space::Rn> path;
    clock_t before{clock()};
    //thread_ptr_.reset(new boost::thread(boost::bind(&animate<state_space::Rn>,img,rrt_based_planner_ptr->getRootVertex())));
    if (rrt_based_planner_ptr->planning()) {
        std::cout<<"time: "<<static_cast<double>((clock()-before))/CLOCKS_PER_SEC<<""
                                                                                   "s"<<std::endl;
        path = rrt_based_planner_ptr->GetPath();
        auto root_vertex=rrt_based_planner_ptr->getRootVertex();
        //visualization
        for(const auto& item: root_vertex){
            _animate(img,item);
        }
        for (std::size_t i = 1; i < path.size(); ++i) {
            cv::line(img, cv::Point(path[i].Vector()[0], path[i].Vector()[1]),
                     cv::Point(path[i - 1].Vector()[0], path[i - 1].Vector()[1]), cv::Scalar{0, 0, 255}, 2);
        }
        double path_length{};
        for(int i=0; i<path.size()-1;++i){
            path_length += planner::distance(path[i],path[i+1]);
        }
        std::cout<<path.size()<<" "<<path_length<<std::endl;
        std::cout<<"nodes: "<<rrt_based_planner_ptr->getTotalNodes()<<std::endl;
        std::cout<<"cc Times: "<<check_times<<std::endl;
        cv::resize(img,img,cv::Size{800,800});
        cv::imshow("check: ",img);
        cv::waitKey(0);
        //cv::imwrite("/home/xcy/"+rrt_based_planner_ptr->getName()+".png",img);
    }
    //thread_ptr_->interrupt();
    //thread_ptr_->join();
}

int main(int argc, char **argv)
{
    logger lg(argv[0]);
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "RRTTest");
    return RUN_ALL_TESTS();
}
