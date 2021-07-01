/**
 * This file is for vdc incremental, and binary collision check
 * @author Yixuan Zhou
 */
#include <gtest/gtest.h>
#include "util/logger.hpp"
#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "opencv2/opencv.hpp"
#include "planner/PlannerMethod.hpp"


enum CHECK_TYPE{
    VDC,
    INCREMENTAL,
    BINARY
};
/**
 * 2D collision_check
 * check bounds and collision
 * @return
 */
inline bool collision_check(const state_space::Rn& test_state){
    static cv::Mat environment{cv::imread("/home/xcy/Cspace/SampleBasedPlanningMethods/map0.png", 0)};
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
bool isPathValid_vdc(const state_space::Rn& from, const state_space::Rn& to,double& check_percent){
    const double min_distance=0.5;
    unsigned int K = ceil(log2(ceil(planner::distance(from,to)/min_distance)));
    unsigned int bits = K;
    K = 1<<K;
    check_percent = static_cast<double>(1)/K;
    if(!collision_check(to))
        return false;
    for(int k=0; k<K;++k){
        check_percent = static_cast<double>(k+1)/K;
        auto intermediate_state = planner::interpolate(from, to, vdc(k,bits)/K);
        if (!collision_check(intermediate_state))
            return false;
    }
    return true;
}
bool isPathValid_incremental(const state_space::Rn& from, const state_space::Rn& to,double& check_percent){
    const double min_distance=0.5;
    unsigned int K= ceil(planner::distance(from,to)/min_distance);
    for(int k=0;k<=K;++k) {
        check_percent = static_cast<double>(k+1)/K;
        auto intermediate_state = planner::interpolate(from, to, (double)k/K);
        if(!collision_check(intermediate_state))
            return false;
    }
    return true;
}
bool isPathValid_binary(const state_space::Rn& from, const state_space::Rn& to,double& check_percent){
    const double min_distance=0.5;
    unsigned int K = ceil(log2(ceil(planner::distance(from,to)/min_distance)));
    unsigned int bits = K ? K:1;
    K = 1<<K;
    check_percent = static_cast<double>(1)/K;
    if(!collision_check(from))
        return false;
    check_percent = static_cast<double>(2)/K;
    if(!collision_check(to))
        return false;

    double checked_length;
    unsigned int k=0;
    for(int i=0; i<=bits;i++){
        for(int j=0; j<(1<<(i-1));j++){
            check_percent = static_cast<double>(k+1)/K;
            k++;
            checked_length = (double)(2*j+1)/(1<<i);
            auto intermediate_state = planner::interpolate(from, to, checked_length);
            if (!collision_check(intermediate_state))
                return false;
        }
    }
    return true;
}

TEST(vdcTest,2dVdcTest){

    const std::string base_path{"/home/xcy/VDC_SAT/times.txt"};
    std::ofstream out_file{base_path,std::ios::out|std::ios::trunc};
    cv::Mat img = cv::imread("/home/xcy/Cspace/SampleBasedPlanningMethods/map0.png");
    Eigen::MatrixX2d bounds;
    bounds.resize(2, 2);
    bounds << Eigen::Vector2d{img.cols, img.rows},
              Eigen::Vector2d::Zero();

    //overall performance
    auto start_state = planner::randomState<state_space::Rn>(2,&bounds);
    auto goal_state = planner::randomState<state_space::Rn>(2,&bounds);
    double return_vdc_index,return_incremental_index,return_binary_index;
    double vdc_index{},incremental_index{},binary_index{};
    int i=0;
    const std::size_t max_iterations=1E6;
    while(i++<max_iterations){
        start_state = planner::randomState<state_space::Rn>(2,&bounds);
        goal_state = planner::randomState<state_space::Rn>(2,&bounds);
        isPathValid_vdc(start_state,goal_state,return_vdc_index);
        vdc_index+=return_vdc_index;
        isPathValid_incremental(start_state,goal_state,return_incremental_index);
        incremental_index+=return_incremental_index;
        isPathValid_binary(start_state,goal_state,return_binary_index);
        binary_index+=return_binary_index;
        out_file<<return_vdc_index<<" "<<return_incremental_index<<" "<<return_binary_index<<std::endl;
    }
    std::cout<<"vdc overall performance: "<<static_cast<double>(vdc_index)/max_iterations<<std::endl;
    std::cout<<"incremental overall performance: "<<static_cast<double>(incremental_index)/max_iterations<<std::endl;
    std::cout<<"binary overall performance: "<<static_cast<double>(binary_index)/max_iterations<<std::endl;
    if(out_file.is_open())
        out_file.close();
}
int main(int argc, char **argv)
{
    logger lg(argv[0]);
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "vdcTest");
    return RUN_ALL_TESTS();
}