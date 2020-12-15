#include <iostream>
#include "StateSpace/SE3.hpp"
#include "RobotModel.h"
#include <ctime>
#include "RobotModel.cpp"
#include "StateSpace/StateSpace.hpp"
static double distanceInR6(const state_space::R6 &start, const state_space::R6& end)
{
    state_space::R6 joint_tangent{end - start};
    for(int i=0; i<6;++i)
    {
        if(joint_tangent[i]>M_PI)
            joint_tangent[i] -= 2*M_PI;
        else if(joint_tangent[i]< -M_PI)
            joint_tangent[i]+=2*M_PI;
    }
    return joint_tangent.norm();
}

static state_space::R6 bezierCurveSampleInJointSpace(const state_space::R6 &start, const state_space::R6& end, double lambda)
{
    state_space::R6 result = state_space::bezierInterpolate(start, end, lambda);
    for(int i=0; i<6;++i)
    {
        if(result[i]>M_PI)
            result[i] -= 2*M_PI;
        else if(result[i]< -M_PI)
            result[i]+=2*M_PI;
    }
    return result;
}

int main()
{
    Eigen::Matrix<double,7,1> pose_with_quaternion{};
    pose_with_quaternion<<-0.000, 0.2155, 0.8865,0.000, sqrt(2)/2, sqrt(2)/2,0.0;
    state_space::SE3 robot_home_configuration{pose_with_quaternion};

    pose_with_quaternion<<0.000, 0.000, 0.122,0,0,1,0;
    state_space::SE3 mount_configuration{pose_with_quaternion};

    pose_with_quaternion<< -0.0405, 0.0, 0.14775,0.000, -0.000, sqrt(2)/2, sqrt(2)/2;
    state_space::SE3 ee_configuration{pose_with_quaternion};

    const std::string model_path = "../config/aubo_i5.yaml";
    RobotModel robotModel(model_path);

    robotModel.addEndEffector(ee_configuration);
    robotModel.addMount(mount_configuration);

    std::cout<<(ee_configuration*2.0).SE3Matrix()<<std::endl;
    state_space::R6 temp_twist{2 * ee_configuration.Vector()};
    std::cout << (state_space::SE3(temp_twist).SE3Matrix()) << std::endl;



}
