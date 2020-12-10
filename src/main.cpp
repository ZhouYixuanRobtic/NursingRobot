#include <iostream>
#include "SE3.h"
#include "RobotModel.h"
#include <ctime>
#include "RobotModel.cpp"


int main()
{
    Eigen::Matrix<double,7,1> pose_with_quaternion{};
    pose_with_quaternion<<-0.000, 0.2155, 0.8865,0.000, sqrt(2)/2, sqrt(2)/2,0.0;
    LieGroup::SE3 robot_home_configuration{pose_with_quaternion};

    pose_with_quaternion<<0.000, 0.000, 0.122,0,0,1,0;
    LieGroup::SE3 mount_configuration{pose_with_quaternion};

    pose_with_quaternion<< -0.0405, 0.0, 0.14775,0.000, -0.000, sqrt(2)/2, sqrt(2)/2;
    LieGroup::SE3 ee_configuration{pose_with_quaternion};

    const std::string model_path = "../config/aubo_i5.yaml";
    RobotModel robotModel(model_path);

    robotModel.addEndEffector(ee_configuration);
    robotModel.addMount(mount_configuration);

    Eigen::VectorXd joint_angles(6);
    joint_angles<<1,1,1,1,1,1;

    //std::cout<<robotModel.fk(joint_angles).matrix()<<std::endl;
    LieGroup::SE_3 desired_pose =(mount_configuration.SE3Matrix().inverse())*robotModel.fk(joint_angles).matrix()*(ee_configuration.SE3Matrix().inverse());
    clock_t start(clock());
    std::vector<Eigen::VectorXd> a = robotModel.allIkSolutions(desired_pose);

    for(const auto & a_e : a)
        std::cout<<a_e.transpose()<<std::endl;
}
