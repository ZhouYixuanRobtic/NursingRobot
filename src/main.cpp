#include <iostream>
#include "SE3.h"
#include "RobotModel.h"
#include <ctime>
#include "RobotModel.cpp"
int main() {
    /*
    Eigen::Matrix<double,7,1> pose_with_quaternion{};
    pose_with_quaternion<<-0.0405, -0.36325, 1.0085,0.500, -0.500, 0.500, 0.500;
    LieGroup::SE3 home_configuration(pose_with_quaternion);
    LieGroup::R6 S1;
    S1<<0,0,1,0,0,0;

    LieGroup::R6 B1;
    B1<<1,0,0,0,-0.36325,0.0405;

    std::cout<<LieGroup::getAdjoint(home_configuration.SE3Matrix())*B1<<std::endl;*/
    Eigen::Matrix<double,7,1> pose_with_quaternion{};
    pose_with_quaternion<<-0.0405, -0.36325, 1.0085,0.500, -0.500, 0.500, 0.500;
    LieGroup::SE3 home_configuration(pose_with_quaternion);

    std::string yaml_path = "../config/aubo_i5.yaml";
    RobotModel robotModel(yaml_path);
    LieGroup::R6 joint_angles;
    joint_angles<<M_PI_2,0,0,0,0,0;
    //clock_t start(clock());
    Eigen::Affine3d ee_pose = robotModel.fk(joint_angles);
    Eigen::VectorXd init_angles(6);
    init_angles<<M_PI_2-0.1,0,0,0,0,0;
    clock_t start(clock());
    if(robotModel.nIk(ee_pose,init_angles))
    {
        clock_t end(clock());
        std::cout<<((double)(end-start))/CLOCKS_PER_SEC<<std::endl;
        std::cout<<init_angles<<std::endl;
    }
}
