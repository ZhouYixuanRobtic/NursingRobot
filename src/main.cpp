#include <iostream>
#include "SE3.h"
#include "RobotModel.h"
#include <ctime>
#include "RobotModel.cpp"
int main()
{

    Eigen::Matrix<double,7,1> pose_with_quaternion{};
    pose_with_quaternion<<-0.0405, 0.0, 0.14775,0.000, -0.000, sqrt(2)/2, sqrt(2)/2;
    LieGroup::SE3 ee_configuration(pose_with_quaternion);


    std::string yaml_path = "../config/aubo_i5.yaml";
    RobotModel robotModel(yaml_path);
    robotModel.addEndEffector(ee_configuration);

    LieGroup::R6 joint_angles;
    joint_angles<<M_PI_2,0,0,0,0,0;

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
