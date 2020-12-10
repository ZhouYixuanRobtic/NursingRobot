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
    joint_angles<<0,0,0,0,0.1,0;

    Eigen::MatrixXd joint_solutions;
    clock_t start(clock());
    if(robotModel.allIkSolutions(joint_solutions,robotModel.fk(joint_angles).matrix()))
    {
        clock_t end(clock());
        std::cout<<1e6*(double)(end-start)/CLOCKS_PER_SEC<<"us"<<std::endl;
        for(int i=0;i<joint_solutions.cols();++i)
            std::cout<<joint_solutions.col(i).transpose()<<std::endl;
    }

}
