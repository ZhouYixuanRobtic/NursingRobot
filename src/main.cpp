#include <iostream>
#include "SE3.h"
#include "RobotModel.h"
#include <ctime>
#include "RobotModel.cpp"

static double distanceInR6(const LieGroup::R6 &start, const LieGroup::R6& end)
{
    LieGroup::R6 joint_tangent{end - start};
    for(int i=0; i<6;++i)
    {
        if(joint_tangent[i]>M_PI)
            joint_tangent[i] -= 2*M_PI;
        else if(joint_tangent[i]< -M_PI)
            joint_tangent[i]+=2*M_PI;
    }
    return joint_tangent.norm();
}

static LieGroup::R6 bzierCurveSampleInJointSpace(const LieGroup::R6 &start, const LieGroup::R6& end,double lambda)
{
    LieGroup::R6 result;
    LieGroup::R6 delta = end-start;
    result = start*1*pow(lambda,0)*pow((1-lambda),5-0)+
            (start-2*delta)*5*pow(lambda,1)*pow((1-lambda),5-1)+
            (start-delta)*10*pow(lambda,2)*pow((1-lambda),5-2)+
            (start+3*delta)*10*pow(lambda,3)*pow((1-lambda),5-3)+
            (start+2*delta)*5*pow(lambda,4)*pow((1-lambda),5-4)+
            end*1*pow(lambda,5)*pow((1-lambda),5-5);
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

    Eigen::VectorXd initial_angles(6);
    initial_angles<<1,1,1,1,-0.1,1;
    std::random_device rd;
    std::default_random_engine randomEngine(rd());
    std::uniform_real_distribution<double> x_distribution(-M_PI,M_PI);
    clock_t start,end;
    double interval{}; int counter =0;
    /* test 1 for non-secutive mode;
    for(int i = 0; i<1E7; ++i)
    {
        Eigen::VectorXd random_angles(6);
        random_angles<<x_distribution(randomEngine),x_distribution(randomEngine),x_distribution(randomEngine)
        ,x_distribution(randomEngine),x_distribution(randomEngine),x_distribution(randomEngine);
        LieGroup::SE3 origin(robotModel.fk(random_angles).matrix());
        Eigen::Affine3d desired_pose{origin.SE3Matrix()};
        start = clock();
        Eigen::MatrixXd joint_solutions;
        RobotModel::IK_SINGULAR_CODE ret=robotModel.allIkSolutions(joint_solutions,desired_pose.matrix());
        end = clock();
        interval += (double)(end-start)/CLOCKS_PER_SEC;
        if(ret != RobotModel::NO_SOLUTIONS)
        {
            LieGroup::SE3 test_result(robotModel.fk(joint_solutions.col(0)).matrix());

        }
    }*/
    for(int i = 0; i<1e4; ++i)
    {
        Eigen::VectorXd random_start_angles(6);
        random_start_angles<<x_distribution(randomEngine),x_distribution(randomEngine),x_distribution(randomEngine)
                ,x_distribution(randomEngine),x_distribution(randomEngine),x_distribution(randomEngine);

        Eigen::VectorXd random_end_angles(6);
        random_end_angles<<x_distribution(randomEngine),x_distribution(randomEngine),x_distribution(randomEngine)
                ,x_distribution(randomEngine),x_distribution(randomEngine),x_distribution(randomEngine);
        double step = 1e-3;
        LieGroup::R6 reference = random_start_angles;
        LieGroup::R6 tangent_reference;
        for(int j=1; j< (double)(1.0+step) /step ;j++)
        {
            LieGroup::R6 random_angles(bzierCurveSampleInJointSpace(random_start_angles,random_end_angles,j*step));
            Eigen::Affine3d desired_pose{robotModel.fk(random_angles).matrix()};
            start = clock();
            LieGroup::R6  result;
            if(j == 1)
                result =robotModel.nearestIkSolution(desired_pose,reference,true);
            else
                result =robotModel.directedNearestIkSolution(desired_pose,reference,tangent_reference);
            end = clock();
            interval += (double)(end-start)/CLOCKS_PER_SEC;
            if((result - random_angles).norm()<1e-4)
            {
                counter++;
            }
            tangent_reference = result - reference;
            for(int k=0; k<6;++k)
            {
                if(tangent_reference[k]>M_PI)
                    tangent_reference[k] -= 2*M_PI;
                else if(tangent_reference[k]< -M_PI)
                    tangent_reference[k]+=2*M_PI;
            }
            reference =result;

        }

    }
    std::cout<<"average time consumption: "<<1e6*interval/1e7 <<"us"<<std::endl;
    std::cout<<"coverage rate "<<(double)counter/1e7<<std::endl;




}
