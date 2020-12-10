//
// Created by xcy on 2020/12/2.
//

#include <iostream>
#include "RobotModel.h"
RobotModel::RobotModel(const std::string &yaml_name)
{
    ee_configuration_ = LieGroup::SE3();
    mount_configuration_ = LieGroup::SE3();
    loadModel(yaml_name);
}
RobotModel::RobotModel(const std::vector<LieGroup::SE3> &all_screw_axes,const LieGroup::SE3 &home_configuration,bool isInBodyFrame)
{
    all_screw_axes_ = all_screw_axes;
    home_configuration_ = home_configuration;
    IN_BODY_ = isInBodyFrame;
    ee_configuration_ = LieGroup::SE3();
    mount_configuration_ = LieGroup::SE3();
    current_joint_angles_ = Eigen::VectorXd(all_screw_axes.size());
    current_joint_angles_.setZero();
}
void RobotModel::loadModel(const std::string &yaml_name)
{
    YAML::Node doc = YAML::LoadFile(yaml_name);
    try
    {
        std::vector<double> pose_with_quaternion = doc["aubo_i5"]["home_configuration"].as<std::vector<double>>();
        Eigen::Matrix<double,7,1> home_pose;
        memcpy(home_pose.data(),pose_with_quaternion.data(), pose_with_quaternion.size()*sizeof(double));
        home_configuration_ = LieGroup::SE3(home_pose);
        IN_BODY_ = doc["aubo_i5"]["isInBodyFrame"].as<bool>();
        std::map<std::string,std::vector<double>> axes_map;
        axes_map = doc["aubo_i5"]["screw_axes"].as<std::map<std::string,std::vector<double>>>();

        for(const auto & it : axes_map)
        {
            if(it.second.size()!=6)
            {
                char buf[100];
                sprintf(buf,"wrong twist, should have 6 elements but have %d elements",(int)it.second.size());
                perror(buf);
            }
            else
            {
                LieGroup::R6 temp_twist;
                memcpy(temp_twist.data(),it.second.data(), temp_twist.size()*sizeof(double));
                all_screw_axes_.emplace_back(LieGroup::SE3(temp_twist));
            }
        }
    }
    catch (YAML::InvalidScalar)
    {
       perror("tagParam.yaml is invalid.");
    }
    current_joint_angles_ = Eigen::VectorXd(all_screw_axes_.size());
    current_joint_angles_.setZero();
}
inline LieGroup::SE_3 RobotModel::fkInSpace(const Eigen::VectorXd &joint_angles)
{
    LieGroup::SE_3 ee_pose{LieGroup::SE_3::Identity()};
    for (int i = 0; i < joint_angles.size(); ++i)
    {
        ee_pose = ee_pose* all_screw_axes_[i](joint_angles[i]).SE3Matrix();
    }
    ee_pose = ee_pose * home_configuration_.SE3Matrix();
    return ee_pose;
}
inline LieGroup::SE_3 RobotModel::fkInBody(const Eigen::VectorXd &joint_angles)
{
    LieGroup::SE_3 ee_pose{home_configuration_.SE3Matrix()};
    for (int i = 0; i < joint_angles.size(); ++i)
    {
        ee_pose = ee_pose* all_screw_axes_[i](joint_angles[i]).SE3Matrix();
    }
    return ee_pose;
}
inline Eigen::MatrixXd RobotModel::jacobianSpace(const Eigen::VectorXd & joint_angles)
{
    Eigen::MatrixXd jacobian_matrix(6,6);
    jacobian_matrix.col(0) = all_screw_axes_[0].Axis();
    LieGroup::SE_3  tmp_matrix = LieGroup::SE_3::Identity();
    for (int i = 1; i < joint_angles.size(); i++)
    {
        tmp_matrix = tmp_matrix * all_screw_axes_[i-1](joint_angles[i-1]).SE3Matrix();
        jacobian_matrix.col(i) = LieGroup::getAdjoint(tmp_matrix) * all_screw_axes_[i].Axis();
    }
    return jacobian_matrix;
}
inline Eigen::MatrixXd RobotModel::jacobianBody(const Eigen::VectorXd & joint_angles)
{
    Eigen::MatrixXd jacobian_matrix(6,6);
    LieGroup::SE_3  tmp_matrix = LieGroup::SE_3::Identity();
    jacobian_matrix.col(joint_angles.size()-1) = all_screw_axes_[joint_angles.size()-1].Axis();
    for (int i = joint_angles.size() - 2; i >= 0; i--)
    {
        tmp_matrix = tmp_matrix * all_screw_axes_[i+1](-joint_angles[i+1]).SE3Matrix();
        jacobian_matrix.col(i) = LieGroup::getAdjoint(tmp_matrix) * all_screw_axes_[i].Axis();
    }
    return jacobian_matrix;
}
inline bool RobotModel::nIkInSpace(const Eigen::Affine3d &desired_pose,Eigen::VectorXd &joint_angles, double eomg, double ev)
{
    /*
     * jacobian iterative method
     */
    int i = 0;
    int max_iterations = 50;
    LieGroup::SE_3 Tfk = fkInSpace(joint_angles);
    LieGroup::SE_3 Tdiff = Tfk.inverse() * desired_pose.matrix();
    LieGroup::R6 Vs = LieGroup::getAdjoint(Tfk)*LieGroup::SE3(Tdiff).Vector();
    bool err = (Vs.block<3,1>(0,0).norm() > eomg || Vs.block<3,1>(3,0).norm() > ev);
    Eigen::MatrixXd Js;
    while (err && i < max_iterations)
    {
        Js = jacobianSpace( joint_angles);
        joint_angles += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
        ++i;
        // iterate
        Tfk = fkInSpace(joint_angles);
        Tdiff = Tfk.inverse() * desired_pose.matrix();
        Vs = LieGroup::getAdjoint(Tfk)*LieGroup::SE3(Tdiff).Vector();
        err = (Vs.block<3,1>(0,0).norm() > eomg || Vs.block<3,1>(3,0).norm() > ev);
    }
    return !err;
}
inline bool RobotModel::nIkInBody(const Eigen::Affine3d &desired_pose,Eigen::VectorXd &joint_angles, double eomg, double ev)
{
    int i = 0;
    int max_iterations = 50;
    LieGroup::SE_3 Tfk = fkInBody(joint_angles).matrix();
    LieGroup::SE_3 Tdiff = Tfk.inverse() * desired_pose.matrix();
    LieGroup::R6 Vb = LieGroup::SE3(Tdiff).Vector();
    bool err = (Vb.block<3,1>(0,0).norm() > eomg || Vb.block<3,1>(3,0).norm() > ev);
    Eigen::MatrixXd Jb;
    while (err && i < max_iterations)
    {
        Jb = jacobianBody( joint_angles);
        joint_angles += Jb.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
        ++i;
        // iterate
        Tfk = fkInBody(joint_angles).matrix();
        Tdiff = Tfk.inverse() * desired_pose.matrix();
        Vb = LieGroup::SE3(Tdiff).Vector();
        err = (Vb.block<3,1>(0,0).norm() > eomg || Vb.block<3,1>(3,0).norm() > ev);
    }
    return !err;
}
inline Eigen::VectorXd RobotModel::nearestIkSolution(const Eigen::Affine3d &desired_pose)
{
    //return the nearest ik solution
    Eigen::VectorXd solution{current_joint_angles_};
    if(nIk(desired_pose,solution))
        return solution;
    else
        return current_joint_angles_;
}
inline double RobotModel::customAtan2(double y, double x)
{
    double eps = 1e-8;
    double angle = 0;
    if((fabs(y) < eps)&&(fabs(x) < eps))
    {
        return 0;
    }
    if(fabs(x) < eps)
        angle = M_PI/2.0*SIGN(y);
    else if(fabs(y) < eps)
    {
        if (SIGN(x) == 1)
            angle = 0;
        else
            angle = M_PI;
    }
    else
    {
        angle = atan2(y, x);
    }

    return angle;
}
inline int RobotModel::allIkSolutions(Eigen::MatrixXd &joint_solutions,const LieGroup::SE_3 &desired_motion)
{
    static double h1=0.1215,d1=0.408,d2=0.376,d=0.8865;
    Eigen::Vector4d p1{0,h1,d,1};
    LieGroup::SE_3 M1{desired_motion*home_configuration_.SE3Matrix().inverse()};
    p1=M1*p1;

    //step 2 solving theta1
    //in case of very small negative value;
    double r1 = p1[0]*p1[0]+p1[1]*p1[1]-h1*h1;
    //r1 = nearZero(r1);
    //r1 = sqrt(r1);
    double theta1[2] = {customAtan2(p1[1]*r1-h1*p1[0],p1[1]*h1+p1[0]*r1),
                      customAtan2(-p1[1]*r1-h1*p1[0],p1[1]*h1-p1[0]*r1)};
    for(int i=0;i<2;++i)
    {
        if(theta1[i] > M_PI)
            theta1[]
    }


    //step 3 solving theta5 theta6
    double r3 = desired_motion(1,2)*cos(theta1[0]) - desired_motion(0,2)*sin(theta1[0]);
    double theta5[4];
    theta5[0] =acos(r3);theta5[1]=-acos(r3);

    r3 = desired_motion(1,2)*cos(theta1[1]) - desired_motion(0,2)*sin(theta1[1]);
    r3 = nearZero(r3-1)+1;
    theta5[2] =acos(r3);theta5[3]=-acos(r3);
    double theta6[4]{}, sum_theta[4]{};
    if(std::abs(1-r3) <1E-8)
    {
        std::cout<<"singularity"<<std::endl;
    }
    else
    {
        double r4 = desired_motion(1,1)*cos(theta1[0]) - desired_motion(0,1)*sin(theta1[0]);
        double r2 = desired_motion(0,0)*sin(theta1[0]) - desired_motion(1,0)*cos(theta1[0]);
        double r5 = desired_motion(0,2)*cos(theta1[0]) + desired_motion(1,2)*sin(theta1[0]);
        theta6[0] = atan2(r4/sin(theta5[0]),r2/sin(theta5[0]));
        theta6[1] = atan2(r4/sin(theta5[1]),r2/sin(theta5[1]));
        sum_theta[0] = atan2(desired_motion(2,2)/sin(theta5[0]),-r5/sin(theta5[0]));
        sum_theta[1] = atan2(desired_motion(2,2)/sin(theta5[1]),-r5/sin(theta5[1]));

        r4 = desired_motion(1,1)*cos(theta1[1]) - desired_motion(0,1)*sin(theta1[1]);
        r2 = desired_motion(0,0)*sin(theta1[1]) - desired_motion(1,0)*cos(theta1[1]);
        r5 = desired_motion(0,2)*cos(theta1[1]) + desired_motion(1,2)*sin(theta1[1]);
        theta6[2] = atan2(r4/sin(theta5[0]),r2/sin(theta5[0]));
        theta6[3] = atan2(r4/sin(theta5[1]),r2/sin(theta5[1]));
        sum_theta[2] = atan2(desired_motion(2,2)/sin(theta5[0]),-r5/sin(theta5[0]));
        sum_theta[3] = atan2(desired_motion(2,2)/sin(theta5[1]),-r5/sin(theta5[1]));
    }
    double theta2[8]{},theta3[8]{},theta4[8]{};

    LieGroup::SE_3 e_i1,e_i5,e_i6;
    e_i1<<cos(theta1[0]),sin(theta1[0]),0,0,
            -sin(theta1[0]),cos(theta1[0]),0,0,
            0,0,1,0,
            0,0,0,1;
    e_i6<<cos(theta6[0]),0,-sin(theta6[0]),sin(theta6[0])*0.8865,
            0,1,0,0,
            sin(theta6[0]),0,cos(theta6[0]),(1-cos(theta6[0]))*0.8865,
            0,0,0,1;
    e_i5<<cos(theta5[0]),sin(theta5[0]),0,-0.1215*sin(theta5[0]),
            -sin(theta5[0]),cos(theta5[0]),0,0.1215*(1-cos(theta6[0])),
            0,0,1,0,
            0,0,0,1;

    LieGroup::SE_3 M3{e_i1*M1*e_i6*e_i5};
    double r6 = M3(0,3)+0.784*sin(sum_theta[0]);
    double r7 = M3(2,3)+0.784*cos(sum_theta[0]);
    double s1 = (r6*r6+ r7*r7-0.408*0.408-0.376*0.376)/(2*0.408*0.376);
    s1 =nearZero(s1-1)+1;
    double alpha =atan2(r6,r7);
    double s2 = (r6*r6+ r7*r7+0.408*0.408-0.376*0.376)/(2*0.408*sqrt(r6*r6+ r7*r7));
    s2 =nearZero(s2-1)+1;
    theta3[0] = acos(s1);
    theta3[1] = -acos(s1);
    theta2[0] = acos(s2)+ alpha;
    theta2[1] = -acos(s2)+ alpha;
    theta4[0] = sum_theta[0]-theta2[0]+theta3[0];
    theta4[1] = sum_theta[0]-theta2[1]+theta3[1];

    e_i1<<cos(theta1[0]),sin(theta1[0]),0,0,
          -sin(theta1[0]),cos(theta1[0]),0,0,
          0,0,1,0,
          0,0,0,1;
    e_i6<<cos(theta6[1]),0,-sin(theta6[1]),sin(theta6[1])*0.8865,
            0,1,0,0,
            sin(theta6[1]),0,cos(theta6[1]),(1-cos(theta6[1]))*0.8865,
            0,0,0,1;
    e_i5<<cos(theta5[1]),sin(theta5[1]),0,-0.1215*sin(theta5[1]),
     -sin(theta5[1]),cos(theta5[1]),0,0.1215*(1-cos(theta6[1])),
     0,0,1,0,
     0,0,0,1;

    M3 =e_i1*M1*e_i6*e_i5;

    r6 = M3(0,3)+0.784*sin(sum_theta[1]);
    r7 = M3(2,3)+0.784*cos(sum_theta[1]);
    s1 = (r6*r6+ r7*r7-0.408*0.408-0.376*0.376)/(2*0.408*0.376);
    s1 =nearZero(s1-1)+1;
    alpha =atan2(r6,r7);
    s2 = (r6*r6+ r7*r7+0.408*0.408-0.376*0.376)/(2*0.408*sqrt(r6*r6+ r7*r7));
    s2 =nearZero(s2-1)+1;
    theta3[2] = acos(s1);
    theta3[3] = -acos(s1);
    theta2[2] = acos(s2) + alpha;
    theta2[3] = -acos(s2)+ alpha;
    theta4[2] = sum_theta[1]-theta2[2]+theta3[2];
    theta4[3] = sum_theta[1]-theta2[3]+theta3[3];

    e_i1<<cos(theta1[1]),sin(theta1[1]),0,0,
            -sin(theta1[1]),cos(theta1[1]),0,0,
            0,0,1,0,
            0,0,0,1;
    e_i6<<cos(theta6[2]),0,-sin(theta6[2]),sin(theta6[2])*0.8865,
            0,1,0,0,
            sin(theta6[2]),0,cos(theta6[2]),(1-cos(theta6[2]))*0.8865,
            0,0,0,1;
    e_i5<<cos(theta5[2]),sin(theta5[2]),0,-0.1215*sin(theta5[2]),
            -sin(theta5[2]),cos(theta5[2]),0,0.1215*(1-cos(theta6[2])),
            0,0,1,0,
            0,0,0,1;
    M3 =e_i1*M1*e_i6*e_i5;
    r6 = M3(0,3)+0.784*sin(sum_theta[2]);
    r7 = M3(2,3)+0.784*cos(sum_theta[2]);
    s1 = (r6*r6+ r7*r7-0.408*0.408-0.376*0.376)/(2*0.408*0.376);
    s1 =nearZero(s1-1)+1;
    alpha =atan2(r6,r7);
    s2 = (r6*r6+ r7*r7+0.408*0.408-0.376*0.376)/(2*0.408*sqrt(r6*r6+ r7*r7));
    s2 =nearZero(s2-1)+1;
    theta3[4] = acos(s1);
    theta3[5] = -acos(s1);
    theta2[4] = acos(s2) + alpha;
    theta2[5] = -acos(s2) + alpha;
    theta4[4] = sum_theta[2]-theta2[4]+theta3[4];
    theta4[5] = sum_theta[2]-theta2[5]+theta3[5];

    e_i1<<cos(theta1[1]),sin(theta1[1]),0,0,
            -sin(theta1[1]),cos(theta1[1]),0,0,
            0,0,1,0,
            0,0,0,1;
    e_i6<<cos(theta6[3]),0,-sin(theta6[3]),sin(theta6[3])*0.8865,
            0,1,0,0,
            sin(theta6[3]),0,cos(theta6[3]),(1-cos(theta6[3]))*0.8865,
            0,0,0,1;
    e_i5<<cos(theta5[3]),sin(theta5[3]),0,-0.1215*sin(theta5[3]),
            -sin(theta5[3]),cos(theta5[3]),0,0.1215*(1-cos(theta6[3])),
            0,0,1,0,
            0,0,0,1;
    M3 =e_i1*M1*e_i6*e_i5;

    r6 = M3(0,3)+0.784*sin(sum_theta[1]);
    r7 = M3(2,3)+0.784*cos(sum_theta[1]);
    s1 = (r6*r6+ r7*r7-0.408*0.408-0.376*0.376)/(2*0.408*0.376);
    s1 =nearZero(s1-1)+1;
    alpha =atan2(r6,r7);
    s2 = (r6*r6+ r7*r7+0.408*0.408-0.376*0.376)/(2*0.408*sqrt(r6*r6+ r7*r7));
    s2 =nearZero(s2-1)+1;
    theta3[6] = acos(s1);
    theta3[7] = -acos(s1);
    theta2[6] = acos(s2) + alpha;
    theta2[7] = -acos(s2) + alpha;
    theta4[6] = sum_theta[1]-theta2[6]+theta3[6];
    theta4[7] = sum_theta[1]-theta2[7]+theta3[7];

    clock_t end(clock());
    std::cout<<(double)(end-start)/CLOCKS_PER_SEC<<std::endl;
}