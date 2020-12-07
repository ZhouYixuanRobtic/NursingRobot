//
// Created by xcy on 2020/12/2.
//

#include <iostream>
#include "RobotModel.h"
RobotModel::RobotModel(const std::string &yaml_name)
{
    loadModel(yaml_name);
}
RobotModel::RobotModel(const std::vector<LieGroup::SE3> &all_screw_axes,const LieGroup::SE3 &home_configuration,bool isInBodyFrame)
{
    all_screw_axes_ = all_screw_axes;
    home_configuration_ = home_configuration;
    IN_BODY_ = isInBodyFrame;
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
}
inline Eigen::Affine3d RobotModel::fkInSpace(const Eigen::VectorXd &joint_angles)
{
    LieGroup::SE_3 ee_pose{LieGroup::SE_3::Identity()};
    for (int i = 0; i < joint_angles.size(); ++i)
    {
        ee_pose = ee_pose* all_screw_axes_[i](joint_angles[i]).SE3Matrix();
    }
    ee_pose = ee_pose * home_configuration_.SE3Matrix();
    return Eigen::Affine3d{ee_pose};
}
inline Eigen::Affine3d RobotModel::fkInBody(const Eigen::VectorXd &joint_angles)
{
    LieGroup::SE_3 ee_pose{home_configuration_.SE3Matrix()};
    for (int i = 0; i < joint_angles.size(); ++i)
    {
        ee_pose = ee_pose* all_screw_axes_[i](joint_angles[i]).SE3Matrix();
    }
    return Eigen::Affine3d{ee_pose};
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
    LieGroup::SE_3 Tfk = fkInSpace(joint_angles).matrix();
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
        Tfk = fkInSpace(joint_angles).matrix();
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
inline Eigen::MatrixXd RobotModel::allIkSolutions(const Eigen::Affine3d &desired_pose)
{

    return Eigen::Matrix4d::Identity();
}