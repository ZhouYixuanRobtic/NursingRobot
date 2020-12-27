//
// Created by xcy on 2020/12/2.
//

#include <iostream>
#include "Kinematics/Kinematics.h"
using namespace kinematics;

Kinematics::Kinematics(const std::string &yaml_name)
{
    ee_configuration_ = state_space::SE3();
    mount_configuration_ = state_space::SE3();
    analytical_ik_func_ = nullptr;
    _loadModel(yaml_name);
}
Kinematics::Kinematics(const std::vector<state_space::SE3> &all_screw_axes, const state_space::SE3 &home_configuration, bool isInBodyFrame)
{
    all_screw_axes_ = all_screw_axes;
    home_configuration_ = home_configuration;
    IN_BODY_ = isInBodyFrame;
    ee_configuration_ = state_space::SE3();
    mount_configuration_ = state_space::SE3();
    analytical_ik_func_ = nullptr;
}
void Kinematics::_loadModel(const std::string &yaml_name)
{
    YAML::Node doc = YAML::LoadFile(yaml_name);
    try
    {
        std::vector<double> pose_with_quaternion = doc["aubo_i5"]["home_configuration"].as<std::vector<double>>();
        Eigen::Matrix<double,7,1> home_pose;
        memcpy(home_pose.data(),pose_with_quaternion.data(), pose_with_quaternion.size()*sizeof(double));
        home_configuration_ = state_space::SE3(home_pose);
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
                state_space::R6 temp_twist;
                memcpy(temp_twist.data(),it.second.data(), temp_twist.size()*sizeof(double));
                all_screw_axes_.emplace_back(state_space::SE3(temp_twist));
            }
        }
    }
    catch (YAML::InvalidScalar)
    {
       perror("tagParam.yaml is invalid.");
    }
}
state_space::SE_3 Kinematics::_fkInSpace(const state_space::JointSpace &joint_angles)
{
    state_space::SE_3 ee_pose{state_space::SE_3::Identity()};
    for (int i = 0; i < joint_angles.size(); ++i)
    {
        ee_pose = ee_pose* all_screw_axes_[i](joint_angles[i]).SE3Matrix();
    }
    ee_pose = ee_pose * home_configuration_.SE3Matrix();
    return ee_pose;
}
state_space::SE_3 Kinematics::_fkInBody(const state_space::JointSpace &joint_angles)
{
    state_space::SE_3 ee_pose{home_configuration_.SE3Matrix()};
    for (int i = 0; i < joint_angles.size(); ++i)
    {
        ee_pose = ee_pose* all_screw_axes_[i](joint_angles[i]).SE3Matrix();
    }
    return ee_pose;
}
Eigen::MatrixXd Kinematics::jacobianSpace(const state_space::JointSpace & joint_angles)
{
    Eigen::MatrixXd jacobian_matrix(6,6);
    jacobian_matrix.col(0) = all_screw_axes_[0].Axis();
    state_space::SE_3  tmp_matrix = state_space::SE_3::Identity();
    for (int i = 1; i < joint_angles.size(); i++)
    {
        tmp_matrix = tmp_matrix * all_screw_axes_[i-1](joint_angles[i-1]).SE3Matrix();
        jacobian_matrix.col(i) = state_space::GetAdjoint(tmp_matrix) * all_screw_axes_[i].Axis();
    }
    return jacobian_matrix;
}
Eigen::MatrixXd Kinematics::jacobianBody(const state_space::JointSpace & joint_angles)
{
    Eigen::MatrixXd jacobian_matrix(6,6);
    state_space::SE_3  tmp_matrix = state_space::SE_3::Identity();
    jacobian_matrix.col(joint_angles.size()-1) = all_screw_axes_[joint_angles.size()-1].Axis();
    for (size_t i = joint_angles.size() - 2; i >= 0; i--)
    {
        tmp_matrix = tmp_matrix * all_screw_axes_[i+1](-joint_angles[i+1]).SE3Matrix();
        jacobian_matrix.col(i) = state_space::GetAdjoint(tmp_matrix) * all_screw_axes_[i].Axis();
    }
    return jacobian_matrix;
}
bool Kinematics::_nIkInSpace(const state_space::SE_3 &desired_pose, state_space::JointSpace &joint_angles, double eomg, double ev)
{
    int i = 0;
    int max_iterations = 50;
    state_space::SE_3 Tfk = _fkInSpace(joint_angles);
    state_space::SE_3 Tdiff = Tfk.inverse() * desired_pose.matrix();
    state_space::R6 Vs = state_space::GetAdjoint(Tfk) * state_space::SE3(Tdiff).Vector();
    bool err = (Vs.block<3,1>(0,0).norm() > eomg || Vs.block<3,1>(3,0).norm() > ev);
    Eigen::MatrixXd Js;
    while (err && i < max_iterations)
    {
        Js = jacobianSpace( joint_angles);
        joint_angles += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
        ++i;
        // iterate
        Tfk = _fkInSpace(joint_angles);
        Tdiff = Tfk.inverse() * desired_pose.matrix();
        Vs = state_space::GetAdjoint(Tfk) * state_space::SE3(Tdiff).Vector();
        err = (Vs.block<3,1>(0,0).norm() > eomg || Vs.block<3,1>(3,0).norm() > ev);
    }
    for(int j =0; j<joint_angles.size(); ++j)
    {
        joint_angles[j] = _nearZero(joint_angles[j]);
    }
    return !err;
}
 bool Kinematics::_nIkInBody(const state_space::SE_3 &desired_pose, state_space::JointSpace &joint_angles, double eomg, double ev)
{
    int i = 0;
    int max_iterations = 50;
    state_space::SE_3 Tfk = _fkInBody(joint_angles).matrix();
    state_space::SE_3 Tdiff = Tfk.inverse() * desired_pose.matrix();
    state_space::R6 Vb = state_space::SE3(Tdiff).Vector();
    bool err = (Vb.block<3,1>(0,0).norm() > eomg || Vb.block<3,1>(3,0).norm() > ev);
    Eigen::MatrixXd Jb;
    while (err && i < max_iterations)
    {
        Jb = jacobianBody( joint_angles);
        joint_angles += Jb.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
        ++i;
        // iterate
        Tfk = _fkInBody(joint_angles).matrix();
        Tdiff = Tfk.inverse() * desired_pose.matrix();
        Vb = state_space::SE3(Tdiff).Vector();
        err = (Vb.block<3,1>(0,0).norm() > eomg || Vb.block<3,1>(3,0).norm() > ev);
    }
    for(int j =0; j<joint_angles.size(); ++j)
    {
        joint_angles[j] = _nearZero(joint_angles[j]);
    }
    return !err;
}
IK_SINGULAR_CODE Kinematics::analyticalIkSolutions(Eigen::MatrixXd &joint_solutions, const state_space::SE_3 &desired_pose, const state_space::JointSpace* ik_references_ptr)
{
    if(analytical_ik_func_)
    {
        return analytical_ik_func_(joint_solutions, home_configuration_, _get_pose_in_first(desired_pose), ik_references_ptr);
    }
    else
        return NO_SOLUTIONS;
}
 bool Kinematics::allValidIkSolutions(Eigen::MatrixXd &joint_solutions, const state_space::SE_3 &desired_pose, const state_space::JointSpace* reference)
{

    IK_SINGULAR_CODE ret_code = analyticalIkSolutions(joint_solutions, desired_pose, nullptr);
    if(ret_code == NO_SOLUTIONS)
    {
        if(reference)
        {
            state_space::JointSpace solution{*reference};
            if(nIk(desired_pose,solution))
            {
                joint_solutions= solution.Vector();
            }
        }
    }
    else if(ret_code == WRIST_SINGULAR)
    {
        /*if have reference, use numerical method to get right singular joint angle*/
        if(reference)
        {
            state_space::JointSpace solution{*reference};
            if(nIk(desired_pose,solution))
            {
                ret_code = analyticalIkSolutions(joint_solutions, desired_pose, &solution);
                if(ret_code==NO_SOLUTIONS)
                {
                    joint_solutions=solution.Vector();
                }
            }
        }
        /*else keep 0 as reference value*/
    }
    return joint_solutions.cols()!=0;

}
state_space::JointSpace Kinematics::nearestIkSolution(const Eigen::Affine3d &desired_pose, const state_space::JointSpace& reference, bool isConsecutive)
{
    //return the nearest ik solution
    state_space::JointSpace solution{reference};
    Eigen::MatrixXd joint_solutions;
    const state_space::JointSpace* reference_ptr = isConsecutive? &solution : nullptr;
    if(allValidIkSolutions(joint_solutions, desired_pose.matrix(), reference_ptr))
    {
        std::vector<double> norm_box;
        for (int i = 0; i < joint_solutions.cols(); ++i)
        {
            norm_box.emplace_back(reference.distance(joint_solutions.col(i)));
        }
        return state_space::JointSpace(joint_solutions.col(std::distance(norm_box.begin(), std::min_element(norm_box.begin(), norm_box.end()))));
    }
    return solution;
}
state_space::JointSpace Kinematics::directedNearestIkSolution(const Eigen::Affine3d &desired_pose, const state_space::JointSpace &reference, const state_space::JointSpace &tangent_reference)
{
    state_space::JointSpace solution{reference};
    Eigen::MatrixXd joint_solutions;
    IK_SINGULAR_CODE ret_code = analyticalIkSolutions(joint_solutions, desired_pose.matrix(), nullptr);
    if(joint_solutions.cols() == 0 || ret_code == WRIST_SINGULAR)
    {
        if(nIk(desired_pose,solution,1e-8,1e-8))
            return solution;
        else
            return reference;
    }
    else
    {
        std::vector<double> norm_box;
        for (int i = 0; i < joint_solutions.cols(); ++i)
        {
            state_space::JointSpace joint_tangent{joint_solutions.col(i) - reference.Vector()};

            norm_box.emplace_back(joint_tangent.distance(tangent_reference));
        }
        return state_space::JointSpace(joint_solutions.col(std::distance(norm_box.begin(), std::min_element(norm_box.begin(), norm_box.end()))));
    }
}