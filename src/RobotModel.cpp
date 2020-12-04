//
// Created by xcy on 2020/12/2.
//

#include "RobotModel.h"
RobotModel::RobotModel(const std::string &urdf_name,bool isInBodyFrame): IN_BODY_(isInBodyFrame)
{

}
RobotModel::RobotModel(const std::vector<LieGroup::SE3> &all_screw_axes,const LieGroup::SE_3 &home_configuration, bool isInBodyFrame) :IN_BODY_(isInBodyFrame)
{
    all_screw_axes_ = all_screw_axes;
    home_configuration_ = home_configuration;
}
inline Eigen::Affine3d RobotModel::fkInSpace(const Eigen::VectorXd &joint_angles)
{
    LieGroup::SE_3 ee_pose{LieGroup::SE_3::Identity()};
    for (int i = 0; i < joint_angles.size(); ++i)
    {
        ee_pose = ee_pose* all_screw_axes_[i](joint_angles[i]).SE3Matrix();
    }
    ee_pose = ee_pose * home_configuration_;
    return Eigen::Affine3d{ee_pose};
}
inline Eigen::Affine3d RobotModel::fkInBody(const Eigen::VectorXd &joint_angles)
{
    LieGroup::SE_3 ee_pose{home_configuration_};
    for (int i = 0; i < joint_angles.size(); ++i)
    {
        ee_pose = ee_pose* all_screw_axes_[i](joint_angles[i]).SE3Matrix();
    }
    return Eigen::Affine3d{ee_pose};
}
inline Eigen::MatrixXd RobotModel::jacobianSpace(const Eigen::VectorXd & joint_angles)
{
    Eigen::MatrixXd jacobian_matrix;
    jacobian_matrix << all_screw_axes_[0].Axis();
    LieGroup::SE_3  tmp_matrix = LieGroup::SE_3::Identity();
    for (int i = 1; i < joint_angles.size(); i++)
    {
        tmp_matrix *= all_screw_axes_[i-1](joint_angles[-1]).SE3Matrix();
        jacobian_matrix << LieGroup::getAdjoint(tmp_matrix) * all_screw_axes_[i].Axis();
    }
    return jacobian_matrix;
}
inline Eigen::MatrixXd RobotModel::jacobianBody(const Eigen::VectorXd & joint_angles)
{
    Eigen::MatrixXd jacobian_matrix;
    LieGroup::SE_3  tmp_matrix = LieGroup::SE_3::Identity();
    jacobian_matrix.col(joint_angles.size()-1) = all_screw_axes_[joint_angles.size()-1].Axis();
    for (int i = joint_angles.size() - 2; i >= 0; i--)
    {
        tmp_matrix *= all_screw_axes_[i+1](joint_angles[i+1]).inverse().SE3Matrix();
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
    int max_iterations = 20;
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
    int max_iterations = 20;
    LieGroup::SE_3 Tfk = fkInSpace(joint_angles).matrix();
    LieGroup::SE_3 Tdiff = Tfk.inverse() * desired_pose.matrix();
    LieGroup::R6 Vb = LieGroup::SE3(Tdiff).Vector();
    bool err = (Vb.block<3,1>(0,0).norm() > eomg || Vb.block<3,1>(3,0).norm() > ev);
    Eigen::MatrixXd Js;
    while (err && i < max_iterations)
    {
        Js = jacobianSpace( joint_angles);
        joint_angles += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
        ++i;
        // iterate
        Tfk = fkInSpace(joint_angles).matrix();
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
    if(IN_BODY_  && nIkInBody(desired_pose,solution) || (!IN_BODY_ && nIkInSpace(desired_pose,solution)))
        return solution;
    else
        return current_joint_angles_;
}
inline Eigen::MatrixXd RobotModel::allIkSolutions(const Eigen::Affine3d &desired_pose)
{

    return Eigen::Matrix4d::Identity();
}