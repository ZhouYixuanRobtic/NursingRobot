/*
 * SE3 library for modeling open-chain manipulator
 */
#include <iostream>
#include "StateSpace/SE3.hpp"
using namespace state_space;

SE_3 SE3::MatrixExp(const se_3 &se3_)
{
    R3 twist{so3ToVec(se3_.block<3,3>(0,0))};
    SO3 SO3_part(twist);

    SE_3 m_ret{};
    if(NearZero(SO3_part.Theta()))
    {
        m_ret<<SO3_part.SO3Matrix(),se3_.block<3,1>(0,3),
                0,0,0,1;
    }
    else
    {
        Eigen::Matrix3d J = Eigen::Matrix3d::Identity()*SO3_part.Theta() +(1-std::cos(SO3_part.Theta()))
                            *SO3_part.Unitso3Matrix() + (SO3_part.Theta()
                            -std::sin(SO3_part.Theta())) *SO3_part.Unitso3Matrix() *SO3_part.Unitso3Matrix();

        m_ret<<SO3_part.SO3Matrix(),J*se3_.block<3,1>(0,3)/SO3_part.Theta(),
                0,0,0,1;
    }


    return m_ret;

}
se_3 SE3::MatrixLog(const SE_3 &SE3_)
{
    SO_3 tmp_matrix{SE3_.block<3,3>(0,0)};
    SO3 SO3_part(tmp_matrix);

    double theta = SO3_part.Theta();
    se_3 m_ret{};
    if(NearZero(theta))
    {
        m_ret<<Eigen::Matrix3d::Zero(),SE3_.block<3,1>(0, 3),
                0,0,0,0;
    }
    else
    {
        Eigen::Matrix3d J_inverse = Eigen::Matrix3d::Identity() - SO3_part.so3Matrix() / 2.0 +
                                    (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2) *SO3_part.so3Matrix() *SO3_part.so3Matrix() / theta;

        m_ret << SO3_part.so3Matrix(), J_inverse * SE3_.block<3,1>(0, 3),
                0,0,0,0;
    }

    return m_ret;
}

SE3::SE3(const R6 &twist)
{
    _twist_3d_ = twist;
    se_3  se_3_{};
    if(!NearZero(Theta()))
        se_3_ =  VecTose3(Vector());
    else
        se_3_<<Eigen::Matrix3d::Identity(),_twist_3d_.block<3,1>(3, 0),
                0,0,0,0;
    SE3_matrix_ = MatrixExp(se_3_);
}
SE3::SE3(const SE_3 &transformation_matrix)
{
    _twist_3d_ = se3ToVec(MatrixLog(transformation_matrix));
    SE3_matrix_ = transformation_matrix;
}
SE3::SE3(const Eigen::Affine3d &transformation_matrix)
{
    *this = SE3(transformation_matrix.matrix());
}
SE3::SE3(const Eigen::Matrix<double, 7, 1> &pose_with_quaternion)
{
    Eigen::Vector3d translation{pose_with_quaternion.block<3,1>(0,0)};
    Eigen::Quaterniond rotation{pose_with_quaternion.block<4,1>(3,0)};
    Eigen::Affine3d transformation_matrix{};
    transformation_matrix.translation() = translation;
    transformation_matrix.linear() = rotation.toRotationMatrix();
    *this = SE3(transformation_matrix);
}
SE3::SE3(const SO3 &SO3_part, const Eigen::Vector3d &translation_part)
{
    SE_3 transformation_matrix;
    transformation_matrix<<SO3_part.SO3Matrix(),translation_part,
                            0,0,0,1;
    *this = SE3(transformation_matrix);
}

adjoint_mat SE3::Adjoint()
{
    adjoint_mat ad_ret{};
    const SE_3 & T = this->SE3_matrix_;
    ad_ret<< T.block<3,3>(0,0),Eigen::Matrix3d::Zero(),
            VecToso3(T.block<3,1>(0,3))*T.block<3,3>(0,0),T.block<3,3>(0,0);
    return ad_ret;
}
R6 SE3::getNormalizedTwist(const R6 &twist)
{
    double theta = twist.block<3, 1>(0, 0).norm();
    if (!NearZero(twist.norm()) && NearZero(theta))
        return twist.normalized();
    else
        return twist / theta;
}