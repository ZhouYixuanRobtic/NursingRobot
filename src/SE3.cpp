/*
 * SE3 library for modeling open-chain manipulator
 */
#include <iostream>
#include "SE3.h"
using namespace LieGroup;

SO_3 SO3::MatrixExp(const so_3 &so3_)
{
    R3 twist{so3ToVec(so3_)};
    double theta = twist.norm();
    if(NearZero(theta))
    {
        return Eigen::Matrix3d::Identity();
    }
    else
    {
        SO_3 unit_so3 = VecToso3(twist.normalized());
        return Eigen::Matrix3d::Identity() + std::sin(theta)*unit_so3 + (1-std::cos(theta))*unit_so3*unit_so3;
    }
}
so_3 SO3::MatrixLog(const SO_3 &SO3_)
{
    double acosinput = (SO3_.trace()-1)/2.0;
    Eigen::Matrix3d m_ret = Eigen::Matrix3d::Zero();
    if(acosinput >=1)
        return m_ret;
    else if(acosinput <=-1)
    {
        Eigen::Vector3d omega;
        if(!NearZero(1+SO3_(2,2)))
            omega = (1.0/std::sqrt(2*(1+SO3_(2,2))))*Eigen::Vector3d(SO3_(0,2),SO3_(1,2),1+SO3_(2,2));
        else if (!NearZero(1+SO3_(1,1)))
            omega = (1.0/std::sqrt(2*(1+SO3_(1,1))))*Eigen::Vector3d(SO3_(0,1),1+SO3_(1,1),SO3_(2,1));
        else
            omega = (1.0/std::sqrt(2*(1+SO3_(0,0))))*Eigen::Vector3d(1+SO3_(0,0),SO3_(1,0),SO3_(2,0));

        m_ret = VecToso3(omega*M_PI);
    }
    else
    {
        Eigen::AngleAxisd rotation_vector(SO3_);
        double theta = rotation_vector.angle();
        //TODO:
        // here exist a multiple value problem,temporarily fixed by using angle axis;
        //double theta = std::acos(acosinput);
        m_ret = theta*(SO3_-SO3_.transpose())/(2.0*sin(theta));
    }
    return m_ret;
}
SO_3 SO3::calSO3() const
{
    so_3 so_3_ = NearZero(Theta()) ? Eigen::Matrix3d::Identity() : VecToso3(Vector());
    return SO_3{MatrixExp(so_3_)};
}
SO3::SO3( SO3 const &SO3_)
{
    twist_2d_ = SO3_.twist_2d_;
}
SO3::SO3(const R3 &twist)
{
    twist_2d_ = twist;
}
SO3::SO3(const SO_3 &rotation_matrix)
{
    //R3 twist = so3ToVec(MatrixLog(rotation_matrix));
    Eigen::AngleAxisd rotation_vector(rotation_matrix);
    R3 twist = rotation_vector.axis()*rotation_vector.angle();
    *this = SO3(twist);
}
SO3::SO3(double roll, double pitch, double yaw)
{
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d rotation_matrix{rollAngle*pitchAngle*yawAngle};
    *this = SO3(rotation_matrix);
}
SO3::SO3(const Eigen::Matrix<double, 4, 1> &quaternion_)
{
    Eigen::Quaterniond quaternion(quaternion_);
    *this = SO3(quaternion.toRotationMatrix());
}
SO3 SO3::operator+(const SO3 & input) const
{
    SO_3 tmp_matrix{this->calSO3()*input.calSO3()};
    return SO3(tmp_matrix);
}
SO3 SO3::operator-(const SO3 &input) const
{
    SO_3 tmp_matrix(input.calSO3().inverse()* this->calSO3());
    return SO3(tmp_matrix);
}
SO3 SO3::operator*(double s) const
{
    SO_3 tmp_matrix(this->calSO3().pow(s));
    return SO3(tmp_matrix);
}
SO3 SO3::interp(double lambda, const SO3 & destination) const
{
    return *this+((destination-*this)*lambda);
}


SE_3 SE3::calSE3() const
{
    se_3  se_3_{};
    if(!NearZero(Theta()))
        se_3_ =  VecTose3(Vector());
    else
        se_3_<<Eigen::Matrix3d::Identity(),twist_3d_.block<3,1>(3,0),
                0,0,0,0;

    return SE_3{MatrixExp(se_3_)};
}

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
        Eigen::Matrix3d J = Eigen::Matrix3d::Identity()*SO3_part.Theta() +(1-std::cos(SO3_part.Theta()))*
                                                                          SO3_part.unitso3Matrix() + (SO3_part.Theta() - std::sin(SO3_part.Theta())) *
                                                                                                     SO3_part.unitso3Matrix() *
                                                                                                     SO3_part.unitso3Matrix();

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

SE3::SE3(const SE3 &SE3_)
{
    twist_3d_ = SE3_.twist_3d_;
}
SE3::SE3(const R6 &twist)
{
    twist_3d_ = twist;
}
SE3::SE3(const SE_3 &transformation_matrix)
{
    R6 twist(se3ToVec(MatrixLog(transformation_matrix)));
    *this = SE3(twist);
}
SE3::SE3(const Eigen::Affine3d &transformation_matrix)
{
    *this = SE3(transformation_matrix.matrix());
}
SE3::SE3(const Eigen::Matrix<double, 7, 1> &pose_with_quaternion)
{
    Eigen::Vector3d translation{pose_with_quaternion.block<3,1>(0,0)};
    Eigen::Quaterniond rotation{pose_with_quaternion.block<4,1>(0,3)};
    Eigen::Affine3d transformation_matrix{};
    transformation_matrix.translation() = translation;
    transformation_matrix.linear() = rotation.toRotationMatrix();
    *this = SE3(transformation_matrix);
}

SE3 SE3::operator+(const SE3 & input)const
{
    SE_3 tmp_matrix{this->calSE3()*input.calSE3()};
    return SE3(tmp_matrix);
}

SE3 SE3::operator-(const SE3 &input)const
{
    SE_3 tmp_matrix(input.calSE3().inverse()* this->calSE3());
    return SE3(tmp_matrix);
}

SE3 SE3::operator*(double s)const
{
    SE_3 tmp_matrix(this->calSE3().pow(s));
    return SE3(tmp_matrix);
}

adjoint_mat SE3::Adjoint()
{
    adjoint_mat ad_ret{};
    SE_3 T{calSE3()};
    ad_ret<< T.block<3,3>(0,0),Eigen::Matrix3d::Zero(),
            VecToso3(T.block<3,1>(0,3))*T.block<3,3>(0,0),T.block<3,3>(0,0);
    return ad_ret;
}
R6 SE3::getNormalizedTwist(const R6 &twist)
{
    double theta = twist.block<3,1>(0,0).norm();
    if(!NearZero(twist.norm())&&NearZero(theta))
        return twist.normalized();
    else
        return twist/theta;
}
SE3 SE3::interp(double lambda, const SE3 &destination) const
{
    return *this+((destination-*this)*lambda);
}