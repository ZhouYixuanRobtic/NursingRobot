#include "StateSpace/SO3.hpp"

using namespace state_space;

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
    double acos_input = (SO3_.trace()-1)/2.0;
    Eigen::Matrix3d m_ret = Eigen::Matrix3d::Zero();
    if(acos_input >=1)
        return m_ret;
    else if(acos_input <=-1)
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
        //Warning:
        // here exist a multiple value problem,temporarily fixed by using angle axis;
        //double theta = std::acos(acosinput);
        m_ret = theta*(SO3_-SO3_.transpose())/(2.0*sin(theta));
    }
    return m_ret;
}

SO3::SO3(const R3 &twist)
{
    _twist_2d_ = twist;
    so_3 so_3_ = NearZero(Theta()) ? Eigen::Matrix3d::Identity() : VecToso3(Vector());
    SO3_MATRIX_ = MatrixExp(so_3_);
}
SO3::SO3(const SO_3 &rotation_matrix)
{
    //R3 twist = so3ToVec(MatrixLog(rotation_matrix));
    Eigen::AngleAxisd rotation_vector(rotation_matrix);
    _twist_2d_ = rotation_vector.axis()*rotation_vector.angle();
    SO3_MATRIX_ = rotation_matrix;
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
