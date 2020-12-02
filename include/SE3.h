#ifndef NURSINGROBOT_SE3_H
#define NURSINGROBOT_SE3_H

#include <Eigen/Dense>
#include <vector>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
namespace LieGroup{

    //exponential coordinate of se(3) s = [w,v]
    typedef Eigen::Matrix<double,6,1> R6;

    //lie algebra se(3) of the lie group SE(3)
    typedef Eigen::Matrix4d se_3;

    //lie group SE(3)
    typedef Eigen::Matrix4d SE_3;

    //exponential coordinate of so(3)
    typedef Eigen::Vector3d R3;

    //lie algebra so(3) of the lie group SO(3)
    typedef Eigen::Matrix3d so_3;

    //lie group SO(3)
    typedef Eigen::Matrix3d SO_3;

    //adjoint matrix 6X6 for Lie bracket
    typedef Eigen::Matrix<double,6,6> adjoint_mat;

    static bool NearZero(const double val)
    {
        return (std::abs(val)<0.000001);
    }

    static so_3 VecToso3(const Eigen::Vector3d & omega)
    {
        so_3 w_x;
        w_x << 0,-omega[2],omega[1],
                omega[2],0,-omega[0],
                -omega[1],omega[0],0;
        return w_x;
    }
    static se_3 VecTose3(const R6 & twist)
    {

        so_3 w_x{VecToso3(twist.block<3,1>(0,0))};
        se_3 se_3_;
        se_3_<<w_x,twist.block<3,1>(3,0),
                0,0,0,0;
        return se_3_;
    }
    static R3 so3ToVec(const so_3 & so3_)
    {
        return R3{-so3_(1,2),so3_(0,2),-so3_(0,1)};
    }
    static R6 se3ToVec(const se_3 & se3_)
    {
        R3 omega{so3ToVec(se3_.block<3,3>(0,0))};
        R6 twist;
        twist<<omega,se3_.block<3,1>(0,3);
        return twist;
    }
    class SO3{
    private:
        R3 twist_2d_{};

        SO_3 calSO3() const;
        so_3 calso3() const {return so_3{VecToso3(Vector())};};
        so_3 calUnitso3() const {return so_3{VecToso3(Axis())};};
    public:
        SO3( SO3 const &SO3_);
        explicit SO3(const R3 & twist);
        explicit SO3(const Eigen::Matrix<double,4,1> & quaternion_);
        explicit SO3(const SO_3 & rotation_matrix);
        ~SO3() = default;

        SO3 operator+(const SO3& input)const;
        SO3 operator-(const SO3& input) const;
        SO3 operator*(double s) const;
        SO3 operator()(double theta){twist_2d_ = Axis()*theta;return *this;};

        R3 Vector() const {return twist_2d_;};
        R3 Axis() const {return twist_2d_.normalized();};
        double Theta() const{return twist_2d_.norm();};
        SO_3 SO3Matrix() const{return calSO3();};
        so_3 unitso3Matrix() const {return calUnitso3();};
        so_3 so3Matrix() const {return calso3();};
        static SO_3 MatrixExp(const so_3 & so3_) ;
        static so_3 MatrixLog(const SO_3 & SO3_) ;

        SO3 inverse();
        SO3 interp(double lambda, const SO3 & destination) const ;
    };

    class SE3 {
    private:
        R6 twist_3d_{};

        SE_3 calSE3() const;
        se_3 calse3(){return se_3{VecTose3(Vector())};};
        se_3 calUnitse3(){return se_3{VecTose3(Axis())};};
    public:
        SE3(const SE3 & SE3_);
        explicit SE3(const R6 & twist);
        explicit SE3(const Eigen::Matrix<double,7,1> & pose_with_quaternion);
        explicit SE3(const Eigen::Affine3d & transformation_matrix);
        explicit SE3(const SE_3 & transformation_matrix);

        ~SE3() = default;

        SE3 operator+(const SE3& input) const;
        SE3 operator-(const SE3& input) const;
        SE3 operator*(double s) const;
        SE3 operator()(double theta){twist_3d_ = Axis()*theta;return *this;}
        R6 Vector() const{return twist_3d_;};
        R6 Axis(){return getNormalizedTwist(twist_3d_);};
        double Theta() const
        {
            /*
             * Only takes pure rotation and translation into account, this is not general.
             * But for robotics, it's sufficient.
             */
            if(!NearZero(twist_3d_.norm()) && NearZero(twist_3d_.block<3,1>(0,0).norm()))
                return twist_3d_.block<3,1>(3,0).norm();
            else
                return twist_3d_.block<3,1>(0,0).norm();
        };
        SE_3 SE3Matrix(){return calSE3();};
        se_3 se3Matrix(){return calse3();};
        se_3 unitse3Matrix(){return calUnitse3();};
        static SE_3 MatrixExp(const se_3 & se3_);
        static se_3 MatrixLog(const SE_3 & SE3_);

        adjoint_mat Adjoint();
        static R6 getNormalizedTwist(const R6 & twist);
        SE3 inverse();
        SE3 interp(double lambda, const SE3 & destination) const;

    };


}



#endif //NURSINGROBOT_SE3_H
