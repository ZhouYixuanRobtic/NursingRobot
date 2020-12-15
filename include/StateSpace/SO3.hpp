//
// Created by xcy on 2020/12/15.
//

#ifndef NURSINGROBOT_SO3_H
#define NURSINGROBOT_SO3_H

#include "StateSpace.hpp"

namespace state_space{

    /**
     * @brief transfers a R3 vector to a so3 matirx
     * @param omega a R3 vector representing w
     * @return a so3 matrix
     */
    static so_3 VecToso3(const Eigen::Vector3d & omega)
    {
        so_3 w_x;
        w_x << 0,-omega[2],omega[1],
                omega[2],0,-omega[0],
                -omega[1],omega[0],0;
        return w_x;
    }

    /**
     * @brief transfers a so3 matrix to a R3 vector
     * @param a_so3 a so3 matrix
     * @return a R3 vector
     */
    static R3 so3ToVec(const so_3 & a_so3)
    {
        return R3{-a_so3(1, 2), a_so3(0, 2), -a_so3(0, 1)};
    }

    class SO3 : public StateSpace<SO3>{
    private:
        R3 _twist_2d_{};
        SO_3 SO3_MATRIX_{};

        /**
         *
         * @return so3 matrix of this SO3 object
         */
        so_3 _calculate_so3() const {return so_3{VecToso3(Vector())};};
        so_3 _calculate_unit_so3() const {return so_3{VecToso3(Axis())};};
    public:

        SO3()
        {
            _twist_2d_ = R3 ::Zero();
            SO3_MATRIX_=SO_3 ::Identity();
        };
        SO3( SO3 const &SO3_): StateSpace(SO3_) {
            _twist_2d_ = SO3_._twist_2d_;
            SO3_MATRIX_ = SO3_.SO3_MATRIX_;
        };
        explicit SO3(const R3 & twist);
        explicit SO3(const Eigen::Matrix<double,4,1> & quaternion_);
        explicit SO3(const SO_3 & rotation_matrix);
        explicit SO3(double roll, double pitch, double yaw);
        ~SO3() override = default;

        R3 Vector() const {return _twist_2d_;};
        R3 Axis() const {return _twist_2d_.normalized();};
        double Theta() const{return _twist_2d_.norm();};
        SO_3 SO3Matrix() const{return SO3_MATRIX_;};
        so_3 Unitso3Matrix() const {return _calculate_unit_so3();};
        so_3 so3Matrix() const {return _calculate_so3();};
        Eigen::Vector4d Quaternion() const {Eigen::Quaterniond rotation{SO3_MATRIX_}; return rotation.coeffs();};

        SO3 operator+(const SO3 & input) const override
        {
            SO_3 result{this->SO3Matrix() * input.SO3Matrix()};
            return SO3(result);
        };
        SO3 operator-(const SO3 & input) const override
        {
            SO_3 tmp_matrix(input.SO3Matrix().inverse() * this->SO3Matrix());
            return SO3(tmp_matrix);
        };
        SO3 operator*(double s) const override
        {
            R3 temp_twist{s * this->Vector()};
            return SO3(temp_twist);
        };
        SO3 operator()(double theta) const override
        {
            R3 temp_twist = this->Axis()*theta;
            return SO3(temp_twist);
        };
        SO3 inverse() const override
        {
            R3 twist = -Axis() * Theta();
            return SO3(twist);
        };
        SO3 random(std::default_random_engine & randomEngine,const Eigen::MatrixXd * bounds_ptr) const override
        {
            //Effective Sampling and Distance Metrics for 3D Rigid Body Path Planning [c]
            //James J. Kuffner
            //use the uniformly sampling unit Quaternion
            std::uniform_real_distribution<double> x_distribution(0,1);
            double s = x_distribution(randomEngine);
            double sigma1 = std::sqrt(1-s);
            double sigma2 = std::sqrt(s);
            double theta1 = 2*M_PI*x_distribution(randomEngine);
            double theta2 = 2*M_PI*x_distribution(randomEngine);
            Eigen::Vector4d random_quaternion{sin(theta1)*sigma1,cos(theta1)*sigma1,sin(theta2)*sigma2,cos(theta2)*sigma2};
            return SO3(random_quaternion);
        };
        double distance(const SO3 & to) const override
        {
            //Smooth invariant interpolation of rotations
            //F.park & B.Ravani
            //pr = ||log(Q1^{-1}*Q2)||, which is bi-invariant
            return (to - *this).Vector().norm();
        };

        static SO_3 MatrixExp(const so_3 & so3_) ;
        static so_3 MatrixLog(const SO_3 & SO3_) ;


    };

}



#endif //NURSINGROBOT_SO3_H
