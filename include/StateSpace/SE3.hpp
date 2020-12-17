#ifndef NURSINGROBOT_SE3_HPP
#define NURSINGROBOT_SE3_HPP

#include "StateSpace.hpp"
#include "SO3.hpp"
namespace state_space {

    /**
     * @brief transfers a se3 matrix to a twist vector
     * @param a_se3
     * @return a twist
     */
    static R6 se3ToVec(const se_3 &a_se3) {
        R6 twist;
        twist << so3ToVec(a_se3.block<3, 3>(0, 0)), a_se3.block<3, 1>(0, 3);
        return twist;
    }

    /**
     * @brief transfers a twist to se3 matirx
     * @param a_twist
     * @return a se3 matrix
     */
    static se_3 VecTose3(const R6 &a_twist) {
        se_3 se_3_;
        se_3_ << VecToso3(a_twist.block<3, 1>(0, 0)), a_twist.block<3, 1>(3, 0),
                0, 0, 0, 0;
        return se_3_;
    }

    /**
     * @brief gets adjoint matrix of SE3 object
     * @param a_SE3
     * @return a 6x6 matrix, which is the adjoint matrix of @a_SE3
     */
    static adjoint_mat GetAdjoint(const SE_3 &a_SE3) {
        adjoint_mat ad_ret{};
        ad_ret << a_SE3.block<3, 3>(0, 0), Eigen::Matrix3d::Zero(),
                VecToso3(a_SE3.block<3, 1>(0, 3)) * a_SE3.block<3, 3>(0, 0), a_SE3.block<3, 3>(0, 0);
        return ad_ret;
    }


    class SE3 : public virtual StateSpace<SE3> {
    protected:
        R6 _twist_3d_{};

        SE_3 SE3_matrix_;

        /**
         *
         * @return se3 matrix of this SE3 object
         */
        se_3 _calculate_se3() const {
            return se_3{VecTose3(Vector())};
        };

        /**
         *
         * @return unit se3 matrix of this SE3 object
         */
        se_3 _calculate_unitse3() const {
            return se_3{VecTose3(Axis())};
        };
    public:

        SE3() {
            _twist_3d_ = R6::Zero();
            SE3_matrix_ = SE_3::Identity();
        };

        SE3(const SE3 &a_SE3) : StateSpace(a_SE3) {
            _twist_3d_ = a_SE3._twist_3d_;
            SE3_matrix_ = a_SE3.SE3_matrix_;
        };

        explicit SE3(const R6 &twist);

        explicit SE3(const Eigen::Matrix<double, 7, 1> &pose_with_quaternion);

        explicit SE3(const Eigen::Affine3d &transformation_matrix);

        explicit SE3(const SE_3 &transformation_matrix);

        explicit SE3(const SO3 &SO3_part, const Eigen::Vector3d &translation_part);

        ~SE3() override = default;

        R6 Vector() const { return _twist_3d_; };

        R6 Axis() const { return getNormalizedTwist(_twist_3d_); };

        double Theta() const {
            /*
             * Only takes pure rotation or translation into account, this is not general.
             * But for robotics, it's sufficient.
             */
            if (!NearZero(_twist_3d_.norm()) && NearZero(_twist_3d_.block<3, 1>(0, 0).norm()))
                return _twist_3d_.block<3, 1>(3, 0).norm();
            else
                return _twist_3d_.block<3, 1>(0, 0).norm();
        };

        SE_3 SE3Matrix() const { return SE3_matrix_; };

        se_3 se3Matrix() const { return _calculate_se3(); };

        se_3 Unitse3Matrix() const { return _calculate_unitse3(); };

        SO3 SO3Part() const {
            R3 omega{_twist_3d_.block<3, 1>(0, 0)};
            return SO3(omega);
        };

        Eigen::Matrix<double, 7, 1> pose_with_quaternion() const {
            Eigen::Matrix<double, 7, 1> result;
            result << SE3_matrix_.block<3, 1>(0, 3), SO3Part().Quaternion();
            return result;
        };

        adjoint_mat Adjoint();

        SE3 operator+(const SE3 &input) const override {
            SE_3 result{this->SE3Matrix() * input.SE3Matrix()};
            return SE3(result);
        };

        SE3 operator-(const SE3 &input) const override {
            SE_3 tmp_matrix(input.SE3Matrix().inverse() * this->SE3Matrix());
            return SE3(tmp_matrix);
        };

        SE3 operator*(double s) const override {
            R6 temp_twist{s * this->Vector()};
            return SE3(temp_twist);
        };
        friend SE3 operator*(double s, const SE3 & input)
        {
            return input*s;
        };
        SE3 operator()(double theta) const override {
            R6 temp_twist = this->Axis() * theta;
            return SE3(temp_twist);
        };

        SE3 inverse() const override {
            R6 twist = -Axis() * Theta();
            return SE3(twist);
        };

        SE3 random(std::default_random_engine &randomEngine, const Eigen::Matrix2Xd *bounds_ptr) const override {
            //TODO:: SE3 random
            return SE3();
        };

        double distance(const SE3 &to) const override {
            //similar to SO3 distance
            return (to - *this).Vector().norm();
        };

        static SE_3 MatrixExp(const se_3 &se3_);

        static se_3 MatrixLog(const SE_3 &SE3_);

        static R6 getNormalizedTwist(const R6 &twist);

    };
}



#endif //NURSINGROBOT_SE3_HPP
