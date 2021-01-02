//
// Created by xcy on 2020/12/15.
//

#ifndef NURSINGROBOT_JOINTSPACE_HPP
#define NURSINGROBOT_JOINTSPACE_HPP

#include <iostream>
#include "StateSpace/StateSpace.hpp"

namespace state_space {

    ALIGNED_CLASS_STL_FORWARD(JointSpace)

    class JointSpace : public virtual StateSpace<JointSpace> {
    protected:
        unsigned int _dimensions_;
        Eigen::VectorXd _data_;

        void _restrict()
        {
            for (int i = 0; i < _data_.size(); ++i) {
                auto& val = _data_[i];
                if (val > _upper_limit_)
                    val -= 2 * _upper_limit_;
                else if (val < _lower_limit_)
                    val += 2 * _lower_limit_;
            }
        }

        enum {
            NeedsToAlign = (sizeof(_data_) % 16) == 0
        };
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)

        JointSpace(const JointSpace& other)
                : StateSpace(other),
                  _dimensions_(other._dimensions_),
                  _data_(other._data_)
        {}

        explicit JointSpace(unsigned int dimensions = 6)
                : _dimensions_(dimensions)
        {
            this->_data_.resize(dimensions);
            this->_data_.setZero();
        };

        explicit JointSpace(const Eigen::VectorXd& data)
                : _dimensions_(data.size())
        {
            this->_data_ = data;
            _restrict();
        };

        explicit JointSpace(const double* data_ptr, unsigned int dimensions)
                : _dimensions_(dimensions)
        {
            this->_data_.resize(dimensions);
            memcpy(this->_data_.data(), data_ptr, dimensions * sizeof(double));
            _restrict();
        };

        ~JointSpace() override = default;

        const Eigen::VectorXd& Vector() const
        { return _data_; };

        Eigen::VectorXd& Vector()
        { return _data_; };

        JointSpace& operator=(const JointSpace& other)
        {
            this->_dimensions_ = other._dimensions_;
            this->_data_ = other._data_;
            return *this;
        };

        JointSpace operator+(const JointSpace& input) const override
        {
            return JointSpace(this->Vector() + input.Vector());
        };

        JointSpace operator-(const JointSpace& input) const override
        {
            return JointSpace(this->Vector() - input.Vector());
        };

        JointSpace operator*(double s) const override
        {
            return JointSpace(s * this->Vector());
        };

        JointSpace operator()(double theta) const override
        {
            return JointSpace(this->Vector().normalized() * theta);
        };

        double& operator[](const size_t index)
        {
            return this->_data_[index];
        };

        double operator[](const long index) const
        {
            return this->_data_[index];
        };

        JointSpace& operator+=(const Eigen::VectorXd& input)
        {
            this->_data_ += input;
            return *this;
        };

        bool operator==(const JointSpace& other) const override
        {
            return _data_ == other._data_;
        }

        const double* data() const override
        {
            return this->_data_.data();
        };

        JointSpace inverse() const override
        {
            return JointSpace(-1 * this->Vector());
        };

        JointSpace random(std::default_random_engine& randomEngine, const Eigen::MatrixX2d* bounds_ptr) const override
        {
            Eigen::VectorXd result(_dimensions_);
            if (bounds_ptr != nullptr) {
                std::uniform_real_distribution<double> x_distribution(0, 1);
                for (int i = 0; i < this->_dimensions_; ++i) {
                    result[i] = bounds_ptr->col(1)[i] +
                                x_distribution(randomEngine) * (bounds_ptr->col(0)[i] - bounds_ptr->col(1)[i]);
                }
            } else {
                std::uniform_real_distribution<double> x_distribution(-M_PI, M_PI);
                for (int i = 0; i < this->_dimensions_; ++i) {
                    result[i] = x_distribution(randomEngine);
                }
            }
            return JointSpace(result);
        };

        double distance(const JointSpace& to) const override
        {
            return (*this - to).norm();
        };

        double distance(const Eigen::VectorXd& input) const
        {
            return this->distance(JointSpace(input));
        };

        double norm() const override
        {
            return this->_data_.norm();
        };

        unsigned int Dimensions() const override
        {
            return _dimensions_;
        };

        size_t size() const
        {
            return this->_data_.size();
        };

        bool isValid(const Eigen::MatrixX2d* bounds_ptr) const
        {
            JointSpace upper_bound_{bounds_ptr->col(0)}, lower_bound_{bounds_ptr->col(1)};
            return !(((*this - upper_bound_).Vector().array() > 0).any() ||
                     ((*this - lower_bound_).Vector().array() < 0).any());

        };

        bool isValid(const Eigen::VectorXd& upper_bound, const Eigen::VectorXd& lower_bound) const
        {
            return !(((_data_ - upper_bound).array() > 0).any() ||
                     ((_data_ - lower_bound).array() < 0).any());
        };


    private:
        const double _lower_limit_{-M_PI};
        const double _upper_limit_{M_PI};
    };

}


#endif //NURSINGROBOT_JOINTSPACE_HPP
