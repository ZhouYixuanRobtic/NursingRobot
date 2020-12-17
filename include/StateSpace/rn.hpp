//
// Created by xcy on 2020/12/16.
//

#ifndef NURSINGROBOT_RN_HPP
#define NURSINGROBOT_RN_HPP

#include "StateSpace/StateSpace.hpp"
namespace state_space{
    class Rn : public virtual StateSpace<Rn>{
    protected:
        int _dimensions_{};
        Eigen::VectorXd _data_;
    public:
        explicit Rn(int dimensions = 3)
        {
            this->_data_.resize(dimensions);
            this->_data_.setZero();
        };
        explicit Rn(const Eigen::VectorXd &data)
        {
            this->_data_ = data;
            this->_dimensions_ = data.size();
        };
        explicit Rn(const double * data_ptr, int dimensions)
        {
            this->_data_.resize(dimensions);
            memcpy(this->_data_.data(),data_ptr,dimensions * sizeof(double));
            this->_dimensions_ = dimensions;
        };
        ~Rn() override = default;

        Eigen::VectorXd Vector() const{return _data_;};

        Rn operator+(const Rn& input)const override
        {
            return Rn(this->Vector()+input.Vector());
        };
        friend Rn operator+(double a, const Rn&input)
        {
            return  Rn(input);
        }
        Rn operator-(const Rn& input)const override
        {
            return Rn(this->Vector()-input.Vector());
        };
        Rn operator*(double s)const override
        {
            return Rn(this->Vector()*s);
        };
        friend Rn operator*(double s, const Rn & input)
        {
            return input*s;
        };
        Rn operator()(double theta)const override
        {
            return Rn(this->Vector().normalized()*theta);
        };
        double& operator[](const long index)
        {
            return this->_data_[index];
        };
        double operator[](const long index)const
        {
            return this->_data_[index];
        };
        void* data()
        {
            return this->_data_.data();
        };
        Rn inverse()const override
        {
            return Rn(-1*this->Vector());
        };
        Rn random(std::default_random_engine & randomEngine, const Eigen::Matrix2Xd* bounds_ptr)const override
        {
            Eigen::VectorXd result(_dimensions_);
            if(bounds_ptr != nullptr)
            {
                std::uniform_real_distribution<double> x_distribution(0, 1);
                for(int i=0; i<this->_dimensions_; ++i)
                {
                    result[i] =  bounds_ptr->col(i)[1] + x_distribution(randomEngine) * (bounds_ptr->col(i)[0]-bounds_ptr->col(i)[1]);
                }
            }
            else
            {
                result.setRandom();
            }
            return Rn(result);
        };
        double norm() const
        {
            return (this->_data_.norm());
        };
        double distance(const Rn & to)const override
        {
            return (*this - to).norm();
        };
        bool isValid(const Eigen::Matrix2Xd* bounds_ptr) const
        {
            return !( ((this->_data_-bounds_ptr->col(0)).array()>0).any()||
                      ((this->_data_-bounds_ptr->col(1)).array()<0).any() );
        };
        bool isValid(const Eigen::VectorXd & upper_bound,const Eigen::VectorXd & lower_bound) const
        {
            return !( ((this->_data_-upper_bound).array()>0).any()||
                      ((this->_data_-lower_bound).array()<0).any() );
        };

    };
}





#endif //NURSINGROBOT_RN_HPP