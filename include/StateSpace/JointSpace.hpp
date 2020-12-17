//
// Created by xcy on 2020/12/15.
//

#ifndef NURSINGROBOT_JOINTSPACE_HPP
#define NURSINGROBOT_JOINTSPACE_HPP

#include "StateSpace/StateSpace.hpp"
namespace state_space{

    class JointSpace: public virtual StateSpace<JointSpace> {
    protected:
        int _dimensions_{};
        Eigen::VectorXd _data_;
        void _restrict()
        {
            for(int i=0; i < _data_.size(); ++i)
            {
                auto & val = _data_[i];
                if(val >= _upper_limit_)
                    val -= 2 * _upper_limit_;
                else if(val <= _lower_limit_)
                    val+= 2 * _lower_limit_;
            }
        }

    public:
        JointSpace(const JointSpace& other)
         : StateSpace(other) {
            this->_dimensions_ = other.Vector().size();
            this->_data_ = other.Vector();
        }
        explicit JointSpace(int dimensions =6)
        {
            this->_data_.resize(dimensions);
            this->_data_.setZero();
            this->_dimensions_ = dimensions;
        };

        explicit JointSpace(const Eigen::VectorXd & data)
        {
            this->_data_ = data;
            _restrict();
            this->_dimensions_ = data.size();
        };
        explicit JointSpace(const double* data_ptr,int dimensions)
        {
            this->_data_.resize(dimensions);
            memcpy(this->_data_.data(), data_ptr, dimensions * sizeof(double));
            _restrict();
            this->_dimensions_ = dimensions;
        };
        ~JointSpace() override =default;

        Eigen::VectorXd Vector() const {return _data_;};
        JointSpace& operator=(const JointSpace& other)
        {
            this->_dimensions_ = other.Vector().size();
            this->_data_ = other.Vector();
            return *this;
        };
        JointSpace operator+(const JointSpace & input)const override
        {
            return JointSpace(this->Vector()+input.Vector());
        };
        JointSpace operator-(const JointSpace & input)const override
        {
            return JointSpace(this->Vector() - input.Vector());
        };
        JointSpace operator*(double s)const override
        {
            return JointSpace(s*this->Vector());
        };
        JointSpace operator()(double theta)const override
        {
            return JointSpace(this->Vector().normalized()*theta);
        };
        double& operator[](const long index)
        {
            return this->_data_[index];
        };
        double operator[](const long index)const
        {
            return this->_data_[index];
        };
        JointSpace& operator+=(const Eigen::VectorXd & input)
        {
            this->_data_+=input;
            return *this;
        };
        void* data()
        {
            return this->_data_.data();
        };
        JointSpace inverse() const override
        {
          return JointSpace(-1*this->Vector());
        };
        JointSpace random(std::default_random_engine & randomEngine, const Eigen::Matrix2Xd* bounds_ptr)const override
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
                std::uniform_real_distribution<double> x_distribution(-M_PI, M_PI);
                for(int i=0; i<this->_dimensions_; ++i)
                {
                    result[i] = x_distribution(randomEngine);
                }
            }
        };
        double distance(const JointSpace & to) const override
        {
            return (*this-to).norm();
        };
        double distance(const Eigen::VectorXd & input) const
        {
            return this->distance(JointSpace(input));
        };
        double norm() const
        {
            return this->_data_.norm();
        };
        int Dimensions() const
        {
            return _dimensions_;
        };
        long size() const
        {
            return this->_data_.size();
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


    private:
        const double _lower_limit_{-M_PI};
        const double _upper_limit_{M_PI};
    };

}



#endif //NURSINGROBOT_JOINTSPACE_HPP
