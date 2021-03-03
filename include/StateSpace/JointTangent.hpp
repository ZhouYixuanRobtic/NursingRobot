#ifndef NURSINGROBOT_JOINTTANGENT_HPP
#define NURSINGROBOT_JOINTTANGENT_HPP

namespace state_space{
    DONT_ALIGN_CLASS_STL_FORWARD(JointTangent)
    class JointTangent {
    protected:
        bool _continuous_;
        unsigned int _dimensions_;
        Eigen::VectorXd _data_;

        //if continuous make the Path shortest
        static void _restrict(double &val)
        {
            int sign = (val < 0) ? -1 : (val > 0);
            val = fabs(fmod(val,2.0 * M_PI));
            val = val > M_PI ? 2.0 * M_PI - val : val;
            val *= -sign;
        }

        void _restrict()
        {
            for (int i = 0; i < _data_.size(); ++i) {
                _restrict(_data_[i]);
            }
        }
    public:
        JointTangent(const JointTangent &other) = default;

        explicit JointTangent(unsigned int dimensions = 6, bool continuous = false)
                :_dimensions_(dimensions),
                 _continuous_(continuous)
        {
            this->_data_.resize(dimensions);
            this->_data_.setZero();
        }
        explicit JointTangent(const Eigen::VectorXd &data,bool continuous = false)
                :_dimensions_(data.size()),
                 _continuous_(continuous),
                 _data_(data)
        {
            if(continuous) _restrict();
        }
        explicit JointTangent(const double *data_ptr, unsigned int dimensions,bool continuous = false)
                : _dimensions_(dimensions),
                  _continuous_(continuous)
        {
            this->_data_.resize(dimensions);
            memcpy(this->_data_.data(), data_ptr, dimensions * sizeof(double));
            if(continuous) _restrict();
        };

        explicit JointTangent(const std::vector<double> &vector_input,bool continuous = false)
                :_continuous_(continuous),
                 _dimensions_(vector_input.size())
        {
            this->_data_.resize(_dimensions_);
            memcpy(this->_data_.data(), vector_input.data(), _dimensions_ * sizeof(double));
            if(continuous) _restrict();
        }

        ~JointTangent()  = default;

        Eigen::VectorXd Vector() const
        { return _data_; };

        bool isContinuous() const
        {return _continuous_;};

        JointTangent &operator=(const JointTangent &other)= default;

        double &operator[](const size_t index)
        {
            return this->_data_[index];
        };

        double operator[](const long index) const
        {
            return this->_data_[index];
        };

        JointTangent  &operator+=(const Eigen::VectorXd &input)
        {
            this->_data_ += input;
            return *this;
        };

        JointTangent  &operator+=(const JointTangent &input)
        {
            this->_data_ += input._data_;
            return *this;
        };

        bool operator==(const JointTangent &other) const
        {
            return _data_ == other._data_;
        }

        const double *data() const
        {
            return this->_data_.data();
        };

        JointTangent inverse() const
        {
            return JointTangent(-1 * this->Vector(),this->_continuous_);
        };

        JointTangent operator+(const JointTangent &input) const
        {
            return JointTangent(this->Vector() + input.Vector(), this->_continuous_);
        };

        JointTangent operator-(const JointTangent &input) const
        {
            return JointTangent(this->Vector() - input.Vector(), this->_continuous_);
        };

        JointTangent operator*(double s) const
        {
            return JointTangent(s * this->Vector(),this->_continuous_);
        };

        double norm() const
        {
            return this->_data_.norm();
        };
    };
}

#endif //NURSINGROBOT_JOINTTANGENT_HPP
