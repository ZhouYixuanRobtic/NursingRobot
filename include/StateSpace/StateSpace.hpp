
#ifndef NURSINGROBOT_STATESPACE_HPP
#define NURSINGROBOT_STATESPACE_HPP
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Core>

namespace state_space{
    //Rn
    typedef Eigen::VectorXd Rn;
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
        return (std::abs(val)<1e-8);
    }
    /**
    * A state space represents the set of possible states for a planning problem.
    * for example Rn SO2 SO3 SE3 SE3
    * This class is abstract and must be subclassed in order to provide actual
    * functionality.
    */
    template <typename T>
    class StateSpace{
    public:
        StateSpace() = default;
        virtual ~StateSpace() = default;

        //
        virtual T operator+(const T & input) const =0;
        virtual T operator-(const T & input) const =0;
        virtual T operator*(double s)  const =0;
        virtual T operator()(double theta)  const =0;

        virtual T inverse() const =0;
        virtual T random(std::default_random_engine & randomEngine,const Eigen::MatrixXd* bounds_ptr) const =0;
        virtual double distance(const T & to) const =0;

    protected:
        int _dimensions=0;
    };


    template <typename T>
    static T randomState(const Eigen::MatrixXd * bounds_ptr)
    {
        return T::random(bounds_ptr);
    }

    /**
     * Finds a state in the direction of @target from @source.state().
     * @param lambda The relative normalized distance from @source, which must
     * inside the range of [0,1]
     *
     * @return A state in the direction of @target from @source (hyper linear interpolation)
     */
    template <typename T>
    static T interpolate(const T& source, const T& target,
                          double lambda)
    {
            return source+(target-source)*lambda;
    }

    /**
     * An overloaded version designed for bezier interpolation
     *
     * @param source The start state
     * @param target The point in the space to extend to
     * @param lambda The relative normalized distance from @source, which must
     * inside the range of [0,1]
     *
     * @return A state in the bezier curve of @target from @source.state(), the control points are fixed
     * and divided equally.
     */
    template <typename T>
    static T bezierInterpolate(const T& source, const T& target,double lambda)
    {
        T delta = target - source;
        T result;
        result = source*(1*pow(lambda,0)*pow((1-lambda),5-0))+
                 (source-delta*2)*(5*pow(lambda,1)*pow((1-lambda),5-1))+
                 (source-delta)*(10*pow(lambda,2)*pow((1-lambda),5-2))+
                 (source+delta*3)*(10*pow(lambda,3)*pow((1-lambda),5-3))+
                 (source+delta*2)*(5*pow(lambda,4)*pow((1-lambda),5-4))+
                 target*(1*pow(lambda,5)*pow((1-lambda),5-5));
        return result;
    }

    /**
     * @brief Calculate the distance between two states
     *
     * @param from Start state
     * @param to End state
     *
     * @return The distance between the states
     */
    template <typename T>
    double distance(const T& from, const T& to)
    {
        return from.distance(to);
    }

    /**
     * @brief Calculate the control points smoothing the give states contained in @state_list
     *
     * @param state_list containing all states needed to be smoothed
     *
     * @return a vector contains all six control points for each segment.
     */
     //TODO: Not implemented
    template <typename T>
    std::vector<std::array<T,6>> getBezierControlPoints(const std::vector<T> & state_list)
    {
        std::vector<std::array<T,6>> control_points_list{};
        return control_points_list;
    }


}



#endif //NURSINGROBOT_STATESPACE_HPP
