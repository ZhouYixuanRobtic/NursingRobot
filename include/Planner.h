//
// Created by xcy on 2020/12/14.
//

#ifndef NURSINGROBOT_PLANNER_H
#define NURSINGROBOT_PLANNER_H
#include <utility>

#include "StateSpace/SE3.hpp"
#include "RobotModel.h"
namespace planner{

    template <typename T>
    static T randomState(const Eigen::Matrix2Xd * bounds_ptr)
    {
        return T().random(bounds_ptr);
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

    template <typename T>
    class Vertex{
    public:
        explicit Vertex(const T & data, const double distance_from_parent =0)
                :_data(data),_parent(nullptr),_distance_from_parent(distance_from_parent){}
    private:
        T _data;
        Vertex* _parent;
        double _distance_from_parent;
    };

    template <typename T>
    class Planner {
    protected:

        int _iter_max{};
        double _step_len{},_goal_sample_rate{};
        T _start,_goal;
        virtual std::vector<T> extractPath(const Vertex<T> & node_end)=0;
    public:
        Planner() =default;
        virtual ~Planner() =default;
        virtual std::vector<T> planning() =0;
        virtual T sample()=0;
        virtual Vertex<T> get_nearest_neighbor(const T & current_state)=0;
        bool isGoalReached(const Vertex<T> & node_end){return planner::distance(node_end._data-_goal)<_step_len;};
    };
}



#endif //NURSINGROBOT_PLANNER_H
