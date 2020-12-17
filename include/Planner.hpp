//
// Created by xcy on 2020/12/14.
//

#ifndef NURSINGROBOT_PLANNER_HPP
#define NURSINGROBOT_PLANNER_HPP
#include <utility>

#include "StateSpace/SE3.hpp"
#include "StateSpace/JointSpace.hpp"
#include "StateSpace/SO3.hpp"
#include "StateSpace/rn.hpp"
#include <unordered_map>
#include <deque>
#include <flann/flann.hpp>
#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <stdexcept>
#include <type_traits>
#include <functional>
#include <cstdlib>
#include <list>
namespace planner{

    template <typename T>
    static T randomState(const Eigen::Matrix2Xd * bounds_ptr)
    {
        static std::random_device rd;
        static std::default_random_engine randomEngine(rd());
        return T().random(randomEngine,bounds_ptr);
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
        explicit Vertex(const T & data, Vertex<T>* parent = nullptr, int dimensions = 6, std::function<void(T,double*)>TToArray = NULL)
                : _state(data), _parent(parent)
                {
                    _vec.resize(dimensions);
                    if(_parent)
                    {
                        _parent->_children.push_back(this);
                    }
                    if(NULL == TToArray)
                    {
                        _vec = data.Vector();
                    }
                    else
                    {
                        TToArray(data,_vec.data());
                    }
                };
        const Vertex<T>* parent() const {return  _parent;};

        int depth() const{
            int n=0;
            Vertex<T>* ancestor = _parent;
            while( ancestor != nullptr)
            {
                n++;
                ancestor = ancestor->_parent;
            }
            return n;
        };

        const T& state() const {return _state;};

        Eigen::VectorXd* coordinates() {return &_vec;};
    private:
        Eigen::VectorXd _vec;
        T _state;
        Vertex* _parent;
        std::list<Vertex<T>*> _children;
    };

}



#endif //NURSINGROBOT_PLANNER_HPP