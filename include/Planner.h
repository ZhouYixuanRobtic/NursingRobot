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
        bool isGoalReached(const Vertex<T> & node_end){return state_space::distance(node_end._data-_goal)<_step_len;};
    };
}



#endif //NURSINGROBOT_PLANNER_H
