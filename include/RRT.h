//
// Created by xcy on 2020/12/15.
//

#ifndef NURSINGROBOT_RRT_H
#define NURSINGROBOT_RRT_H
#include "Planner.h"
namespace planner{
    template <typename T>
    class RRT: public Planner<T> {
    protected:
        std::vector<T> extractPath(const Vertex<T> & node_end) override;
        bool not_in_collision(){return true;};
        T sample() override;
        Vertex<T> get_nearest_neighbor(const T & current_state) override;
        Vertex<T> generate_new_state(const Vertex<T> & nearest_vertex, const T & rand_state);

        Vertex<T>* _head;
        Vertex<T>* _tail;
    public:
        explicit RRT(const T & start, const T & goal, int iter_max, double step_len,
                     double goal_sample_rate);
        ~RRT();
        std::vector<T> planning() override;
    };
}



#endif //NURSINGROBOT_RRT_H
