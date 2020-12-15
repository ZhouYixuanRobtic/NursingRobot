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
        std::vector<state_space::Rn> extractPath(const Vertex<T> & node_end) override;
        bool not_in_collision(){return true;};
        state_space::Rn sample() override;
        Vertex<T> get_nearest_neighbor(const state_space::Rn & current_state) override;
        Vertex<T> generate_new_state(const Vertex<T> & nearest_vertex, const state_space::Rn & rand_state);

        Vertex<T>* _head;
        Vertex<T>* _tail;
    public:
        explicit RRT(const state_space::Rn & start, const state_space::Rn & goal, int iter_max, double step_len,
                     double goal_sample_rate);
        ~RRT();
        std::vector<state_space::Rn> planning() override;
    };
}



#endif //NURSINGROBOT_RRT_H
