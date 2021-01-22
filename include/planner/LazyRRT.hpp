#ifndef NURSINGROBOT_LAZYRRT_HPP
#define NURSINGROBOT_LAZYRRT_HPP
#include "planner/RRT.hpp"
namespace planner{

    template<typename T>
    class LazyRRT : public RRT<T>{
    protected:
        void _removeVertex(Vertex<T>* & state_vertex)
        {
            std::size_t point_id = this->_kd_tree_hash_map.at(this->_node_map.at(state_vertex->state()));
            this->_kd_tree->removePoint(point_id);
            this->_node_map.erase(state_vertex->state());

            //remove self from parent list
            if(state_vertex->parent()){
                for(auto iter =state_vertex->parent()->children().cbegin(); iter != state_vertex->parent()->children().cend();++iter){
                    if( *iter == state_vertex){
                        state_vertex->parent()->children().erase(iter);
                        break;
                    }
                }
            }

            //remove all children
            for(auto & item : state_vertex->children()){
                item->parent() = nullptr;
                _removeVertex(item);
            }
        }

    public:
        LazyRRT(const LazyRRT<T> &) = delete;

        LazyRRT &operator=(const LazyRRT<T> &) = delete;

        LazyRRT(std::function<size_t(T)> hashT, std::size_t dimensions, bool forward = true,
                std::function<T(double *)> arrayToT = NULL,
                std::function<void(T, double *)> TToArray = NULL)
                : RRT<T>(hashT,dimensions,forward,arrayToT,TToArray)
        {

        };

         ~LazyRRT() = default;

        bool planning() override
        {
            // validity check
            if(!this->argsCheck())
                return false;
            time_t start(clock());
            double time{};
            for (int i = 0; i < this->_iter_max; ++i) {
                time = (double) (clock() - start) / CLOCKS_PER_SEC;
                if (time >= this->_time_limit) {
                    LOG(ERROR) << "No path find within " << this->_time_limit << " seconds"
                               << " now iterates " << i << "times";
                    return false;
                }
                Vertex<T> *new_vertex;

                double r = fabs(planner::randomState<state_space::Rn>(1, nullptr)[0]);

                if (r < this->_goal_bias)
                    new_vertex = this->_steer(this->_goal, nullptr,false);
                else
                    new_vertex = this->_steer(this->_sample(), nullptr,false);
                if (new_vertex && this->_isGoalReached(new_vertex)) {
                    this->_tail = new_vertex;
                    bool solution_found = true;
                    //here the path doesn't contain goal state;
                    std::vector<Vertex<T>*> path;
                    this->_extract_path(path);
                    for(int index=path.size()-2 ; index>=0 && solution_found; --index){
                        if(!this->_isStateValid(path[index]->parent()->state(),path[index]->state(),true)){
                            _removeVertex(path[index]);
                            solution_found = false;
                        }
                    }
                    if(solution_found)
                        return true;
                }

            }
            LOG(ERROR) << "No path find within " << this->_iter_max<< " iterations";
            return false;
        }
    };


}





#endif //NURSINGROBOT_LAZYRRT_HPP
