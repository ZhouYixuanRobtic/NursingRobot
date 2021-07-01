#ifndef NURSINGROBOT_LAZYRRT_HPP
#define NURSINGROBOT_LAZYRRT_HPP
#include "planner/RRT.hpp"
namespace planner{

    template<typename T,typename Distance>
    class LazyRRT : public RRT<T,Distance>{
    protected:
        void _removeVertex(Vertex<T>* state_vertex)
        {
            if(state_vertex== nullptr)
                return;
            this->_tree_ptr->removeState(state_vertex);

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
        }/*
        void _animate(cv::Mat& draw, Vertex<T> *  vertex)
        {
            cv::circle(draw, cv::Point((int)vertex->state().Vector()[0], (int)vertex->state().Vector()[1]), 3, cv::Scalar{255, 0, 0}, 1);
            for(auto & it : vertex->children())
            {
                cv::line(draw, cv::Point((int)vertex->state().Vector()[0], (int)vertex->state().Vector()[1]),
                         cv::Point((int)it->state().Vector()[0], (int)it->state().Vector()[1]), cv::Scalar{255, 0, 0}, 2);
                _animate(draw,it);
            }
        }*/
    public:
        LazyRRT(const LazyRRT<T,Distance> &) = delete;

        LazyRRT &operator=(const LazyRRT<T,Distance> &) = delete;

        LazyRRT(std::function<size_t(T)> hashT, std::size_t dimensions,
                std::function<T(double *)> arrayToT = NULL,
                std::function<void(T, double *)> TToArray = NULL)
                : RRT<T,Distance>(hashT,dimensions,arrayToT,TToArray)
        {

        };

         ~LazyRRT() = default;

        bool planning() override
        {
            // validity check
            if(!this->_isPlanConstructed || !this->argsCheck())
                return false;
            time_t start(clock());
            double time{};
            for (int i = 0; i < this->_iter_max; ++i) {
                time = (double) (clock() - start) / CLOCKS_PER_SEC;
                if (time >= this->_time_limit) {
                    LOG(ERROR) << "No path find within " << this->_time_limit << " seconds"
                               << " now iterates " << i << "times";
                    this->_total_nodes = this->_tree_ptr->TreeSize();
                    return false;
                }

                Vertex<T> *new_vertex;
                double r = fabs(planner::randomState<state_space::Rn>(1, nullptr)[0]);

                if (r < this->_goal_bias)
                    new_vertex = this->_steer(this->_tree_ptr,this->_tree_ptr->Goal(), nullptr,false);
                else
                    new_vertex = this->_steer(this->_tree_ptr,this->_sample(), nullptr,false);
                if (new_vertex && this->_isReached(new_vertex,this->_tree_ptr->Goal(),false)) {
                    bool solution_found = true;
                    //here the path doesn't contain goal state;
                    std::vector<Vertex<T>*> path;
                    this->_tree_ptr->_extract_path(path,new_vertex);
                    for(int index=path.size()-2 ; index>=0 && solution_found; --index){
                        if(!this->_isStateValid(path[index]->parent()->state(),path[index]->state(),true)){
                            _removeVertex(path[index]);
                            solution_found = false;
                        }
                    }
                    if(solution_found && this->_isReached(new_vertex,this->_tree_ptr->Goal(),true))
                    {
                        this->_tree_ptr->addState(this->_tree_ptr->Goal(),new_vertex);
                        this->_tail = this->_tree_ptr->LastVertex();
                        this->_total_nodes = this->_tree_ptr->TreeSize();
                        return true;
                    }
                }
            }
            this->_total_nodes = this->_tree_ptr->TreeSize();
            LOG(ERROR) << "No path find within " << this->_iter_max<< " iterations";
            return false;
        }
        std::string getName() const override{
            return "Lazy_RRT";
        }
        std::size_t getTotalNodes() const override{
            return this->_total_nodes;
        };
        std::vector<Vertex<T>*> getRootVertex() override
        {
            std::vector<Vertex<T>*> results;
            results.template emplace_back(this->_tree_ptr->RootVertex());
            return results;
        }
    };


}





#endif //NURSINGROBOT_LAZYRRT_HPP
