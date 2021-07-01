#ifndef NURSINGROBOT_LAZYRRTCONNECT_HPP
#define NURSINGROBOT_LAZYRRTCONNECT_HPP
namespace planner{
    template<typename T, typename Distance>
    class LazyRRTConnect : public RRTConnect<T,Distance>{
    protected:
        void _removeVertex(Vertex<T>* state_vertex, std::shared_ptr<Tree<T,Distance>> & tree)
        {
            if(state_vertex == nullptr)
                return;
            tree->removeState(state_vertex);
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
                _removeVertex(item,tree);
            }

        }

        using RRTConnect<T,Distance>::_steer;
    public:
        LazyRRTConnect(const LazyRRTConnect<T,Distance> &) = delete;

        LazyRRTConnect &operator=(const LazyRRTConnect<T,Distance> &) = delete;

        LazyRRTConnect(std::function<size_t(T)> hashT, std::size_t dimensions,
                std::function<T(double *)> arrayToT = NULL,
                std::function<void(T, double *)> TToArray = NULL)
            : RRTConnect<T,Distance>(hashT,dimensions,arrayToT,TToArray)
        {

        };
        ~LazyRRTConnect() = default;

        bool planning() override
        {
            // validity check
            if(!this->_isPlanConstructed||!this->argsCheck())
                return false;

            //when start state is near goal, check is the path valid
            if ((!this->_is_step_relative && this->_isReached(this->_tree_ptr->RootVertex(),this->_tree_ptr->Goal(),true))
                || this->_is_step_relative && this->_d_min <= std::numeric_limits<double>::epsilon()) {
                this->_tail = this->_tree_ptr->RootVertex();
                this->_other_tail = this->_other_tree_ptr->RootVertex();
                return true;
            }
            this->start = (clock());
            double time{};
            for (int i = 0; i < this->_iter_max; ++i) {
                time = (double) (clock() - this->start) / CLOCKS_PER_SEC;
                if (time >= this->_time_limit) {
                    LOG(ERROR) << "No path find within " << this->_time_limit << " seconds"
                               << " now iterates " << i << "times";
                    this->_total_nodes = this->_tree_ptr->TreeSize()+this->_other_tree_ptr->TreeSize();
                    return false;
                }
                Vertex<T> *new_vertex=_steer(this->_tree_ptr,this->_sample(), nullptr,false);
                if (new_vertex) {
                    auto temp_tail = this->_connect(new_vertex,false);
                    if(temp_tail){
                        bool solution_found1{true},solution_found2{true};
                        //here the path doesn't contain goal state;
                        std::vector<Vertex<T>*> path;
                        this->_tree_ptr->_extract_path(path,new_vertex);
                        for(int index=path.size()-2 ; index>=0 && solution_found1; --index){
                            if(!this->_isStateValid(path[index]->parent()->state(),path[index]->state(),true)){
                                _removeVertex(path[index],this->_tree_ptr);
                                solution_found1 = false;
                            }
                        }
                        path.clear();
                        this->_other_tree_ptr->_extract_path(path,temp_tail);
                        for(int index=path.size()-2 ; index>=0 && solution_found2; --index){
                            if(!this->_isStateValid(path[index]->parent()->state(),path[index]->state(),true)){
                                _removeVertex(path[index],this->_other_tree_ptr);
                                solution_found2 = false;
                            }
                        }
                        if(solution_found1 && solution_found2 && this->_isReached(new_vertex,temp_tail->state(),true))
                        {
                            this->_tail = new_vertex;
                            this->_other_tail = temp_tail;
                            this->reverse = i %2 == 1;
                            this->_total_nodes = this->_tree_ptr->TreeSize()+this->_other_tree_ptr->TreeSize();
                            return true;
                        }
                    }
                }

                if(this->_other_tree_ptr->TreeSize()< this->_tree_ptr->TreeSize())
                    this->_tree_ptr.swap(this->_other_tree_ptr);
            }
            this->_total_nodes = this->_tree_ptr->TreeSize()+this->_other_tree_ptr->TreeSize();
            LOG(ERROR) << "No path find within " << this->_iter_max<< " iterations";
            return false;
        }
        std::string getName() const override{
            return "Lazy_RRT_Connect";
        }
        std::size_t getTotalNodes() const override{
            return this->_total_nodes;
        };
        std::vector<Vertex<T>*> getRootVertex() override
        {
            std::vector<Vertex<T>*> results;
            results.template emplace_back(this->_tree_ptr->RootVertex());
            results.template emplace_back(this->_other_tree_ptr->RootVertex());
            return results;
        }
    };

}

#endif //NURSINGROBOT_LAZYRRTCONNECT_HPP
