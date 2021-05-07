#ifndef NURSINGROBOT_RRTCONNECT_HPP
#define NURSINGROBOT_RRTCONNECT_HPP
#include "planner/RRT.hpp"
namespace planner{

    template<typename T,typename Distance>
    class RRTConnect: public RRT<T,Distance>{
    protected:
        enum {
            TRAPPED,
            REACHED,
            ADVANCED
        }steer_status;
        std::shared_ptr<Tree<T,Distance>> _other_tree_ptr;

        Vertex<T>* _connect(Vertex<T>* vertex,bool check_collision)
        {
            _steer(_other_tree_ptr,vertex->state(), nullptr,check_collision);
            while(steer_status == ADVANCED)
            {
                _steer(_other_tree_ptr,vertex->state(), nullptr,check_collision);
            }
            return steer_status == REACHED ? this->_other_tree_ptr->LastVertex() : nullptr;
        }

        Vertex<T>* _other_tail;

        bool reverse{false};


        Vertex <T> * _steer(std::shared_ptr<Tree<T,Distance>> & tree_ptr,const T &rand_state, Vertex <T> *source, bool check_collision) override
        {
            auto nearest_vertex = RRT<T,Distance>::_steer(tree_ptr,rand_state,source,check_collision);
            steer_status = nearest_vertex != nullptr ? ADVANCED : TRAPPED;
            steer_status = (nearest_vertex != nullptr && this->_isReached(nearest_vertex,rand_state,check_collision)) ? REACHED : steer_status;

            return nearest_vertex;
        }

    public:
        RRTConnect(const RRTConnect<T,Distance> &) = delete;

        RRTConnect &operator=(const RRTConnect<T,Distance> &) = delete;

        RRTConnect(std::function<size_t(T)> hashT, std::size_t dimensions,
                   std::function<T(double *)> arrayToT = NULL,
                   std::function<void(T, double *)> TToArray = NULL)
                : RRT<T,Distance>(hashT,dimensions,arrayToT,TToArray),
                _other_tree_ptr(std::make_shared<Tree<T,Distance>>(hashT,dimensions,arrayToT,TToArray))
        {

        };
        ~RRTConnect() = default;

        void constructPlan(const PLAN_REQUEST<T,Distance> &rrtRequest) override
        {
            RRT<T,Distance>::constructPlan(rrtRequest);
            _other_tree_ptr->setStartState(this->GoalState());
            _other_tree_ptr->setGoalState(this->StartState());
        }

        std::vector<T> GetPath() override
        {
            std::vector<T> path1;
            this-> _tree_ptr->extract_path(path1,this->_tail, reverse);
            std::vector<T> path2;
            _other_tree_ptr->extract_path(path2,_other_tail,!reverse);
            if(!reverse)
                path1.insert(path1.end(),path2.begin(),path2.end());
            else
                path2.insert(path2.end(),path1.begin(),path1.end());
            return reverse ? path2 : path1;
        }

        bool planning() override
        {
            // validity check
            if(!this->_isPlanConstructed||!this->argsCheck())
                return false;
            //when start state is near goal, check is the path valid
            if ((!this->_is_step_relative && this->_isReached(this->_tree_ptr->RootVertex(),this->_tree_ptr->Goal(),true))
                || this->_is_step_relative && this->_d_min <= std::numeric_limits<double>::epsilon()) {
                this->_tail = this->_tree_ptr->RootVertex();
                _other_tail = _other_tree_ptr->RootVertex();
                return true;
            }
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
                    new_vertex = _steer(this->_tree_ptr,this->_tree_ptr->Goal(), nullptr,true);
                else
                    new_vertex = _steer(this->_tree_ptr,this->_sample(), nullptr,true);
                if (new_vertex) {
                    auto temp_tail = _connect(new_vertex,true);
                    if(temp_tail){
                        this->_tail = new_vertex;
                        _other_tail = temp_tail;
                        reverse = i%2 == 1;
                        return true;
                    }
                }
                this->_tree_ptr.swap(_other_tree_ptr);
            }
            LOG(ERROR) << "No path find within " << this->_iter_max<< " iterations";
            return false;
        }
    };
}




#endif //NURSINGROBOT_RRTCONNECT_HPP
