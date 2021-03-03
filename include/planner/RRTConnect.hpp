#ifndef NURSINGROBOT_RRTCONNECT_HPP
#define NURSINGROBOT_RRTCONNECT_HPP
#include "planner/RRT.hpp"
namespace planner{

    template<typename T>
    class RRTConnect: public RRT<T>{
    protected:
        

    public:
        RRTConnect(const RRTConnect<T> &) = delete;

        RRTConnect &operator=(const RRTConnect<T> &) = delete;

        RRTConnect(std::function<size_t(T)> hashT, std::size_t dimensions, bool forward = true,
                   std::function<T(double *)> arrayToT = NULL,
                   std::function<void(T, double *)> TToArray = NULL)
                : RRT<T>(hashT,dimensions,forward,arrayToT,TToArray)
        {

        };
        ~RRTConnect() = default;

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

                }

            }
            LOG(ERROR) << "No path find within " << this->_iter_max<< " iterations";
            return false;
        }
    };
}




#endif //NURSINGROBOT_RRTCONNECT_HPP
