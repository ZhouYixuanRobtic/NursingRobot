#ifndef NURSINGROBOT_IRRTSTAR_HPP
#define NURSINGROBOT_IRRTSTAR_HPP
#include "planner/RRTStar.hpp"
namespace planner{
    template<typename T,typename Distance>
    class IRRTStar : public RRTStar<T,Distance>{
    protected:

    public:
        IRRTStar(const IRRTStar<T,Distance> &) = delete;

        IRRTStar &operator=(const IRRTStar<T,Distance> &) = delete;

        IRRTStar(std::function<size_t(T)> hashT, std::size_t dimensions, bool forward = true,
                std::function<T(double *)> arrayToT = NULL,
                std::function<void(T, double *)> TToArray = NULL)
                : RRTStar<T,Distance>(hashT,dimensions,forward,arrayToT,TToArray)
        {

        };
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
                if (new_vertex && this->_isReached(new_vertex)) {

                }

            }
            LOG(ERROR) << "No path find within " << this->_iter_max<< " iterations";
            return false;
        }
    };
}


#endif //NURSINGROBOT_IRRTSTAR_HPP
