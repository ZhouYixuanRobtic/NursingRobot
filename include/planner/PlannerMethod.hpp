#ifndef NURSINGROBOT_PLANNERMETHOD_HPP
#define NURSINGROBOT_PLANNERMETHOD_HPP
#include "planner/RRT.hpp"
#include "planner/LazyRRT.hpp"
#include "planner/RRTConnect.hpp"
#include "planner/LazyRRTConnect.hpp"
//#include "planner/RRTStar.hpp"
//#include "planner/IRRTStar.hpp"
namespace planner{
    template<typename T,typename Distance= flann::L1<double>>
    std::shared_ptr<RRT<T,Distance>> createPlanner(const PLANNER_TYPE &planner_type, std::size_t dimensions = 6)
    {
        switch (planner_type) {
            case RRT_SIMPLE:
                return std::shared_ptr<RRT<T,Distance>>(new RRT<T,Distance>(planner::hash<T>, dimensions));
            case RRT_CONNECT:
                return std::shared_ptr<RRT<T,Distance>>(new RRTConnect<T,Distance>(planner::hash<T>, dimensions));
            case LAZY_RRT:
                return std::shared_ptr<RRT<T,Distance>>(new LazyRRT<T,Distance>(planner::hash<T>, dimensions));
            case LAZY_RRT_CONNECT:
                return std::shared_ptr<RRT<T,Distance>>(new LazyRRTConnect<T,Distance>(planner::hash<T>, dimensions));
           // case RRT_STAR:
           //     return std::shared_ptr<RRT<T,Distance>>(new RRTStar<T,Distance>(planner::hash<T>, dimensions));
           // case INFORMED_RRT_STAR:
          //      return std::shared_ptr<RRT<T,Distance>>(new IRRTStar<T,Distance>(planner::hash<T>, dimensions));
            default:
                return std::shared_ptr<RRT<T,Distance>>(new RRT<T,Distance>(planner::hash<T>, dimensions));
        }
    }
}

#endif //NURSINGROBOT_PLANNERMETHOD_HPP
