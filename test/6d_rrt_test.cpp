#include "StateSpace/JointSpace.hpp"
#include "StateSpace/rn.hpp"
#include "RRT.hpp"


/**
 *   test RRT with kd tree on R2 space and R3 space and J6 space
 */

int main(int argc, char** argv)
{
   using SPCIFIC_STATE=state_space::JointSpace;
   int dimensions = 6;
   auto start_state = planner::randomState<SPCIFIC_STATE>(nullptr,dimensions);
   auto goal_state = planner::randomState<SPCIFIC_STATE>(nullptr,dimensions);

   std::tr1::shared_ptr<planner::RRT<SPCIFIC_STATE>> rrt_2d(
           new planner::RRT<SPCIFIC_STATE>(start_state,goal_state,planner::hash<SPCIFIC_STATE>,start_state.Dimensions()));

    rrt_2d->setStepLen(0.01);
    if(rrt_2d->planning())
    {
        std::vector<SPCIFIC_STATE> path = rrt_2d->GetPath();
        for(const auto& it: path)
        {
            std::cout<<it.Vector().transpose()<<std::endl;
        }
    }
}
