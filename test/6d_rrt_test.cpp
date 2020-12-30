#define EIGEN_MKL_USE_ALL
#include "StateSpace/JointSpace.hpp"
#include "StateSpace/rn.hpp"
#include "StateSpace/SE3.hpp"
#include "StateSpace/SO3.hpp"
#include "planner/RRT.hpp"


int main(int argc, char** argv)
{
   using SPCIFIC_STATE=state_space::JointSpace;
   int dimensions = 4;
   auto start_state = planner::randomState<SPCIFIC_STATE>(nullptr,dimensions);
   auto goal_state = planner::randomState<SPCIFIC_STATE>(nullptr,dimensions);

   std::shared_ptr<planner::RRT<SPCIFIC_STATE>> rrt_2d = std::make_shared<planner::RRT<SPCIFIC_STATE>>(start_state,goal_state,planner::hash<SPCIFIC_STATE>,start_state.Dimensions());

   rrt_2d->setStepLen(0.01);
   clock_t start(clock());
   if(rrt_2d->planning())
   {
       clock_t end(clock());
       std::cout<<(double)(end-start)/CLOCKS_PER_SEC<<std::endl;
       std::vector<SPCIFIC_STATE> path = rrt_2d->GetPath();
       for(const auto& it: path)
       {
           std::cout<<it.Vector().transpose()<<std::endl;
       }
   }
}