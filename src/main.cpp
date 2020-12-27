#include <iostream>
#include "StateSpace/SE3.hpp"
#include "StateSpace/SO3.hpp"
#include "Kinematics/Kinematics.h"
#include <ctime>
#include "StateSpace/StateSpace.hpp"
#include "planner/Planner.hpp"

int main()
{
    Eigen::MatrixX2d bounds(2,2);
    bounds<<680,0,
            480,0;
    state_space::Rn start_state = planner::randomState<state_space::Rn>(&bounds,2);
    state_space::Rn goal_state = planner::randomState<state_space::Rn>(&bounds,2);
    std::cout<<goal_state.Vector().transpose()<<std::endl;
    std::cout<<start_state.Vector().transpose()<<std::endl;
}
