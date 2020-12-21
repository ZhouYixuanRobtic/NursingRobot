#include <iostream>
#include "StateSpace/SE3.hpp"
#include "StateSpace/SO3.hpp"
#include "RobotModel.h"
#include <ctime>
#include "RobotModel.cpp"
#include "StateSpace/StateSpace.hpp"
#include "Planner.hpp"
int main()
{
    Eigen::MatrixX2d bounds(3,2);
    bounds<<0.6,0.4,
            0.6,0.4,
            0.6,0.4;
    double b=0.1;
    int iter=-1;
    while(++iter<100)
    {
        state_space::SE3 a=planner::randomState<state_space::SE3>(&bounds);
        std::cout<<a.SE3Matrix()<<std::endl;
    }


}