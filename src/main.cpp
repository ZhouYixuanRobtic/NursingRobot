#define Eigen_MKL_USE_ALL

#include <iostream>
#include <Eigen/Core>
#include "RobotModel/joint_model.h"
#include "StateSpace/SO3.hpp"
#include "StateSpace/SE3.hpp"
#include "planner/Planner.hpp"

int main()
{

    state_space::SO_3 B;
    B << 1, 2, 3, 4, 5, 6, 7, 7, 9;
    state_space::SO3::projectToSO3(B);


}
