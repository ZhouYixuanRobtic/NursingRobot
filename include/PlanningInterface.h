//
// Created by xcy on 2020/12/14.
//

#ifndef NURSINGROBOT_PLANNINGINTERFACE_H
#define NURSINGROBOT_PLANNINGINTERFACE_H
#include "RobotModel.h"
#include "StateSpace/SE3.hpp"
class PlanningInterface {
private:
    state_space::R6 joint_goal_{};
    state_space::SE3 cartesian_goal_{};

public:

};


#endif //NURSINGROBOT_PLANNINGINTERFACE_H
