#ifndef NURSINGROBOT_PLANNINGINTERFACE_H
#define NURSINGROBOT_PLANNINGINTERFACE_H

#include "planner/CartesianPlanner.h"
#include "planner/RRT.hpp"

namespace planner{
    class PlanningInterface{
    private:
        std::shared_ptr<RRT<state_space::JointSpace>> _rrt_ptr;
        my_collision_detection::MoveItCollisionHelperPtr _moveit_collision_helper_ptr;
    public:
        explicit PlanningInterface(const PLANNER_TYPE & planner_type,
                                   const std::string &group_name,
                                   const std::string &yaml_name,
                                   const my_kinematics::analytical_ik_handled_t &analytical_ik_func);
        ~PlanningInterface() = default;

        bool computeJointPath(state_space::vector_JointSpace &path,
                              const PLAN_REQUEST<state_space::JointSpace> &plan_request);

        bool computeCartesianPath(state_space::vector_JointSpace &path,
                                  )
    };


}



#endif //NURSINGROBOT_PLANNINGINTERFACE_H
