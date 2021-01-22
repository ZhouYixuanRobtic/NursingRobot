#ifndef NURSINGROBOT_PLANNINGINTERFACE_H
#define NURSINGROBOT_PLANNINGINTERFACE_H

#include "planner/CartesianPlanner.h"
#include "planner/PlannerMethod.hpp"

namespace planner{
    class PlanningInterface{
    private:
        std::shared_ptr<RRT<state_space::JointSpace>> _rrt_ptr;
        my_collision_detection::MoveItCollisionHelperPtr _moveit_collision_helper_ptr;
        std::size_t _joint_nums;
    public:
        explicit PlanningInterface(const PLANNER_TYPE & planner_type,
                                   const std::string &group_name,
                                   const std::string &yaml_name,
                                   const my_kinematics::analytical_ik_handled_t &analytical_ik_func);
        ~PlanningInterface() = default;

        const my_collision_detection::MoveItCollisionHelperPtr &getMoveItCollisionHelperPtr()
        {
            return _moveit_collision_helper_ptr;
        }

        state_space::JointSpace getRandomValidJointState();

        bool computeJointPath(state_space::vector_JointSpace &path,
                              const PLAN_REQUEST<state_space::JointSpace> &plan_request);

        bool computeJointPath(state_space::vector_JointSpace &path,
                              const state_space::JointSpace & goal,
                              double time_limit = 5);

        double computeCartesianPath(const state_space::vector_SE3 &wayPoints,
                                    state_space::vector_JointSpace &trajectory,
                                    const MaxEEFStep &max_step,
                                    const JumpThreshold &jump_threshold,
                                    const std::string &reference_frame);

    };


}



#endif //NURSINGROBOT_PLANNINGINTERFACE_H
