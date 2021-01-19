#include "planner/PlanningInterface.h"
namespace planner{
    PlanningInterface::PlanningInterface(const PLANNER_TYPE & planner_type,
                                         const std::string &group_name,
                                         const std::string &yaml_name,
                                         const my_kinematics::analytical_ik_handled_t &analytical_ik_func)
    :_moveit_collision_helper_ptr(new my_collision_detection::MoveItCollisionHelper(group_name,yaml_name,analytical_ik_func))
    {
        _rrt_ptr = createPlanner<state_space::JointSpace>(planner_type,6);
    }

    bool PlanningInterface::computeJointPath(state_space::vector_JointSpace &path,
                                             const PLAN_REQUEST<state_space::JointSpace> &plan_request)
    {

        _rrt_ptr->setStepLen(0.005);
        _rrt_ptr->setGoalMaxDist(0.005);
        _rrt_ptr->constructPlan(plan_request);

        _rrt_ptr->setStateValidator(std::function<bool(const state_space::JointSpace &, const state_space::JointSpace &)>(
                std::bind(&my_collision_detection::MoveItCollisionHelper::isPathValid<state_space::JointSpace>,
                          _moveit_collision_helper_ptr, std::placeholders::_1, std::placeholders::_2)));

        bool found_path = _rrt_ptr->planning();
        if(found_path)
            path = _rrt_ptr->GetPath();
        return found_path;
    }

}