#include "planner/PlanningInterface.h"

namespace planner {
    PlanningInterface::PlanningInterface(const PLANNER_TYPE &planner_type,
                                         const std::string &group_name,
                                         const std::string &yaml_name,
                                         const my_kinematics::analytical_ik_handled_t &analytical_ik_func)
            : _moveit_collision_helper_ptr(
            new my_collision_detection::MoveItCollisionHelper(group_name, yaml_name, analytical_ik_func)),
              _joint_nums(_moveit_collision_helper_ptr->getKinematicsPtr()->getJointNums())
    {
        _rrt_ptr = createPlanner<state_space::JointSpace>(planner_type, _joint_nums);
        _rrt_ptr->setStateValidator(
                std::function<bool(const state_space::JointSpace &, const state_space::JointSpace &)>(
                        std::bind(&my_collision_detection::MoveItCollisionHelper::isPathValid<state_space::JointSpace>,
                                  _moveit_collision_helper_ptr, std::placeholders::_1, std::placeholders::_2)));
    }

    state_space::JointSpace PlanningInterface::getRandomValidJointState()
    {
        auto start_state = planner::randomState<state_space::JointSpace>(_joint_nums);
        clock_t start(clock());
        double time{};
        while (true) {
            time = (double) (clock() - start) / CLOCKS_PER_SEC;
            start_state = planner::randomState<state_space::JointSpace>(_joint_nums);
            if (_moveit_collision_helper_ptr->isStateValid(start_state))
                return start_state;
            if (time > 5) {
                LOG(WARNING) << "Generate valid state failed out of 5 s";
                break;
            }
        }
        return start_state;
    }

    bool PlanningInterface::computeJointPath(state_space::vector_JointSpace &path,
                                             const PLAN_REQUEST<state_space::JointSpace> &plan_request)
    {
        _rrt_ptr->constructPlan(plan_request);

        bool found_path = _rrt_ptr->planning();
        if (found_path)
            path = _rrt_ptr->GetPath();
        return found_path;
    }

    bool PlanningInterface::computeJointPath(state_space::vector_JointSpace &path,
                                             const state_space::JointSpace &goal,
                                             double time_limit)
    {
        auto plan_request = PLAN_REQUEST<state_space::JointSpace>(_moveit_collision_helper_ptr->getCurrentJointAngles(),
                                                                  goal,
                                                                  time_limit,
                                                                  0.1,
                                                                  0.1);

        return computeJointPath(path, plan_request);
    }

    double PlanningInterface::computeCartesianPath(const state_space::vector_SE3 &wayPoints,
                                                   state_space::vector_JointSpace &trajectory,
                                                   const MaxEEFStep &max_step, const JumpThreshold &jump_threshold,
                                                   const std::string &reference_frame)
    {
        auto current_joint_angles = _moveit_collision_helper_ptr->getCurrentJointAngles();
        double percentage_solved = CartesianPlanner::computeCartesianPath(current_joint_angles, wayPoints, trajectory,
                                                                          max_step, jump_threshold, reference_frame,
                                                                          _moveit_collision_helper_ptr);
        //add different first point trajectory
        if(percentage_solved == 1.0 && planner::distance(current_joint_angles,trajectory[0]) > 1e-2){
            state_space::vector_JointSpace joint_path;
            auto plan_request = PLAN_REQUEST<state_space::JointSpace>(_moveit_collision_helper_ptr->getCurrentJointAngles(),
                                                                      trajectory[0],
                                                                      5,0.005,0.005,false);
            if(computeJointPath(joint_path,plan_request)){
                auto end_iter = joint_path.end();
                std::advance(end_iter,-2);
                trajectory.insert(trajectory.begin(),joint_path.begin(),end_iter);
            }else{
                percentage_solved = (double)(trajectory.size()-1)/trajectory.size();
            }
        }
        return percentage_solved;

    }

}